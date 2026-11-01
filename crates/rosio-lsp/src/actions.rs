//! Code actions on rosio diagnostics.
//!
//! A finding whose fix is an exact replacement applies it, and any finding can be waived: a
//! finding a team has decided about, with the reason it was accepted, so a CI gate stops
//! grading it while every report still shows it.
use crate::atlas::{Atlas, Evidence};
use crate::paths::Paths;
use lsp_types::Url;
use serde_json::{json, Value};
use std::path::Path;

/// The waiver file `rosio drift` reads by convention, at the workspace root.
pub const WAIVER_FILE: &str = "rosio-waivers.yaml";

/// The reason a person has to write before the waiver means anything.
const REASON_PLACEHOLDER: &str = "write why this finding is accepted";

/// One `quickfix` per rosio diagnostic in the request that is not waived yet.
pub fn code_actions(workspace: &Path, params: &Value) -> Value {
    let path = workspace.join(WAIVER_FILE);
    let Ok(uri) = Url::from_file_path(&path) else {
        return json!([]);
    };
    let existing = std::fs::read_to_string(&path).ok();
    let diagnostics = params["context"]["diagnostics"].as_array();
    let actions: Vec<Value> = diagnostics
        .into_iter()
        .flatten()
        .filter(|diagnostic| diagnostic["source"] == "rosio")
        .filter_map(|diagnostic| {
            let finding = diagnostic["data"]["finding"].as_str()?;
            if existing
                .as_deref()
                .is_some_and(|text| text.contains(finding))
            {
                return None;
            }
            let mut action = json!({
                "title": format!("Waive this finding in {WAIVER_FILE}"),
                "kind": "quickfix",
                "diagnostics": [diagnostic],
            });
            match waiver_edit(&uri, existing.as_deref(), finding) {
                Ok(edit) => action["edit"] = edit,
                Err(reason) => action["disabled"] = json!({"reason": reason}),
            }
            Some(action)
        })
        .collect();
    Value::Array(actions)
}

/// The fix a finding states as exact replacements, offered only when every line it cites still
/// holds the text once as a whole word, so an edited or moved line is never rewritten.
pub fn fix_actions(atlas: &Atlas, paths: &Paths, params: &Value) -> Vec<Value> {
    let diagnostics = params["context"]["diagnostics"].as_array();
    let mut offered = std::collections::BTreeSet::new();
    diagnostics
        .into_iter()
        .flatten()
        .filter(|diagnostic| diagnostic["source"] == "rosio")
        .filter_map(|diagnostic| {
            let id = diagnostic["data"]["finding"].as_str()?;
            let finding = atlas.findings.iter().find(|f| f.fields.id == id)?;
            let edits = &finding.fields.edits;
            if edits.is_empty() || !offered.insert(id.to_string()) {
                return None;
            }
            let mut changes = serde_json::Map::new();
            for edit in edits {
                let row = Evidence {
                    file: edit.file.clone(),
                    root: edit.root.clone(),
                    line_hint: edit.line_hint,
                    locator: None,
                };
                let path = paths.resolve(&row)?;
                let line = edit.line_hint?.checked_sub(1)?;
                let text = std::fs::read_to_string(&path).ok()?;
                let start = whole_word(text.lines().nth(line as usize)?, &edit.find)?;
                let uri = crate::paths::uri(&path)?;
                let at = |offset: usize| json!({"line": line, "character": offset});
                let change = json!({"range": {"start": at(start), "end": at(start + edit.find.encode_utf16().count())}, "newText": edit.replace});
                changes
                    .entry(uri.to_string())
                    .or_insert_with(|| json!([]))
                    .as_array_mut()?
                    .push(change);
            }
            let first = &edits[0];
            Some(json!({
                "title": format!("Replace {} with {}", first.find, first.replace),
                "kind": "quickfix",
                "isPreferred": true,
                "diagnostics": [diagnostic],
                "edit": {"changes": changes},
            }))
        })
        .collect()
}

/// The UTF-16 column of the one whole-word occurrence of `word` in `line`, if there is exactly one.
fn whole_word(line: &str, word: &str) -> Option<usize> {
    let part = |c: Option<char>| c.is_some_and(|c| c.is_alphanumeric() || c == '_');
    let hits: Vec<usize> = line
        .match_indices(word)
        .filter(|(at, _)| {
            !part(line[..*at].chars().next_back()) && !part(line[at + word.len()..].chars().next())
        })
        .map(|(at, _)| at)
        .collect();
    match hits.as_slice() {
        [at] => Some(line[..*at].encode_utf16().count()),
        _ => None,
    }
}

/// The edit that appends one waiver, creating the file when there is none.
fn waiver_edit(uri: &Url, existing: Option<&str>, finding: &str) -> Result<Value, &'static str> {
    let entry = format!(
        "  - key: {}\n    reason: {REASON_PLACEHOLDER}\n",
        quoted(finding)
    );
    match existing {
        None => Ok(json!({
            "documentChanges": [
                {"kind": "create", "uri": uri, "options": {"ignoreIfExists": true}},
                {
                    "textDocument": {"uri": uri, "version": null},
                    "edits": [{"range": range_at(0, 0), "newText": format!("waivers:\n{entry}")}]
                }
            ]
        })),
        Some(text) => {
            let position = waiver_position(text)?;
            let prefix = &text[..position.unwrap_or(text.len())];
            let (line, character) = end_of(prefix);
            let mut insert = String::new();
            if !prefix.is_empty() && !prefix.ends_with('\n') {
                insert.push('\n');
            }
            if position.is_none() {
                insert.push_str("waivers:\n");
            }
            insert.push_str(&entry);
            let proposed = format!("{prefix}{insert}{}", &text[prefix.len()..]);
            let parsed: serde_yaml_ng::Value = serde_yaml_ng::from_str(&proposed)
                .map_err(|_| "Cannot safely append to this waiver list. Edit it manually.")?;
            let mut expected: serde_yaml_ng::Value = if text.trim().is_empty() {
                serde_yaml_ng::Value::Mapping(Default::default())
            } else {
                serde_yaml_ng::from_str(text).map_err(|_| "Cannot read this waiver file.")?
            };
            let key = serde_yaml_ng::Value::String("waivers".into());
            let mapping = expected
                .as_mapping_mut()
                .ok_or("Expected a waiver mapping.")?;
            let list = mapping
                .entry(key)
                .or_insert_with(|| serde_yaml_ng::Value::Sequence(Vec::new()));
            list.as_sequence_mut()
                .ok_or("Expected a waiver list.")?
                .push(
                    serde_yaml_ng::from_str::<Vec<serde_yaml_ng::Value>>(&entry)
                        .map_err(|_| "Cannot construct a waiver entry.")?
                        .remove(0),
                );
            if parsed != expected {
                return Err("Cannot safely append to this waiver list. Edit it manually.");
            }
            Ok(json!({
                "documentChanges": [{
                    "textDocument": {"uri": uri, "version": null},
                    "edits": [{"range": range_at(line, character), "newText": insert}]
                }]
            }))
        }
    }
}

fn waiver_position(text: &str) -> Result<Option<usize>, &'static str> {
    const REFUSAL: &str =
        "Cannot safely edit this waiver file. Use an unanchored block list under waivers.";
    if text.trim().is_empty() {
        return Ok(None);
    }
    let document: serde_yaml_ng::Value = serde_yaml_ng::from_str(text).map_err(|_| REFUSAL)?;
    let mapping = document.as_mapping().ok_or(REFUSAL)?;
    let Some(waivers) = mapping.get(serde_yaml_ng::Value::String("waivers".into())) else {
        return Ok(None);
    };
    if !waivers.is_sequence()
        || text
            .split_whitespace()
            .any(|word| word.starts_with(['&', '*']))
    {
        return Err(REFUSAL);
    }
    let mut offset = 0;
    let mut header = false;
    for line in text.split_inclusive('\n') {
        let trimmed = line.trim();
        if !header {
            if let Some(rest) = line.strip_prefix("waivers:") {
                if !rest.trim().is_empty() && !rest.trim().starts_with('#') {
                    return Err(REFUSAL);
                }
                header = true;
            }
        } else if !trimmed.is_empty() && !trimmed.starts_with('#') && !line.starts_with([' ', '\t'])
        {
            return Ok(Some(offset));
        }
        offset += line.len();
    }
    if header {
        Ok(Some(text.len()))
    } else {
        Err(REFUSAL)
    }
}

/// A finding id holds `:` and `|`, so it is always written single quoted.
fn quoted(text: &str) -> String {
    format!("'{}'", text.replace('\'', "''"))
}

fn range_at(line: u32, character: u32) -> Value {
    json!({"start": {"line": line, "character": character}, "end": {"line": line, "character": character}})
}

/// The position just after the last character, in UTF-16 units as LSP counts them.
fn end_of(text: &str) -> (u32, u32) {
    let line = text.matches('\n').count() as u32;
    let last = text.rsplit('\n').next().unwrap_or("");
    (line, last.encode_utf16().count() as u32)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn a_fix_replaces_one_whole_word_or_nothing() {
        assert_eq!(
            whole_word("    stop_distnce: 0.35", "stop_distnce"),
            Some(4)
        );
        assert_eq!(
            whole_word(r#"canTransform("base_link", "laser_frame")"#, "laser_frame"),
            Some(27)
        );
        assert_eq!(
            whole_word("laser_frame_2: laser_frame", "laser_frame"),
            Some(15)
        );
        assert_eq!(
            whole_word("a: laser_frame, b: laser_frame", "laser_frame"),
            None
        );
        assert_eq!(whole_word("max_range: 8.0", "laser_frame"), None);
    }

    fn request(finding: &str) -> Value {
        json!({"context": {"diagnostics": [
            {"source": "rosio", "message": "m", "data": {"finding": finding},
             "range": range_at(0, 0)},
            {"source": "clangd", "message": "other", "range": range_at(0, 0)}
        ]}})
    }

    #[test]
    fn commented_headers_preserve_existing_waivers() {
        let uri = Url::parse("file:///waivers.yaml").unwrap();
        let edit = waiver_edit(
            &uri,
            Some("waivers: # accepted\n  - key: old\n    reason: ok\n"),
            "new",
        )
        .unwrap();
        let text = edit["documentChanges"][0]["edits"][0]["newText"]
            .as_str()
            .unwrap();
        assert!(!text.contains("waivers:"));
    }

    #[test]
    fn block_list_insertion_precedes_the_next_top_level_key() {
        let uri = Url::parse("file:///waivers.yaml").unwrap();
        let edit = waiver_edit(
            &uri,
            Some("waivers: # accepted\n  - key: old\n    reason: ok\nother: true\n"),
            "new",
        )
        .unwrap();
        assert_eq!(
            edit["documentChanges"][0]["edits"][0]["range"]["start"]["line"],
            3
        );
    }

    #[test]
    fn unsafe_waiver_shapes_are_disabled() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("target/unsafe-waivers");
        std::fs::create_dir_all(&dir).unwrap();
        for text in [
            "waivers: []",
            "waivers: [{key: old, reason: ok}]",
            "waivers: &accepted\n  - key: old",
            "waivers: *accepted",
            "waivers: false",
            "waivers:\n  key: old",
            "waivers: null",
            "waivers:\n- key: old\n  reason: ok",
            "waivers:\n    - key: old\n      reason: ok",
            "{other: true}",
        ] {
            std::fs::write(dir.join(WAIVER_FILE), text).unwrap();
            let actions = code_actions(&dir, &request("new"));
            assert!(
                actions[0]["disabled"]["reason"].is_string(),
                "{text}: {actions}"
            );
            assert!(actions[0].get("edit").is_none());
        }
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn a_new_file_is_created_with_the_list_and_one_waiver() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("target")
            .join(format!("rosio-actions-new-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let actions = code_actions(&dir, &request("find:param.orphan_block|param:/a%7Cb|-"));
        assert_eq!(actions.as_array().unwrap().len(), 1);
        let changes = &actions[0]["edit"]["documentChanges"];
        assert_eq!(changes[0]["kind"], "create");
        let text = changes[1]["edits"][0]["newText"].as_str().unwrap();
        assert_eq!(
            text,
            "waivers:\n  - key: 'find:param.orphan_block|param:/a%7Cb|-'\n    reason: write why this finding is accepted\n"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn an_existing_file_gets_the_waiver_appended_once() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("target")
            .join(format!("rosio-actions-old-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join(WAIVER_FILE),
            "waivers:\n  - key: 'find:x|y|-'\n    reason: ok",
        )
        .unwrap();
        let actions = code_actions(&dir, &request("find:z|w|-"));
        let edit = &actions[0]["edit"]["documentChanges"][0]["edits"][0];
        assert_eq!(edit["range"]["start"], json!({"line": 2, "character": 14}));
        assert!(edit["newText"]
            .as_str()
            .unwrap()
            .starts_with("\n  - key: 'find:z|w|-'"));
        assert_eq!(code_actions(&dir, &request("find:x|y|-")), json!([]));
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
