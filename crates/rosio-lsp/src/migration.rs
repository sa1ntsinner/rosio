use lsp_types::{Diagnostic, DiagnosticSeverity, NumberOrString, Position, Range, Url};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn file_key(uri: &Url) -> Url {
    let Ok(path) = uri.to_file_path() else {
        return uri.clone();
    };
    #[cfg(windows)]
    let path = {
        let mut text = path.to_string_lossy().into_owned();
        if text.as_bytes().get(1) == Some(&b':') {
            let drive = text[..1].to_ascii_lowercase();
            text.replace_range(..1, &drive);
        }
        PathBuf::from(text)
    };
    Url::from_file_path(path).unwrap_or_else(|_| uri.clone())
}

#[derive(Default)]
pub(crate) struct Migration {
    pub plan: Option<Value>,
    pub rows: BTreeMap<Url, Vec<(Diagnostic, Value)>>,
}

pub(crate) fn root(workspace: &Path) -> PathBuf {
    workspace
        .ancestors()
        .find(|p| p.join(".git").exists())
        .unwrap_or(workspace)
        .to_owned()
}

pub(crate) fn range(text: &str, line: usize, column: usize, found: &str) -> Option<Range> {
    let line = line.checked_sub(1)?;
    let column = column.checked_sub(1)?;
    let offset = text
        .split_inclusive('\n')
        .take(line)
        .map(str::len)
        .sum::<usize>();
    let row = text.get(offset..)?.split('\n').next()?;
    if row.chars().count() < column {
        return None;
    }
    let prefix: String = row.chars().take(column).collect();
    let start = offset + prefix.len();
    if !text.get(start..)?.starts_with(found) {
        return None;
    }
    let first = Position::new(line as u32, prefix.encode_utf16().count() as u32);
    let mut end = first;
    for ch in found.chars() {
        if ch == '\n' {
            end.line += 1;
            end.character = 0;
        } else {
            end.character += ch.len_utf16() as u32;
        }
    }
    Some(Range::new(first, end))
}

fn offset(text: &str, position: Position) -> Option<usize> {
    let base = text
        .split_inclusive('\n')
        .take(position.line as usize)
        .map(str::len)
        .sum::<usize>();
    let mut units = 0;
    for (i, ch) in text.get(base..)?.char_indices() {
        if units == position.character {
            return Some(base + i);
        }
        if ch == '\n' {
            return None;
        }
        units += ch.len_utf16() as u32;
    }
    (units == position.character).then_some(text.len())
}

impl Migration {
    pub fn replace(&mut self, plan: Value, workspace: &Path, documents: &BTreeMap<Url, String>) {
        self.rows.clear();
        let root = plan["root"]
            .as_str()
            .map(PathBuf::from)
            .unwrap_or_else(|| root(workspace));
        for change in plan["changes"].as_array().into_iter().flatten() {
            let Some(file) = change["file"].as_str() else {
                continue;
            };
            let path = root.join(file);
            let Ok(uri) = Url::from_file_path(&path) else {
                continue;
            };
            let uri = file_key(&uri);
            let text = documents
                .iter()
                .find(|(key, _)| file_key(key) == uri)
                .map(|(_, text)| text.clone())
                .or_else(|| std::fs::read_to_string(&path).ok());
            let Some(text) = text else {
                continue;
            };
            let Some(range) = range(
                &text,
                change["line"].as_u64().unwrap_or(0) as usize,
                change["column"].as_u64().unwrap_or(1) as usize,
                change["found"].as_str().unwrap_or(""),
            ) else {
                continue;
            };
            let diagnostic = Diagnostic {
                range,
                severity: Some(if change["effect"] != "deprecated" {
                    DiagnosticSeverity::WARNING
                } else {
                    DiagnosticSeverity::INFORMATION
                }),
                code: Some(NumberOrString::String(
                    change["rule"].as_str().unwrap_or("").into(),
                )),
                code_description: change["source"]
                    .as_str()
                    .and_then(|s| Url::parse(s).ok())
                    .map(|href| lsp_types::CodeDescription { href }),
                source: Some("rosio migrate".into()),
                message: format!(
                    "{} ({} in {})",
                    change["say"].as_str().unwrap_or(""),
                    change["effect"].as_str().unwrap_or(""),
                    change["release"].as_str().unwrap_or("")
                ),
                ..Default::default()
            };
            self.rows
                .entry(uri)
                .or_default()
                .push((diagnostic, change.clone()));
        }
        self.plan = Some(plan);
    }

    pub fn edited(&mut self, uri: &Url, text: &str) {
        if let Some(rows) = self.rows.get_mut(&file_key(uri)) {
            rows.retain(|(d, c)| {
                offset(text, d.range.start)
                    .zip(offset(text, d.range.end))
                    .and_then(|(a, b)| text.get(a..b))
                    == c["found"].as_str()
            });
        }
    }

    pub fn union(
        &self,
        findings: BTreeMap<Url, Vec<Diagnostic>>,
    ) -> BTreeMap<Url, Vec<Diagnostic>> {
        let mut result: BTreeMap<Url, Vec<Diagnostic>> = BTreeMap::new();
        for (uri, rows) in findings {
            result.entry(file_key(&uri)).or_default().extend(rows);
        }
        for (uri, rows) in &self.rows {
            result
                .entry(uri.clone())
                .or_default()
                .extend(rows.iter().map(|(d, _)| d.clone()));
        }
        result
    }

    pub fn actions(&self, params: &Value) -> Vec<Value> {
        let Some(rows) = params["textDocument"]["uri"]
            .as_str()
            .and_then(|s| Url::parse(s).ok())
            .and_then(|u| self.rows.get(&file_key(&u)))
        else {
            return vec![];
        };
        let only = params["context"]["only"].as_array();
        let allows = |kind: &str| {
            only.is_none_or(|items| {
                items
                    .iter()
                    .filter_map(Value::as_str)
                    .any(|s| kind == s || kind.starts_with(&format!("{s}.")))
            })
        };
        let uri = &params["textDocument"]["uri"];
        let mut result = vec![];
        let mut edits = vec![];
        for (diagnostic, change) in rows {
            let Some(replacement) = change["replacement"].as_str() else {
                continue;
            };
            let edit = json!({"range":diagnostic.range,"newText":replacement});
            edits.push(edit.clone());
            let requested: Option<Range> = serde_json::from_value(params["range"].clone()).ok();
            if allows("quickfix")
                && requested.is_some_and(|r| {
                    r.start <= diagnostic.range.end && diagnostic.range.start <= r.end
                })
            {
                result.push(json!({"title":format!("Apply: {}", replacement.lines().next().unwrap_or("")),"kind":"quickfix","isPreferred":true,"diagnostics":[diagnostic],"edit":{"changes":{uri.as_str().unwrap_or(""):[edit]}}}));
            }
        }
        if allows("source.fixAll.rosioMigration") && !edits.is_empty() {
            result.push(json!({"title":"rosio: apply every automatic migration change in this file","kind":"source.fixAll.rosioMigration","edit":{"changes":{uri.as_str().unwrap_or(""):edits}}}));
        }
        result
    }
}

pub(crate) fn run(
    workspace: &Path,
    command: &[String],
    verb: &str,
    args: &Value,
) -> Result<Value, String> {
    let end = command
        .iter()
        .position(|s| s == "extract")
        .unwrap_or(command.len());
    let (program, prefix) = command[..end]
        .split_first()
        .ok_or("Engine command is empty")?;
    let mut process = std::process::Command::new(program);
    process.args(prefix).args([
        "migrate",
        verb,
        ".",
        "--json",
        "--to",
        args["to"].as_str().ok_or("Missing target release")?,
    ]);
    if let Some(from) = args["from"].as_str() {
        process.args(["--from", from]);
    }
    let output = process
        .current_dir(workspace)
        .output()
        .map_err(|e| crate::extraction::spawn_error(program, &e))?;
    answer(
        verb,
        output.status.success(),
        &output.stdout,
        &output.stderr,
    )
}

/// A verify whose build failed still answers: its data holds every error, matched to the plan.
fn answer(verb: &str, success: bool, stdout: &[u8], stderr: &[u8]) -> Result<Value, String> {
    let envelope: Option<Value> = serde_json::from_slice(stdout).ok();
    if let Some(envelope) = &envelope {
        if verb == "verify" && envelope["data"]["packages"].is_array() {
            return Ok(envelope["data"].clone());
        }
    }
    if !success {
        let stderr = String::from_utf8_lossy(stderr).into_owned();
        if stderr.contains("No module named rosio") {
            return Err(crate::extraction::engine_missing("rosio"));
        }
        return Err(stderr);
    }
    let envelope = envelope.ok_or("The engine did not answer in JSON")?;
    if envelope["ok"] != true {
        return Err(envelope.to_string());
    }
    Ok(envelope["data"].clone())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn code_points_become_utf16_ranges() {
        assert_eq!(
            range("abc def", 1, 5, "def"),
            Some(Range::new(Position::new(0, 4), Position::new(0, 7)))
        );
        assert_eq!(
            range("a\u{1f600} def", 1, 4, "def"),
            Some(Range::new(Position::new(0, 4), Position::new(0, 7)))
        );
        assert_eq!(
            range("a\u{1f600} foo\nbar\u{1f600}!", 1, 4, "foo\nbar\u{1f600}"),
            Some(Range::new(Position::new(0, 4), Position::new(1, 5)))
        );
        assert_eq!(range("abc", 1, 2, "no"), None);
    }

    fn fixture() -> (Migration, Url) {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR"));
        let uri =
            file_key(&Url::from_file_path(root(workspace).join("migration-test.txt")).unwrap());
        let documents = [(uri.clone(), "a\u{1f600} old\nkeep".into())].into();
        let mut migration = Migration::default();
        migration.replace(json!({"from":"humble","to":"lyrical","summary":{},"changes":[{
            "file":"migration-test.txt","line":1,"column":4,"found":"old","replacement":"new\nvalue",
            "rule":"test.rule","say":"Change this","effect":"removed","release":"lyrical","source":"https://example.org/rule"
        }]}), workspace, &documents);
        (migration, uri)
    }

    #[test]
    fn migration_and_findings_publish_together() {
        let (migration, uri) = fixture();
        let finding = Diagnostic {
            message: "finding".into(),
            ..Default::default()
        };
        let union = migration.union([(uri.clone(), vec![finding.clone()])].into());
        assert_eq!(union[&uri].len(), 2);
        assert_eq!(union[&uri][0].message, "finding");
        assert_eq!(union[&uri][1].source.as_deref(), Some("rosio migrate"));
        assert_eq!(union[&uri][1].severity, Some(DiagnosticSeverity::WARNING));
        assert_eq!(union[&uri][1].message, "Change this (removed in lyrical)");
        let cleared = Migration::default().union([(uri.clone(), vec![finding])].into());
        assert_eq!(cleared[&uri].len(), 1);
    }

    #[test]
    fn edits_drop_only_stale_matches() {
        let (mut migration, uri) = fixture();
        migration.edited(&uri, "a\u{1f600} old\nchanged elsewhere");
        assert_eq!(migration.rows[&uri].len(), 1);
        migration.edited(&uri, "a\u{1f600} new\nkeep");
        assert!(migration.rows[&uri].is_empty());
    }

    #[test]
    fn encoded_file_uris_find_edits_and_merge_diagnostics() {
        let (mut migration, uri) = fixture();
        let encoded = if cfg!(windows) {
            Url::parse(&uri.as_str().replacen("c:/", "C%3A/", 1)).unwrap()
        } else {
            uri.clone()
        };
        let actions = migration
            .actions(&json!({"textDocument":{"uri":encoded},"context":{"only":["source"]}}));
        assert_eq!(actions.len(), 1);
        let union = migration.union([(encoded.clone(), vec![Diagnostic::default()])].into());
        assert_eq!(union.len(), 1);
        assert_eq!(union.values().next().unwrap().len(), 2);
        migration.edited(&encoded, "a\u{1f600} new\nkeep");
        assert!(migration.rows[&uri].is_empty());
    }

    #[test]
    fn a_failed_build_still_answers_verify() {
        let failed =
            br#"{"ok":false,"data":{"ok":false,"packages":[{"name":"a","status":"failed"}]}}"#;
        assert_eq!(
            answer("verify", false, failed, b"").unwrap()["packages"][0]["name"],
            "a"
        );
        assert!(answer("plan", false, failed, b"broken").is_err());
        assert_eq!(
            answer(
                "verify",
                false,
                b"",
                b"Docker must be installed and running."
            )
            .unwrap_err(),
            "Docker must be installed and running."
        );
    }

    #[test]
    fn quickfix_replaces_the_exact_match() {
        let (mut migration, uri) = fixture();
        let mut params = json!({"textDocument":{"uri":uri},"range":{"start":{"line":0,"character":4},"end":{"line":0,"character":7}},"context":{}});
        let actions = migration.actions(&params);
        assert_eq!(actions.len(), 2);
        assert_eq!(actions[0]["title"], "Apply: new");
        assert_eq!(actions[0]["isPreferred"], true);
        assert_eq!(
            actions[0]["edit"]["changes"][uri.as_str()][0],
            json!({"range":{"start":{"line":0,"character":4},"end":{"line":0,"character":7}},"newText":"new\nvalue"})
        );
        params["context"] = json!({"only":["source"]});
        assert_eq!(migration.actions(&params).len(), 1);
        migration.rows.get_mut(&uri).unwrap()[0]
            .1
            .as_object_mut()
            .unwrap()
            .remove("replacement");
        assert!(migration.actions(&params).is_empty());
    }
}
