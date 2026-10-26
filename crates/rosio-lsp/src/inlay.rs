//! Parameter candidate outcomes beside their source lines.
use crate::atlas::{string, Atlas};
use crate::index::Index;
use crate::paths::normalize;
use lsp_types::{
    InlayHint, InlayHintLabel, InlayHintTooltip, MarkupContent, MarkupKind, Position, Range, Url,
};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Default)]
struct HintGroup {
    labels: BTreeSet<String>,
    tooltips: BTreeSet<String>,
}

/// Build one hint per source line and profile, retaining every chain in the tooltip.
pub(crate) fn hints(atlas: &Atlas, index: &Index, document: &Url, range: Range) -> Vec<InlayHint> {
    let Ok(path) = document.to_file_path() else {
        return Vec::new();
    };
    let path = normalize(&path);
    let profiles: BTreeSet<_> = atlas
        .entities
        .iter()
        .filter(|(_, e)| e.fields.kind == "profile")
        .map(|(id, _)| id.as_str())
        .chain(
            atlas
                .entities
                .values()
                .flat_map(|e| e.fields.candidates.iter().map(|c| string(c, "profile"))),
        )
        .filter(|p| !p.is_empty())
        .collect();
    let mut grouped: BTreeMap<(Position, &str), HintGroup> = BTreeMap::new();
    for entity in atlas.entities.values().filter(|e| e.fields.kind == "param") {
        for candidate in &entity.fields.candidates {
            let Some(site) = index.row(&candidate["source"]) else {
                continue;
            };
            if site.path != path {
                continue;
            }
            let position = Position::new(site.line, index.line_length(&path, site.line));
            if position < range.start || position > range.end {
                continue;
            }
            let profile = string(candidate, "profile");
            let mut chain: Vec<_> = entity
                .fields
                .candidates
                .iter()
                .filter(|c| string(c, "profile") == profile)
                .collect();
            chain.sort_by_key(|c| (c["rank"].as_u64().unwrap_or(u64::MAX), c.to_string()));
            // One write site cited twice is one candidate, not a value it wins over.
            chain.dedup();
            let Some(label) = outcome(
                string(entity, "state"),
                candidate,
                &chain,
                index,
                index.line_text(&path, site.line).unwrap_or(""),
            ) else {
                continue;
            };
            let group = grouped.entry((position, profile)).or_default();
            group.labels.insert(label);
            group
                .tooltips
                .insert(tooltip(&entity.fields.label, profile, &chain, index));
        }
    }
    grouped
        .into_iter()
        .map(|((position, profile), group)| {
            let mut label = group.labels.into_iter().collect::<Vec<_>>().join("; ");
            if profiles.len() > 1 {
                label.push_str(&format!(" in {}", slug(profile)));
            }
            InlayHint {
                position,
                label: InlayHintLabel::String(label),
                kind: None,
                text_edits: None,
                tooltip: Some(InlayHintTooltip::MarkupContent(MarkupContent {
                    kind: MarkupKind::Markdown,
                    value: group.tooltips.into_iter().collect::<Vec<_>>().join("\n\n"),
                })),
                padding_left: Some(true),
                padding_right: None,
                data: None,
            }
        })
        .collect()
}

/// What a line should say about its value, or nothing when the line already says it all.
///
/// Most lines of a parameters file hold the only value their parameter gets, and a hint on
/// every one of them is noise that hides the few that matter: a value that loses, a value
/// the launch file rewrites, a value that wins over another, a block no node reads.
fn outcome(
    state: &str,
    candidate: &Value,
    chain: &[&Value],
    index: &Index,
    line: &str,
) -> Option<String> {
    match state {
        "owner_name_mismatch" => return Some("no node is named this".into()),
        "inert_by_design" => return Some("plugin not selected".into()),
        _ => {}
    }
    if candidate["wins"] == true {
        let others = chain.len().saturating_sub(1);
        if others > 0 {
            let values = if others == 1 { "value" } else { "values" };
            return Some(format!("wins over {others} other {values}"));
        }
        let inline = line
            .split_once(':')
            .is_some_and(|(_, rest)| !rest.trim().is_empty());
        return (inline && !shows_value(line, &candidate["value"]))
            .then(|| format!("= {}", candidate["value"]));
    }
    if let Some(location) = chain
        .iter()
        .find(|c| c["wins"] == true)
        .and_then(|c| source_location(&c["source"], index))
    {
        return Some(format!("shadowed by {location}"));
    }
    let kind = string(candidate, "shadow_kind");
    Some(if kind.is_empty() {
        "no winning value".into()
    } else {
        kind.replace('_', " ")
    })
}

fn slug(profile: &str) -> String {
    crate::ids::parse(profile)
        .ok()
        .and_then(|id| id.segments.into_iter().next())
        .unwrap_or_else(|| profile.to_owned())
}

fn source_location(source: &Value, index: &Index) -> Option<String> {
    let site = index.row(source)?;
    let id = crate::ids::parse(&site.file_id).ok()?;
    Some(format!(
        "{}:{}",
        id.segments.get(1)?,
        site.line.saturating_add(1)
    ))
}

fn tooltip(label: &str, profile: &str, chain: &[&Value], index: &Index) -> String {
    let mut text = format!(
        "**{}** ({})\n\n",
        crate::hover::escape(label),
        crate::hover::escape(&slug(profile))
    );
    for candidate in chain {
        let location =
            source_location(&candidate["source"], index).unwrap_or_else(|| "source unknown".into());
        text.push_str(&format!(
            "Rank {}: {}, set by {}, at {}.\n\n",
            candidate["rank"],
            crate::hover::escape(&candidate["value"].to_string()),
            crate::hover::escape(string(candidate, "mechanism")),
            crate::hover::escape(&location)
        ));
    }
    text.trim_end().to_owned()
}

/// Compare the complete YAML value, so a key or a comment cannot hide a rewrite.
fn shows_value(line: &str, value: &Value) -> bool {
    let parsed = serde_yaml_ng::from_str::<Value>(line.trim()).ok();
    if let Some(fields) = parsed.as_ref().and_then(Value::as_object) {
        return fields.len() == 1 && fields.values().next() == Some(value);
    }
    line.split_once(" = ").is_some_and(|(_, text)| {
        serde_yaml_ng::from_str::<Value>(text.trim().trim_end_matches(';'))
            .ok()
            .as_ref()
            == Some(value)
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::{uri, Paths};
    use serde_json::json;
    use std::path::Path;

    fn fixture() -> Value {
        serde_json::from_str(include_str!("../tests/fixtures/params.atlas.json")).unwrap()
    }

    fn build(value: Value, range: Range) -> Vec<InlayHint> {
        let atlas = Atlas::parse(&value.to_string()).unwrap();
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let paths = Paths::new(&root, &atlas, &BTreeMap::new());
        let index = Index::new(&atlas, &paths);
        let document = uri(&root.join("crates/rosio-lsp/tests/fixtures/params.yaml")).unwrap();
        hints(&atlas, &index, &document, range)
    }

    fn full_range() -> Range {
        Range::new(Position::new(0, 0), Position::new(100, 0))
    }

    fn label(hint: &InlayHint) -> &str {
        let InlayHintLabel::String(label) = &hint.label else {
            panic!("Expected text")
        };
        label
    }

    #[test]
    fn winners_losers_and_inactive_parameters() {
        let hints = build(fixture(), full_range());
        let labels: Vec<_> = hints.iter().map(label).collect();
        assert_eq!(
            labels,
            [
                "= 2",
                "shadowed by crates/rosio-lsp/tests/fixtures/params.yaml:6",
                "wins over 1 other value",
                "no node is named this",
                "plugin not selected",
            ]
        );
        assert!(hints
            .iter()
            .all(|h| h.kind.is_none() && h.padding_left == Some(true)));
        let Some(InlayHintTooltip::MarkupContent(tooltip)) = &hints[1].tooltip else {
            panic!("Expected Markdown")
        };
        assert_eq!(tooltip.kind, MarkupKind::Markdown);
        assert!(
            tooltip.value.find("Rank 0: 4").unwrap() < tooltip.value.find("Rank 1: 3").unwrap()
        );
        assert!(tooltip.value.contains("set by yaml"));
        assert!(tooltip.value.contains("params.yaml:5"));
    }

    #[test]
    fn a_line_that_holds_its_only_value_says_nothing() {
        let hints = build(
            fixture(),
            Range::new(Position::new(2, 0), Position::new(2, 12)),
        );
        assert!(hints.is_empty());
    }

    #[test]
    fn profiles_and_duplicate_sites_stay_separate_and_sorted() {
        let mut value = fixture();
        value["entities"]["profile:other"] = json!({"kind":"profile"});
        let candidate = value["entities"]["param:/example|speed"]["candidates"][0].clone();
        let mut other = candidate.clone();
        other["profile"] = json!("profile:other");
        other["wins"] = json!(false);
        other["shadow_kind"] = json!("not_read_in_profile");
        value["entities"]["param:/example|speed"]["candidates"] =
            json!([other, candidate.clone(), candidate]);
        let range = Range::new(Position::new(2, 0), Position::new(2, 12));
        let hints = build(value, range);
        assert_eq!(hints.len(), 1);
        assert_eq!(label(&hints[0]), "not read in profile in other");
    }

    #[test]
    fn source_free_winner_uses_shadow_kind_and_range_checks_hint_position() {
        let mut value = fixture();
        value["entities"]["param:/example|old"]["candidates"][1]["source"] = Value::Null;
        let hints = build(value, Range::new(Position::new(4, 0), Position::new(5, 0)));
        assert_eq!(hints.len(), 1);
        assert_eq!(label(&hints[0]), "overridden");
        assert!(build(
            fixture(),
            Range::new(Position::new(2, 0), Position::new(2, 11))
        )
        .is_empty());
    }

    #[test]
    fn value_comparison_uses_the_value_and_preserves_types() {
        assert!(shows_value("  speed: 1 # units", &json!(1)));
        assert!(!shows_value("  speed: 10 # 1", &json!(1)));
        assert!(!shows_value("  speed: '1'", &json!(1)));
        assert!(!shows_value("  speed: 1", &json!("1")));
        assert!(shows_value("  names: [1, 2]", &json!([1, 2])));
        assert!(shows_value(
            "  names: [first, 'second']",
            &json!(["first", "second"])
        ));
        assert!(shows_value("  speed: 1.0", &json!(1.0)));
        assert!(!shows_value("  speed: 1", &json!(1.0)));
        assert!(shows_value("params['speed'] = 2", &json!(2)));
        assert!(shows_value("  name: 'hello'", &json!("hello")));
        assert!(shows_value("  name: hello", &json!("hello")));
        assert!(shows_value("  name: \"a # b\" # note", &json!("a # b")));
    }
}
