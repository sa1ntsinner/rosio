//! Short Markdown descriptions of the atlas entries a line holds.
//!
//! A hover answers one question fast: what is this line in the system. A parameter says its
//! value, who owns it and what set it. One line that sets a parameter for a whole launch group
//! reads as one card naming every node it reaches, not as one card per node.
use crate::atlas::{string, Atlas};
use crate::index::{Index, Site};
use crate::paths::normalize;
use lsp_types::{Hover, HoverContents, MarkupContent, MarkupKind, Url};
use serde_json::Value;
use std::collections::BTreeMap;

/// How many owners a grouped parameter card names before it counts the rest.
const OWNERS_SHOWN: usize = 6;

/// How many candidates of one chain a card lists.
const CANDIDATES_SHOWN: usize = 8;

/// Describe the entities and edges with evidence on the cursor line.
pub fn at(atlas: &Atlas, index: &Index, uri: &Url, line: u32) -> Option<Hover> {
    let mut parameters: BTreeMap<String, Vec<&str>> = BTreeMap::new();
    let mut sections = Vec::new();
    for id in index.at(uri, line) {
        let approximate = index.owners[id]
            .iter()
            .any(|site| site.line == line && site.approximate && on(site, uri));
        if let Some(entity) = atlas.entities.get(id).map(|e| &e.raw) {
            if string(entity, "kind") == "param" {
                let card = parameter_card(entity, index, approximate);
                parameters.entry(card).or_default().push(id);
                continue;
            }
            let mut card = entity_card(entity, approximate);
            card.push_str(&findings(atlas, index, id, uri, line));
            sections.push(card);
        } else if let Some(edge) = atlas.edges.get(id).map(|e| &e.raw) {
            sections.push(edge_card(atlas, edge));
        }
    }
    for (card, owners) in parameters {
        let mut text = owners_line(&owners);
        text.push_str(&card);
        for id in owners.iter().take(1) {
            text.push_str(&findings(atlas, index, id, uri, line));
        }
        sections.insert(0, text);
    }
    if sections.is_empty() {
        return None;
    }
    Some(Hover {
        contents: HoverContents::Markup(MarkupContent {
            kind: MarkupKind::Markdown,
            value: sections.join("\n\n---\n\n"),
        }),
        range: None,
    })
}

/// The heading of a parameter card: its name and the node or nodes it belongs to.
fn owners_line(ids: &[&str]) -> String {
    let name = ids
        .first()
        .map(|id| id.rsplit('|').next().unwrap_or(id))
        .unwrap_or_default();
    let owners: Vec<String> = ids.iter().map(|id| owner(id)).collect();
    let shown = owners
        .iter()
        .take(OWNERS_SHOWN)
        .map(|o| escape(o))
        .collect::<Vec<_>>()
        .join(", ");
    let more = owners.len().saturating_sub(OWNERS_SHOWN);
    let rest = if more > 0 {
        format!(" and {more} more")
    } else {
        String::new()
    };
    if owners.len() == 1 {
        format!("**{}** on {shown}", escape(name))
    } else {
        format!(
            "**{}** on {} nodes: {shown}{rest}",
            escape(name),
            owners.len()
        )
    }
}

/// The node a parameter id names, or what can be said when it has no stable name.
fn owner(id: &str) -> String {
    let body = id.strip_prefix("param:").unwrap_or(id);
    let node = body.rsplit_once('|').map_or(body, |(node, _)| node);
    if node.starts_with('~') {
        "an unnamed node".into()
    } else {
        node.replace("%7C", "|")
    }
}

/// The body of a parameter card, the same text for every owner that shares the chain.
fn parameter_card(entity: &Value, index: &Index, approximate: bool) -> String {
    let mut text = String::new();
    if approximate {
        text.push_str(" (found by name)");
    }
    let candidates = entity["candidates"].as_array().cloned().unwrap_or_default();
    let mut profiles: BTreeMap<&str, Vec<&Value>> = BTreeMap::new();
    for candidate in &candidates {
        profiles
            .entry(string(candidate, "profile"))
            .or_default()
            .push(candidate);
    }
    if profiles.is_empty() {
        text.push_str(&format!("\n\nState: {}.", words(string(entity, "state"))));
    }
    for (profile, mut chain) in profiles {
        chain.sort_by_key(|c| c["rank"].as_u64().unwrap_or(u64::MAX));
        let winner = chain.iter().find(|c| c["wins"] == true);
        match winner {
            Some(winner) => text.push_str(&format!(
                "\n\n`{}` in {}, set by {} at {}",
                value_text(&winner["value"]),
                escape(slug(profile)),
                mechanism(winner),
                place(&winner["source"], index)
            )),
            None => text.push_str(&format!(
                "\n\nNo value wins in {}: {}.",
                escape(slug(profile)),
                words(string(entity, "state"))
            )),
        }
        if chain.len() > 1 {
            for candidate in chain.iter().take(CANDIDATES_SHOWN) {
                let outcome = if candidate["wins"] == true {
                    "wins".to_string()
                } else {
                    words(string(candidate, "shadow_kind"))
                };
                text.push_str(&format!(
                    "\n- `{}` {}, {} ({outcome})",
                    value_text(&candidate["value"]),
                    mechanism(candidate),
                    place(&candidate["source"], index)
                ));
            }
        }
    }
    text
}

fn entity_card(entity: &Value, approximate: bool) -> String {
    let mut text = format!(
        "**{}** · {}{}\n\n{}",
        escape(string(entity, "label")),
        escape(string(entity, "kind")),
        if approximate { " (found by name)" } else { "" },
        stated_by(&entity["layers"])
    );
    if let Some(conflicts) = entity["conflicts"].as_array().filter(|c| !c.is_empty()) {
        let fields: Vec<String> = conflicts
            .iter()
            .filter_map(Value::as_str)
            .map(escape)
            .collect();
        text.push_str(&format!(" The layers disagree on {}.", fields.join(", ")));
    }
    text
}

fn edge_card(atlas: &Atlas, edge: &Value) -> String {
    let label = |id: &str| {
        atlas
            .entities
            .get(id)
            .map(|e| string(&e.raw, "label").to_owned())
            .unwrap_or_else(|| "unknown".into())
    };
    format!(
        "{} **{}** {}\n\n{}{}",
        escape(&label(string(edge, "src"))),
        words(string(edge, "kind")),
        escape(&label(string(edge, "dst"))),
        qos(edge),
        stated_by(&edge["layers"])
    )
}

/// The QoS an endpoint states in its first profile that states any, as a sentence.
fn qos(edge: &Value) -> String {
    let stated = |profile: &Value, key: &str| {
        profile[format!("qos.{key}")]["value"]
            .as_str()
            .map(str::to_owned)
            .or_else(|| {
                profile[format!("qos.{key}")]["value"]
                    .as_u64()
                    .map(|n| n.to_string())
            })
    };
    let parts = edge["effective"]
        .as_object()
        .into_iter()
        .flat_map(|profiles| profiles.values())
        .map(|profile| {
            if let Some(preset) = stated(profile, "preset") {
                return vec![format!("the {preset} preset")];
            }
            ["reliability", "durability", "history", "depth"]
                .iter()
                .filter_map(|key| {
                    stated(profile, key).map(|value| match *key {
                        "depth" => format!("depth {value}"),
                        _ => value,
                    })
                })
                .collect()
        })
        .find(|parts: &Vec<String>| !parts.is_empty());
    parts.map_or_else(String::new, |parts| {
        format!("QoS: {}. ", escape(&parts.join(", ")))
    })
}

/// Whether a site is in the document, whatever spelling of its path the client sends.
fn on(site: &Site, uri: &Url) -> bool {
    uri.to_file_path()
        .is_ok_and(|path| normalize(&path) == normalize(&site.path))
}

/// Findings about an entity, except those whose diagnostic already sits on this line.
fn findings(atlas: &Atlas, index: &Index, id: &str, uri: &Url, line: u32) -> String {
    let here = |finding: &&Value| {
        finding["where"]
            .as_array()
            .into_iter()
            .flatten()
            .any(|row| {
                index
                    .row(row)
                    .is_some_and(|site| site.line == line && on(site, uri))
            })
    };
    atlas
        .findings_for(id)
        .iter()
        .filter(|finding| !here(finding))
        .map(|finding| {
            format!(
                "\n\n{}: {}",
                escape(string(finding, "severity")),
                escape(string(finding, "title"))
            )
        })
        .collect()
}

fn stated_by(layers: &Value) -> String {
    let named: Vec<&str> = layers
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    match named.as_slice() {
        [] => "Known by name only.".into(),
        [one] => format!("Stated by the {one} layer."),
        many => format!("Stated by the {} layers.", many.join(", ")),
    }
}

/// How a value got where it is, in words.
fn mechanism(candidate: &Value) -> String {
    if string(candidate, "note").starts_with("SetParameter") {
        return "SetParameter for the launch group".into();
    }
    match string(candidate, "mechanism") {
        "yaml_base" | "launch_params_file" => "a parameters file".into(),
        "yaml_overlay_merge" => "a merged parameters file".into(),
        "yaml_wildcard" => "a wildcard block".into(),
        "launch_inline_dict" => "the launch file".into(),
        "launch_rewritten_yaml" => "RewrittenYaml".into(),
        "ros_args_p" => "a -p argument".into(),
        "ros_args_params_file" => "--params-file".into(),
        "python_override" => "a launcher script".into(),
        "param_set_runtime" => "ros2 param set".into(),
        "code_default" => "the default in the source".into(),
        "observed_runtime" => "the running node".into(),
        "commented_out" => "a commented out line".into(),
        other => words(other),
    }
}

/// The file and line a source row points at, package relative.
fn place(source: &Value, index: &Index) -> String {
    let file = string(source, "file");
    let file = file
        .strip_prefix("file:")
        .unwrap_or(file)
        .replacen('|', "/", 1);
    let line = index
        .row(source)
        .map(|site| site.line.saturating_add(1).to_string())
        .or_else(|| source.get("line_hint").map(Value::to_string));
    match line {
        Some(line) => format!("{}:{line}", escape(&file)),
        None => escape(&file),
    }
}

fn value_text(value: &Value) -> String {
    match value {
        Value::String(text) => text.clone(),
        other => other.to_string(),
    }
}

fn slug(profile: &str) -> &str {
    profile.strip_prefix("profile:").unwrap_or(profile)
}

fn words(text: &str) -> String {
    escape(&text.replace('_', " "))
}

pub(crate) fn escape(text: &str) -> String {
    text.chars()
        .flat_map(|c| {
            if matches!(c, '\\' | '`' | '*' | '_' | '[' | ']' | '<' | '>') {
                vec!['\\', c]
            } else {
                vec![c]
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::{uri, Paths};
    use serde_json::json;
    use std::{collections::BTreeMap, path::Path};

    fn markdown(hover: Hover) -> String {
        let HoverContents::Markup(content) = hover.contents else {
            panic!("Expected Markdown")
        };
        content.value
    }

    #[test]
    fn fixture_hover() {
        let atlas = Atlas::parse(include_str!("../tests/fixtures/ws_static.atlas.json")).unwrap();
        let paths = Paths::new(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            &atlas,
            &BTreeMap::new(),
        );
        let index = Index::new(&atlas, &paths);
        let site = &index.owners["frame:base_footprint"][0];
        let text = markdown(at(&atlas, &index, &uri(&site.path).unwrap(), site.line).unwrap());
        assert!(!text.contains("found by name"), "a link cites its own line");
        assert!(text.contains("base\\_footprint"));
    }

    fn parameter_atlas(owners: &[&str], cited: Value) -> Atlas {
        let file = "file:@repo|tests/fixtures/ws_static/src/fx_desc/urdf/fx_robot.urdf";
        let source = json!({"file": file, "locator": "link[base_footprint]", "root": "src"});
        let mut entities = serde_json::Map::new();
        for owner in owners {
            entities.insert(
                format!("param:{owner}|speed"),
                json!({"kind": "param", "label": "speed", "state": "effective",
                "candidates": [
                    {"rank": 0, "wins": true, "value": 2, "mechanism": "yaml_base",
                     "profile": "profile:demo", "source": source},
                    {"rank": 1, "wins": false, "value": 1, "mechanism": "code_default",
                     "profile": "profile:demo", "source": source, "shadow_kind": "overridden"}
                ]}),
            );
        }
        Atlas::parse(
            &json!({"schema_version": "1.3", "entities": entities,
                "findings": [{"id": "f", "severity": "warn", "title": "Speed differs", "where": cited,
                              "about": {"entity": format!("param:{}|speed", owners[0])}}]})
            .to_string(),
        )
        .unwrap()
    }

    fn parameter_hover(atlas: &Atlas) -> String {
        let paths = Paths::new(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            atlas,
            &BTreeMap::new(),
        );
        let index = Index::new(atlas, &paths);
        let site = index.owners.values().next().unwrap()[0].clone();
        markdown(at(atlas, &index, &uri(&site.path).unwrap(), site.line).unwrap())
    }

    #[test]
    fn a_parameter_names_its_owner_its_value_and_what_set_it() {
        let text = parameter_hover(&parameter_atlas(&["/controller"], json!([])));
        for expected in [
            "**speed** on /controller",
            "`2` in demo, set by a parameters file",
            "`1` the default in the source",
            "(overridden)",
            "Speed differs",
        ] {
            assert!(text.contains(expected), "missing {expected}: {text}");
        }
        assert!(!text.contains("{\""), "no raw json: {text}");
    }

    #[test]
    fn a_finding_on_the_hovered_line_is_left_to_its_diagnostic() {
        let atlas = parameter_atlas(
            &["/controller"],
            json!([{"file": "file:@repo|tests/fixtures/ws_static/src/fx_desc/urdf/fx_robot.urdf",
                    "locator": "link[base_footprint]", "root": "src"}]),
        );
        let text = parameter_hover(&atlas);
        assert!(!text.contains("Speed differs"), "{text}");
    }

    #[test]
    fn an_endpoint_states_its_qos_and_nothing_it_did_not_state() {
        let preset = json!({"effective": {"profile:a": {"qos.preset": {"value": "sensor_data"}}}});
        assert_eq!(qos(&preset), "QoS: the sensor\\_data preset. ");
        let policies = json!({"effective": {"profile:a": {
            "qos.reliability": {"value": "reliable"}, "qos.durability": {"value": null},
            "qos.depth": {"value": 10}}}});
        assert_eq!(qos(&policies), "QoS: reliable, depth 10. ");
        assert_eq!(qos(&json!({})), "");
    }

    #[test]
    fn one_line_for_many_nodes_is_one_card() {
        let text = parameter_hover(&parameter_atlas(&["/a", "/b", "/c"], json!([])));
        assert!(text.contains("**speed** on 3 nodes: /a, /b, /c"), "{text}");
        assert_eq!(
            text.matches("set by a parameters file").count(),
            1,
            "{text}"
        );
    }
}
