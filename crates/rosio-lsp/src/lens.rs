//! Endpoint peers beside the source sites that create them.
use crate::atlas::{Atlas, Edge};
use crate::index::Index;
use crate::paths::normalize;
use lsp_types::{CodeLens, Command, Position, Range, Url};
use serde_json::json;
use std::collections::{BTreeMap, BTreeSet};

/// Build one lens per line, opening the first endpoint card in edge id order.
pub(crate) fn lenses(atlas: &Atlas, index: &Index, document: &Url) -> Vec<CodeLens> {
    let Ok(path) = document.to_file_path() else {
        return Vec::new();
    };
    let path = normalize(&path);
    let profiles: BTreeSet<&str> = atlas
        .edges
        .values()
        .flat_map(|edge| edge.fields.common.facts.iter())
        .filter(|fact| fact.confidence != "unknown" && fact.profile != "*")
        .map(|fact| fact.profile.as_str())
        .chain(
            atlas
                .entities
                .iter()
                .filter(|(_, entity)| entity.fields.kind == "profile")
                .map(|(id, _)| id.as_str()),
        )
        .collect();
    let profiles = if profiles.is_empty() {
        BTreeSet::from(["*"])
    } else {
        profiles
    };
    let mut peers: BTreeMap<(&str, &str, &str), BTreeSet<&str>> = BTreeMap::new();
    for edge in atlas
        .edges
        .values()
        .filter(|e| is_endpoint(atlas, &e.fields))
    {
        for profile in &profiles {
            if participates(&edge.fields, profile) {
                peers
                    .entry((&edge.fields.dst, &edge.fields.common.kind, profile))
                    .or_default()
                    .insert(&edge.fields.src);
            }
        }
    }
    let mut lines: BTreeMap<u32, BTreeMap<&str, (String, &str)>> = BTreeMap::new();
    for (id, edge) in &atlas.edges {
        if !is_endpoint(atlas, &edge.fields) {
            continue;
        }
        let (opposite, noun) = match edge.fields.common.kind.as_str() {
            "subscribes" => ("publishes", "publisher"),
            "publishes" => ("subscribes", "subscriber"),
            "calls" => ("serves", "server"),
            _ => ("calls", "client"),
        };
        let titles: Vec<_> = profiles
            .iter()
            .filter(|profile| participates(&edge.fields, profile))
            .map(|profile| {
                let nodes = peers.get(&(edge.fields.dst.as_str(), opposite, *profile));
                let labels = nodes
                    .into_iter()
                    .flatten()
                    .map(|id| atlas.entities[*id].fields.label.as_str())
                    .collect();
                let mut title = peer_title(labels, noun);
                if incompatible(atlas, &edge.fields, profile) {
                    title.push_str(" · QoS incompatible");
                }
                (*profile, title, nodes)
            })
            .collect();
        if titles.is_empty() {
            continue;
        }
        // A library wrapper's own topic (tf2, diagnostics) has peers outside the workspace.
        let wrapped = edge.fields.common.facts.iter().any(|fact| {
            fact.values
                .get("via_known_wrapper")
                .is_some_and(|w| !w.is_null() && w != false)
        });
        if wrapped
            && titles
                .iter()
                .all(|(_, _, nodes)| nodes.is_none_or(BTreeSet::is_empty))
        {
            continue;
        }
        let different = titles
            .iter()
            .any(|(_, title, nodes)| title != &titles[0].1 || nodes != &titles[0].2);
        let title = if different {
            titles
                .iter()
                .map(|(profile, title, _)| {
                    format!(
                        "{}: {title}",
                        profile.strip_prefix("profile:").unwrap_or(profile)
                    )
                })
                .collect::<Vec<_>>()
                .join(" · ")
        } else {
            titles[0].1.clone()
        };
        for site in index.owners.get(id).into_iter().flatten() {
            if site.path == path {
                lines
                    .entry(site.line)
                    .or_default()
                    .insert(id, (title.clone(), &edge.fields.dst));
            }
        }
    }
    lines
        .into_iter()
        .filter_map(|(line, endpoints)| {
            let target = endpoints.values().next()?.1;
            let named = endpoints.len() > 1;
            let title = endpoints
                .values()
                .map(|(title, topic)| {
                    if named {
                        let label = atlas
                            .entities
                            .get(*topic)
                            .map_or(*topic, |e| e.fields.label.as_str());
                        format!("{label}: {title}")
                    } else {
                        title.clone()
                    }
                })
                .collect::<Vec<_>>()
                .join(" · ");
            let position = Position::new(line, 0);
            Some(CodeLens {
                range: Range::new(position, position),
                command: Some(Command::new(
                    title,
                    "rosio.showEntity".into(),
                    Some(vec![json!(target)]),
                )),
                data: None,
            })
        })
        .collect()
}

fn participates(edge: &Edge, profile: &str) -> bool {
    edge.common.facts.iter().any(|fact| {
        fact.confidence != "unknown" && (fact.profile == "*" || fact.profile == profile)
    })
}

fn is_endpoint(atlas: &Atlas, edge: &Edge) -> bool {
    let destination = match edge.common.kind.as_str() {
        "publishes" | "subscribes" => &["topic"][..],
        "serves" | "calls" => &["service", "action"][..],
        _ => return false,
    };
    atlas
        .entities
        .get(&edge.src)
        .is_some_and(|e| e.fields.kind == "node")
        && atlas
            .entities
            .get(&edge.dst)
            .is_some_and(|e| destination.contains(&e.fields.kind.as_str()))
}

fn peer_title(mut labels: Vec<&str>, noun: &str) -> String {
    labels.sort();
    let count = labels.len();
    if count == 0 {
        return format!("no {noun}");
    }
    let shown = labels.into_iter().take(3).collect::<Vec<_>>().join(", ");
    let more = if count > 3 {
        format!(" and {} more", count - 3)
    } else {
        String::new()
    };
    let plural = if count == 1 { "" } else { "s" };
    format!("{count} {noun}{plural}: {shown}{more}")
}

fn incompatible(atlas: &Atlas, edge: &Edge, profile: &str) -> bool {
    let label = atlas.entities[&edge.src].fields.label.as_str();
    atlas.findings_for(&edge.dst).iter().any(|finding| {
        finding["rule"] == "qos.incompatible"
            && finding
                .get("profile")
                .and_then(|value| value.as_str())
                .is_none_or(|scope| scope == "*" || scope == profile)
            && ["title", "say", "detail", "fix"].iter().any(|field| {
                finding[field]
                    .as_str()
                    .is_some_and(|text| names_node(text, label))
            })
    })
}

fn names_node(text: &str, label: &str) -> bool {
    if label.is_empty() {
        return false;
    }
    let name_character = |c: char| c.is_alphanumeric() || c == '_' || c == '/';
    text.match_indices(label).any(|(offset, _)| {
        !text[..offset]
            .chars()
            .next_back()
            .is_some_and(name_character)
            && !text[offset + label.len()..]
                .chars()
                .next()
                .is_some_and(name_character)
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::{uri, Paths};
    use std::path::Path;

    fn fixture_lenses(text: &str) -> Vec<CodeLens> {
        let atlas = Atlas::parse(text).unwrap();
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let paths = Paths::new(&root, &atlas, &BTreeMap::new());
        let index = Index::new(&atlas, &paths);
        let document = uri(&root.join("crates/rosio-lsp/tests/fixtures/endpoints.cpp")).unwrap();
        assert!(lenses(&atlas, &index, &Url::parse("untitled:other").unwrap()).is_empty());
        assert!(lenses(&atlas, &index, &uri(&root.join("other.cpp")).unwrap()).is_empty());
        lenses(&atlas, &index, &document)
    }

    #[test]
    fn peers_require_usable_facts_in_the_same_profile() {
        let mut atlas: serde_json::Value =
            serde_json::from_str(include_str!("../tests/fixtures/endpoints.atlas.json")).unwrap();
        atlas["findings"] = json!([]);
        for edge in atlas["edges"].as_object_mut().unwrap().values_mut() {
            if edge["src"] == "node:/scan_filter" {
                edge["facts"] =
                    json!([{"layer":"coded","profile":"profile:other","confidence":"certain"}]);
            }
            if edge["src"] == "node:/fake_scan" {
                edge["facts"] = json!([{"layer":"coded","profile":"*","confidence":"unknown"}]);
            }
        }
        let lenses = fixture_lenses(&atlas.to_string());
        assert_eq!(
            lenses[0].command.as_ref().unwrap().title,
            "demo: no publisher · other: 1 publisher: /scan_filter"
        );
        for edge in atlas["edges"].as_object_mut().unwrap().values_mut() {
            if edge["src"] == "node:/scan_filter" {
                edge["facts"][0]["profile"] = json!("*");
            }
        }
        assert_eq!(
            fixture_lenses(&atlas.to_string())[0]
                .command
                .as_ref()
                .unwrap()
                .title,
            "1 publisher: /scan_filter"
        );
    }

    #[test]
    fn wildcard_sites_include_profiles_without_endpoint_facts() {
        let mut atlas: serde_json::Value =
            serde_json::from_str(include_str!("../tests/fixtures/endpoints.atlas.json")).unwrap();
        atlas["findings"] = json!([]);
        atlas["entities"]["profile:real"] = json!({"kind":"profile"});
        let listener = "edge:subscribes|node:/listener|topic:/scan|scan";
        atlas["edges"][listener]["facts"][0]["profile"] = json!("*");
        for edge in atlas["edges"].as_object_mut().unwrap().values_mut() {
            if edge["src"] != "node:/listener" {
                edge["facts"][0]["profile"] = json!("profile:demo");
            }
        }
        let title = fixture_lenses(&atlas.to_string())[0]
            .command
            .as_ref()
            .unwrap()
            .title
            .clone();
        assert!(title.contains("demo: 2 publishers"));
        assert!(title.contains("real: no publisher"));
        for fact in atlas["edges"][listener]["facts"].as_array_mut().unwrap() {
            fact["confidence"] = json!("unknown");
        }
        assert!(fixture_lenses(&atlas.to_string())
            .iter()
            .all(|lens| lens.range.start.line != 0));
    }

    #[test]
    fn endpoint_sites_peers_qos_and_shared_lines() {
        let lenses = fixture_lenses(include_str!("../tests/fixtures/endpoints.atlas.json"));
        assert_eq!(lenses.len(), 3);
        let titles: Vec<_> = lenses
            .iter()
            .map(|lens| lens.command.as_ref().unwrap().title.as_str())
            .collect();
        assert_eq!(
            titles,
            [
                "2 publishers: /fake_scan, /scan_filter · QoS incompatible",
                "no subscriber",
                "/output: no subscriber · /scan: 2 publishers: /fake_scan, /scan_filter · QoS incompatible",
            ]
        );
        for (line, lens) in lenses.iter().enumerate() {
            assert_eq!(
                lens.range,
                Range::new(Position::new(line as u32, 0), Position::new(line as u32, 0))
            );
            let command = lens.command.as_ref().unwrap();
            assert_eq!(command.command, "rosio.showEntity");
            assert_eq!(
                command.arguments,
                Some(vec![json!(if line == 0 {
                    "topic:/scan"
                } else {
                    "topic:/output"
                })])
            );
        }
    }

    #[test]
    fn a_wrapper_endpoint_with_no_peer_gets_no_lens() {
        let mut atlas: serde_json::Value =
            serde_json::from_str(include_str!("../tests/fixtures/endpoints.atlas.json")).unwrap();
        let output = "edge:publishes|node:/listener|topic:/output|output";
        atlas["edges"][output]["facts"][0]["values"] = json!({"via_known_wrapper": "tf2_ros"});
        let lenses = fixture_lenses(&atlas.to_string());
        assert!(lenses.iter().all(|lens| lens.range.start.line != 1));
        assert_eq!(lenses.len(), 2);
    }

    #[test]
    fn services_actions_and_reverse_directions() {
        let fixture = include_str!("../tests/fixtures/endpoints.atlas.json");
        for kind in ["service", "action"] {
            let text = fixture
                .replace("topic:", &format!("{kind}:"))
                .replace("\"topic\"", &format!("\"{kind}\""))
                .replace("publishes", "serves")
                .replace("subscribes", "calls");
            let lenses = fixture_lenses(&text);
            assert!(lenses[0]
                .command
                .as_ref()
                .unwrap()
                .title
                .starts_with("2 servers: /fake_scan, /scan_filter"));
            assert_eq!(lenses[1].command.as_ref().unwrap().title, "no client");
        }
        let reversed = fixture
            .replace("publishes", "swapped")
            .replace("subscribes", "publishes")
            .replace("swapped", "subscribes");
        let lenses = fixture_lenses(&reversed);
        assert!(lenses[0]
            .command
            .as_ref()
            .unwrap()
            .title
            .starts_with("2 subscribers:"));
        assert_eq!(lenses[1].command.as_ref().unwrap().title, "no publisher");
    }

    #[test]
    fn labels_are_limited_sorted_and_names_match_whole_nodes() {
        assert_eq!(
            peer_title(vec!["/z", "/b", "/a", "/d", "/c"], "publisher"),
            "5 publishers: /a, /b, /c and 2 more"
        );
        assert_eq!(peer_title(vec!["/a"], "subscriber"), "1 subscriber: /a");
        assert!(names_node("/listener requests reliable", "/listener"));
        assert!(!names_node(
            "/listener_extra requests reliable",
            "/listener"
        ));
        assert!(!names_node("/ns/listener requests reliable", "/listener"));
        let fixture = include_str!("../tests/fixtures/endpoints.atlas.json");
        let text = fixture
            .replace("/listener never", "/listener_extra never")
            .replace("while /listener requests", "while /listener_extra requests");
        assert_eq!(
            fixture_lenses(&text)[0].command.as_ref().unwrap().title,
            "2 publishers: /fake_scan, /scan_filter"
        );
    }
}
