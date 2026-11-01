//! Custom view requests and standard document queries.
mod graph;
mod params;
mod robot;
mod trees;

use crate::atlas::{string, Atlas};
use crate::index::Index;
use crate::paths::uri;
use crate::{hover, navigation};
use lsp_types::{CodeLensParams, InlayHintParams, TextDocumentPositionParams, Url};
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::BTreeMap;

#[derive(Serialize)]
struct SourceLocation {
    uri: Url,
    line: u32,
}

#[derive(Serialize)]
struct FindingRow<'a> {
    id: &'a Value,
    rule: &'a Value,
    severity: &'a Value,
    title: &'a Value,
    say: &'a Value,
    fix: &'a Value,
    fixable: bool,
    locations: Vec<SourceLocation>,
}

/// A value the atlas could not resolve, with the line that asked for it.
#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct UnresolvedRow<'a> {
    id: &'a Value,
    want: &'a Value,
    expr: &'a Value,
    reason: &'a Value,
    owner: &'a str,
    resolvable_by: &'a Value,
    locations: Vec<SourceLocation>,
}

#[derive(Serialize)]
struct SearchHit<'a> {
    id: &'a str,
    kind: &'a Value,
    label: &'a Value,
    layers: &'a Value,
}

pub(crate) fn query(
    atlas: &Atlas,
    index: &Index,
    method: &str,
    params: Value,
) -> Result<Value, String> {
    match method {
        "textDocument/hover" | "textDocument/definition" | "textDocument/references" => {
            let position: TextDocumentPositionParams =
                serde_json::from_value(params).map_err(|e| e.to_string())?;
            let uri = &position.text_document.uri;
            let line = position.position.line;
            match method {
                "textDocument/hover" => serde_json::to_value(hover::at(atlas, index, uri, line)),
                "textDocument/definition" => {
                    serde_json::to_value(navigation::definitions(atlas, index, uri, line))
                }
                _ => serde_json::to_value(navigation::references(atlas, index, uri, line)),
            }
            .map_err(|e| e.to_string())
        }
        "textDocument/codeLens" => {
            let params: CodeLensParams =
                serde_json::from_value(params).map_err(|e| e.to_string())?;
            serde_json::to_value(crate::lens::lenses(atlas, index, &params.text_document.uri))
                .map_err(|e| e.to_string())
        }
        "textDocument/inlayHint" => {
            let params: InlayHintParams =
                serde_json::from_value(params).map_err(|e| e.to_string())?;
            serde_json::to_value(crate::inlay::hints(
                atlas,
                index,
                &params.text_document.uri,
                params.range,
            ))
            .map_err(|e| e.to_string())
        }
        "workspace/symbol" => {
            let query = required_string(&params, "query")?.to_lowercase();
            Ok(json!(atlas
                .entities
                .iter()
                .filter(|(_, v)| string(v, "label").to_lowercase().contains(&query))
                .filter_map(|(id, entity)| {
                    let location = index
                        .owners
                        .get(id)
                        .into_iter()
                        .flatten()
                        .find_map(|site| site.location(index))
                        .or_else(|| {
                            atlas
                                .edges_touching(id)
                                .flat_map(|(edge_id, _)| {
                                    index.owners.get(edge_id).into_iter().flatten()
                                })
                                .find_map(|site| site.location(index))
                        })?;
                    Some(json!({"name":string(entity,"label"),"kind":13,"location":location}))
                })
                .collect::<Vec<_>>()))
        }
        "rosio/search" => search(atlas, &params),
        "rosio/entity" => {
            let id = required_string(&params, "id")?;
            let Some(mut entity) = atlas.entities.get(id).map(|entity| entity.raw.clone()) else {
                return Ok(Value::Null);
            };
            entity["sites"] = site_rows(index, id);
            entity["relations"] = relations(atlas, id);
            locate_candidates(index, &mut entity);
            entity["findings"] = json!(atlas
                .findings_for(id)
                .iter()
                .map(|f| &f["id"])
                .collect::<Vec<_>>());
            Ok(entity)
        }
        "rosio/findings" => {
            let minimum = params
                .get("minSeverity")
                .and_then(Value::as_str)
                .unwrap_or("info");
            let Some(rank) = severity_rank(minimum) else {
                return Err("minSeverity must be error, warn or info".into());
            };
            let rows: Vec<_> = atlas
                .findings
                .iter()
                .filter(|f| severity_rank(string(f, "severity")).unwrap_or(2) <= rank)
                .map(|finding| FindingRow {
                    id: &finding["id"],
                    rule: &finding["rule"],
                    severity: &finding["severity"],
                    title: &finding["title"],
                    say: &finding["say"],
                    fix: &finding["fix"],
                    fixable: finding["edits"]
                        .as_array()
                        .is_some_and(|rows| !rows.is_empty()),
                    locations: locations(index, &finding["where"]),
                })
                .collect();
            serde_json::to_value(rows).map_err(|e| e.to_string())
        }
        "rosio/children" => Ok(children(atlas, &params)),
        "rosio/launchTree" => Ok(trees::launch_tree(atlas, index, &params)),
        "rosio/frameTree" => Ok(trees::frame_tree(atlas, index, &params)),
        "rosio/paramTree" => Ok(params::param_tree(atlas, index, &params)),
        "rosio/graph" => Ok(graph::graph(atlas, index, &params)),
        "rosio/robotModel" => Ok(robot::robot_model(atlas, index, &params)),
        "rosio/unresolved" => {
            let rows: Vec<_> = atlas
                .unresolved
                .iter()
                .map(|entry| UnresolvedRow {
                    id: &entry["id"],
                    want: &entry["want"],
                    expr: &entry["expr"],
                    reason: &entry["reason"],
                    owner: atlas
                        .entities
                        .get(string(entry, "owner"))
                        .map_or(string(entry, "owner"), |owner| owner.fields.label.as_str()),
                    resolvable_by: &entry["resolvable_by"],
                    locations: locations(index, &entry["evidence"]),
                })
                .collect();
            serde_json::to_value(rows).map_err(|e| e.to_string())
        }
        _ => Err(format!("Unknown method {method}")),
    }
}

/// The editor locations of a list of provenance rows, skipping rows outside every root.
fn locations(index: &Index, rows: &Value) -> Vec<SourceLocation> {
    rows.as_array()
        .into_iter()
        .flatten()
        .filter_map(|row| {
            let site = index.row(row)?;
            Some(SourceLocation {
                uri: uri(&site.path)?,
                line: site.line,
            })
        })
        .collect()
}

fn required_string<'a>(params: &'a Value, key: &str) -> Result<&'a str, String> {
    params
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("Missing string parameter {key}"))
}
fn severity_rank(value: &str) -> Option<u8> {
    match value {
        "error" => Some(0),
        "warn" => Some(1),
        "info" => Some(2),
        _ => None,
    }
}
/// One row per place an entity is written, once each even when several layers cite it.
fn site_rows(index: &Index, id: &str) -> Value {
    let mut seen = std::collections::BTreeSet::new();
    let mut rows = Vec::new();
    for site in index.owners.get(id).into_iter().flatten() {
        let Some(uri) = uri(&site.path) else {
            continue;
        };
        if seen.insert((uri.to_string(), site.line)) {
            rows.push(json!({"uri": uri, "line": site.line, "approximate": site.approximate}));
        }
    }
    Value::Array(rows)
}

/// Give every parameter candidate the editor location of its source row, so a card can
/// open the line that set the value.
fn locate_candidates(index: &Index, entity: &mut Value) {
    let Some(candidates) = entity.get_mut("candidates").and_then(Value::as_array_mut) else {
        return;
    };
    for candidate in candidates {
        let Some(site) = candidate.get("source").and_then(|source| index.row(source)) else {
            continue;
        };
        if let Some(uri) = uri(&site.path) {
            candidate["at"] =
                json!({"uri": uri, "line": site.line, "approximate": site.approximate});
        }
    }
}

/// The effective value of a parameter in its first profile, for a card that lists many.
fn effective_value(entity: &crate::atlas::Record<crate::atlas::Entity>) -> Option<&Value> {
    if entity.fields.kind != "param" {
        return None;
    }
    entity
        .fields
        .effective
        .as_object()?
        .values()
        .next()?
        .get("value")?
        .get("value")
}

/// Most relations an entity card lists, so a package with a thousand files stays readable.
const RELATION_LIMIT: usize = 400;

/// The edges that touch an entity, as the card lists them: what kind, which way, to what.
fn relations(atlas: &Atlas, id: &str) -> Value {
    let mut rows = Vec::new();
    for (_, edge) in atlas.edges_touching(id) {
        let fields = &edge.fields;
        let (direction, other) = if fields.src == id {
            ("out", &fields.dst)
        } else if fields.dst == id {
            ("in", &fields.src)
        } else {
            continue;
        };
        let entity = atlas.entities.get(other);
        let mut row = json!({
            "kind": fields.common.kind,
            "direction": direction,
            "id": other,
            "otherKind": entity.map_or("", |e| e.fields.kind.as_str()),
            "label": entity.map_or("", |e| e.fields.label.as_str()),
            "layers": fields.common.layers,
        });
        if let Some(value) = entity.and_then(effective_value) {
            row["value"] = value.clone();
        }
        row["facts"] = edge_facts(&edge.raw);
        rows.push(row);
        if rows.len() == RELATION_LIMIT {
            break;
        }
    }
    Value::Array(rows)
}
/// What each layer stated about one edge, the QoS of a publisher or a subscription above all,
/// without the evidence rows a card does not show.
fn edge_facts(edge: &Value) -> Value {
    let facts = edge["facts"].as_array().into_iter().flatten();
    Value::Array(
        facts
            .map(|fact| {
                json!({
                    "layer": fact["layer"],
                    "profile": fact["profile"],
                    "values": fact["values"],
                })
            })
            .collect(),
    )
}

fn search(atlas: &Atlas, params: &Value) -> Result<Value, String> {
    let query = required_string(params, "query")?.to_lowercase();
    let kinds = params.get("kinds").and_then(Value::as_array);
    let limit = params.get("limit").and_then(Value::as_u64).unwrap_or(100) as usize;
    let mut matches: Vec<_> = atlas
        .entities
        .iter()
        .filter_map(|(id, entity)| {
            let label = string(entity, "label").to_lowercase();
            if !label.contains(&query)
                || kinds.is_some_and(|ks| {
                    !ks.iter()
                        .any(|k| k.as_str() == Some(string(entity, "kind")))
                })
            {
                return None;
            }
            let rank = if label == query {
                0
            } else if label.starts_with(&query) {
                1
            } else {
                2
            };
            Some((rank, label, id, entity))
        })
        .collect();
    matches.sort_by(|a, b| (&a.0, &a.1, &a.2).cmp(&(&b.0, &b.1, &b.2)));
    let rows: Vec<_> = matches
        .into_iter()
        .take(limit)
        .map(|(_, _, id, e)| SearchHit {
            id,
            kind: &e["kind"],
            label: &e["label"],
            layers: &e["layers"],
        })
        .collect();
    serde_json::to_value(rows).map_err(|e| e.to_string())
}
fn children(atlas: &Atlas, params: &Value) -> Value {
    let mut groups: BTreeMap<String, Vec<String>> = BTreeMap::new();
    if let Some(by_kind) = atlas.indexes["by_kind"].as_object() {
        for (kind, ids) in by_kind {
            groups.insert(
                kind.clone(),
                ids.as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(Value::as_str)
                    .map(str::to_owned)
                    .collect(),
            );
        }
    } else {
        for (id, entity) in &atlas.entities {
            groups
                .entry(string(entity, "kind").into())
                .or_default()
                .push(id.clone());
        }
    }
    groups.retain(|kind, _| {
        params
            .get("kind")
            .and_then(Value::as_str)
            .is_none_or(|wanted| wanted == kind)
    });
    for ids in groups.values_mut() {
        ids.retain(|id| {
            atlas.entities.get(id).is_some_and(|entity| {
                params
                    .get("profile")
                    .and_then(Value::as_str)
                    .is_none_or(|profile| {
                        entity["profiles"].as_array().is_some_and(|ps| {
                            ps.iter().any(|p| {
                                p.as_str().is_some_and(|p| {
                                    p == profile || p.strip_prefix("profile:") == Some(profile)
                                })
                            })
                        })
                    })
            })
        });
        ids.sort();
        ids.dedup();
    }
    json!(groups)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    #[test]
    fn ranked_search_and_children() {
        let atlas = Atlas::parse(r#"{"schema_version":"1.1","entities":{
            "topic:/cmd_vel":{"kind":"topic","label":"cmd_vel","layers":["coded"],"profiles":["profile:p"]},
            "node:/a":{"kind":"node","label":"a_cmd_vel","layers":[]},
            "topic:/cmd_vel_out":{"kind":"topic","label":"cmd_vel_out","layers":[]}}}"#).unwrap();
        let result = search(&atlas, &json!({"query":"CMD_VEL"})).unwrap();
        assert_eq!(result[0]["label"], "cmd_vel");
        assert_eq!(result[1]["label"], "cmd_vel_out");
        assert_eq!(
            search(&atlas, &json!({"query":"cmd_vel","kinds":["node"]}))
                .unwrap()
                .as_array()
                .unwrap()
                .len(),
            1
        );
        assert_eq!(
            children(&atlas, &json!({"profile":"p"}))["topic"],
            json!(["topic:/cmd_vel"])
        );
    }
}
