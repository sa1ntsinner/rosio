use crate::atlas::{string, Atlas};
use crate::ids;
use crate::index::Index;
use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, BTreeSet};

pub(super) fn profile(params: &Value) -> Option<String> {
    params["profile"].as_str().map(|p| {
        if p.starts_with("profile:") {
            p.to_owned()
        } else {
            format!("profile:{p}")
        }
    })
}

pub(super) fn facts<'a>(record: &'a Value, profile: Option<&str>) -> Vec<&'a Value> {
    let mut rows: Vec<_> = record["facts"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|f| {
            profile.is_none_or(|p| string(f, "profile") == p || string(f, "profile") == "*")
        })
        .collect();
    rows.sort_by_key(|f| {
        (
            string(f, "profile") == "*",
            string(f, "profile"),
            string(f, "layer"),
        )
    });
    rows
}

pub(super) fn visible(record: &Value, profile: Option<&str>) -> bool {
    if profile.is_none() || !facts(record, profile).is_empty() {
        return true;
    }
    if record["facts"].as_array().is_some_and(|f| !f.is_empty()) {
        return false;
    }
    record["profiles"]
        .as_array()
        .is_none_or(|ps| ps.is_empty() || ps.iter().any(|p| p.as_str() == profile || p == "*"))
}

pub(super) fn values(record: &Value, profile: Option<&str>) -> Value {
    let mut result = Map::new();
    for fact in facts(record, profile) {
        if let Some(fields) = fact["values"].as_object() {
            for (key, value) in fields {
                result.entry(key.clone()).or_insert_with(|| value.clone());
            }
        }
    }
    Value::Object(result)
}

fn location(index: &Index, record: &Value, profile: Option<&str>) -> Option<Value> {
    facts(record, profile)
        .iter()
        .flat_map(|f| super::locations(index, &f["evidence"]))
        .next()
        .and_then(|at| serde_json::to_value(at).ok())
}

fn put_location(row: &mut Value, index: &Index, record: &Value, profile: Option<&str>) {
    if let Some(at) = location(index, record, profile) {
        row["location"] = at;
    }
}

fn node(atlas: &Atlas, id: &str) -> Value {
    json!({"id": id, "label": atlas.entities.get(id).map(|e| &e.fields.label)})
}

fn launch_kind(segment: &str) -> &str {
    match segment.split('(').next().unwrap_or("") {
        "Include" | "IncludeLaunchDescription" => "include",
        "Node" | "LifecycleNode" => "node",
        "ComposableNode" => "composable",
        "ComposableNodeContainer" => "container",
        "Group" | "GroupAction" => "group",
        _ => "other",
    }
}

pub(super) fn launch_tree(atlas: &Atlas, index: &Index, params: &Value) -> Value {
    let wanted = profile(params);
    let mut profiles = Vec::new();
    for (id, entity) in &atlas.entities {
        if entity.fields.kind != "profile" || wanted.as_ref().is_some_and(|p| p != id) {
            continue;
        }
        let p = Some(id.as_str());
        let stated = values(entity, p);
        let mut entries = Vec::new();
        for (file_id, file) in &atlas.entities {
            if file.fields.kind != "launchfile" || !visible(file, p) {
                continue;
            }
            if stated["launch_files"]
                .as_array()
                .is_some_and(|files| !files.iter().any(|f| f == file_id))
            {
                continue;
            }
            let mut rows = BTreeMap::new();
            let mut include_scopes = Vec::new();
            for (_, edge) in atlas.edges_touching(file_id).filter(|(_, e)| {
                e.fields.common.kind == "contains" && e.fields.src == *file_id && visible(e, p)
            }) {
                let ent_id = &edge.fields.dst;
                let Some(ent) = atlas
                    .entities
                    .get(ent_id)
                    .filter(|e| e.fields.kind == "launchent" && visible(e, p))
                else {
                    continue;
                };
                let Ok(parsed) = ids::parse(ent_id) else {
                    continue;
                };
                let Some(segment) = parsed.segments.last() else {
                    continue;
                };
                let v = values(ent, p);
                if let (Some(path), Some(file)) = (
                    v["entity_path"].as_array(),
                    v["included_file"]
                        .as_str()
                        .and_then(|id| id.strip_prefix("launchfile:")),
                ) {
                    let mut scope = parsed.segments[..2].to_vec();
                    scope.extend(path.iter().filter_map(Value::as_str).map(str::to_owned));
                    include_scopes.push((ent_id.clone(), scope, format!("file:{file}")));
                }
                let nodes: BTreeMap<_, _> = atlas
                    .edges_touching(ent_id)
                    .map(|(_, e)| e)
                    .filter(|e| {
                        e.fields.common.kind == "launches"
                            && e.fields.src == *ent_id
                            && visible(e, p)
                    })
                    .filter_map(|e| {
                        atlas
                            .entities
                            .get(&e.fields.dst)
                            .filter(|n| n.fields.kind == "node")?;
                        Some((e.fields.dst.clone(), node(atlas, &e.fields.dst)))
                    })
                    .collect();
                let mut row = json!({
                    "id": ent_id, "kind": launch_kind(segment), "label": segment,
                    "nodes": nodes.into_values().collect::<Vec<_>>(),
                    "package": v["pkg"], "executable": v["executable"], "namespace": v["namespace"],
                    "conditions": v.get("conditions").cloned().unwrap_or(json!([])),
                    "gates": v.get("gates").cloned().unwrap_or(json!([])),
                    "parameter_inputs": v.get("parameter_inputs").cloned().unwrap_or(json!([])),
                    "remaps": v.get("remaps").cloned().unwrap_or(json!([]))
                });
                put_location(&mut row, index, ent, p);
                rows.insert(ent_id.clone(), row);
            }
            let mut parents = BTreeMap::new();
            for ent_id in rows.keys() {
                let mut prefix = ent_id.as_str();
                while let Some((parent, _)) = prefix.rsplit_once('|') {
                    if rows.contains_key(parent) {
                        parents.insert(ent_id.clone(), parent.to_owned());
                        break;
                    }
                    prefix = parent;
                }
            }
            for ent_id in rows.keys() {
                let Ok(parsed) = ids::parse(ent_id) else {
                    continue;
                };
                let candidates: Vec<_> = include_scopes
                    .iter()
                    .filter(|(_, scope, file)| {
                        scope.len() < parsed.segments.len()
                            && parsed.segments.starts_with(scope)
                            && parents
                                .get(ent_id)
                                .is_none_or(|id| id.split('|').count() < scope.len())
                            && facts(&atlas.entities[ent_id], p).iter().any(|fact| {
                                fact["evidence"]
                                    .as_array()
                                    .into_iter()
                                    .flatten()
                                    .any(|row| row["file"] == *file)
                            })
                    })
                    .collect();
                if let [candidate] = candidates.as_slice() {
                    parents.insert(ent_id.clone(), candidate.0.clone());
                }
            }
            let children = nested_rows(&rows, &parents);
            let v = values(file, p);
            let mut row = json!({"id":file_id,"label":file.fields.label,
                "arg_declarations":v.get("arg_declarations").cloned().unwrap_or(json!([])),
                "args":v.get("args").cloned().unwrap_or(json!([])),"children":children});
            put_location(&mut row, index, file, p);
            entries.push(row);
        }
        let command = stated.get("command").cloned().unwrap_or_else(|| {
            stated["entrypoints"]
                .as_array()
                .map_or(Value::Null, |entries| {
                    json!(entries
                        .iter()
                        .filter_map(Value::as_str)
                        .collect::<Vec<_>>()
                        .join("\n"))
                })
        });
        profiles.push(json!({"id":id,"label":entity.fields.label,
            "command":command,"entries":entries}));
    }
    json!(profiles)
}

fn nested_rows(rows: &BTreeMap<String, Value>, parents: &BTreeMap<String, String>) -> Vec<Value> {
    fn branch(
        id: &str,
        rows: &BTreeMap<String, Value>,
        parents: &BTreeMap<String, String>,
        seen: &mut BTreeSet<String>,
    ) -> Value {
        let mut row = rows[id].clone();
        if !seen.insert(id.to_owned()) {
            row["children"] = json!([]);
            return row;
        }
        let children: Vec<_> = rows
            .keys()
            .filter(|child| parents.get(*child).is_some_and(|p| p == id))
            .filter(|child| !seen.contains(*child))
            .cloned()
            .collect();
        row["children"] = json!(children
            .iter()
            .map(|child| branch(child, rows, parents, seen))
            .collect::<Vec<_>>());
        row
    }
    let mut seen = BTreeSet::new();
    let mut roots = Vec::new();
    for id in rows.keys().filter(|id| !parents.contains_key(*id)) {
        roots.push(branch(id, rows, parents, &mut seen));
    }
    for id in rows.keys() {
        if !seen.contains(id) {
            roots.push(branch(id, rows, parents, &mut seen));
        }
    }
    roots
}

pub(super) fn frame_tree(atlas: &Atlas, index: &Index, params: &Value) -> Value {
    let wanted = profile(params);
    let p = wanted.as_deref();
    let mut rows = BTreeMap::new();
    for (id, frame) in &atlas.entities {
        if frame.fields.kind != "frame" || !visible(frame, p) {
            continue;
        }
        let mut row = json!({"id":id,"label":frame.fields.label,
            "joint":null,"joint_type":null,"model":null});
        put_location(&mut row, index, frame, p);
        for kind in ["broadcasts_tf", "listens_tf"] {
            let mut nodes = BTreeMap::new();
            for (_, edge) in atlas.edges_touching(id).filter(|(_, e)| {
                e.fields.common.kind == kind && e.fields.dst == *id && visible(e, p)
            }) {
                let Some(source) = atlas
                    .entities
                    .get(&edge.fields.src)
                    .filter(|e| e.fields.kind == "node")
                else {
                    continue;
                };
                let mut n = node(atlas, &edge.fields.src);
                if let Some(at) = location(index, edge, p).or_else(|| location(index, source, p)) {
                    n["location"] = at;
                }
                nodes.entry(edge.fields.src.clone()).or_insert(n);
            }
            row[if kind == "broadcasts_tf" {
                "broadcasters"
            } else {
                "listeners"
            }] = json!(nodes.into_values().collect::<Vec<_>>());
        }
        let mut findings: Vec<_> = atlas
            .findings_for(id)
            .iter()
            .filter(|f| {
                p.is_none_or(|p| {
                    f["profiles"]
                        .as_array()
                        .is_none_or(|ps| ps.is_empty() || ps.iter().any(|v| v == p || v == "*"))
                })
            })
            .map(|f| f["id"].clone())
            .collect();
        findings.sort_by(|a, b| a.as_str().cmp(&b.as_str()));
        row["findings"] = json!(findings);
        rows.insert(id.clone(), row);
    }
    let mut links: BTreeMap<String, Vec<(String, Value)>> = BTreeMap::new();
    let mut parented = BTreeSet::new();
    for edge in atlas
        .edges
        .values()
        .filter(|e| e.fields.common.kind == "tf_parent" && visible(e, p))
    {
        if !rows.contains_key(&edge.fields.src) || !rows.contains_key(&edge.fields.dst) {
            continue;
        }
        let mut row = rows[&edge.fields.src].clone();
        row["parent"] = json!(edge.fields.dst);
        let v = values(edge, p);
        for key in ["joint", "joint_type", "model", "xyz", "rpy"] {
            if let Some(value) = v.get(key) {
                row[key] = value.clone();
            }
        }
        put_location(&mut row, index, edge, p);
        links
            .entry(edge.fields.dst.clone())
            .or_default()
            .push((edge.fields.src.clone(), row));
        parented.insert(edge.fields.src.clone());
    }
    for children in links.values_mut() {
        children.sort_by(|a, b| a.0.cmp(&b.0));
    }
    fn branch(
        id: &str,
        mut row: Value,
        links: &BTreeMap<String, Vec<(String, Value)>>,
        path: &mut BTreeSet<String>,
        seen: &mut BTreeSet<String>,
    ) -> Value {
        seen.insert(id.to_owned());
        path.insert(id.to_owned());
        let model = row["model"].clone();
        row["children"] = json!(links
            .get(id)
            .into_iter()
            .flatten()
            .filter(|(_, child)| {
                // one robot model's edges never continue another model's chain
                model.is_null() || child["model"].is_null() || child["model"] == model
            })
            .filter_map(|(child, row)| {
                if path.contains(child) {
                    return None;
                }
                Some(branch(child, row.clone(), links, path, seen))
            })
            .collect::<Vec<_>>());
        path.remove(id);
        row
    }
    let mut roots = Vec::new();
    let mut seen = BTreeSet::new();
    for (id, row) in rows.iter().filter(|(id, _)| !parented.contains(*id)) {
        roots.push(branch(
            id,
            row.clone(),
            &links,
            &mut BTreeSet::new(),
            &mut seen,
        ));
    }
    for (id, row) in &rows {
        if !seen.contains(id) {
            roots.push(branch(
                id,
                row.clone(),
                &links,
                &mut BTreeSet::new(),
                &mut seen,
            ));
        }
    }
    json!(roots)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::Paths;

    fn fixture() -> (Atlas, Index) {
        let atlas = Atlas::parse(include_str!("../../tests/fixtures/trees.atlas.json")).unwrap();
        let workspace = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let paths = Paths::new(&workspace, &atlas, &BTreeMap::new());
        let index = Index::new(&atlas, &paths);
        (atlas, index)
    }

    #[test]
    fn launch_profiles_nesting_values_and_locations() {
        let (atlas, index) = fixture();
        let result =
            super::super::query(&atlas, &index, "rosio/launchTree", json!({"profile":"p"}))
                .unwrap();
        assert_eq!(result.as_array().unwrap().len(), 1);
        assert_eq!(
            result[0]["command"],
            json!("ros2 launch demo parent.launch.py")
        );
        let entries = result[0]["entries"].as_array().unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0]["args"], json!([["mode", "real"]]));
        assert_eq!(entries[0]["arg_declarations"], json!(["mode"]));
        let rows = entries[0]["children"].as_array().unwrap();
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0]["kind"], "container");
        assert_eq!(rows[0]["executable"], "component_container");
        assert_eq!(rows[1]["kind"], "group");
        assert_eq!(rows[1]["children"][0]["kind"], "node");
        assert_eq!(
            rows[1]["children"][0]["gates"][0]["mechanism"],
            "IfCondition"
        );
        assert_eq!(rows[2]["kind"], "include");
        assert_eq!(rows[2]["label"], "Include(child|file.launch.py)");
        assert_eq!(rows[2]["location"]["line"], 8);
        let camera = &rows[2]["children"][0];
        assert_eq!(camera["kind"], "composable");
        assert_eq!(camera["namespace"], "/robot");
        assert_eq!(camera["package"], "demo");
        assert_eq!(
            camera["nodes"],
            json!([{"id":"node:/camera","label":"camera"}])
        );
        assert_eq!(camera["parameter_inputs"][0]["values"]["rate"], 20);
        assert_eq!(camera["remaps"], json!([["image", "/image", "node"]]));
        assert_eq!(camera["location"]["line"], 3);
        assert!(camera["location"]["uri"]
            .as_str()
            .unwrap()
            .ends_with("/include_child.launch.py"));
        assert_eq!(
            result,
            launch_tree(&atlas, &index, &json!({"profile":"profile:p"}))
        );
        assert_eq!(
            launch_tree(&atlas, &index, &json!({}))
                .as_array()
                .unwrap()
                .len(),
            2
        );
        assert_eq!(
            launch_tree(&atlas, &index, &json!({"profile":"missing"})),
            json!([])
        );
    }

    #[test]
    fn missing_source_keeps_the_launch_row_without_a_location() {
        let (atlas, _) = fixture();
        let workspace = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("missing-workspace");
        let paths = Paths::new(&workspace, &atlas, &BTreeMap::new());
        let index = Index::new(&atlas, &paths);
        let result = launch_tree(&atlas, &index, &json!({"profile":"p"}));
        let include = &result[0]["entries"][0]["children"][2];
        assert_eq!(include["kind"], "include");
        assert!(include.get("location").is_none());
        let camera = &include["children"][0];
        assert_eq!(camera["kind"], "composable");
        assert!(camera.get("location").is_none());
    }

    #[test]
    fn frame_direction_geometry_nodes_findings_and_profiles() {
        let (atlas, index) = fixture();
        let result =
            super::super::query(&atlas, &index, "rosio/frameTree", json!({"profile":"p"})).unwrap();
        assert_eq!(result.as_array().unwrap().len(), 2);
        assert_eq!(result[0]["id"], "frame:base");
        assert!(result[0].get("parent").is_none());
        assert_eq!(result[0]["location"]["line"], 1);
        assert_eq!(result[0]["children"].as_array().unwrap().len(), 1);
        let camera = &result[0]["children"][0];
        assert_eq!(camera["id"], "frame:camera");
        assert_eq!(camera["parent"], "frame:base");
        assert_eq!(camera["joint"], "camera_mount");
        assert_eq!(camera["joint_type"], "fixed");
        assert_eq!(camera["model"], "file:@repo|robot.urdf");
        assert_eq!(camera["xyz"], json!([1, 0, 0]));
        assert_eq!(camera["rpy"], json!([0, 0, 0]));
        assert_eq!(camera["location"]["line"], 7);
        assert_eq!(camera["broadcasters"].as_array().unwrap().len(), 1);
        assert_eq!(camera["broadcasters"][0]["location"]["line"], 11);
        assert_eq!(result[1]["id"], "frame:lookup");
        assert_eq!(result[1]["listeners"][0]["id"], "node:/listener");
        assert_eq!(
            result[1]["findings"],
            json!(["find:no_publisher", "find:not_in_urdf"])
        );
        assert_eq!(
            frame_tree(&atlas, &index, &json!({"profile":"q"}))[0]["children"][0]["id"],
            "frame:sim"
        );
        assert_eq!(
            frame_tree(&atlas, &index, &json!({}))[0]["children"]
                .as_array()
                .unwrap()
                .len(),
            2
        );
        assert_eq!(
            result,
            frame_tree(&atlas, &index, &json!({"profile":"profile:p"}))
        );
    }

    #[test]
    fn same_basename_includes_use_their_source_files_without_renaming_nodes() {
        let atlas = Atlas::parse(r#"{"schema_version":"1.6","entities":{
            "profile:p":{"kind":"profile","facts":[{"profile":"profile:p","values":{"launch_files":["launchfile:p|parent.py"]}}]},
            "launchfile:p|parent.py":{"kind":"launchfile"},
            "launchent:p|parent.py|Include(shared.py@a)":{"kind":"launchent","facts":[{"profile":"profile:p","values":{"entity_path":["Include(shared.py)"],"included_file":"launchfile:a|shared.py"}}]},
            "launchent:p|parent.py|Include(shared.py@b)":{"kind":"launchent","facts":[{"profile":"profile:p","values":{"entity_path":["Include(shared.py)"],"included_file":"launchfile:b|shared.py"}}]},
            "launchent:p|parent.py|Include(shared.py)|Node(a)":{"kind":"launchent","facts":[{"profile":"profile:p","evidence":[{"file":"file:a|shared.py","line_hint":4}]}]},
            "launchent:p|parent.py|Include(shared.py)|Node(b)":{"kind":"launchent","facts":[{"profile":"profile:p","evidence":[{"file":"file:b|shared.py","line_hint":7}]}]}
        },"edges":{
            "a":{"kind":"contains","src":"launchfile:p|parent.py","dst":"launchent:p|parent.py|Include(shared.py@a)"},
            "b":{"kind":"contains","src":"launchfile:p|parent.py","dst":"launchent:p|parent.py|Include(shared.py@b)"},
            "c":{"kind":"contains","src":"launchfile:p|parent.py","dst":"launchent:p|parent.py|Include(shared.py)|Node(a)"},
            "d":{"kind":"contains","src":"launchfile:p|parent.py","dst":"launchent:p|parent.py|Include(shared.py)|Node(b)"}
        }}"#).unwrap();
        let result = launch_tree(&atlas, &Index::default(), &json!({"profile":"p"}));
        let rows = result[0]["entries"][0]["children"].as_array().unwrap();
        assert_eq!(rows.len(), 2);
        assert_eq!(
            rows[0]["children"][0]["id"],
            "launchent:p|parent.py|Include(shared.py)|Node(a)"
        );
        assert_eq!(
            rows[1]["children"][0]["id"],
            "launchent:p|parent.py|Include(shared.py)|Node(b)"
        );
    }

    #[test]
    fn cycles_terminate_and_alternative_parents_survive() {
        let atlas = Atlas::parse(
            r#"{"schema_version":"1.6","entities":{
            "frame:a":{"kind":"frame"},"frame:b":{"kind":"frame"},"frame:c":{"kind":"frame"}},
            "edges":{"one":{"kind":"tf_parent","src":"frame:a","dst":"frame:b"},
            "two":{"kind":"tf_parent","src":"frame:b","dst":"frame:a"},
            "three":{"kind":"tf_parent","src":"frame:a","dst":"frame:c"}}}"#,
        )
        .unwrap();
        let result = frame_tree(&atlas, &Index::default(), &json!({}));
        assert_eq!(result[0]["id"], "frame:c");
        assert_eq!(result[0]["children"][0]["id"], "frame:a");
        assert_eq!(result[0]["children"][0]["children"][0]["id"], "frame:b");
        assert_eq!(
            result[0]["children"][0]["children"][0]["children"],
            json!([])
        );
    }

    #[test]
    fn two_models_sharing_frame_names_stay_two_chains() {
        let mut entities = serde_json::Map::new();
        let mut edges = serde_json::Map::new();
        for i in 0..9 {
            entities.insert(format!("frame:f{i}"), json!({"kind":"frame"}));
        }
        for model in ["a", "b"] {
            for i in 0..8 {
                edges.insert(
                    format!("{model}{i}"),
                    json!({"kind":"tf_parent","src":format!("frame:f{}", i + 1),
                    "dst":format!("frame:f{i}"),
                    "facts":[{"profile":"*","values":{"model":format!("file:{model}|robot.urdf")}}]}),
                );
            }
        }
        let text = json!({"schema_version":"1.6","entities":entities,"edges":edges}).to_string();
        let atlas = Atlas::parse(&text).unwrap();
        let result = frame_tree(&atlas, &Index::default(), &json!({}));
        fn count(row: &Value) -> usize {
            1 + row["children"]
                .as_array()
                .unwrap()
                .iter()
                .map(count)
                .sum::<usize>()
        }
        assert_eq!(result.as_array().unwrap().len(), 1);
        assert_eq!(count(&result[0]), 17);
        let a = &result[0]["children"][0];
        assert_eq!(a["model"], "file:a|robot.urdf");
        assert_eq!(a["children"].as_array().unwrap().len(), 1);
        assert_eq!(a["children"][0]["model"], "file:a|robot.urdf");
    }
}
