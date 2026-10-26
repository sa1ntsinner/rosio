use super::trees::profile;
use crate::atlas::{string, Atlas};
use crate::ids;
use crate::index::Index;
use crate::paths::uri;
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// The source row of a candidate as the editor opens it, with the package path a person reads.
fn place(index: &Index, source: &Value) -> Value {
    let mut row = json!({"locator": source["locator"]});
    if let Ok(id) = ids::parse(string(source, "file")) {
        row["file"] = json!(id.segments.join("/").trim_start_matches("@repo/"));
    }
    if let Some(site) = index.row(source) {
        if let Some(uri) = uri(&site.path) {
            row["at"] = json!({"uri": uri, "line": site.line, "approximate": site.approximate});
        }
    }
    row
}

/// Every parameter of a profile, grouped by the node that owns it: the value that wins, the
/// line that set it, and every candidate it beat.
pub(super) fn param_tree(atlas: &Atlas, index: &Index, params: &Value) -> Value {
    let wanted = profile(params);
    let mut nodes: BTreeMap<String, Vec<Value>> = BTreeMap::new();
    for (id, entity) in &atlas.entities {
        if entity.fields.kind != "param" {
            continue;
        }
        let Some((owner, _)) = id
            .strip_prefix("param:")
            .and_then(|body| body.rsplit_once('|'))
        else {
            continue;
        };
        let in_profile = |row: &Value| {
            wanted
                .as_deref()
                .is_none_or(|p| string(row, "profile") == p)
        };
        let candidates: Vec<Value> = entity
            .fields
            .candidates
            .iter()
            .filter(|c| in_profile(c))
            .map(|c| {
                let mut row = json!({
                    "rank": c["rank"],
                    "wins": c["wins"],
                    "value": c["value"],
                    "valueType": c["value_type"],
                    "mechanism": c["mechanism"],
                    "shadowKind": c["shadow_kind"],
                });
                row["source"] = place(index, &c["source"]);
                row
            })
            .collect();
        let effective = entity
            .fields
            .effective
            .as_object()
            .and_then(|by_profile| match &wanted {
                Some(p) => by_profile.get(p),
                None => by_profile.values().next(),
            });
        if wanted.is_some()
            && candidates.is_empty()
            && effective.is_none()
            && !entity
                .fields
                .profiles
                .iter()
                .any(|p| Some(p) == wanted.as_ref())
        {
            continue;
        }
        let mut row = json!({
            "id": id,
            "name": entity.fields.label,
            "state": entity.raw["state"],
            "layers": entity.fields.layers,
            "findings": atlas.findings_for(id).len(),
            "candidates": candidates,
        });
        if let Some(value) = effective.map(|e| &e["value"]) {
            row["value"] = value["value"].clone();
            row["valueType"] = value["value_type"].clone();
            row["source"] = place(index, &value["source"]);
        }
        nodes.entry(format!("node:{owner}")).or_default().push(row);
    }
    let rows: Vec<Value> = nodes
        .into_iter()
        .map(|(node, mut rows)| {
            rows.sort_by(|a, b| string(a, "name").cmp(string(b, "name")));
            let label = atlas.entities.get(&node).map_or_else(
                || node.trim_start_matches("node:").to_owned(),
                |e| e.fields.label.clone(),
            );
            json!({"id": node, "label": label, "params": rows})
        })
        .collect();
    json!({"profile": wanted, "nodes": rows})
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::Paths;

    const ATLAS: &str = r#"{"schema_version":"1.9","entities":{
        "node:/scan_filter":{"kind":"node","label":"/scan_filter","profiles":["profile:rover"]},
        "param:/scan_filter|max_range":{"kind":"param","label":"max_range","state":"effective",
          "layers":["declared","coded"],"profiles":["profile:rover"],
          "candidates":[
            {"profile":"profile:rover","rank":0,"wins":true,"value":10.0,"value_type":"float",
             "mechanism":"launch_inline_dict","source":{"file":"file:rover_bringup|launch/bringup.launch.py","locator":"parameters","root":"src"}},
            {"profile":"profile:rover","rank":1,"wins":false,"value":8.0,"value_type":"float",
             "mechanism":"launch_params_file","shadow_kind":"value_changed","shadowed_by":0,
             "source":{"file":"file:rover_bringup|config/rover.yaml","locator":"scan_filter.ros__parameters.max_range","root":"src"}},
            {"profile":"profile:sim","rank":0,"wins":true,"value":4.0,"value_type":"float",
             "mechanism":"launch_params_file","source":{"file":"file:rover_bringup|config/sim.yaml","locator":"x","root":"src"}}],
          "effective":{"profile:rover":{"value":{"value":10.0,"value_type":"float",
             "source":{"file":"file:rover_bringup|launch/bringup.launch.py","locator":"parameters","root":"src"}}}}},
        "param:/safety_stop|stop_distnce":{"kind":"param","label":"stop_distnce","state":"effective",
          "profiles":["profile:rover"],"candidates":[],"effective":{}}
      },
      "findings":[{"id":"find:param.set_undeclared|param:/safety_stop%7Cstop_distnce|-","about":{"entity":"param:/safety_stop|stop_distnce"}}]}"#;

    fn tree(params: Value) -> Value {
        let atlas = Atlas::parse(ATLAS).unwrap();
        let workspace = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let index = Index::new(&atlas, &Paths::new(&workspace, &atlas, &BTreeMap::new()));
        param_tree(&atlas, &index, &params)
    }

    #[test]
    fn parameters_group_by_node_with_the_chain_of_one_profile() {
        let result = tree(json!({"profile": "rover"}));
        assert_eq!(result["profile"], "profile:rover");
        let nodes = result["nodes"].as_array().unwrap();
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes[0]["label"], "/safety_stop");
        assert_eq!(nodes[0]["params"][0]["findings"], 1);
        assert!(nodes[0]["params"][0].get("value").is_none());
        let range = &nodes[1]["params"][0];
        assert_eq!(range["value"], 10.0);
        assert_eq!(
            range["source"]["file"],
            "rover_bringup/launch/bringup.launch.py"
        );
        let chain = range["candidates"].as_array().unwrap();
        assert_eq!(chain.len(), 2);
        assert_eq!(chain[1]["shadowKind"], "value_changed");
        assert_eq!(
            chain[1]["source"]["locator"],
            "scan_filter.ros__parameters.max_range"
        );
    }

    #[test]
    fn another_profile_keeps_only_its_own_rows() {
        let result = tree(json!({"profile": "profile:sim"}));
        let nodes = result["nodes"].as_array().unwrap();
        assert_eq!(nodes.len(), 1);
        let chain = nodes[0]["params"][0]["candidates"].as_array().unwrap();
        assert_eq!(chain.len(), 1);
        assert_eq!(chain[0]["value"], 4.0);
        assert!(nodes[0]["params"][0].get("value").is_none());
    }
}
