use super::graph::{finding_in_profile, findings};
use super::trees::{facts, profile, values, visible};
use crate::atlas::{string, Atlas};
use crate::index::Index;
use crate::paths::uri;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

/// Most links, and separately most broadcasts, one answer carries.
const LIMIT: usize = 1000;
/// Joint values copied only when the atlas states them.
const JOINT_KEYS: [&str; 6] = ["xyz", "rpy", "axis", "limit", "mimic", "unreadable"];

fn site(index: &Index, row: &Value) -> Option<Value> {
    let site = index.row(row)?;
    Some(json!({"uri": uri(&site.path)?, "line": site.line, "approximate": site.approximate}))
}

/// The first evidence row of the profile's facts that cites the model file.
fn at_in(index: &Index, record: &Value, p: Option<&str>, model: &str) -> Option<Value> {
    facts(record, p)
        .iter()
        .flat_map(|f| f["evidence"].as_array().into_iter().flatten())
        .filter(|row| row["file"] == model)
        .find_map(|row| site(index, row))
}

fn worst(atlas: &Atlas, id: &str, p: Option<&str>) -> Option<&'static str> {
    atlas
        .findings_for(id)
        .into_iter()
        .filter(|f| finding_in_profile(f, p))
        .filter_map(|f| match f["severity"].as_str() {
            Some("error") => Some((0, "error")),
            Some("warn") => Some((1, "warn")),
            Some("info") => Some((2, "info")),
            _ => None,
        })
        .min()
        .map(|(_, severity)| severity)
}

/// Model file id to the profiles that load it: a launch entity reading it as a robot model,
/// or a urdf fact stated for one profile.
fn loaded(atlas: &Atlas) -> BTreeMap<&str, BTreeSet<&str>> {
    let mut out: BTreeMap<&str, BTreeSet<&str>> = BTreeMap::new();
    for entity in atlas.entities.values() {
        if entity.fields.kind != "launchent" {
            continue;
        }
        for fact in facts(entity, None) {
            for read in fact["values"]["file_reads"]
                .as_array()
                .into_iter()
                .flatten()
            {
                if read["role"] == "robot_model" {
                    out.entry(string(read, "file"))
                        .or_default()
                        .insert(string(fact, "profile"));
                }
            }
        }
    }
    for edge in atlas.edges.values() {
        if edge.fields.common.kind != "tf_parent" {
            continue;
        }
        for fact in facts(edge, None) {
            let model = fact["values"]["model"].as_str();
            if let (Some(model), Some(p)) = (model, fact["profile"].as_str()) {
                if p != "*" {
                    out.entry(model).or_default().insert(p);
                }
            }
        }
    }
    for profiles in out.values_mut() {
        profiles.retain(|p| p.starts_with("profile:"));
    }
    out
}

/// One urdf or xacro model with its links, joints, roots and the TF broadcasts into it.
pub(super) fn robot_model(atlas: &Atlas, index: &Index, params: &Value) -> Value {
    let wanted = profile(params);
    let p = wanted.as_deref();
    let loaded = loaded(atlas);
    let mut models = Vec::new();
    for (id, entity) in &atlas.entities {
        if entity.fields.kind != "file" || !entity["attrs"]["strategy"].is_string() {
            continue;
        }
        let at = atlas
            .entities
            .values()
            .filter(|e| e.fields.kind == "frame")
            .flat_map(|e| facts(e, None))
            .flat_map(|f| f["evidence"].as_array().into_iter().flatten())
            .filter(|row| row["file"] == id.as_str())
            .find_map(|row| site(index, row))
            .map(|mut at| {
                at["line"] = json!(0);
                at["approximate"] = json!(true);
                at
            });
        let attrs = &entity["attrs"];
        let mut row = json!({
            "id": id,
            "label": entity.fields.label,
            "robotName": attrs["robot_name"],
            "strategy": attrs["strategy"],
            "profiles": loaded.get(id.as_str()).cloned().unwrap_or_default(),
        });
        if let Some(missing) = attrs.get("missing_meshes") {
            row["missingMeshes"] = missing.clone();
        }
        if let Some(at) = at {
            row["at"] = at;
        }
        models.push(row);
    }
    let chosen = match params["model"].as_str() {
        Some(model) => models
            .iter()
            .any(|m| m["id"] == model)
            .then(|| model.to_owned()),
        None => {
            let loads = |m: &&Value| {
                m["profiles"]
                    .as_array()
                    .is_some_and(|ps| ps.iter().any(|v| p.is_none_or(|p| v == p)))
            };
            models
                .iter()
                .find(loads)
                .or(models.first())
                .and_then(|m| m["id"].as_str())
                .map(str::to_owned)
        }
    };
    let mut answer = json!({
        "profile": wanted,
        "model": chosen,
        "models": models,
        "links": [],
        "joints": [],
        "roots": [],
        "broadcasts": [],
        "truncated": false,
        "limit": LIMIT,
    });
    let Some(model) = chosen else {
        return answer;
    };
    let mut links = BTreeMap::new();
    for (id, frame) in &atlas.entities {
        let listed = frame["attrs"]["urdf_models"]
            .as_array()
            .is_some_and(|ms| ms.iter().any(|m| m == model.as_str()));
        if frame.fields.kind != "frame" || !listed || !visible(frame, p) {
            continue;
        }
        let mut row = json!({
            "id": id,
            "label": frame.fields.label,
            "layers": frame.fields.layers,
            "findings": findings(atlas, id, p),
        });
        if let Some(severity) = worst(atlas, id, p) {
            row["worst"] = json!(severity);
        }
        if let Some(at) = at_in(index, frame, p, &model) {
            row["at"] = at;
        }
        // Old atlases hold no geometry, which is not the same as a link with no shape.
        if let Some(geometry) = frame["attrs"]["urdf_geometry"].get(model.as_str()) {
            for key in ["visuals", "collisions"] {
                row[key] = geometry.get(key).cloned().unwrap_or_else(|| json!([]));
            }
        }
        links.insert(id.as_str(), row);
    }
    let mut truncated = links.len() > LIMIT;
    let links: BTreeMap<_, _> = links.into_iter().take(LIMIT).collect();
    let mut joints: BTreeMap<String, Value> = BTreeMap::new();
    for (id, edge) in &atlas.edges {
        let fields = &edge.fields;
        if fields.common.kind != "tf_parent"
            || !links.contains_key(fields.src.as_str())
            || !links.contains_key(fields.dst.as_str())
        {
            continue;
        }
        let ours: Vec<_> = facts(edge, p)
            .into_iter()
            .filter(|f| f["values"]["model"] == model.as_str())
            .collect();
        let Some(first) = ours.first() else {
            continue;
        };
        let v = &first["values"];
        let name = string(v, "joint");
        let mut row = json!({
            "id": id,
            "name": name,
            "type": v["joint_type"],
            "parent": fields.dst,
            "child": fields.src,
        });
        for key in JOINT_KEYS {
            if let Some(value) = v.get(key) {
                row[key] = value.clone();
            }
        }
        if let Some(at) = at_in(index, edge, p, &model) {
            row["at"] = at;
        }
        joints.entry(name.to_owned()).or_insert(row);
    }
    let children: BTreeSet<&str> = joints
        .values()
        .filter_map(|j| j["child"].as_str())
        .collect();
    let roots: Vec<&str> = links
        .keys()
        .copied()
        .filter(|id| !children.contains(id))
        .collect();
    let mut broadcasts = Vec::new();
    for (id, edge) in &atlas.edges {
        let fields = &edge.fields;
        if fields.common.kind != "broadcasts_tf" || !visible(edge, p) {
            continue;
        }
        let v = values(edge, p);
        let parent_name = v["parent"].as_str();
        let parent = parent_name
            .map(|name| format!("frame:{}", name.trim_start_matches('/')))
            .filter(|id| atlas.entities.contains_key(id));
        let touches = links.contains_key(fields.dst.as_str())
            || parent.as_deref().is_some_and(|id| links.contains_key(id));
        if !touches {
            continue;
        }
        let mut row = json!({
            "id": id,
            "node": fields.src,
            "nodeLabel": atlas.entities.get(&fields.src).map(|e| &e.fields.label),
            "child": fields.dst,
            "layers": fields.common.layers,
        });
        if let Some(parent) = parent {
            row["parent"] = json!(parent);
        }
        if let Some(name) = parent_name {
            row["parentName"] = json!(name);
        }
        for (key, out) in [("condition", "condition"), ("static", "static")] {
            if let Some(value) = v.get(key).filter(|v| !v.is_null()) {
                row[out] = value.clone();
            }
        }
        if let Some(at) = super::graph::at(index, edge, p) {
            row["at"] = at;
        }
        broadcasts.push(row);
    }
    truncated |= broadcasts.len() > LIMIT;
    broadcasts.truncate(LIMIT);
    answer["links"] = json!(links.into_values().collect::<Vec<_>>());
    answer["joints"] = json!(joints.into_values().collect::<Vec<_>>());
    answer["roots"] = json!(roots);
    answer["broadcasts"] = json!(broadcasts);
    answer["truncated"] = json!(truncated);
    answer
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::Paths;

    fn fixture(params: Value) -> Value {
        let atlas =
            Atlas::parse(include_str!("../../tests/fixtures/ws_static.atlas.json")).unwrap();
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let src = root.join("tests/fixtures/ws_static/src");
        let overrides = BTreeMap::from([("src".to_owned(), src.display().to_string())]);
        let index = Index::new(&atlas, &Paths::new(&root, &atlas, &overrides));
        robot_model(&atlas, &index, &params)
    }

    fn row<'a>(rows: &'a Value, key: &str, value: &str) -> &'a Value {
        rows.as_array()
            .unwrap()
            .iter()
            .find(|r| r[key] == value)
            .unwrap()
    }

    #[test]
    fn the_fixture_robot_states_its_joints_as_the_urdf_writes_them() {
        let model = "file:fx_desc|urdf/fx_robot.urdf";
        let result = fixture(json!({"model": model}));
        assert_eq!(result["model"], model);
        assert_eq!(result["models"].as_array().unwrap().len(), 5);
        assert_eq!(
            result["models"][0]["id"],
            "file:fx_desc|urdf/fx_broken.urdf"
        );
        assert_eq!(
            result["models"][0]["missingMeshes"],
            json!(["package://fx_desc/meshes/fx_absent_body.stl"])
        );
        assert!(result["models"][1]["at"]["uri"]
            .as_str()
            .unwrap()
            .ends_with("fx_desc/urdf/fx_robot.urdf"));
        assert_eq!(result["roots"], json!(["frame:base_footprint"]));
        let swivel = row(&result["joints"], "name", "caster_swivel_joint");
        assert_eq!(swivel["type"], "revolute");
        assert_eq!(swivel["parent"], "frame:base_link");
        assert_eq!(swivel["child"], "frame:caster_swivel_link");
        assert_eq!(swivel["xyz"], json!([0.45, 0.0, 0.0]));
        assert_eq!(swivel["axis"], json!([0.0, 0.0, 1.0]));
        assert_eq!(swivel["limit"]["effort"], 40.0);
        assert_eq!(swivel["at"]["line"], 63);
        let base = row(&result["joints"], "name", "base_joint");
        assert!(base.get("axis").is_none() && base.get("limit").is_none());
        let tow = row(&result["links"], "id", "frame:tow_link");
        assert_eq!(tow["layers"], json!(["declared"]));
        assert!(tow["at"]["uri"]
            .as_str()
            .unwrap()
            .ends_with("fx_desc/urdf/fx_robot.urdf"));
        assert_eq!(tow["at"]["approximate"], false);
        assert_eq!(tow["visuals"], json!([]));
        let base = row(&result["links"], "id", "frame:base_link");
        let mesh = &base["visuals"][0]["geometry"];
        assert_eq!(mesh["filename"], "package://fx_desc/meshes/fx_body.stl");
        assert_eq!(mesh["file"], "file:fx_desc|meshes/fx_body.stl");
        assert_eq!(
            base["collisions"][0]["geometry"]["size"],
            json!([0.6, 0.45, 0.2])
        );
        let sensor = row(&result["links"], "id", "frame:sensor_link");
        assert!(sensor["visuals"][0].get("origin").is_none());
        assert_eq!(
            sensor["visuals"][0]["material"]["rgba"],
            json!([0.92, 0.4, 0.0, 1.0])
        );
        assert!(result["links"]
            .as_array()
            .unwrap()
            .iter()
            .all(|l| l["id"] != "frame:trailer_link"));
        assert_eq!(result, fixture(json!({"model": model})));
        assert_eq!(
            fixture(json!({}))["model"],
            "file:fx_desc|urdf/fx_broken.urdf"
        );
        assert_eq!(fixture(json!({"model": "file:nope"}))["model"], Value::Null);
    }

    const ATLAS: &str = r#"{"schema_version":"1.13","entities":{
        "file:bot|a.urdf":{"kind":"file","label":"a.urdf","attrs":{"strategy":"urdf","robot_name":"bot"}},
        "file:bot|b.urdf":{"kind":"file","label":"b.urdf","attrs":{"strategy":"urdf","robot_name":"bot"}},
        "file:bot|launch.py":{"kind":"file","label":"launch.py"},
        "frame:base":{"kind":"frame","label":"base","layers":["declared","coded"],"attrs":{"urdf_models":["file:bot|a.urdf","file:bot|b.urdf"]}},
        "frame:arm":{"kind":"frame","label":"arm","layers":["declared"],"attrs":{"urdf_models":["file:bot|b.urdf"]}},
        "frame:odom":{"kind":"frame","label":"odom","layers":["coded"]},
        "frame:map":{"kind":"frame","label":"map","layers":["coded"]},
        "node:/odometry":{"kind":"node","label":"/odometry"},
        "launchent:bot|launch.py|Node(rsp)":{"kind":"launchent","facts":[
          {"layer":"declared","profile":"profile:real","values":{"file_reads":[{"file":"file:bot|b.urdf","role":"robot_model"}]}}]}
      },"edges":{
        "j":{"kind":"tf_parent","src":"frame:arm","dst":"frame:base","facts":[{"layer":"declared","profile":"*",
          "values":{"model":"file:bot|b.urdf","joint":"arm_joint","joint_type":"revolute","axis":[0,0,1]}}]},
        "c":{"kind":"tf_parent","src":"frame:base","dst":"frame:odom","facts":[{"layer":"coded","profile":"profile:real",
          "values":{"node":"node:/odometry","source":"coded_broadcast"}}]},
        "b1":{"kind":"broadcasts_tf","src":"node:/odometry","dst":"frame:base","layers":["coded"],"facts":[{"layer":"coded","profile":"profile:real",
          "values":{"parent":"odom","child":"base","static":false,"condition":"publish_tf"}}]},
        "b2":{"kind":"broadcasts_tf","src":"node:/odometry","dst":"frame:odom","layers":["coded"],"facts":[{"layer":"coded","profile":"profile:real",
          "values":{"parent":"map","child":"odom"}}]}
      },
      "findings":[{"id":"f1","severity":"warn","about":{"entity":"frame:base"}},
        {"id":"f2","severity":"error","profiles":["profile:sim"],"about":{"entity":"frame:base"}}]}"#;

    fn run(params: Value) -> Value {
        robot_model(&Atlas::parse(ATLAS).unwrap(), &Index::default(), &params)
    }

    #[test]
    fn the_loaded_model_is_the_default_and_absent_geometry_stays_absent() {
        let result = run(json!({"profile": "real"}));
        assert_eq!(result["profile"], "profile:real");
        assert_eq!(result["model"], "file:bot|b.urdf");
        assert_eq!(result["models"][0]["profiles"], json!([]));
        assert_eq!(result["models"][1]["profiles"], json!(["profile:real"]));
        assert_eq!(result["roots"], json!(["frame:base"]));
        let joints = result["joints"].as_array().unwrap();
        assert_eq!(joints.len(), 1, "a coded tf_parent is no joint of a model");
        assert_eq!(joints[0]["axis"], json!([0, 0, 1]));
        assert!(joints[0].get("xyz").is_none() && joints[0].get("rpy").is_none());
        let base = row(&result["links"], "id", "frame:base");
        assert_eq!(base["findings"], 1);
        assert!(base.get("visuals").is_none() && base.get("collisions").is_none());
        assert_eq!(base["worst"], "warn");
        assert_eq!(run(json!({}))["links"][1]["worst"], "error");
        let broadcasts = result["broadcasts"].as_array().unwrap();
        assert_eq!(broadcasts.len(), 1);
        assert_eq!(broadcasts[0]["node"], "node:/odometry");
        assert_eq!(broadcasts[0]["parent"], "frame:odom");
        assert_eq!(broadcasts[0]["child"], "frame:base");
        assert_eq!(broadcasts[0]["condition"], "publish_tf");
        assert_eq!(
            run(json!({"profile": "sim", "model": "file:bot|b.urdf"}))["broadcasts"],
            json!([])
        );
    }

    #[test]
    fn a_large_model_is_cut_at_the_limit() {
        let mut entities = serde_json::Map::new();
        entities.insert(
            "file:m|r.urdf".into(),
            json!({"kind":"file","attrs":{"strategy":"urdf"}}),
        );
        for i in 0..LIMIT + 5 {
            entities.insert(
                format!("frame:l{i:04}"),
                json!({"kind":"frame","attrs":{"urdf_models":["file:m|r.urdf"]}}),
            );
        }
        let text = json!({"schema_version":"1.13","entities":entities}).to_string();
        let result = robot_model(&Atlas::parse(&text).unwrap(), &Index::default(), &json!({}));
        assert_eq!(result["truncated"], true);
        assert_eq!(result["links"].as_array().unwrap().len(), LIMIT);
    }
}
