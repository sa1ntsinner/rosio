use super::trees::{facts, profile};
use crate::atlas::Atlas;
use crate::index::Index;
use crate::paths::uri;
use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, BTreeSet};

const LINK_KINDS: [&str; 4] = ["publishes", "subscribes", "serves", "calls"];
const CHANNEL_KINDS: [&str; 3] = ["topic", "service", "action"];
const QOS_KEYS: [&str; 5] = ["reliability", "durability", "history", "depth", "preset"];
const QUIET_NAMES: &[&str] = &[
    "/diagnostics",
    "/parameter_events",
    "/rosout",
    "/tf",
    "/tf_static",
];
const QUIET_SUFFIXES: &[&str] = &[
    "/describe_parameters",
    "/get_parameter_types",
    "/get_parameters",
    "/get_type_description",
    "/list_parameters",
    "/set_parameters",
    "/set_parameters_atomically",
];
const LIFECYCLE_TAILS: &[&str] = &[
    "change_state",
    "get_available_states",
    "get_available_transitions",
    "get_state",
    "get_transition_graph",
    "transition_event",
];
/// Most nodes and channels one answer carries.
const LIMIT: usize = 400;

fn in_profile(record: &Value, p: Option<&str>) -> bool {
    p.is_none_or(|p| {
        record["profiles"]
            .as_array()
            .is_some_and(|ps| ps.iter().any(|v| v == p || v == "*"))
            || !facts(record, Some(p)).is_empty()
    })
}

pub(super) fn findings(atlas: &Atlas, id: &str, p: Option<&str>) -> usize {
    atlas
        .findings_for(id)
        .iter()
        .filter(|f| finding_in_profile(f, p))
        .count()
}

pub(super) fn finding_in_profile(finding: &Value, p: Option<&str>) -> bool {
    p.is_none_or(|p| {
        finding["profiles"]
            .as_array()
            .is_none_or(|ps| ps.is_empty() || ps.iter().any(|v| v == p || v == "*"))
    })
}

/// The worst severity of the findings about this endpoint: about the edge itself, or about its
/// channel and citing the line the endpoint is made on.
fn problem(
    atlas: &Atlas,
    index: &Index,
    id: &str,
    edge: &Value,
    p: Option<&str>,
) -> Option<&'static str> {
    let sites: Vec<_> = facts(edge, p)
        .iter()
        .flat_map(|f| f["evidence"].as_array().into_iter().flatten())
        .filter_map(|row| index.row(row))
        .collect();
    let cites_this = |finding: &Value| {
        finding["where"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|row| index.row(row))
            .any(|site| {
                sites
                    .iter()
                    .any(|own| own.path == site.path && own.line == site.line)
            })
    };
    let about_edge = atlas.findings_for(id);
    let about_channel = atlas.findings_for(edge["dst"].as_str().unwrap_or_default());
    about_edge
        .into_iter()
        .chain(about_channel.into_iter().filter(|f| cites_this(f)))
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

/// The first evidence row of the profile's facts that maps to a local file.
pub(super) fn at(index: &Index, record: &Value, p: Option<&str>) -> Option<Value> {
    facts(record, p)
        .iter()
        .flat_map(|f| f["evidence"].as_array().into_iter().flatten())
        .find_map(|row| {
            let site = index.row(row)?;
            let uri = uri(&site.path)?;
            Some(json!({"uri": uri, "line": site.line, "approximate": site.approximate}))
        })
}

/// The precomputed effective block of one profile, or of the first profile without one.
fn effective<'a>(record: &'a Value, p: Option<&str>) -> Option<&'a Map<String, Value>> {
    let by_profile = record["effective"].as_object()?;
    match p {
        Some(p) => by_profile.get(p).or_else(|| by_profile.get("*")),
        None => by_profile.values().next(),
    }?
    .as_object()
}

/// The message type of a channel when its `uses_type` edges in the profile name exactly one.
fn channel_type(atlas: &Atlas, id: &str, p: Option<&str>) -> Option<String> {
    let types: BTreeSet<String> = atlas
        .edges_touching(id)
        .map(|(_, e)| e)
        .filter(|e| e.fields.common.kind == "uses_type" && e.fields.src == id && in_profile(e, p))
        .filter_map(|e| crate::ids::parse(&e.fields.dst).ok())
        .map(|t| t.segments.join("/"))
        .collect();
    match types.len() {
        1 => types.into_iter().next(),
        _ => None,
    }
}

fn library_end(atlas: &Atlas, edge: &Value, p: Option<&str>) -> bool {
    let Some(channel) = edge["dst"]
        .as_str()
        .and_then(|id| crate::ids::parse(id).ok())
    else {
        return false;
    };
    let name = &channel.segments[0];
    let quiet = QUIET_NAMES.contains(&name.as_str())
        || QUIET_SUFFIXES.iter().any(|tail| name.ends_with(tail));
    let wrapper = edge["attrs"]["via_known_wrapper"] == true
        || facts(edge, p)
            .iter()
            .any(|f| f["values"]["via_known_wrapper"] == true);
    if (wrapper && quiet) || name.contains("/_action/") {
        return true;
    }
    if !matches!(edge["kind"].as_str(), Some("serves" | "publishes")) {
        return false;
    }
    if quiet {
        return true;
    }
    let Some((parent, tail)) = name.rsplit_once('/') else {
        return false;
    };
    let Some(owner_id) = edge["src"].as_str() else {
        return false;
    };
    let Some(owner) = atlas.entities.get(owner_id) else {
        return false;
    };
    LIFECYCLE_TAILS.contains(&tail)
        && crate::ids::parse(owner_id).is_ok_and(|id| !id.synthetic && id.segments[0] == parent)
        && facts(owner, None).iter().any(|f| {
            f["values"]["form"] == "lifecycle_node"
                || f["values"]["lifecycle_state"]
                    .as_str()
                    .is_some_and(|state| !state.is_empty())
        })
}

/// Nodes, the topics, services and actions they use, and the endpoint edges between them.
pub(super) fn graph(atlas: &Atlas, index: &Index, params: &Value) -> Value {
    let wanted = profile(params);
    let p = wanted.as_deref();
    let mut entities: BTreeMap<&str, Value> = BTreeMap::new();
    for (id, entity) in &atlas.entities {
        let kind = entity.fields.kind.as_str();
        if (kind != "node" && !CHANNEL_KINDS.contains(&kind)) || !in_profile(entity, p) {
            continue;
        }
        let mut row = json!({
            "id": id,
            "label": entity.fields.label,
            "kind": kind,
            "layers": entity.fields.layers,
            "findings": findings(atlas, id, p),
        });
        if kind == "node" {
            row["nameSource"] = entity["name_source"].clone();
        } else if let Some(name) = channel_type(atlas, id, p) {
            row["type"] = json!(name);
        }
        if let Some(at) = at(index, entity, p) {
            row["at"] = at;
        }
        entities.insert(id, row);
    }
    let edges: Vec<_> = atlas
        .edges
        .iter()
        .filter(|(_, e)| {
            LINK_KINDS.contains(&e.fields.common.kind.as_str())
                && in_profile(e, p)
                && entities.contains_key(e.fields.src.as_str())
                && entities.contains_key(e.fields.dst.as_str())
        })
        .collect();
    let focus = params["focus"].as_str();
    if let Some(focus) = focus {
        let mut reached = BTreeSet::from([focus]);
        let mut frontier = reached.clone();
        for _ in 0..params["depth"].as_u64().unwrap_or(1) {
            if frontier.is_empty() {
                break;
            }
            let mut next = BTreeSet::new();
            for (_, e) in &edges {
                let (src, dst) = (e.fields.src.as_str(), e.fields.dst.as_str());
                for (from, to) in [(src, dst), (dst, src)] {
                    if frontier.contains(from) && reached.insert(to) {
                        next.insert(to);
                    }
                }
            }
            frontier = next;
        }
        entities.retain(|id, _| reached.contains(id));
    }
    let library_only: BTreeSet<_> = edges
        .iter()
        .filter(|(_, e)| library_end(atlas, e, p))
        .map(|(_, e)| e.fields.dst.as_str())
        .collect();
    let edges: Vec<_> = edges
        .into_iter()
        .filter(|(_, e)| params["wrappers"] == true || !library_end(atlas, e, p))
        .collect();
    let shown: BTreeSet<_> = edges.iter().map(|(_, e)| e.fields.dst.as_str()).collect();
    entities.retain(|id, _| !library_only.contains(id) || shown.contains(id));
    let truncated = entities.len() > LIMIT;
    if truncated {
        let mut order: Vec<_> = entities
            .iter()
            .map(|(id, row)| (Some(*id) != focus, row["findings"] == 0, *id))
            .collect();
        order.sort();
        let keep: BTreeSet<&str> = order.into_iter().take(LIMIT).map(|(_, _, id)| id).collect();
        entities.retain(|id, _| keep.contains(id));
    }
    let mut links = Vec::new();
    for (id, edge) in edges {
        let fields = &edge.fields;
        if !entities.contains_key(fields.src.as_str())
            || !entities.contains_key(fields.dst.as_str())
        {
            continue;
        }
        let stated = effective(edge, p);
        let get = |key: &str| {
            stated
                .and_then(|m| m.get(key))
                .map(|v| &v["value"])
                .filter(|v| !v.is_null())
        };
        let wrapper = library_end(atlas, edge, p);
        let mut row = json!({
            "id": id,
            "src": fields.src,
            "dst": fields.dst,
            "kind": fields.common.kind,
            "layers": fields.common.layers,
            "agreement": fields.common.agreement,
            "conflicts": fields.common.conflicts,
            "wrapper": wrapper,
        });
        let qos: Map<String, Value> = QOS_KEYS
            .iter()
            .filter_map(|k| Some(((*k).to_owned(), get(&format!("qos.{k}"))?.clone())))
            .collect();
        if !qos.is_empty() {
            row["qos"] = Value::Object(qos);
        }
        if let Some(raw) = get("raw_name") {
            row["rawName"] = raw.clone();
        }
        if let Some(at) = at(index, edge, p) {
            row["at"] = at;
        }
        if let Some(severity) = problem(atlas, index, id, edge, p) {
            row["problem"] = json!(severity);
        }
        links.push(row);
    }
    let (nodes, channels): (Vec<Value>, Vec<Value>) = entities
        .into_values()
        .partition(|row| row["kind"] == "node");
    json!({
        "profile": wanted,
        "nodes": nodes,
        "channels": channels,
        "links": links,
        "truncated": truncated,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::Paths;

    const ATLAS: &str = r#"{"schema_version":"1.9","entities":{
        "node:/scan_filter":{"kind":"node","label":"/scan_filter","name_source":"literal",
          "layers":["declared","coded"],"profiles":["profile:rover"]},
        "node:/safety_stop":{"kind":"node","label":"/safety_stop","name_source":"literal",
          "layers":["coded"],"profiles":["profile:rover"]},
        "node:~sim_pkg|sim_node|/|75ed3551":{"kind":"node","label":"sim_node","name_source":"runtime_randomised",
          "layers":["declared"],"profiles":["profile:sim"]},
        "topic:/scan_filtered":{"kind":"topic","label":"/scan_filtered","layers":["coded"],"profiles":["profile:rover"]},
        "topic:/cmd_vel":{"kind":"topic","label":"/cmd_vel","layers":["coded"],"profiles":["profile:rover","profile:sim"]},
        "topic:/diagnostics":{"kind":"topic","label":"/diagnostics","layers":["observed"],"profiles":["profile:rover"]},
        "type:sensor_msgs|msg|LaserScan":{"kind":"type","label":"LaserScan"}
      },"edges":{
        "edge:publishes|node:/scan_filter|topic:/scan_filtered|a":{"kind":"publishes",
          "src":"node:/scan_filter","dst":"topic:/scan_filtered","key":"a","layers":["coded"],
          "agreement":"single_layer","conflicts":[],"profiles":["profile:rover"],
          "facts":[{"layer":"coded","profile":"profile:rover","values":{"raw_name":"scan_filtered"},
            "evidence":[{"file":"file:@repo|examples/rover_ws/src/rover_perception/src/scan_filter.cpp","line_hint":12}]}],
          "effective":{"profile:rover":{"qos.depth":{"value":10},"qos.reliability":{"value":"reliable"},
            "qos.durability":{"value":null},"raw_name":{"value":"scan_filtered"}}}},
        "edge:subscribes|node:/safety_stop|topic:/scan_filtered|b":{"kind":"subscribes",
          "src":"node:/safety_stop","dst":"topic:/scan_filtered","key":"b","layers":["coded","observed"],
          "agreement":"differ","conflicts":["qos.reliability"],"profiles":["profile:rover"]},
        "edge:publishes|node:/safety_stop|topic:/cmd_vel|c":{"kind":"publishes",
          "src":"node:/safety_stop","dst":"topic:/cmd_vel","key":"c","profiles":["profile:rover"]},
        "edge:publishes|node:/safety_stop|topic:/diagnostics|d":{"kind":"publishes",
          "src":"node:/safety_stop","dst":"topic:/diagnostics","key":"d","profiles":["profile:rover"],
          "facts":[{"layer":"observed","profile":"profile:rover","values":{"via_known_wrapper":true}}]},
        "edge:subscribes|node:~sim_pkg%7Csim_node%7C/%7C75ed3551|topic:/cmd_vel|e":{"kind":"subscribes",
          "src":"node:~sim_pkg|sim_node|/|75ed3551","dst":"topic:/cmd_vel","key":"e","profiles":["profile:sim"]},
        "edge:uses_type|topic:/scan_filtered|type:sensor_msgs%7Cmsg%7CLaserScan|-":{"kind":"uses_type",
          "src":"topic:/scan_filtered","dst":"type:sensor_msgs|msg|LaserScan","key":"-","profiles":["profile:rover"]}
      },
      "findings":[{"id":"find:qos.differ|x|-","profiles":["profile:rover"],"about":{"entity":"topic:/scan_filtered"}},
        {"id":"find:rate.below_declared|y|-","severity":"warn","profiles":["profile:rover"],
         "about":{"edge":{"src":"node:/safety_stop","kind":"publishes","dst":"topic:/cmd_vel","key":"c"}}}]}"#;

    fn run(params: Value) -> Value {
        let atlas = Atlas::parse(ATLAS).unwrap();
        let workspace = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let index = Index::new(&atlas, &Paths::new(&workspace, &atlas, &BTreeMap::new()));
        graph(&atlas, &index, &params)
    }

    #[test]
    fn a_link_carries_the_worst_finding_about_it_and_no_other() {
        let rover = run(json!({"profile": "rover"}));
        let link = |id: &str| {
            rover["links"]
                .as_array()
                .unwrap()
                .iter()
                .find(|l| l["id"] == id)
                .unwrap()
                .clone()
        };
        assert_eq!(
            link("edge:publishes|node:/safety_stop|topic:/cmd_vel|c")["problem"],
            "warn"
        );
        let publish = link("edge:publishes|node:/scan_filter|topic:/scan_filtered|a");
        assert!(
            publish.get("problem").is_none(),
            "a channel finding that cites no line marks no link"
        );
    }

    fn ids(rows: &Value) -> Vec<&str> {
        rows.as_array()
            .unwrap()
            .iter()
            .map(|r| r["id"].as_str().unwrap())
            .collect()
    }

    #[test]
    fn a_profile_keeps_its_own_nodes_channels_and_links() {
        let rover = run(json!({"profile": "rover"}));
        assert_eq!(rover["profile"], "profile:rover");
        assert_eq!(
            ids(&rover["nodes"]),
            ["node:/safety_stop", "node:/scan_filter"]
        );
        assert_eq!(rover["channels"].as_array().unwrap().len(), 2);
        assert_eq!(rover["links"].as_array().unwrap().len(), 3);
        assert_eq!(rover["truncated"], false);
        let scan = &rover["channels"][1];
        assert_eq!(scan["type"], "sensor_msgs/msg/LaserScan");
        assert_eq!(scan["findings"], 1);
        let sim = run(json!({"profile": "profile:sim"}));
        assert_eq!(sim["nodes"][0]["id"], "node:~sim_pkg|sim_node|/|75ed3551");
        assert_eq!(sim["nodes"][0]["label"], "sim_node");
        assert_eq!(sim["nodes"][0]["nameSource"], "runtime_randomised");
        assert_eq!(ids(&sim["channels"]), ["topic:/cmd_vel"]);
        assert_eq!(sim["links"][0]["kind"], "subscribes");
        assert_eq!(rover, run(json!({"profile": "profile:rover"})));
    }

    #[test]
    fn focus_returns_the_neighbourhood_within_depth_hops() {
        let one = run(json!({"profile": "rover", "focus": "node:/scan_filter"}));
        assert_eq!(ids(&one["nodes"]), ["node:/scan_filter"]);
        assert_eq!(ids(&one["channels"]), ["topic:/scan_filtered"]);
        assert_eq!(one["links"].as_array().unwrap().len(), 1);
        let two = run(json!({"profile": "rover", "focus": "node:/scan_filter", "depth": 2}));
        assert_eq!(
            ids(&two["nodes"]),
            ["node:/safety_stop", "node:/scan_filter"]
        );
        assert_eq!(two["links"].as_array().unwrap().len(), 2);
        let missing = run(json!({"focus": "node:/nobody"}));
        assert_eq!(missing["nodes"], json!([]));
        assert_eq!(missing["links"], json!([]));
    }

    #[test]
    fn links_state_qos_wrapper_and_site_only_when_the_atlas_does() {
        let hidden = run(json!({"profile": "rover"}));
        assert!(!ids(&hidden["channels"]).contains(&"topic:/diagnostics"));
        let result = run(json!({"profile": "rover", "wrappers": true}));
        let links = result["links"].as_array().unwrap();
        let diagnostics = &links[1];
        assert_eq!(diagnostics["dst"], "topic:/diagnostics");
        assert_eq!(diagnostics["wrapper"], true);
        assert!(diagnostics.get("qos").is_none());
        let publish = &links[2];
        assert_eq!(publish["src"], "node:/scan_filter");
        assert_eq!(publish["dst"], "topic:/scan_filtered");
        assert_eq!(publish["wrapper"], false);
        assert_eq!(
            publish["qos"],
            json!({"depth": 10, "reliability": "reliable"})
        );
        assert_eq!(publish["rawName"], "scan_filtered");
        assert!(publish["at"]["uri"]
            .as_str()
            .unwrap()
            .ends_with("rover_perception/src/scan_filter.cpp"));
        let subscribe = &links[3];
        assert_eq!(subscribe["kind"], "subscribes");
        assert_eq!(subscribe["src"], "node:/safety_stop");
        assert_eq!(subscribe["agreement"], "differ");
        assert_eq!(subscribe["conflicts"], json!(["qos.reliability"]));
        assert!(subscribe.get("qos").is_none());

        let mut fixture: Value = serde_json::from_str(ATLAS).unwrap();
        fixture["entities"]["node:/safety_stop"]["facts"] =
            json!([{"values":{"form":"lifecycle_node"}}]);
        let atlas = Atlas::parse(&fixture.to_string()).unwrap();
        for (kind, name, wrapper, hidden) in [
            ("publishes", "/dock/_action/feedback", false, true),
            ("calls", "/dock/_action/send_goal", false, true),
            ("serves", "/safety_stop/get_parameters", false, true),
            ("calls", "/safety_stop/get_parameters", false, false),
            ("serves", "/safety_stop/get_state", false, true),
            ("serves", "/scan_filter/get_state", false, false),
            ("publishes", "/parameter_events", false, true),
            ("publishes", "/rosout", false, true),
            ("subscribes", "/clock", false, false),
            ("subscribes", "/tf", true, true),
            ("subscribes", "/scan", true, false),
        ] {
            let edge = json!({"kind":kind,"src":"node:/safety_stop",
                "dst":crate::ids::build("topic", &[name]),
                "facts":[{"profile":"profile:rover","values":{"via_known_wrapper":wrapper}}]});
            assert_eq!(
                library_end(&atlas, &edge, Some("profile:rover")),
                hidden,
                "{name}"
            );
        }
    }

    #[test]
    fn a_large_graph_is_cut_at_the_limit_keeping_findings_first() {
        let mut entities = Map::new();
        for i in 0..LIMIT + 10 {
            entities.insert(
                format!("topic:/t{i:04}"),
                json!({"kind":"topic","label":format!("/t{i:04}")}),
            );
        }
        let text = json!({"schema_version":"1.9","entities":entities,
            "findings":[{"id":"f","about":{"entity":"topic:/t0409"}}]})
        .to_string();
        let atlas = Atlas::parse(&text).unwrap();
        let result = graph(&atlas, &Index::default(), &json!({}));
        assert_eq!(result["truncated"], true);
        let kept = ids(&result["channels"]);
        assert_eq!(kept.len(), LIMIT);
        assert!(kept.contains(&"topic:/t0409"));
        assert!(!kept.contains(&"topic:/t0408"));
        assert_eq!(kept[0], "topic:/t0000");
    }
}
