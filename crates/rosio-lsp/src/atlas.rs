//! The additive schema view. Entity JSON is retained for custom requests.
use serde::{de::DeserializeOwned, Deserialize, Deserializer, Serialize, Serializer};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

/// A portable extraction root.
#[derive(Clone, Debug, Default, Deserialize, Serialize)]
#[serde(default)]
pub struct Root {
    pub kind: String,
    pub label: String,
    pub hash: String,
    pub path: Option<String>,
}

/// Machine independent extraction metadata.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Generated {
    pub roots: Vec<Root>,
    pub packages: BTreeMap<String, BTreeMap<String, String>>,
}

/// Inline source provenance.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(default)]
pub struct Evidence {
    pub file: String,
    pub locator: Option<String>,
    pub line_hint: Option<u32>,
    pub root: String,
}

/// Typed fields alongside the unchanged JSON needed by custom requests.
#[derive(Clone, Debug)]
pub struct Record<T> {
    pub fields: T,
    pub raw: Value,
}

impl<'de, T: DeserializeOwned> Deserialize<'de> for Record<T> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let raw = Value::deserialize(deserializer)?;
        if !raw.is_object() {
            return Err(serde::de::Error::custom("Atlas record must be an object"));
        }
        let fields = T::deserialize(&raw).map_err(serde::de::Error::custom)?;
        Ok(Self { fields, raw })
    }
}

impl<T> Serialize for Record<T> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.raw.serialize(serializer)
    }
}

impl<T> std::ops::Deref for Record<T> {
    type Target = Value;
    fn deref(&self) -> &Self::Target {
        &self.raw
    }
}

/// Layer claims retain their open values and inline provenance.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Fact {
    pub layer: String,
    pub profile: String,
    pub confidence: String,
    pub evidence: Vec<Evidence>,
    pub values: Value,
}

/// Common entity fields, with open vocabularies for additive schema versions.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Entity {
    pub id: String,
    pub kind: String,
    pub label: String,
    pub layers: Vec<String>,
    pub profiles: Vec<String>,
    pub agreement: String,
    pub conflicts: Vec<String>,
    pub facts: Vec<Fact>,
    pub attrs: Value,
    pub effective: Value,
    pub candidates: Vec<Value>,
}

/// Edge identity is read from fields rather than its handle.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Edge {
    #[serde(flatten)]
    pub common: Entity,
    pub src: String,
    pub dst: String,
    pub key: String,
}

/// A finding and its source locations.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Finding {
    pub id: String,
    pub rule: String,
    pub severity: String,
    pub title: String,
    pub say: String,
    pub fix: String,
    pub about: Value,
    #[serde(rename = "where")]
    pub locations: Vec<Evidence>,
    pub edits: Vec<Edit>,
}

/// One exact replacement a finding's fix makes on the line it cites.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Edit {
    pub file: String,
    pub root: String,
    pub line_hint: Option<u32>,
    pub find: String,
    pub replace: String,
}

/// Whether a layer was collected.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Layer {
    pub present: bool,
}

/// The atlas fields used by the editor.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Atlas {
    pub schema_version: String,
    pub generated: Generated,
    pub entities: BTreeMap<String, Record<Entity>>,
    pub edges: BTreeMap<String, Record<Edge>>,
    pub findings: Vec<Record<Finding>>,
    pub unresolved: Vec<Value>,
    pub counters: Value,
    pub layers: BTreeMap<String, Layer>,
    pub indexes: Value,
    /// Edge ids by each end, built by `parse`.
    #[serde(skip)]
    ends: BTreeMap<String, BTreeSet<String>>,
    /// Finding positions by the entity or edge they are about, built by `parse`.
    #[serde(skip)]
    subjects: BTreeMap<String, Vec<usize>>,
}

impl Atlas {
    /// Read an additive version of schema 1.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut atlas: Self =
            serde_json::from_str(text).map_err(|e| format!("Invalid atlas: {e}"))?;
        let Some((major, minor)) = atlas.schema_version.split_once('.') else {
            return Err("Invalid atlas schema_version, expected major.minor".into());
        };
        if major.parse::<u32>().is_err() || minor.parse::<u32>().is_err() {
            return Err("Invalid atlas schema_version, expected major.minor".into());
        }
        if major != "1" {
            return Err(format!(
                "Unsupported atlas schema {}, expected major 1",
                atlas.schema_version
            ));
        }
        for (id, edge) in &atlas.edges {
            for end in [&edge.fields.src, &edge.fields.dst] {
                atlas
                    .ends
                    .entry(end.clone())
                    .or_default()
                    .insert(id.clone());
            }
        }
        let mut subjects: BTreeMap<String, Vec<usize>> = BTreeMap::new();
        for (position, finding) in atlas.findings.iter().enumerate() {
            let about = &finding["about"];
            let mut about_ids = vec![string(about, "entity")];
            let edge = &about["edge"];
            if edge.is_object() {
                about_ids.extend(
                    atlas
                        .edges_touching(edge["src"].as_str().unwrap_or(""))
                        .filter(|(_, row)| {
                            ["src", "dst", "kind", "key"]
                                .iter()
                                .all(|k| edge[k] == row[k])
                        })
                        .map(|(id, _)| id.as_str()),
                );
            }
            for id in about_ids {
                let rows = subjects.entry(id.to_owned()).or_default();
                if rows.last() != Some(&position) {
                    rows.push(position);
                }
            }
        }
        atlas.subjects = subjects;
        Ok(atlas)
    }

    /// Findings directly about an entity or edge, in atlas order.
    pub fn findings_for<'a>(&'a self, id: &str) -> Vec<&'a Value> {
        self.subjects
            .get(id)
            .into_iter()
            .flatten()
            .map(|position| &self.findings[*position].raw)
            .collect()
    }

    /// The edges with this id at either end, in edge id order.
    pub fn edges_touching<'a>(
        &'a self,
        id: &str,
    ) -> impl Iterator<Item = (&'a String, &'a Record<Edge>)> {
        self.ends
            .get(id)
            .into_iter()
            .flatten()
            .filter_map(|edge| self.edges.get_key_value(edge))
    }
}

/// Read an optional string without manufacturing a value.
pub fn string<'a>(value: &'a Value, key: &str) -> &'a str {
    value.get(key).and_then(Value::as_str).unwrap_or("")
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    #[test]
    fn additive_schema() {
        assert!(Atlas::parse(r#"{"schema_version":"1.999","future":true}"#).is_ok());
        assert!(Atlas::parse(r#"{"schema_version":"2.0"}"#)
            .unwrap_err()
            .contains("major 1"));
        assert!(Atlas::parse("{}").is_err());
        assert!(Atlas::parse("{").is_err());
    }
    #[test]
    fn record_types_and_unknown_fields() {
        let text = r#"{"schema_version":"1.2","entities":{"topic:/a":{"id":"topic:/a","kind":"topic","label":"a","new_field":{"x":1}}}}"#;
        let atlas = Atlas::parse(text).unwrap();
        assert_eq!(atlas.entities["topic:/a"].fields.kind, "topic");
        assert_eq!(atlas.entities["topic:/a"].raw["new_field"]["x"], 1);
        assert!(Atlas::parse(r#"{"schema_version":"1.1","entities":{"topic:/a":42}}"#).is_err());
        assert!(
            Atlas::parse(r#"{"schema_version":"1.1","entities":{"topic:/a":{"layers":42}}}"#)
                .is_err()
        );
    }
    #[test]
    fn findings_and_edges_are_indexed_by_subject() {
        let atlas = Atlas::parse(
            r#"{"schema_version":"1.9","edges":{
            "e2":{"kind":"publishes","src":"node:/a","dst":"topic:/t","key":"k"},
            "e1":{"kind":"subscribes","src":"node:/b","dst":"topic:/t","key":"k"},
            "e3":{"kind":"loop","src":"node:/a","dst":"node:/a"}},
            "findings":[
            {"id":"f1","about":{"edge":{"src":"node:/a","kind":"publishes","dst":"topic:/t","key":"k"}}},
            {"id":"f2","about":{"entity":"topic:/t"}},
            {"id":"f3","about":{"entity":"e2","edge":{"src":"node:/a","kind":"publishes","dst":"topic:/t","key":"k"}}},
            {"id":"f4","about":{"edge":{"src":"node:/a","kind":"publishes","dst":"topic:/t"}}},
            {"id":"f5","about":{"edge":{"src":"node:/a","kind":"loop","dst":"node:/a"}}},
            {"id":"f6"}]}"#,
        )
        .unwrap();
        let ids = |id: &str| -> Vec<&str> {
            atlas
                .findings_for(id)
                .iter()
                .map(|f| f["id"].as_str().unwrap())
                .collect()
        };
        assert_eq!(ids("e2"), ["f1", "f3"]);
        assert_eq!(ids("topic:/t"), ["f2"]);
        assert_eq!(ids("e3"), ["f5"]);
        assert_eq!(ids(""), ["f1", "f4", "f5", "f6"]);
        assert!(ids("e1").is_empty());
        let touching = |id: &str| -> Vec<&str> {
            atlas
                .edges_touching(id)
                .map(|(id, _)| id.as_str())
                .collect()
        };
        assert_eq!(touching("topic:/t"), ["e1", "e2"]);
        assert_eq!(touching("node:/a"), ["e2", "e3"]);
        assert!(touching("node:/c").is_empty());
    }
    #[test]
    fn fixture() {
        let atlas = Atlas::parse(include_str!("../tests/fixtures/ws_static.atlas.json")).unwrap();
        assert_eq!(atlas.findings.len(), 3);
        assert!(!atlas.entities.is_empty());
    }
}
