//! One source index per atlas load, with cached file contents.
use crate::atlas::{string, Atlas, Evidence};
use crate::paths::{normalize, uri, Paths};
use lsp_types::{Location, Position, Range, Url};
use serde::Deserialize;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// One owner's source site.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Site {
    pub file_id: String,
    pub line: u32,
    pub approximate: bool,
    pub path: PathBuf,
    pub layer: String,
}

impl Site {
    /// The full line range, measured in UTF-16 code units.
    pub fn location(&self, index: &Index) -> Option<Location> {
        Some(Location {
            uri: uri(&self.path)?,
            range: Range::new(
                Position::new(self.line, 0),
                Position::new(self.line, index.line_length(&self.path, self.line)),
            ),
        })
    }
}

/// Owner lookup and cached source text.
#[derive(Default)]
pub struct Index {
    pub owners: BTreeMap<String, Vec<Site>>,
    files: BTreeMap<PathBuf, Option<String>>,
    rows: BTreeMap<Evidence, Site>,
    by_line: BTreeMap<(PathBuf, u32), BTreeSet<String>>,
    file_ids: BTreeMap<PathBuf, BTreeSet<String>>,
    paths: BTreeMap<(String, String), Option<PathBuf>>,
}

impl Index {
    /// Collect every inline evidence row in entities, edges and findings.
    pub fn new(atlas: &Atlas, paths: &Paths) -> Self {
        let mut index = Self::default();
        for (id, value) in atlas
            .entities
            .iter()
            .map(|(id, row)| (id, &row.raw))
            .chain(atlas.edges.iter().map(|(id, row)| (id, &row.raw)))
        {
            index.walk(id, value, "", paths);
        }
        for finding in &atlas.findings {
            index.walk(string(finding, "id"), &finding["where"], "", paths);
        }
        // A blind spot is not an owner of its line, so its rows resolve without joining
        // the line index that hover and navigation read.
        for row in atlas
            .unresolved
            .iter()
            .flat_map(|entry| entry["evidence"].as_array().into_iter().flatten())
        {
            if let Ok(row) = Evidence::deserialize(row) {
                if let Some(site) = index.resolve(&row, "", paths) {
                    index.rows.insert(row, site);
                }
            }
        }
        for (owner, sites) in &mut index.owners {
            sites.sort();
            sites.dedup();
            for site in sites {
                index
                    .by_line
                    .entry((site.path.clone(), site.line))
                    .or_default()
                    .insert(owner.clone());
            }
        }
        index
    }

    fn walk(&mut self, owner: &str, value: &Value, layer: &str, paths: &Paths) {
        match value {
            Value::Object(map) => {
                let layer = map.get("layer").and_then(Value::as_str).unwrap_or(layer);
                if map
                    .get("file")
                    .and_then(Value::as_str)
                    .is_some_and(|f| f.starts_with("file:"))
                {
                    if let Ok(row) = Evidence::deserialize(value) {
                        if let Some(site) = self.resolve(&row, layer, paths) {
                            self.rows.insert(row, site.clone());
                            self.owners.entry(owner.into()).or_default().push(site);
                        }
                    }
                }
                for child in map.values() {
                    self.walk(owner, child, layer, paths);
                }
            }
            Value::Array(rows) => {
                for row in rows {
                    self.walk(owner, row, layer, paths);
                }
            }
            _ => {}
        }
    }

    fn resolve(&mut self, row: &Evidence, layer: &str, paths: &Paths) -> Option<Site> {
        let path = self
            .paths
            .entry((row.file.clone(), row.root.clone()))
            .or_insert_with(|| paths.resolve(row))
            .clone()?;
        let text = self
            .files
            .entry(path.clone())
            .or_insert_with(|| std::fs::read_to_string(&path).ok());
        let line = if let Some(hint) = row.line_hint.filter(|n| *n > 0) {
            hint - 1
        } else {
            let symbol = symbol_of(row.locator.as_deref().unwrap_or(""));
            text.as_deref()
                .and_then(|text| find_symbol(text, symbol))
                .unwrap_or(0)
        };
        self.file_ids
            .entry(path.clone())
            .or_default()
            .insert(row.file.clone());
        Some(Site {
            file_id: row.file.clone(),
            line,
            approximate: row.line_hint.filter(|n| *n > 0).is_none(),
            path,
            layer: layer.into(),
        })
    }

    /// Look up a provenance row without reading its source again.
    pub fn row(&self, value: &Value) -> Option<&Site> {
        let row = Evidence::deserialize(value).ok()?;
        self.rows.get(&row)
    }

    /// Owners whose evidence names this document line.
    pub fn at(&self, document: &Url, line: u32) -> Vec<&str> {
        let Ok(path) = document.to_file_path() else {
            return Vec::new();
        };
        let path = normalize(&path);
        self.by_line
            .get(&(path, line))
            .into_iter()
            .flatten()
            .map(String::as_str)
            .collect()
    }

    /// File ids cited at an absolute path, across roots and packages.
    pub fn file_ids(&self, path: &Path) -> Vec<&str> {
        self.file_ids
            .get(&normalize(path))
            .into_iter()
            .flatten()
            .map(String::as_str)
            .collect()
    }

    /// Cached source line width for diagnostics and navigation.
    pub fn line_length(&self, path: &Path, line: u32) -> u32 {
        self.line_text(path, line)
            .map(|line| line.encode_utf16().count() as u32)
            .unwrap_or(0)
    }

    /// Cached source text at a resolved provenance line.
    pub fn line_text(&self, path: &Path, line: u32) -> Option<&str> {
        self.files
            .get(path)
            .and_then(Option::as_deref)
            .and_then(|t| t.lines().nth(line as usize))
    }
}

/// The word a locator names, the one to look for in the file.
///
/// A launch trail ends in the action that made the entity, `ComposableNode(controller_server)`,
/// and its argument is what the file spells, unless it is a content digest such as
/// `Node(h1f2e3d4c)`. A dotted or scoped locator, `controller_server.ros__parameters.FollowPath`
/// or `nav2::Planner`, ends in its own name.
fn symbol_of(locator: &str) -> &str {
    let last = locator.rsplit('/').next().unwrap_or("");
    if let Some((action, argument)) = last.split_once('(') {
        let argument = argument.trim_end_matches(')').trim_matches(['\'', '"']);
        return if argument.is_empty() || is_digest(argument) {
            action
        } else {
            argument
        };
    }
    last.rsplit("::")
        .next()
        .unwrap_or("")
        .rsplit(['.', '['])
        .next()
        .unwrap_or("")
        .trim_end_matches([']', ')', '\'', '"'])
}

/// The digest discriminator of a launch step: `h` and eight hex digits.
fn is_digest(text: &str) -> bool {
    text.len() == 9 && text.starts_with('h') && text[1..].chars().all(|c| c.is_ascii_hexdigit())
}

/// The first line that uses `symbol` as a whole word, preferring a line that is not an
/// import: a launch file imports `SetParameter` long before it calls it.
fn find_symbol(text: &str, symbol: &str) -> Option<u32> {
    if symbol.is_empty() {
        return None;
    }
    let word = |c: char| c.is_alphanumeric() || c == '_';
    let uses = |line: &str| {
        line.match_indices(symbol).any(|(offset, _)| {
            !line[..offset].chars().next_back().is_some_and(word)
                && !line[offset + symbol.len()..]
                    .chars()
                    .next()
                    .is_some_and(word)
        })
    };
    let is_import = |line: &str| {
        let line = line.trim_start();
        line.starts_with("import ") || line.starts_with("from ") || line.starts_with("#include")
    };
    let lines: Vec<&str> = text.lines().collect();
    lines
        .iter()
        .position(|line| uses(line) && !is_import(line))
        .or_else(|| lines.iter().position(|line| uses(line)))
        .map(|line| line as u32)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    #[test]
    fn locators_name_the_word_the_file_spells() {
        let trail = "Include(navigation_launch.py)/ComposableNode(controller_server)";
        assert_eq!(symbol_of(trail), "controller_server");
        assert_eq!(symbol_of("Node(h1f2e3d4c)"), "Node");
        assert_eq!(
            symbol_of("controller_server.ros__parameters.FollowPath"),
            "FollowPath"
        );
        assert_eq!(
            symbol_of("nav2_controller::ControllerServer"),
            "ControllerServer"
        );
        assert_eq!(symbol_of("SetParameter"), "SetParameter");
        assert_eq!(symbol_of("link[base_link]"), "base_link");
    }
    #[test]
    fn an_import_is_the_last_resort() {
        let text = "from launch_ros.actions import SetParameter\n\n    SetParameter('a', 1),\n";
        assert_eq!(find_symbol(text, "SetParameter"), Some(2));
        assert_eq!(find_symbol("import os\n", "os"), Some(0));
    }
    #[test]
    fn whole_word_only() {
        assert_eq!(find_symbol("foobar\nfoo", "foo"), Some(1));
        assert_eq!(find_symbol("foobar", "foo"), None);
        assert_eq!(find_symbol("x", ""), None);
    }
    #[test]
    fn fixture_sites_and_inverse() {
        let atlas = Atlas::parse(include_str!("../tests/fixtures/ws_static.atlas.json")).unwrap();
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let paths = Paths::new(&root, &atlas, &BTreeMap::new());
        let index = Index::new(&atlas, &paths);
        let sites = &index.owners["frame:base_footprint"];
        let site = sites
            .iter()
            .find(|s| s.file_id == "file:fx_desc|urdf/fx_robot.urdf")
            .unwrap();
        assert!(!site.approximate);
        assert_eq!(
            index.line_text(&site.path, site.line).map(str::trim),
            Some("<link name=\"base_footprint\"/>"),
            "the link element, not the first joint naming it"
        );
        assert!(index.file_ids(&site.path).contains(&site.file_id.as_str()));
        assert!(index
            .at(&uri(&site.path).unwrap(), site.line)
            .contains(&"frame:base_footprint"));
    }
}
