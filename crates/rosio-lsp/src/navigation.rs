//! Navigation over entity and edge evidence.
use crate::atlas::Atlas;
use crate::index::{Index, Site};
use crate::paths::normalize;
use lsp_types::{Location, Url};

/// Other sites of cursor entities, with declared claims first.
pub fn definitions(atlas: &Atlas, index: &Index, uri: &Url, line: u32) -> Vec<Location> {
    let mut sites = Vec::new();
    for id in index.at(uri, line) {
        if atlas.entities.contains_key(id) {
            sites.extend(index.owners[id].iter());
        }
    }
    let path = uri.to_file_path().ok().map(|p| normalize(&p));
    sites.retain(|s| !(Some(&s.path) == path.as_ref() && s.line == line));
    locations(sites, index)
}

/// All entity sites and all incident edge sites.
pub fn references(atlas: &Atlas, index: &Index, uri: &Url, line: u32) -> Vec<Location> {
    let mut sites = Vec::new();
    for id in index.at(uri, line) {
        if !atlas.entities.contains_key(id) {
            continue;
        }
        sites.extend(index.owners[id].iter());
        for (edge_id, _) in atlas.edges_touching(id) {
            sites.extend(index.owners.get(edge_id).into_iter().flatten());
        }
    }
    locations(sites, index)
}

fn locations(mut sites: Vec<&Site>, index: &Index) -> Vec<Location> {
    sites.sort_by_key(|site| (layer_rank(&site.layer), &site.path, site.line));
    let mut locations = Vec::new();
    for site in sites {
        if let Some(location) = site.location(index) {
            if !locations.contains(&location) {
                locations.push(location);
            }
        }
    }
    locations
}
fn layer_rank(layer: &str) -> usize {
    ["declared", "coded", "observed", "recorded"]
        .iter()
        .position(|v| *v == layer)
        .unwrap_or(4)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::{uri, Paths};
    use std::{collections::BTreeMap, path::Path};
    #[test]
    fn fixture_navigation() {
        let atlas = Atlas::parse(include_str!("../tests/fixtures/ws_static.atlas.json")).unwrap();
        let paths = Paths::new(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            &atlas,
            &BTreeMap::new(),
        );
        let index = Index::new(&atlas, &paths);
        let site = &index.owners["frame:base_footprint"][0];
        let document = uri(&site.path).unwrap();
        let definitions = definitions(&atlas, &index, &document, site.line);
        assert!(!definitions.is_empty());
        assert!(definitions
            .iter()
            .all(|l| l.uri != document || l.range.start.line != site.line));
        assert!(references(&atlas, &index, &document, site.line).len() > definitions.len());
    }
}
