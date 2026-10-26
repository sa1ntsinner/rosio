//! Finding diagnostics derived from indexed provenance.
use crate::atlas::{string, Atlas};
use crate::index::Index;
use lsp_types::{
    Diagnostic, DiagnosticRelatedInformation, DiagnosticSeverity, NumberOrString, Url,
};
use serde_json::json;
use std::collections::BTreeMap;

/// Build diagnostics by document, preserving atlas finding order.
pub fn build(atlas: &Atlas, index: &Index) -> BTreeMap<Url, Vec<Diagnostic>> {
    let mut result: BTreeMap<Url, Vec<Diagnostic>> = BTreeMap::new();
    for finding in &atlas.findings {
        let id = string(finding, "id");
        let locations: Vec<_> = finding["where"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|row| index.row(row).and_then(|site| site.location(index)))
            .collect();
        for (position, location) in locations.iter().enumerate() {
            let related: Vec<_> = locations
                .iter()
                .enumerate()
                .filter(|(other, _)| *other != position)
                .map(|(_, location)| DiagnosticRelatedInformation {
                    location: location.clone(),
                    message: string(finding, "title").into(),
                })
                .collect();
            result
                .entry(location.uri.clone())
                .or_default()
                .push(Diagnostic {
                    range: location.range,
                    severity: Some(match string(finding, "severity") {
                        "error" => DiagnosticSeverity::ERROR,
                        "warn" => DiagnosticSeverity::WARNING,
                        _ => DiagnosticSeverity::INFORMATION,
                    }),
                    code: Some(NumberOrString::String(string(finding, "rule").into())),
                    source: Some("rosio".into()),
                    message: message(finding),
                    related_information: (!related.is_empty()).then_some(related),
                    data: Some(json!({"finding":id})),
                    ..Default::default()
                });
        }
    }
    result
}

/// The sentence that says what to change, or the title when a finding has none.
fn message(finding: &serde_json::Value) -> String {
    match string(finding, "say") {
        "" => string(finding, "title").into(),
        say => say.into(),
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::paths::Paths;
    use std::path::Path;
    #[test]
    fn fixture_diagnostics() {
        let atlas = Atlas::parse(include_str!("../tests/fixtures/ws_static.atlas.json")).unwrap();
        let paths = Paths::new(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            &atlas,
            &BTreeMap::new(),
        );
        let index = Index::new(&atlas, &paths);
        let diagnostics = build(&atlas, &index);
        assert_eq!(diagnostics.values().map(Vec::len).sum::<usize>(), 1);
        assert!(diagnostics
            .keys()
            .all(|uri| !uri.path().ends_with("fx_absent_plugin.xml")));
        let diagnostic = diagnostics
            .iter()
            .find(|(uri, _)| uri.path().ends_with("fx_needs_arg.urdf.xacro"))
            .unwrap()
            .1
            .first()
            .unwrap();
        assert_eq!(diagnostic.severity, Some(DiagnosticSeverity::WARNING));
        assert_eq!(
            diagnostic.code,
            Some(NumberOrString::String("extract.input_refused".into()))
        );
    }
    #[test]
    fn related_locations_and_line_ranges() {
        let atlas = Atlas::parse(&json!({"schema_version":"1.1","findings":[{
            "id":"finding","rule":"param.shadowed","severity":"error","title":"Values differ","say":"Edit this source.",
            "where":[{"file":"file:@repo|Cargo.toml","line_hint":1},{"file":"file:@repo|Cargo.toml","line_hint":2}]
        }]}).to_string()).unwrap();
        let paths = Paths::new(
            Path::new(env!("CARGO_MANIFEST_DIR")),
            &atlas,
            &BTreeMap::new(),
        );
        let index = Index::new(&atlas, &paths);
        let diagnostics = build(&atlas, &index);
        let rows = diagnostics.values().next().unwrap();
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0].range.start.line, 0);
        assert!(rows[0].range.end.character > 0);
        assert_eq!(rows[0].severity, Some(DiagnosticSeverity::ERROR));
        assert_eq!(
            rows[0].related_information.as_ref().unwrap()[0]
                .location
                .range
                .start
                .line,
            1
        );
        assert_eq!(rows[0].data, Some(json!({"finding":"finding"})));
    }
}
