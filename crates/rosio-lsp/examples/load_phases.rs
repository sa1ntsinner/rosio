//! Time each phase of an atlas load: `cargo run --release --example load_phases -- <workspace>`.
use rosio_lsp::{atlas::Atlas, diagnostics, index::Index, paths::Paths};
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Instant;

fn main() {
    let workspace = PathBuf::from(std::env::args().nth(1).unwrap_or_else(|| ".".into()));
    let mut clock = Instant::now();
    let mut lap = |phase: &str| {
        eprintln!(
            "{phase:<12} {:>8.1} ms",
            clock.elapsed().as_secs_f64() * 1e3
        );
        clock = Instant::now();
    };
    let Ok(text) = std::fs::read_to_string(workspace.join("rosio-out/atlas.json")) else {
        eprintln!("no atlas under {}", workspace.display());
        return;
    };
    lap("read");
    let Ok(atlas) = Atlas::parse(&text) else {
        eprintln!("unreadable atlas");
        return;
    };
    lap("parse");
    let paths = Paths::new(&workspace, &atlas, &BTreeMap::new());
    lap("paths");
    let index = Index::new(&atlas, &paths);
    lap("index");
    let published = diagnostics::build(&atlas, &index);
    lap("diagnostics");
    eprintln!("{} documents with diagnostics", published.len());
}
