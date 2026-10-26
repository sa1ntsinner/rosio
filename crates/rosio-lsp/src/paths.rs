//! Mapping portable evidence to local paths.
use crate::atlas::{Atlas, Evidence};
use crate::ids;
use lsp_types::Url;
use std::collections::BTreeMap;
use std::path::{Component, Path, PathBuf};

/// Root and package locations for one checkout.
pub struct Paths {
    workspace: PathBuf,
    bundled: bool,
    roots: BTreeMap<String, PathBuf>,
    packages: BTreeMap<String, BTreeMap<String, PathBuf>>,
}

impl Paths {
    /// Explicit root overrides take precedence over generated locations.
    pub fn new(workspace: &Path, atlas: &Atlas, overrides: &BTreeMap<String, String>) -> Self {
        let bundled = is_bundle(workspace);
        let configured = configured_roots(workspace);
        let mut roots = BTreeMap::new();
        for root in &atlas.generated.roots {
            let stated = overrides.get(&root.label).or(root.path.as_ref());
            if let Some(path) = stated.or_else(|| configured.get(&root.kind)) {
                if let Some(path) = local_root(workspace, path) {
                    roots.insert(root.label.clone(), path);
                }
            }
        }
        for (label, path) in overrides {
            if let Some(path) = local_root(workspace, path) {
                roots.insert(label.clone(), path);
            }
        }
        let mut packages = BTreeMap::new();
        for (package, locations) in &atlas.generated.packages {
            let mut resolved = BTreeMap::new();
            for (label, relative) in locations {
                if let Some(root) = roots.get(label) {
                    if let Some(path) = relative_join(root, relative) {
                        resolved.insert(label.clone(), path);
                    }
                }
            }
            packages.insert(package.clone(), resolved);
        }
        // Older atlases do not carry the package map.
        if atlas.generated.packages.is_empty() {
            for (label, root) in &roots {
                discover(root, label, &mut packages);
            }
        }
        Self {
            workspace: normalize(workspace),
            bundled,
            roots,
            packages,
        }
    }

    /// Resolve a source row only when its file exists locally.
    pub fn resolve(&self, row: &Evidence) -> Option<PathBuf> {
        let id = ids::parse(&row.file).ok()?;
        if id.kind != "file" {
            return None;
        }
        let package = &id.segments[0];
        let relative = &id.segments[1];
        let path = if self.bundled {
            let files = self.workspace.join("files");
            let base = if package == "@repo" {
                files
            } else {
                relative_join(&files, package)?
            };
            relative_join(&base, relative)
        } else if package == "@repo" {
            relative_join(&self.workspace, relative)
        } else {
            let locations = self.packages.get(package)?;
            let base = locations.get(&row.root)?;
            relative_join(base, relative)
        }?;
        path.is_file().then_some(path)
    }

    /// The configured directory for a root label.
    pub fn root(&self, label: &str) -> Option<&Path> {
        self.roots.get(label).map(PathBuf::as_path)
    }
}

fn discover(root: &Path, label: &str, packages: &mut BTreeMap<String, BTreeMap<String, PathBuf>>) {
    if let Ok(manifest) = std::fs::read_to_string(root.join("package.xml")) {
        if let Some(name) = manifest
            .split("<name>")
            .nth(1)
            .and_then(|s| s.split("</name>").next())
        {
            packages
                .entry(name.trim().into())
                .or_default()
                .entry(label.into())
                .or_insert_with(|| normalize(root));
        }
        return;
    }
    let Ok(entries) = std::fs::read_dir(root) else {
        return;
    };
    let mut dirs: Vec<_> = entries
        .filter_map(Result::ok)
        .filter(|e| {
            e.file_type().is_ok_and(|t| t.is_dir())
                && !e.file_name().to_string_lossy().starts_with('.')
                && !matches!(
                    e.file_name().to_str(),
                    Some("target" | "build" | "__pycache__")
                )
        })
        .map(|e| e.path())
        .collect();
    dirs.sort();
    for dir in dirs {
        discover(&dir, label, packages);
    }
}

/// Root paths by kind from the workspace's `rosio.yaml`, for kinds it names exactly once.
///
/// An atlas states where a root sits only inside a git checkout, and a colcon workspace is
/// usually not one (its packages are). Where the roots are on this machine is what the
/// configuration says, relative to the directory that holds it.
fn configured_roots(workspace: &Path) -> BTreeMap<String, String> {
    let Some((base, text)) = [workspace.to_path_buf(), workspace.join("..")]
        .into_iter()
        .find_map(|dir| {
            Some((
                dir.clone(),
                std::fs::read_to_string(dir.join("rosio.yaml")).ok()?,
            ))
        })
    else {
        return BTreeMap::new();
    };
    let Ok(config) = serde_yaml_ng::from_str::<serde_json::Value>(&text) else {
        return BTreeMap::new();
    };
    let mut found: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for root in config["roots"].as_array().into_iter().flatten() {
        if let (Some(kind), Some(path)) = (root["kind"].as_str(), root["path"].as_str()) {
            let Some(full) = local_root(&base, path) else {
                continue;
            };
            found
                .entry(kind.into())
                .or_default()
                .push(full.to_string_lossy().into_owned());
        }
    }
    found
        .into_iter()
        .filter_map(|(kind, paths)| (paths.len() == 1).then(|| (kind, paths[0].clone())))
        .collect()
}

fn local_root(base: &Path, stated: &str) -> Option<PathBuf> {
    let path = Path::new(stated);
    if !path.is_absolute()
        && (stated.starts_with('/') || stated.starts_with('\\') || stated.contains(':'))
    {
        return None;
    }
    Some(normalize(&base.join(path)))
}

/// An unpacked `rosio bundle`: its sources sit under `files/` beside `manifest.json`.
pub fn is_bundle(dir: &Path) -> bool {
    dir.join("manifest.json").is_file() && dir.join("files").is_dir()
}

fn relative_join(base: &Path, relative: &str) -> Option<PathBuf> {
    if relative.contains('\\') || relative.contains(':') {
        return None;
    }
    let path = Path::new(relative);
    if path.is_absolute()
        || path
            .components()
            .any(|c| !matches!(c, Component::Normal(_) | Component::CurDir))
    {
        return None;
    }
    Some(normalize(&base.join(path)))
}

/// Normalize disk paths, retaining OS path semantics.
pub fn normalize(path: &Path) -> PathBuf {
    let mut lexical = PathBuf::new();
    for part in path.components() {
        match part {
            Component::CurDir => {}
            Component::ParentDir => {
                lexical.pop();
            }
            _ => lexical.push(part.as_os_str()),
        }
    }
    let mut ancestor = lexical.as_path();
    let mut missing = Vec::new();
    loop {
        if let Ok(mut resolved) = ancestor.canonicalize() {
            for component in missing.iter().rev() {
                resolved.push(component);
            }
            return resolved;
        }
        let (Some(name), Some(parent)) = (ancestor.file_name(), ancestor.parent()) else {
            return lexical;
        };
        missing.push(name.to_owned());
        ancestor = parent;
    }
}

/// Convert an absolute disk path to an editor URI.
pub fn uri(path: &Path) -> Option<Url> {
    Url::from_file_path(path).ok()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use serde_json::json;
    #[test]
    fn missing_evidence_has_no_location() {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR"));
        let paths = Paths::new(workspace, &Atlas::default(), &BTreeMap::new());
        assert!(paths
            .resolve(&Evidence {
                file: "file:@repo|missing-source.cpp".into(),
                ..Default::default()
            })
            .is_none());
    }

    #[test]
    fn foreign_absolute_roots_are_not_relative_to_the_workspace() {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR"));
        let foreign = if cfg!(windows) {
            "/opt/ros/jazzy"
        } else {
            "D:\\ros\\jazzy"
        };
        let atlas: Atlas = serde_json::from_value(json!({"generated": {
            "roots": [{"label":"install", "path": foreign}],
            "packages": {"p":{"install":"share/p"}}
        }}))
        .unwrap();
        let paths = Paths::new(workspace, &atlas, &BTreeMap::new());
        assert!(paths.root("install").is_none());
        let paths = Paths::new(
            workspace,
            &atlas,
            &BTreeMap::from([("install".into(), foreign.into())]),
        );
        assert!(paths.root("install").is_none());
        assert!(paths
            .resolve(&Evidence {
                file: "file:p|launch/localization_launch.py".into(),
                root: "install".into(),
                ..Default::default()
            })
            .is_none());

        let configured = workspace.join("../../target/foreign-root-config");
        std::fs::create_dir_all(&configured).unwrap();
        std::fs::write(
            configured.join("rosio.yaml"),
            format!("roots:\n  - kind: install\n    path: {foreign:?}\n"),
        )
        .unwrap();
        let atlas: Atlas = serde_json::from_value(json!({"generated": {
            "roots": [{"label":"install", "kind":"install"}],
            "packages": {"p":{"install":"share/p"}}
        }}))
        .unwrap();
        let paths = Paths::new(&configured, &atlas, &BTreeMap::new());
        assert!(paths.root("install").is_none());
        std::fs::remove_dir_all(configured).unwrap();
    }

    #[test]
    fn a_workspace_outside_git_takes_its_roots_from_the_configuration() {
        let workspace =
            std::env::temp_dir().join(format!("rosio-config-roots-{}", std::process::id()));
        std::fs::create_dir_all(&workspace).unwrap();
        std::fs::write(
            workspace.join("rosio.yaml"),
            "roots:\n  - kind: src\n    path: src\n  - kind: install\n    path: install\n",
        )
        .unwrap();
        std::fs::create_dir_all(workspace.join("src/p/config")).unwrap();
        std::fs::write(workspace.join("src/p/config/a.yaml"), "").unwrap();
        let atlas: Atlas = serde_json::from_value(json!({"generated": {
            "roots": [{"kind":"src","label":"src","hash":"a"},{"kind":"install","label":"install","hash":"b"}],
            "packages": {"p":{"src":"p","install":"p/share/p"}}
        }}))
        .unwrap();
        let paths = Paths::new(&workspace, &atlas, &BTreeMap::new());
        assert_eq!(
            paths.resolve(&Evidence {
                file: "file:p|config/a.yaml".into(),
                root: "src".into(),
                ..Default::default()
            }),
            Some(normalize(&workspace.join("src/p/config/a.yaml")))
        );
        std::fs::remove_dir_all(&workspace).unwrap();
    }

    #[test]
    fn maps_bundle_evidence_to_files() {
        let workspace =
            std::env::temp_dir().join(format!("rosio-bundle-paths-{}", std::process::id()));
        std::fs::create_dir_all(workspace.join("files")).unwrap();
        std::fs::write(workspace.join("manifest.json"), "{}").unwrap();
        let atlas: Atlas = serde_json::from_value(json!({"generated": {
            "roots": [{"label":"src", "path":"original/src"}],
            "packages": {"p":{"src":"nested/p"}}
        }}))
        .unwrap();
        let paths = Paths::new(&workspace, &atlas, &BTreeMap::new());
        for (file, relative) in [
            ("file:p|a.cpp", "p/a.cpp"),
            ("file:@repo|notes.yaml", "notes.yaml"),
        ] {
            let source = workspace.join("files").join(relative);
            std::fs::create_dir_all(source.parent().unwrap()).unwrap();
            std::fs::write(&source, "").unwrap();
            assert_eq!(
                paths.resolve(&Evidence {
                    file: file.into(),
                    root: "src".into(),
                    ..Default::default()
                }),
                Some(normalize(&workspace.join("files").join(relative)))
            );
        }
        std::fs::remove_dir_all(workspace).unwrap();
    }

    #[test]
    fn maps_roots_packages_and_repo() {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR"));
        let atlas: Atlas = serde_json::from_value(json!({"generated": {
            "roots": [{"label":"src", "path":"wrong"}],
            "packages": {"p":{"src":"fixtures"}}
        }}))
        .unwrap();
        let paths = Paths::new(
            workspace,
            &atlas,
            &BTreeMap::from([("src".into(), "tests".into())]),
        );
        let row = Evidence {
            file: "file:p|endpoints.cpp".into(),
            root: "src".into(),
            ..Default::default()
        };
        assert_eq!(
            paths.resolve(&row),
            Some(normalize(&workspace.join("tests/fixtures/endpoints.cpp")))
        );
        let row = Evidence {
            file: "file:@repo|Cargo.toml".into(),
            ..Default::default()
        };
        assert_eq!(
            paths.resolve(&row),
            Some(normalize(&workspace.join("Cargo.toml")))
        );
        assert!(paths
            .resolve(&Evidence {
                file: "file:@repo|../secret".into(),
                ..Default::default()
            })
            .is_none());
    }
}
