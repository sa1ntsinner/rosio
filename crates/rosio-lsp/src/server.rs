//! Synchronous LSP transport and atlas lifecycle.
use crate::atlas::{string, Atlas};
use crate::diagnostics;
use crate::extraction::Extraction;
use crate::index::Index;
use crate::paths::{uri, Paths};
use crossbeam_channel::{unbounded, Sender};
use lsp_server::{Connection, Message, Notification, Request, Response};
use lsp_types::Url;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::time::Duration;

#[derive(Debug, Deserialize)]
#[serde(default, rename_all = "camelCase")]
struct Options {
    atlas_path: String,
    roots: BTreeMap<String, String>,
    extract_command: Vec<String>,
    extract_on_save: bool,
    code_lens: bool,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            atlas_path: "rosio-out/atlas.json".into(),
            roots: BTreeMap::new(),
            extract_command: vec!["rosio".into(), "extract".into(), ".".into()],
            extract_on_save: false,
            code_lens: true,
        }
    }
}

struct Loaded {
    atlas: Atlas,
    index: Index,
    paths: Paths,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct Summary<'a> {
    loaded: bool,
    schema_version: Option<&'a str>,
    counters: Value,
    layers: Vec<&'a str>,
    profiles: Vec<String>,
    roots: Vec<&'a crate::atlas::Root>,
    findings: BTreeMap<&'static str, usize>,
    atlas_path: PathBuf,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<&'a str>,
}

struct State {
    workspace: PathBuf,
    options: Options,
    loaded: Option<Loaded>,
    error: Option<String>,
    published: BTreeSet<Url>,
    migration: crate::migration::Migration,
    documents: BTreeMap<Url, String>,
    migration_serial: u64,
    open: BTreeSet<Url>,
    notified_missing: bool,
    inlay_refresh: bool,
    next_refresh: u64,
}

/// The nearest directory at or above `start` that holds the atlas, for an editor that opens a
/// subdirectory of the workspace (Zed roots a single file at its own folder).
fn atlas_ancestor(start: &std::path::Path, atlas: &str) -> Option<PathBuf> {
    if std::path::Path::new(atlas).is_absolute() {
        return None;
    }
    start
        .ancestors()
        .find(|dir| dir.join(atlas).is_file())
        .map(PathBuf::from)
}

impl State {
    fn atlas_path(&self) -> PathBuf {
        let configured = self.workspace.join(&self.options.atlas_path);
        if !configured.exists() && crate::paths::is_bundle(&self.workspace) {
            return self.workspace.join("atlas.json");
        }
        configured
    }

    fn reload(&mut self, sender: &Sender<Message>) {
        let result = std::fs::read_to_string(self.atlas_path())
            .map_err(|e| format!("Cannot read atlas: {e}"))
            .and_then(|text| Atlas::parse(&text));
        match result {
            Ok(atlas) => {
                let paths = Paths::new(&self.workspace, &atlas, &self.options.roots);
                let index = Index::new(&atlas, &paths);
                self.loaded = Some(Loaded {
                    atlas,
                    index,
                    paths,
                });
                self.error = None;
                self.publish(sender);
                notify(sender, "rosio/atlasChanged", self.summary());
                if self.inlay_refresh {
                    self.next_refresh += 1;
                    let _ = sender.send(Message::Request(Request::new(
                        format!("inlay-refresh-{}", self.next_refresh).into(),
                        "workspace/inlayHint/refresh".into(),
                        Value::Null,
                    )));
                }
            }
            Err(error) => {
                log(sender, 1, &error);
                self.error = Some(error);
                if self.loaded.is_none() && !self.notified_missing {
                    let program = self.options.extract_command.first();
                    let message = match program {
                        Some(program) if !crate::extraction::engine_found(program) => {
                            json!({"type":2,"message":crate::extraction::engine_missing(program)})
                        }
                        _ => {
                            json!({"type":3,"message":"No usable rosio atlas. Run rosio extract to create one."})
                        }
                    };
                    notify(sender, "window/showMessage", message);
                    self.notified_missing = true;
                }
            }
        }
    }

    fn publish(&mut self, sender: &Sender<Message>) {
        let findings = self
            .loaded
            .as_ref()
            .map(|l| diagnostics::build(&l.atlas, &l.index))
            .unwrap_or_default();
        let diagnostics = self.migration.union(findings);
        let current: BTreeSet<_> = diagnostics.keys().cloned().collect();
        for uri in self.published.union(&current) {
            notify(
                sender,
                "textDocument/publishDiagnostics",
                json!({"uri":uri,"diagnostics":diagnostics.get(uri).cloned().unwrap_or_default()}),
            );
        }
        self.published = current;
    }

    fn summary(&self) -> Value {
        let mut result = Summary {
            loaded: self.loaded.is_some(),
            schema_version: None,
            counters: json!({}),
            layers: Vec::new(),
            profiles: Vec::new(),
            roots: Vec::new(),
            findings: [("error", 0), ("warn", 0), ("info", 0)].into(),
            atlas_path: self.atlas_path(),
            error: self.error.as_deref(),
        };
        if let Some(loaded) = &self.loaded {
            let atlas = &loaded.atlas;
            result.schema_version = Some(&atlas.schema_version);
            result.counters = atlas.counters.clone();
            result.layers = atlas
                .layers
                .iter()
                .filter(|(_, v)| v.present)
                .map(|(k, _)| k.as_str())
                .collect();
            result.profiles = atlas
                .entities
                .iter()
                .filter(|(_, v)| string(v, "kind") == "profile")
                .filter_map(|(id, _)| {
                    crate::ids::parse(id)
                        .ok()
                        .and_then(|p| p.segments.into_iter().next())
                })
                .collect();
            result.roots = atlas.generated.roots.iter().collect();
            for severity in ["error", "warn", "info"] {
                result.findings.insert(
                    severity,
                    atlas
                        .findings
                        .iter()
                        .filter(|v| string(v, "severity") == severity)
                        .count(),
                );
            }
        }
        json!(result)
    }

    fn query(&self, method: &str, params: Value) -> Result<Value, String> {
        if method == "rosio/summary" {
            return Ok(self.summary());
        }
        if method == "textDocument/codeLens" && !self.options.code_lens {
            return Ok(json!([]));
        }
        let Some(loaded) = &self.loaded else {
            return Ok(match method {
                "rosio/children" => json!({}),
                "rosio/entity" | "textDocument/hover" => Value::Null,
                "rosio/graph" => json!({"profile": null, "nodes": [], "channels": [],
                    "links": [], "truncated": false}),
                "rosio/robotModel" => json!({"profile": null, "model": null, "models": [],
                    "links": [], "joints": [], "roots": [], "broadcasts": [],
                    "truncated": false, "limit": 1000}),
                _ => json!([]),
            });
        };
        crate::queries::query(&loaded.atlas, &loaded.index, method, params)
    }
}

/// Serve one connection, including initialization and orderly shutdown.
pub fn run(connection: Connection) -> Result<(), String> {
    let (id, params) = connection.initialize_start().map_err(|e| e.to_string())?;
    let options: Options = match params.get("initializationOptions").filter(|v| !v.is_null()) {
        Some(options) => serde_json::from_value(options.clone())
            .map_err(|e| format!("Invalid initialization options: {e}"))?,
        None => Options::default(),
    };
    let folder = params["workspaceFolders"]
        .as_array()
        .and_then(|folders| folders.first())
        .and_then(|folder| folder["uri"].as_str())
        .or_else(|| params["rootUri"].as_str());
    let workspace = folder
        .and_then(|s| Url::parse(s).ok())
        .and_then(|u| u.to_file_path().ok())
        .or_else(|| std::env::current_dir().ok())
        .ok_or("No workspace folder")?;
    let workspace = atlas_ancestor(&workspace, &options.atlas_path).unwrap_or(workspace);
    let watcher = params
        .pointer("/capabilities/workspace/didChangeWatchedFiles/dynamicRegistration")
        == Some(&Value::Bool(true));
    let progress =
        params.pointer("/capabilities/window/workDoneProgress") == Some(&Value::Bool(true));
    connection
        .initialize_finish(
            id,
            json!({
                "capabilities": {
                    "inlayHintProvider": true,
                    "codeLensProvider": {"resolveProvider": false},
                    "codeActionProvider": {"codeActionKinds": ["quickfix", "source.fixAll.rosioMigration"]},
                    "hoverProvider": true,
                    "definitionProvider": true,
                    "referencesProvider": true,
                    "workspaceSymbolProvider": true,
                    "executeCommandProvider": {"commands": ["rosio-lsp.reload", "rosio-lsp.extract", "rosio-lsp.migratePlan", "rosio-lsp.migrateClear", "rosio-lsp.migrateApply", "rosio-lsp.migrateVerify"]},
                    "textDocumentSync": {"openClose": true, "change": 1, "save": true}
                },
                "serverInfo": {"name": "rosio-lsp", "version": env!("CARGO_PKG_VERSION")}
            }),
        )
        .map_err(|e| e.to_string())?;
    let mut state = State {
        workspace,
        options,
        loaded: None,
        error: None,
        published: BTreeSet::new(),
        migration: Default::default(),
        documents: BTreeMap::new(),
        migration_serial: 0,
        open: BTreeSet::new(),
        notified_missing: false,
        inlay_refresh: params.pointer("/capabilities/workspace/inlayHint/refreshSupport")
            == Some(&Value::Bool(true)),
        next_refresh: 0,
    };
    if watcher {
        let registration = json!({"registrations": [{
            "id": "rosio-atlas",
            "method": "workspace/didChangeWatchedFiles",
            "registerOptions": {"watchers": [{"globPattern": watcher_glob(&state.atlas_path()), "kind": 7}]}
        }]});
        connection
            .sender
            .send(Message::Request(Request::new(
                String::from("atlas-watcher").into(),
                "client/registerCapability".into(),
                registration,
            )))
            .map_err(|e| e.to_string())?;
    }
    state.reload(&connection.sender);
    event_loop(&connection, &mut state, progress)
}

fn event_loop(connection: &Connection, state: &mut State, progress: bool) -> Result<(), String> {
    let (completed, results) = unbounded::<Result<String, String>>();
    let mut extraction = Extraction::default();
    let (migration_done, migration_results) = unbounded();
    let mut shutdown = false;
    // One worker reads and decimates meshes and owns their cache, so the loop never waits on
    // one and the answers still arrive in the order they were asked for.
    let (meshes, mesh_requests) = unbounded::<(Request, Option<PathBuf>)>();
    let mesh_sender = connection.sender.clone();
    std::thread::spawn(move || {
        let mut cache = crate::mesh::Cache::default();
        for (request, path) in mesh_requests {
            let answer = cache.answer(string(&request.params, "file"), path.as_deref());
            respond(&mesh_sender, request, Ok(answer), -32602);
        }
    });
    loop {
        while let Ok(result) = results.try_recv() {
            extraction.finish(&connection.sender, result);
            state.reload(&connection.sender);
        }
        while let Ok((serial, request, result)) = migration_results.try_recv() {
            let result: Result<Value, String> = result;
            if serial == state.migration_serial {
                if let Ok(plan) = &result {
                    state
                        .migration
                        .replace(plan.clone(), &state.workspace, &state.documents);
                    state.publish(&connection.sender);
                    notify(&connection.sender, "rosio/migrationPlan", plan.clone());
                }
                respond(&connection.sender, request, result, -32603);
            } else {
                respond(
                    &connection.sender,
                    request,
                    Err("Migration request superseded".into()),
                    -32800,
                );
            }
        }
        if !shutdown {
            extraction.start(
                &state.workspace,
                &state.options.extract_command,
                &connection.sender,
                &completed,
                progress,
            );
        }
        match connection.receiver.recv_timeout(Duration::from_millis(50)) {
            Ok(Message::Request(request)) => {
                if request.method == "shutdown" {
                    shutdown = true;
                    respond(&connection.sender, request, Ok(Value::Null), -32600);
                } else if shutdown {
                    respond(
                        &connection.sender,
                        request,
                        Err("Server is shutting down".into()),
                        -32600,
                    );
                } else if request.method == "workspace/executeCommand" {
                    let command = string(&request.params, "command");
                    if command == "rosio-lsp.migratePlan" || command == "rosio-lsp.migrateApply" {
                        state.migration_serial += 1;
                        let serial = state.migration_serial;
                        let workspace = state.workspace.clone();
                        let engine = state.options.extract_command.clone();
                        let args = request.params["arguments"][0].clone();
                        let apply = command == "rosio-lsp.migrateApply";
                        let done = migration_done.clone();
                        std::thread::spawn(move || {
                            let result = (|| {
                                let applied = if apply {
                                    Some(crate::migration::run(
                                        &workspace, &engine, "apply", &args,
                                    )?)
                                } else {
                                    None
                                };
                                let mut plan =
                                    crate::migration::run(&workspace, &engine, "plan", &args)?;
                                if let Some(applied) = applied {
                                    plan["applied"] = applied;
                                }
                                Ok(plan)
                            })();
                            let _ = done.send((serial, request, result));
                        });
                        continue;
                    }
                    if command == "rosio-lsp.migrateVerify" {
                        let workspace = state.workspace.clone();
                        let engine = state.options.extract_command.clone();
                        let args = request.params["arguments"][0].clone();
                        let sender = connection.sender.clone();
                        std::thread::spawn(move || {
                            let result =
                                crate::migration::run(&workspace, &engine, "verify", &args);
                            respond(&sender, request, result, -32603);
                        });
                        continue;
                    }
                    let result = match command {
                        "rosio-lsp.migrateClear" => {
                            state.migration_serial += 1;
                            state.migration = Default::default();
                            state.publish(&connection.sender);
                            notify(&connection.sender, "rosio/migrationPlan", Value::Null);
                            Ok(Value::Null)
                        }
                        "rosio-lsp.reload" => {
                            state.reload(&connection.sender);
                            Ok(Value::Null)
                        }
                        "rosio-lsp.extract" => {
                            extraction.request(Duration::ZERO);
                            Ok(Value::Null)
                        }
                        _ => Err(format!("Unknown command {command}")),
                    };
                    respond(&connection.sender, request, result, -32602);
                } else if request.method == "rosio/mesh" {
                    let path = state.loaded.as_ref().and_then(|l| {
                        crate::mesh::locate(&l.atlas, &l.paths, string(&request.params, "file"))
                    });
                    let _ = meshes.send((request, path));
                } else if request.method == "textDocument/codeAction" {
                    let mut actions =
                        crate::actions::code_actions(&state.workspace, &request.params);
                    if let Some(rows) = actions.as_array_mut() {
                        if let Some(loaded) = state.loaded.as_ref() {
                            let fixes = crate::actions::fix_actions(
                                &loaded.atlas,
                                &loaded.paths,
                                &request.params,
                            );
                            rows.splice(0..0, fixes);
                        }
                        rows.extend(state.migration.actions(&request.params));
                    }
                    respond(&connection.sender, request, Ok(actions), -32602);
                } else {
                    let known = matches!(
                        request.method.as_str(),
                        "textDocument/hover"
                            | "textDocument/inlayHint"
                            | "textDocument/codeLens"
                            | "textDocument/definition"
                            | "textDocument/references"
                            | "workspace/symbol"
                            | "rosio/summary"
                            | "rosio/search"
                            | "rosio/entity"
                            | "rosio/findings"
                            | "rosio/children"
                            | "rosio/launchTree"
                            | "rosio/frameTree"
                            | "rosio/paramTree"
                            | "rosio/graph"
                            | "rosio/robotModel"
                            | "rosio/unresolved"
                    );
                    let result = if known {
                        state.query(&request.method, request.params.clone())
                    } else {
                        Err(format!("Unknown method {}", request.method))
                    };
                    respond(
                        &connection.sender,
                        request,
                        result,
                        if known { -32602 } else { -32601 },
                    );
                }
            }
            Ok(Message::Notification(notification)) => {
                if notification.method == "exit" {
                    return if shutdown {
                        Ok(())
                    } else {
                        Err("Exit before shutdown".into())
                    };
                }
                if !shutdown {
                    handle_notification(state, &mut extraction, &connection.sender, notification);
                }
            }
            Ok(Message::Response(_)) => {}
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return Ok(()),
        }
    }
}

fn handle_notification(
    state: &mut State,
    extraction: &mut Extraction,
    sender: &Sender<Message>,
    notification: Notification,
) {
    match notification.method.as_str() {
        "workspace/didChangeWatchedFiles" => {
            let atlas_uri = uri(&state.atlas_path());
            if notification.params["changes"]
                .as_array()
                .is_some_and(|changes| {
                    changes.iter().any(|change| {
                        change["uri"]
                            .as_str()
                            .and_then(|s| Url::parse(s).ok())
                            .as_ref()
                            == atlas_uri.as_ref()
                    })
                })
            {
                state.reload(sender);
            }
        }
        "textDocument/didChange" => {
            if let (Some(uri), Some(text)) = (
                notification.params["textDocument"]["uri"]
                    .as_str()
                    .and_then(|s| Url::parse(s).ok()),
                notification.params["contentChanges"]
                    .as_array()
                    .and_then(|c| c.last())
                    .and_then(|c| c["text"].as_str()),
            ) {
                state.migration.edited(&uri, text);
                state.documents.insert(uri, text.into());
                state.publish(sender);
            }
        }
        "textDocument/didOpen" | "textDocument/didClose" | "textDocument/didSave" => {
            let Some(uri) = notification.params["textDocument"]["uri"]
                .as_str()
                .and_then(|s| Url::parse(s).ok())
            else {
                return;
            };
            match notification.method.as_str() {
                "textDocument/didOpen" => {
                    if let Some(text) = notification.params["textDocument"]["text"].as_str() {
                        state.migration.edited(&uri, text);
                        state.documents.insert(uri.clone(), text.into());
                        state.publish(sender);
                    }
                    state.open.insert(uri);
                }
                "textDocument/didClose" => {
                    state.open.remove(&uri);
                    state.documents.remove(&uri);
                }
                _ if state.options.extract_on_save => {
                    let cited = state.loaded.as_ref().is_some_and(|loaded| {
                        uri.to_file_path()
                            .ok()
                            .is_some_and(|p| !loaded.index.file_ids(&p).is_empty())
                    });
                    if cited {
                        extraction.request(Duration::from_millis(300));
                    }
                }
                _ => {}
            }
        }
        _ => {}
    }
}

fn respond(sender: &Sender<Message>, request: Request, result: Result<Value, String>, code: i32) {
    let response = match result {
        Ok(value) => Response::new_ok(request.id, value),
        Err(error) => Response::new_err(request.id, code, error),
    };
    let _ = sender.send(Message::Response(response));
}
pub(crate) fn notify(sender: &Sender<Message>, method: &str, params: Value) {
    let _ = sender.send(Message::Notification(Notification::new(
        method.into(),
        params,
    )));
}
pub(crate) fn log(sender: &Sender<Message>, level: u32, message: &str) {
    notify(
        sender,
        "window/logMessage",
        json!({"type":level,"message":message}),
    );
}

fn watcher_glob(path: &Path) -> String {
    let path = path.to_string_lossy().replace('\\', "/");
    let path = if let Some(unc) = path.strip_prefix("//?/UNC/") {
        format!("//{unc}")
    } else {
        path.strip_prefix("//?/").unwrap_or(&path).to_owned()
    };
    let mut glob = String::new();
    for character in path.chars() {
        match character {
            '[' | ']' | '*' | '?' | '{' | '}' => {
                glob.push('[');
                glob.push(character);
                glob.push(']');
            }
            _ => glob.push(character),
        }
    }
    glob
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    #[test]
    fn options_and_watcher_paths() {
        let options: Options = serde_json::from_value(json!({"extractOnSave":true})).unwrap();
        assert!(options.extract_on_save);
        assert_eq!(options.atlas_path, "rosio-out/atlas.json");
        assert_eq!(options.extract_command, ["rosio", "extract", "."]);
        assert_eq!(
            watcher_glob(Path::new("/work space/[src]/atlas.json")),
            "/work space/[[]src[]]/atlas.json"
        );
        assert_eq!(
            watcher_glob(Path::new(r"\\?\C:\work\atlas.json")),
            "C:/work/atlas.json"
        );
    }

    #[test]
    fn an_unpacked_bundle_is_read_without_configuration() {
        let workspace =
            std::env::temp_dir().join(format!("rosio-bundle-server-{}", std::process::id()));
        std::fs::create_dir_all(workspace.join("files")).unwrap();
        std::fs::write(workspace.join("manifest.json"), "{}").unwrap();
        let state = State {
            workspace: workspace.clone(),
            options: Options::default(),
            loaded: None,
            error: None,
            published: BTreeSet::new(),
            migration: Default::default(),
            documents: BTreeMap::new(),
            migration_serial: 0,
            open: BTreeSet::new(),
            notified_missing: false,
            inlay_refresh: false,
            next_refresh: 0,
        };
        assert_eq!(state.atlas_path(), workspace.join("atlas.json"));
        std::fs::remove_dir_all(&workspace).unwrap();
    }

    #[test]
    fn a_subdirectory_finds_the_atlas_of_its_workspace() {
        let workspace = std::env::temp_dir().join(format!("rosio-ancestor-{}", std::process::id()));
        let inner = workspace.join("src/pkg/config");
        std::fs::create_dir_all(&inner).unwrap();
        std::fs::create_dir_all(workspace.join("rosio-out")).unwrap();
        std::fs::write(workspace.join("rosio-out/atlas.json"), "{}").unwrap();
        assert_eq!(
            atlas_ancestor(&inner, "rosio-out/atlas.json"),
            Some(workspace.clone())
        );
        std::fs::remove_dir_all(&workspace).unwrap();
    }
}
