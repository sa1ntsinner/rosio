#![allow(clippy::unwrap_used)]
use lsp_server::{Connection, Message, Notification, Request};
use lsp_types::Url;
use serde_json::{json, Value};
use std::path::{Path, PathBuf};
use std::time::Duration;

struct Client {
    connection: Connection,
    thread: Option<std::thread::JoinHandle<Result<(), String>>>,
    next_id: i32,
    notifications: Vec<Notification>,
    server_requests: Vec<Request>,
}
impl Client {
    fn start(root: &Path, options: Value) -> Self {
        Self::start_with_refresh(root, options, false)
    }
    fn start_with_refresh(root: &Path, options: Value, refresh: bool) -> Self {
        let (server, connection) = Connection::memory();
        let thread = std::thread::spawn(move || rosio_lsp::server::run(server));
        let mut client = Self {
            connection,
            thread: Some(thread),
            next_id: 0,
            notifications: Vec::new(),
            server_requests: Vec::new(),
        };
        let initialize = client.request("initialize", json!({"processId":null,"capabilities":{"workspace":{"didChangeWatchedFiles":{"dynamicRegistration":true},"inlayHint":{"refreshSupport":refresh}},"window":{"workDoneProgress":true}},
            "workspaceFolders":[{"uri":Url::from_file_path(root).unwrap(),"name":"fixture"}],"initializationOptions":options}));
        assert_eq!(initialize["capabilities"]["hoverProvider"], true);
        assert_eq!(initialize["capabilities"]["inlayHintProvider"], true);
        assert_eq!(
            initialize["capabilities"]["codeLensProvider"],
            json!({"resolveProvider":false})
        );
        client.notify("initialized", json!({}));
        client
    }
    fn notify(&self, method: &str, params: Value) {
        self.connection
            .sender
            .send(Message::Notification(Notification::new(
                method.into(),
                params,
            )))
            .unwrap();
    }
    fn request(&mut self, method: &str, params: Value) -> Value {
        self.next_id += 1;
        let id = self.next_id.into();
        self.connection
            .sender
            .send(Message::Request(Request::new(id, method.into(), params)))
            .unwrap();
        loop {
            match self
                .connection
                .receiver
                .recv_timeout(Duration::from_secs(10))
                .unwrap()
            {
                Message::Response(response) => {
                    assert!(response.error.is_none(), "{:?}", response.error);
                    return response.result.unwrap();
                }
                Message::Notification(notification) => self.notifications.push(notification),
                Message::Request(request) => {
                    self.connection
                        .sender
                        .send(Message::Response(lsp_server::Response::new_ok(
                            request.id.clone(),
                            Value::Null,
                        )))
                        .unwrap();
                    self.server_requests.push(request);
                }
            }
        }
    }
    fn stop(mut self) {
        self.request("shutdown", Value::Null);
        self.notify("exit", Value::Null);
        self.thread.take().unwrap().join().unwrap().unwrap();
    }
}
fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}
fn fixture() -> &'static str {
    "crates/rosio-lsp/tests/fixtures/ws_static.atlas.json"
}

#[test]
fn fixture_over_memory_transport() {
    let root = root();
    let mut client = Client::start(&root, json!({"atlasPath":fixture()}));
    let summary = client.request("rosio/summary", Value::Null);
    assert_eq!(summary["loaded"], true);
    assert_eq!(summary["findings"]["warn"], 3);
    assert!(client
        .server_requests
        .iter()
        .any(|r| r.method == "client/registerCapability"));
    assert!(client.notifications.iter().any(|notification| {
        notification.method == "textDocument/publishDiagnostics"
            && notification.params["uri"]
                .as_str()
                .unwrap_or("")
                .ends_with("fx_desc/xacro/fx_needs_arg.urdf.xacro")
            && notification.params["diagnostics"][0]["code"] == "extract.input_refused"
    }));
    let entity = client.request("rosio/entity", json!({"id":"frame:base_footprint"}));
    let site = entity["sites"]
        .as_array()
        .unwrap()
        .iter()
        .find(|s| s["uri"].as_str().unwrap().ends_with("urdf/fx_robot.urdf"))
        .unwrap();
    let position =
        json!({"textDocument":{"uri":site["uri"]},"position":{"line":site["line"],"character":0}});
    let hover = client.request("textDocument/hover", position.clone());
    assert!(!hover["contents"]["value"].as_str().unwrap().is_empty());
    let definitions = client.request("textDocument/definition", position.clone());
    assert!(!definitions.as_array().unwrap().is_empty());
    let references = client.request("textDocument/references", position);
    assert!(!references.as_array().unwrap().is_empty());
    let results = client.request("rosio/search", json!({"query":"cmd_vel","kinds":["topic"]}));
    assert!(!results.as_array().unwrap().is_empty());
    assert!(results
        .as_array()
        .unwrap()
        .iter()
        .all(|e| e["kind"] == "topic"));
    let symbols = client.request("workspace/symbol", json!({"query":"cmd_vel"}));
    assert!(!symbols.as_array().unwrap().is_empty());
    let findings = client.request("rosio/findings", json!({"minSeverity":"warn"}));
    assert_eq!(findings.as_array().unwrap().len(), 3);
    let blind = client.request("rosio/unresolved", json!({}));
    let blind = blind.as_array().unwrap();
    assert_eq!(blind.len(), 5);
    let plugin = blind
        .iter()
        .find(|row| row["expr"] == "fx_missing_library")
        .unwrap();
    assert_eq!(plugin["owner"], "fx_plugins");
    assert_eq!(plugin["resolvableBy"][0], "add the library target");
    assert!(plugin["locations"][0]["uri"]
        .as_str()
        .unwrap()
        .ends_with("fx_tools_plugin.xml"));
    let groups = client.request("rosio/children", json!({"kind":"topic"}));
    assert!(!groups["topic"].as_array().unwrap().is_empty());
    let mesh = client.request(
        "rosio/mesh",
        json!({"file":"file:fx_desc|meshes/fx_body.stl"}),
    );
    assert_eq!(mesh["triangleCount"], 1);
    assert_eq!(
        mesh["bounds"],
        json!({"min":[0.0,0.0,0.0],"max":[1.0,1.0,0.0]})
    );
    let absent = client.request(
        "rosio/mesh",
        json!({"file":"file:fx_desc|meshes/fx_absent_body.stl"}),
    );
    assert_eq!(absent["refused"], "file_missing");
    client.stop();
}

fn scratch(name: &str) -> PathBuf {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("target/lsp-tests")
        .join(format!("{name}-{}", std::process::id()));
    std::fs::create_dir_all(&path).unwrap();
    path
}

#[test]
fn absent_and_malformed_atlas_are_usable() {
    let directory = scratch("absent");
    let mut client = Client::start(&directory, json!({"atlasPath":"missing.json"}));
    let summary = client.request("rosio/summary", Value::Null);
    assert_eq!(summary["loaded"], false);
    assert!(summary["error"].is_string());
    assert_eq!(client.request("rosio/launchTree", json!({})), json!([]));
    assert_eq!(client.request("rosio/frameTree", json!({})), json!([]));
    client.request(
        "workspace/executeCommand",
        json!({"command":"rosio-lsp.reload"}),
    );
    assert_eq!(
        client
            .notifications
            .iter()
            .filter(|n| n.method == "window/showMessage")
            .count(),
        1
    );
    client.stop();
}

#[test]
fn a_missing_engine_says_how_to_install_it() {
    let directory = scratch("no-engine");
    let program = "rosio-engine-that-is-not-installed";
    let mut client = Client::start(
        &directory,
        json!({"atlasPath":"missing.json","extractCommand":[program,"extract","."]}),
    );
    client.request(
        "workspace/executeCommand",
        json!({"command":"rosio-lsp.extract"}),
    );
    client.wait_notification("$/progress", "end");
    let messages: Vec<&str> = client
        .notifications
        .iter()
        .filter(|n| n.method == "window/showMessage")
        .map(|n| n.params["message"].as_str().unwrap())
        .collect();
    assert_eq!(messages.len(), 2, "{messages:?}");
    for message in messages {
        assert!(message.contains(program), "{message}");
        assert!(message.contains("`pipx install rosio` or `pip install rosio`"));
        assert!(!message.contains('\n'));
    }
    client.stop();
}

#[test]
fn reload_retains_previous_atlas_and_clears_diagnostics() {
    let directory = scratch("reload");
    let file = directory.join("atlas.json");
    std::fs::write(&file, include_str!("fixtures/ws_static.atlas.json")).unwrap();
    let source = root().join("tests/fixtures/ws_static/src");
    let mut client = Client::start(
        &directory,
        json!({"atlasPath":"atlas.json","roots":{"src":source}}),
    );
    assert_eq!(client.request("rosio/summary", Value::Null)["loaded"], true);
    std::fs::write(&file, "{broken").unwrap();
    client.request(
        "workspace/executeCommand",
        json!({"command":"rosio-lsp.reload"}),
    );
    let summary = client.request("rosio/summary", Value::Null);
    assert_eq!(summary["loaded"], true);
    assert_eq!(summary["findings"]["warn"], 3);
    assert!(summary["error"].is_string());
    client.notifications.clear();
    std::fs::write(&file, r#"{"schema_version":"1.999"}"#).unwrap();
    client.notify(
        "workspace/didChangeWatchedFiles",
        json!({"changes":[{"uri":Url::from_file_path(&file).unwrap(),"type":2}]}),
    );
    let summary = client.request("rosio/summary", Value::Null);
    assert_eq!(summary["findings"]["warn"], 0);
    assert!(summary.get("error").is_none());
    let cleared: Vec<_> = client
        .notifications
        .iter()
        .filter(|n| n.method == "textDocument/publishDiagnostics")
        .collect();
    assert_eq!(cleared.len(), 1);
    assert!(cleared.iter().all(|n| n.params["diagnostics"] == json!([])));
    client.stop();
}

#[test]
fn extraction_command_child() {
    let Ok(config) = std::fs::read_to_string("extract-child-config.json") else {
        return;
    };
    let config: Value = serde_json::from_str(&config).unwrap();
    let previous = std::fs::read_to_string("runs.txt").unwrap_or_default();
    let count = previous.parse::<usize>().unwrap_or(0) + 1;
    assert!(
        !Path::new("active").exists(),
        "Extraction workers overlapped"
    );
    std::fs::write("active", "").unwrap();
    std::fs::write("runs.txt", count.to_string()).unwrap();
    let release = format!("release-{count}");
    let start = std::time::Instant::now();
    while !Path::new(&release).exists() {
        assert!(
            start.elapsed() < Duration::from_secs(8),
            "Child was never released"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    std::fs::copy(config["atlas"].as_str().unwrap(), "atlas.json").unwrap();
    std::fs::remove_file("active").unwrap();
    eprintln!("fixture extraction stderr");
}

impl Client {
    fn wait_notification(&mut self, method: &str, kind: &str) {
        let start = std::time::Instant::now();
        loop {
            assert!(start.elapsed() < Duration::from_secs(10));
            match self
                .connection
                .receiver
                .recv_timeout(Duration::from_secs(10))
                .unwrap()
            {
                Message::Notification(notification) => {
                    let matches = notification.method == method
                        && notification.params["value"]["kind"] == kind;
                    self.notifications.push(notification);
                    if matches {
                        return;
                    }
                }
                Message::Request(request) => {
                    self.connection
                        .sender
                        .send(Message::Response(lsp_server::Response::new_ok(
                            request.id.clone(),
                            Value::Null,
                        )))
                        .unwrap();
                    self.server_requests.push(request);
                }
                Message::Response(response) => panic!("Unexpected response: {response:?}"),
            }
        }
    }
}

#[test]
fn extraction_runs_off_thread_and_queues_one_rerun() {
    let directory = scratch("extract");
    let source = root().join("tests/fixtures/ws_static/src");
    let atlas = root().join(fixture());
    std::fs::write(
        directory.join("extract-child-config.json"),
        json!({"atlas":atlas}).to_string(),
    )
    .unwrap();
    let mut client = Client::start(
        &directory,
        json!({"atlasPath":"atlas.json","roots":{"src":source},
        "extractCommand":[std::env::current_exe().unwrap(),"--exact","extraction_command_child","--nocapture"]}),
    );
    client.request(
        "workspace/executeCommand",
        json!({"command":"rosio-lsp.extract"}),
    );
    client.wait_notification("$/progress", "begin");
    for _ in 0..4 {
        client.request(
            "workspace/executeCommand",
            json!({"command":"rosio-lsp.extract"}),
        );
    }
    // A query still answers while the child is waiting.
    assert!(client.request("rosio/summary", Value::Null)["loaded"].is_boolean());
    std::fs::write(directory.join("release-1"), "").unwrap();
    client.wait_notification("$/progress", "end");
    client.wait_notification("$/progress", "begin");
    std::fs::write(directory.join("release-2"), "").unwrap();
    client.wait_notification("$/progress", "end");
    assert_eq!(client.request("rosio/summary", Value::Null)["loaded"], true);
    assert_eq!(
        std::fs::read_to_string(directory.join("runs.txt")).unwrap(),
        "2"
    );
    assert_eq!(
        client
            .notifications
            .iter()
            .filter(|n| n.method == "$/progress" && n.params["value"]["kind"] == "begin")
            .count(),
        2
    );
    assert!(client
        .notifications
        .iter()
        .any(|n| n.method == "window/logMessage"
            && n.params["message"]
                .as_str()
                .unwrap_or("")
                .contains("fixture extraction stderr")));
    client.stop();
}

#[test]
fn saving_cited_files_debounces_extraction() {
    let directory = scratch("save");
    let source = root().join("tests/fixtures/ws_static/src");
    let atlas = root().join(fixture());
    std::fs::copy(&atlas, directory.join("atlas.json")).unwrap();
    std::fs::write(
        directory.join("extract-child-config.json"),
        json!({"atlas":atlas}).to_string(),
    )
    .unwrap();
    std::fs::write(directory.join("release-1"), "").unwrap();
    let mut client = Client::start(
        &directory,
        json!({"atlasPath":"atlas.json","roots":{"src":source},"extractOnSave":true,
        "extractCommand":[std::env::current_exe().unwrap(),"--exact","extraction_command_child","--nocapture"]}),
    );
    let uncited = Url::from_file_path(directory.join("unrelated.txt")).unwrap();
    client.notify(
        "textDocument/didSave",
        json!({"textDocument":{"uri":uncited}}),
    );
    let file = Url::from_file_path(source.join("fx_desc/urdf/fx_robot.urdf")).unwrap();
    client.notify(
        "textDocument/didOpen",
        json!({"textDocument":{"uri":file,"languageId":"xml","version":1,"text":""}}),
    );
    for _ in 0..3 {
        client.notify("textDocument/didSave", json!({"textDocument":{"uri":file}}));
    }
    client.request("rosio/summary", Value::Null);
    assert!(!directory.join("runs.txt").exists());
    client.wait_notification("$/progress", "begin");
    client.wait_notification("$/progress", "end");
    assert_eq!(
        std::fs::read_to_string(directory.join("runs.txt")).unwrap(),
        "1"
    );
    client.notify(
        "textDocument/didClose",
        json!({"textDocument":{"uri":file}}),
    );
    client.stop();
}

#[test]
fn parameter_hints_over_memory_transport_and_reload_refresh() {
    let root = root();
    for refresh in [false, true] {
        let mut client = Client::start_with_refresh(
            &root,
            json!({"atlasPath":"crates/rosio-lsp/tests/fixtures/params.atlas.json"}),
            refresh,
        );
        let document =
            Url::from_file_path(root.join("crates/rosio-lsp/tests/fixtures/params.yaml")).unwrap();
        let params = json!({"textDocument":{"uri":document},
            "range":{"start":{"line":0,"character":0},"end":{"line":8,"character":0}}});
        let hints = client.request("textDocument/inlayHint", params.clone());
        assert_eq!(hints.as_array().unwrap().len(), 5);
        assert_eq!(hints[0]["label"], "= 2");
        assert_eq!(
            hints[1]["label"],
            "shadowed by crates/rosio-lsp/tests/fixtures/params.yaml:6"
        );
        assert_eq!(hints[3]["label"], "no node is named this");
        assert_eq!(hints[0]["paddingLeft"], true);
        assert!(hints[0].get("kind").is_none());
        assert_eq!(hints[0]["tooltip"]["kind"], "markdown");
        let initial_refreshes = client
            .server_requests
            .iter()
            .filter(|r| r.method == "workspace/inlayHint/refresh")
            .count();
        assert_eq!(initial_refreshes, usize::from(refresh));
        client.request(
            "workspace/executeCommand",
            json!({"command":"rosio-lsp.reload"}),
        );
        assert_eq!(client.request("textDocument/inlayHint", params), hints);
        let refreshes: Vec<_> = client
            .server_requests
            .iter()
            .filter(|r| r.method == "workspace/inlayHint/refresh")
            .collect();
        assert_eq!(refreshes.len(), 2 * usize::from(refresh));
        if refresh {
            assert_ne!(refreshes[0].id, refreshes[1].id);
            assert!(refreshes.iter().all(|r| r.params.is_null()));
        }
        client.stop();
    }
}

#[test]
fn endpoint_lenses_over_memory_transport_and_disabled_option() {
    let root = root();
    let document =
        Url::from_file_path(root.join("crates/rosio-lsp/tests/fixtures/endpoints.cpp")).unwrap();
    let params = json!({"textDocument":{"uri":document}});
    for enabled in [true, false] {
        let mut options =
            json!({"atlasPath":"crates/rosio-lsp/tests/fixtures/endpoints.atlas.json"});
        if !enabled {
            options["codeLens"] = json!(false);
        }
        let mut client = Client::start(&root, options);
        let result = client.request("textDocument/codeLens", params.clone());
        if enabled {
            let lenses: Vec<lsp_types::CodeLens> = serde_json::from_value(result.clone()).unwrap();
            assert_eq!(lenses.len(), 3);
            assert_eq!(lenses[0].range.start.line, 0);
            assert_eq!(
                lenses[0].command.as_ref().unwrap().title,
                "2 publishers: /fake_scan, /scan_filter · QoS incompatible"
            );
            assert_eq!(
                lenses[0].command.as_ref().unwrap().command,
                "rosio.showEntity"
            );
            assert_eq!(
                lenses[0].command.as_ref().unwrap().arguments,
                Some(vec![json!("topic:/scan")])
            );
            assert_eq!(lenses[1].command.as_ref().unwrap().title, "no subscriber");
            assert_eq!(lenses[2].range.start.line, 2);
            client.request(
                "workspace/executeCommand",
                json!({"command":"rosio-lsp.reload"}),
            );
            assert_eq!(
                client.request("textDocument/codeLens", params.clone()),
                result
            );
        } else {
            assert_eq!(result, json!([]));
        }
        client.stop();
    }
    let directory = scratch("lens-absent");
    let mut client = Client::start(&directory, json!({"atlasPath":"missing.json"}));
    assert_eq!(client.request("textDocument/codeLens", params), json!([]));
    client.stop();
}

#[test]
fn healthy_loads_notify_without_diagnostics() {
    let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("target/healthy-reload");
    std::fs::create_dir_all(&directory).unwrap();
    let file = directory.join("atlas.json");
    let _ = std::fs::remove_file(&file);
    let mut client = Client::start(&directory, json!({"atlasPath":"atlas.json"}));
    client.request("rosio/summary", Value::Null);
    assert!(!client
        .notifications
        .iter()
        .any(|n| n.method == "rosio/atlasChanged"));
    for count in [1, 2] {
        std::fs::write(
            &file,
            json!({"schema_version":"1.5", "counters":{"entities":count}}).to_string(),
        )
        .unwrap();
        client.request(
            "workspace/executeCommand",
            json!({"command":"rosio-lsp.reload"}),
        );
        let summary = client.request("rosio/summary", Value::Null);
        let changed: Vec<_> = client
            .notifications
            .iter()
            .filter(|n| n.method == "rosio/atlasChanged")
            .collect();
        assert_eq!(changed.len(), count);
        assert_eq!(
            changed.last().unwrap().params["atlasPath"],
            summary["atlasPath"]
        );
        assert_eq!(
            changed.last().unwrap().params["counters"],
            summary["counters"]
        );
        assert_eq!(
            changed.last().unwrap().params["findings"],
            summary["findings"]
        );
        assert!(!client
            .notifications
            .iter()
            .any(|n| n.method == "textDocument/publishDiagnostics"));
    }
    client.stop();
    std::fs::remove_dir_all(directory).unwrap();
}

#[test]
fn trees_over_memory_transport() {
    let directory = scratch("trees");
    std::fs::write(
        directory.join("atlas.json"),
        include_str!("fixtures/trees.atlas.json"),
    )
    .unwrap();
    let mut client = Client::start(&directory, json!({"atlasPath":"atlas.json"}));
    let launches = client.request("rosio/launchTree", json!({"profile":"p"}));
    assert_eq!(
        launches[0]["entries"][0]["children"]
            .as_array()
            .unwrap()
            .len(),
        3
    );
    let frames = client.request("rosio/frameTree", json!({"profile":"p"}));
    assert_eq!(frames[0]["children"][0]["id"], "frame:camera");
    client.stop();
}
