const { test } = require("node:test");
const assert = require("node:assert/strict");
const { buildSync } = require("esbuild");
const vm = require("node:vm");
const path = require("node:path");

function load(file, globals, exports = "") {
  const fs = require("node:fs");
  const result = buildSync({ stdin: { contents: fs.readFileSync(file, "utf8") + exports, resolveDir: path.dirname(path.resolve(file)), loader: "ts" }, bundle: true, platform: "node", format: "cjs", external: ["vscode", "vscode-languageclient/node"], write: false });
  const context = { exports: {}, module: { exports: {} }, setTimeout, clearTimeout, ...globals };
  vm.runInNewContext(result.outputFiles[0].text, context);
  return context.module.exports;
}

class Element {
  constructor(tag) { this.tag = tag; this.children = []; this.attrs = {}; }
  append(child) { this.children.push(child); }
  setAttribute(key, value) { this.attrs[key] = value; }
  addEventListener() {}
  get textContent() { return this.children.map(c => typeof c === "string" ? c : c.textContent).join(" "); }
}
const document = { createElement: tag => new Element(tag), createTextNode: text => text, addEventListener() {} };
function card() {
  return load("webview/main.ts", { document, window: { addEventListener() {} }, acquireVsCodeApi: () => ({ postMessage() {} }) }, "\nexport { layerLedger, parameterChain }; ");
}
const fact = (layer, profile, value) => ({ layer, profile, confidence: "certain", values: { value } });
const entity = { kind: "node", profiles: ["profile:real", "profile:sim"], conflicts: [], facts: [fact("coded", "*", "fallback"), fact("declared", "profile:real", "real value"), fact("observed", "profile:sim", "sim value")] };

test("ledger separates profiles and repeats wildcard fallback", () => {
  const ledger = card().layerLedger(entity);
  const sections = ledger.children.filter(child => child.tag === "details");
  assert.equal(sections.length, 2);
  assert.match(sections[0].textContent, /real/);
  assert.match(sections[0].textContent, /fallback/);
  assert.doesNotMatch(sections[0].textContent, /sim value/);
  assert.match(sections[1].textContent, /sim value/);
  assert.match(sections[1].textContent, /fallback/);
  assert.doesNotMatch(sections[1].textContent, /real value/);
});

test("single profile has no profile heading and overrides fallback", () => {
  const ledger = card().layerLedger({ ...entity, profiles: ["profile:real"], facts: [fact("coded", "profile:real", "specific"), fact("coded", "*", "fallback")] });
  assert.doesNotMatch(ledger.textContent, /profile:real|fallback/);
  assert.match(ledger.textContent, /specific/);
});

test("each candidate identifies its profile", () => {
  const chain = card().parameterChain({ candidates: ["profile:real", "profile:sim"].map(profile => ({ profile, rank: 0, wins: true, value: 1, mechanism: "yaml" })) });
  assert.match(chain.textContent, /real/);
  assert.match(chain.textContent, /sim/);
});

test("healthy atlas notification triggers view refresh", async () => {
  let client;
  class LanguageClient {
    constructor() { client = this; this.handlers = {}; }
    onNotification(method, handler) { this.handlers[method] = handler; return { dispose() {} }; }
    async start() {}
    needsStop() { return false; }
  }
  class EventEmitter {
    constructor() { this.listeners = []; this.event = fn => this.listeners.push(fn); }
    fire() { for (const listener of this.listeners) listener(); }
    dispose() {}
  }
  const vscode = { EventEmitter, workspace: { getConfiguration: () => ({ get: (_key, fallback) => fallback }) } };
  const { Server } = load("src/server.ts", { process, require: name => name === "vscode" ? vscode : name.startsWith("node:") ? require(name) : { LanguageClient } });
  const server = new Server({ appendLine() {} });
  let changes = 0;
  server.onDidChange(() => changes++);
  await server.start();
  const before = changes;
  assert.equal(typeof client.handlers["rosio/atlasChanged"], "function");
  client.handlers["rosio/atlasChanged"]({ atlasPath: "atlas.json", counters: {}, findings: { error: 0, warn: 0, info: 0 } });
  await new Promise(resolve => setTimeout(resolve, 200));
  assert.equal(changes, before + 1);
  server.dispose();
});

test("parameter ledgers retain separate single layer profiles", () => {
  const ledger = card().layerLedger({ ...entity, kind: "param" });
  assert.equal(ledger.children.filter(child => child.tag === "details").length, 2);
});

test("migration model groups packages, counts changes and describes empty state", () => {
  const { migrationGroups, MIGRATION_EMPTY, targetReleases } = load("src/migrationModel.ts", {});
  const change = (pkg, file, line) => ({ package: pkg, file, line, column: 1, rule: "test" });
  const rows = migrationGroups({ changes: [change("z", "b", 2), change("a", "c", 1), change("z", "a", 4)] });
  assert.equal(rows.length, 2);
  assert.equal(rows[0].package, "a");
  assert.equal(rows[1].count, 2);
  assert.equal(rows[1].changes[0].file, "a");
  assert.equal(migrationGroups().length, 0);
  assert.match(MIGRATION_EMPTY, /Plan migration/);
  assert.equal(targetReleases("jazzy").join(","), "kilted,lyrical");
  assert.equal(targetReleases("lyrical").length, 0);
  assert.equal(targetReleases().length, 5);
});

test("migration sections put the changes a person makes first and name them by their first clause", () => {
  const model = load("src/migrationModel.ts", {});
  const hand = { package: "driver", file: "src/driver/package.xml", line: 11, column: 3, rule: "dependency.unreleased",
    effect: "removed", release: "jazzy", say: "gazebo_ros is not released for jazzy; move to gz sim", source: "s", found: "gazebo_ros" };
  const rewrite = { ...hand, file: "src/driver/CMakeLists.txt", line: 4, rule: "cmake.minimum_required",
    say: "CMake 4 refuses old minimums", replacement: "3.8", found: "3.0.2" };
  const sections = model.migrationSections({ from: "humble", to: "lyrical", changes: [rewrite, hand], summary: {} });
  assert.equal(sections.map((s) => `${s.kind} ${s.count}`).join(", "), "hand 1, automatic 1");
  assert.equal(model.changeTitle(hand), "gazebo_ros is not released for jazzy");
  assert.equal(model.changeTitle({ ...hand, title: "gazebo_ros" }), "gazebo_ros");
  assert.equal(model.changeDescription(hand), "package.xml:11");
  assert.equal(model.changeDescription({ ...hand, effect: "deprecated" }), "package.xml:11  warns only");
  assert.equal(model.planMessage({ from: "humble", to: "lyrical", changes: [rewrite, hand] }),
    "1 change needs you, 1 is an exact rewrite.");
  assert.equal(model.planMessage({ from: "humble", to: "iron", changes: [] }),
    "Nothing in this workspace changes from humble to iron.");
  assert.equal(model.planMessage(), undefined);
});

test("a build in the target shows only what failed, errors the plan misses first", () => {
  const model = load("src/migrationModel.ts", {});
  const error = (status, line) => ({ file: "src/driver/src/driver.cpp", line, status, matches: [],
    message: `driver.cpp: 'foo' was not declared in this scope\n  12 | foo();` });
  const build = { from: "humble", to: "lyrical", changes: [], summary: {}, image: "ros:lyrical-ros-base", ok: false,
    packages: [
      { name: "planner", status: "skipped", errors: [], all_errors_listed: false },
      { name: "driver", status: "failed", errors: [error("listed", 3), error("not_listed", 12)], all_errors_listed: false },
      { name: "msgs", status: "finished", errors: [], all_errors_listed: false },
    ] };
  const section = model.buildSection(build);
  assert.equal(`${section.label}: ${section.description}`, "Build in lyrical: 1 of 3 built");
  assert.equal(section.packages.map((p) => p.name).join(","), "driver,planner");
  assert.equal(model.buildPackageDescription(section.packages[0]), "2 errors, 1 not in the plan");
  assert.equal(model.buildPackageDescription(section.packages[1]), "not built");
  assert.equal(model.buildErrors(section.packages[0]).map((e) => e.line).join(","), "12,3");
  assert.equal(model.buildErrorTitle(error("listed", 3)), "driver.cpp: 'foo' was not declared in this scope");
  assert.equal(model.buildErrorWhere(error("listed", 3)), "driver.cpp:3");
  assert.equal(model.buildErrorWhere({ ...error("listed", 3), file: null }), "no line");
  assert.equal(model.buildMessage(build), "2 of 3 packages do not build in lyrical; 1 error is not in the plan.");
  assert.equal(model.buildPackageLabel("@setup"), "Container setup");
  assert.equal(model.buildMessage({ ...build, ok: true, packages: [build.packages[2]] }), "Every package builds in lyrical.");
  assert.equal(model.buildSection(), undefined);
});

const launchEntity = (fields) => ({ kind: "node", nodes: [], package: null, executable: null, namespace: "/",
  conditions: [], gates: [], parameter_inputs: [], remaps: [], children: [], ...fields });

test("launch rows read as the command, the node and what runs it", () => {
  const model = load("src/launchModel.ts", {});
  const rsp = launchEntity({ label: "Node(robot_state_publisher@45d4c79b)", package: "robot_state_publisher",
    executable: "robot_state_publisher", nodes: [{ id: "node:/robot_state_publisher", label: "/robot_state_publisher" }] });
  const include = launchEntity({ kind: "include", label: "Include(navigation_launch.py)", children: [rsp, { ...rsp, id: "other" }] });
  const profile = { id: "profile:nav", label: "nav2 with rviz", entries: [{ id: "f", label: "launch/bringup_launch.py",
    arg_declarations: [], args: [], children: [include] }],
    command: "ros2 launch nav2_bringup bringup_launch.py\nros2 launch nav2_bringup rviz_launch.py" };
  assert.equal(model.profileTitle(profile), "nav");
  assert.equal(model.profileDescription(profile), "1 node");
  assert.equal(model.fileTitle(profile.entries[0]), "bringup_launch.py");
  assert.equal(model.fileDescription({ ...profile.entries[0], id: "launchfile:nav2_bringup|launch/bringup_launch.py",
    args: [["use_sim_time", "false"]] }), "nav2_bringup  use_sim_time:=false");
  assert.equal(model.fileDescription({ ...profile.entries[0], id: "launchfile:@repo|launch/a.py" }), "");
  assert.equal(model.entityTitle(include), "navigation_launch.py");
  assert.equal(model.entityTitle(rsp), "/robot_state_publisher");
  assert.equal(model.entityDescription(rsp), "robot_state_publisher/robot_state_publisher");
  assert.equal(model.entityTitle(launchEntity({ kind: "include", label: "Include(50%_robot.launch.py)" })), "50%_robot.launch.py");
});

test("an unnamed node never shows its synthetic id and conditions read as written", () => {
  const model = load("src/launchModel.ts", {});
  const rviz = launchEntity({ label: "Node(navigation/rviz2@8507585b)", package: "rviz2", executable: "rviz2",
    namespace: "/navigation", conditions: ["IfCondition(LaunchConfiguration('use_namespace'))"],
    nodes: [{ id: "node:~rviz2|rviz2|/navigation|8507585b", label: "rviz2/rviz2@/navigation" }] });
  assert.equal(model.entityTitle(rviz), "rviz2");
  assert.equal(model.entityDescription(rviz), "rviz2/rviz2  unnamed in /navigation  if use_namespace");
  assert.equal(model.conditionText("UnlessCondition(LaunchConfiguration('slam'))"), "unless slam");
  assert.equal(model.conditionText({ kind: "UnlessCondition", expression: "headless" }), "unless headless");
  const amcl = launchEntity({ kind: "composable", label: "ComposableNode(amcl)", package: "nav2_amcl",
    nodes: [{ id: "node:/amcl", label: "/amcl" }] });
  assert.equal(model.entityDescription(amcl), "nav2_amcl");
});

test("a node whose condition is false in the profile is shown as not started and not counted", () => {
  const model = load("src/launchModel.ts", {});
  const gate = (outcome) => ({ mechanism: "IfCondition", expression: "IfCondition(LaunchConfiguration('use_namespace'))", outcome });
  const off = launchEntity({ label: "Node(rviz2@1)", gates: [gate("false")], nodes: [{ id: "node:~a", label: "x" }] });
  const on = launchEntity({ label: "Node(rviz2@2)", gates: [gate("true")], nodes: [{ id: "node:~b", label: "y" }] });
  assert.equal(model.started(off), false);
  assert.equal(model.started(on), true);
  assert.equal(model.notStartedReason(off), "Not started in this profile: if use_namespace is false.");
  assert.equal(model.notStartedReason(on), undefined);
  const profile = { id: "profile:p", label: "p", command: null, entries: [{ id: "f", label: "a.py", arg_declarations: [], args: [], children: [off, on] }] };
  assert.equal(model.profileDescription(profile), "1 node");
});

test("a frame names its joint, its broadcaster or its lookup, and the model only when two models disagree", () => {
  const model = load("src/launchModel.ts", {});
  const frame = (fields) => ({ id: "frame:imu_link", label: "imu_link", joint: null, joint_type: null, model: null,
    broadcasters: [], listeners: [], findings: [], children: [], ...fields });
  const waffle = frame({ joint: "imu_joint", joint_type: "fixed", model: "file:tb3|urdf/turtlebot3_waffle.urdf" });
  const create = frame({ joint: "imu_joint", joint_type: "fixed", model: "file:tb4|urdf/icreate/create3.urdf.xacro" });
  assert.equal(model.frameDescription(waffle, [waffle]), "fixed imu_joint");
  assert.equal(model.frameDescription(waffle, [waffle, create]), "fixed imu_joint  turtlebot3_waffle.urdf");
  const odom = frame({ broadcasters: ["/ekf", "/amcl", "/sim", "/bridge"].map((label) => ({ id: label, label })) });
  assert.equal(model.frameDescription(odom), "broadcast by /ekf, /amcl and 2 more");
  const lookup = frame({ listeners: [{ id: "node:/scan_filter", label: "/scan_filter" }] });
  assert.equal(model.frameDescription(lookup), "looked up by /scan_filter");
});

test("several robot models get a tree each, and code only frames come last", () => {
  const model = load("src/launchModel.ts", {});
  const frame = (id, fields) => ({ id: `frame:${id}`, label: id, joint: null, joint_type: null, model: null,
    broadcasters: [], listeners: [], findings: [], children: [], ...fields });
  const tb3 = "file:tb3|urdf/turtlebot3_waffle.urdf";
  const tb4 = "file:tb4|urdf/standard/turtlebot4.urdf.xacro";
  const base = frame("base_link", { model: tb3, children: [frame("imu_link", { model: tb3 }), frame("imu_link", { model: tb4 })] });
  const roots = [frame("base_footprint", { children: [base] }), frame("laser_frame", { listeners: [{ id: "n", label: "/n" }] })];
  const groups = model.frameGroups(roots);
  assert.equal(groups.map((g) => g.model).join(" "), `${tb3} ${tb4} `);
  assert.equal(groups[0].roots[0].children[0].children.length, 1);
  assert.equal(groups[1].roots[0].children[0].children[0].model, tb4);
  assert.equal(groups[2].roots[0].id, "frame:laser_frame");
  assert.equal(JSON.stringify(model.modelTitle(tb4)), JSON.stringify({ title: "turtlebot4.urdf.xacro", pkg: "tb4" }));
  const one = [frame("base_footprint", { children: [frame("base_link", { model: tb3 })] }), frame("laser_frame")];
  assert.equal(model.frameGroups(one).map((g) => g.roots.length).join(), "2");
});

test("a failed package whose errors the plan lists only counts them", () => {
  const model = load("src/migrationModel.ts", {});
  const row = { name: "a", status: "failed", all_errors_listed: true,
    errors: [{ file: null, line: null, message: "m", status: "listed", matches: [] }] };
  assert.equal(model.buildPackageDescription(row), "1 error");
});

test("the server is the setting, then the bundled binary, then rosio-lsp on PATH", () => {
  const { serverCommand } = load("src/serverPath.ts", { process, require });
  const bundled = (platform) => path.join("ext", "server", platform === "win32" ? "rosio-lsp.exe" : "rosio-lsp");
  const seen = [];
  const exists = (file) => { seen.push(file); return true; };
  assert.equal(serverCommand(" /opt/rosio-lsp ", "ext", "linux", exists), "/opt/rosio-lsp");
  assert.deepEqual(seen, []);
  assert.equal(serverCommand("", "ext", "linux", exists), bundled("linux"));
  assert.equal(serverCommand(undefined, "ext", "win32", exists), bundled("win32"));
  assert.equal(serverCommand("", "ext", "darwin", () => false), "rosio-lsp");
});
