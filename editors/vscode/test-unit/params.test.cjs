const { test } = require("node:test");
const assert = require("node:assert/strict");
const { buildSync } = require("esbuild");
const vm = require("node:vm");
const path = require("node:path");

function model() {
  const result = buildSync({ entryPoints: [path.join(__dirname, "../src/paramsModel.ts")], bundle: true, platform: "node", format: "cjs", write: false });
  const context = { module: { exports: {} } };
  vm.runInNewContext(result.outputFiles[0].text, context);
  return context.module.exports;
}

const at = (line) => ({ uri: "file:///ws/src/rover_bringup/config/rover.yaml", line });
const range = {
  id: "param:/scan_filter|max_range", name: "max_range", state: "effective", layers: ["declared", "coded"], findings: 0,
  value: 10.0, valueType: "float", source: { file: "rover_bringup/launch/bringup.launch.py", at: { uri: "file:///b.py", line: 27 } },
  candidates: [
    { rank: 0, wins: true, value: 10.0, valueType: "float", mechanism: "launch_inline_dict", shadowKind: null, source: { file: "rover_bringup/launch/bringup.launch.py", at: { uri: "file:///b.py", line: 27 } } },
    { rank: 1, wins: false, value: 8.0, valueType: "float", mechanism: "launch_params_file", shadowKind: "value_changed", source: { file: "rover_bringup/config/rover.yaml", at: at(3) } },
  ],
};

test("values read the way ROS writes them", () => {
  const { formatValue } = model();
  assert.equal(formatValue(10, "float"), "10.0");
  assert.equal(formatValue(0.35, "float"), "0.35");
  assert.equal(formatValue(3, "int"), "3");
  assert.equal(formatValue(true, "bool"), "true");
  assert.equal(formatValue("map", "string"), '"map"');
  assert.equal(formatValue([1, 2], "int_array"), "[1, 2]");
  assert.equal(formatValue(undefined, null), "");
  assert.equal(formatValue('<?xml version="1.0"?>' + "x".repeat(2000), "string"), "XML, 2.0 KB");
});

test("a parameter row says its value, and a beaten line says so", () => {
  const m = model();
  assert.equal(m.paramDescription(range), "10.0");
  assert.equal(m.paramIcon(range), "layers");
  assert.equal(m.candidateDescription(range.candidates[0]), "wins at bringup.launch.py:28");
  assert.equal(m.candidateDescription(range.candidates[1]), "overridden at rover.yaml:4");
  assert.equal(m.mechanismName(range.candidates[1]), "parameters file");
});

test("a parameter nobody sets in this profile is not given a value", () => {
  const m = model();
  const loose = { ...range, value: undefined, valueType: undefined, source: undefined, candidates: [], findings: 1, state: "owner_name_mismatch" };
  assert.equal(m.paramDescription(loose), "no node of this name runs");
  assert.equal(m.paramIcon(loose), "warning");
});

test("a node line counts what was overridden", () => {
  const m = model();
  const quiet = { ...range, id: "param:/scan_filter|min_range", name: "min_range", candidates: [range.candidates[0]] };
  assert.equal(m.nodeDescription({ id: "node:/scan_filter", label: "/scan_filter", params: [range, quiet] }), "2 parameters, 1 overridden");
  assert.equal(m.nodeDescription({ id: "node:/a", label: "/a", params: [quiet] }), "1 parameter");
  assert.equal(m.nodeDescription({ id: "node:/a", label: "/a", params: [{ ...quiet, findings: 2 }] }), "1 parameter, 2 findings");
  const orphan = { ...quiet, state: "owner_name_mismatch" };
  assert.equal(m.nodeDescription({ id: "node:/odometry", label: "/odometry", params: [orphan] }), "configured, never launched");
});

test("search matches node and name together", () => {
  const m = model();
  const tree = { profile: "profile:rover", nodes: [{ id: "node:/scan_filter", label: "/scan_filter", params: [range] }] };
  const picks = m.paramPicks(tree);
  assert.equal(picks.length, 1);
  assert.equal(picks[0].label, "max_range");
  assert.equal(picks[0].description, "10.0");
  assert.equal(picks[0].detail, "/scan_filter · rover_bringup/launch/bringup.launch.py");
});
