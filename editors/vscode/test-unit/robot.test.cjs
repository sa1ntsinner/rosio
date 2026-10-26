const { test } = require("node:test");
const assert = require("node:assert/strict");
const { buildSync } = require("esbuild");
const vm = require("node:vm");
const path = require("node:path");

function load(file) {
  const result = buildSync({ entryPoints: [path.join(__dirname, "../webview", file)], bundle: true, platform: "node", format: "cjs", write: false });
  const context = { module: { exports: {} } };
  vm.runInNewContext(result.outputFiles[0].text, context);
  return context.module.exports;
}

const { place, project, rpyRotation, rotate, outline } = load("robotModel.ts");

const close = (actual, expected) => actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < 1e-9, `${actual} vs ${expected}`));
const joint = (parent, child, xyz, rpy) => ({ id: `j:${child}`, name: child, type: "fixed", parent, child, xyz, rpy });

test("a yaw of a quarter turn takes x onto y", () => {
  close(rotate(rpyRotation([0, 0, Math.PI / 2]), [1, 0, 0]), [0, 1, 0]);
});

test("a child origin is taken in its parent's rotated frame", () => {
  const { poses, unplaced } = place({
    roots: ["frame:base"],
    links: [{ id: "frame:base" }, { id: "frame:arm" }, { id: "frame:tool" }],
    joints: [
      joint("frame:base", "frame:arm", [1, 0, 0], [0, 0, Math.PI / 2]),
      joint("frame:arm", "frame:tool", [1, 0, 0], [0, 0, 0]),
    ],
  });
  close(poses.get("frame:arm").t, [1, 0, 0]);
  close(poses.get("frame:tool").t, [1, 1, 0]);
  assert.deepEqual([...unplaced], []);
});

test("a joint without a readable origin leaves its child unplaced instead of at zero", () => {
  const { poses, unplaced } = place({
    roots: ["frame:base"],
    links: [{ id: "frame:base" }, { id: "frame:lidar" }, { id: "frame:lost" }],
    joints: [{ ...joint("frame:base", "frame:lidar", undefined, [0, 0, 0]), unreadable: ["xyz"] }],
  });
  assert.equal(poses.has("frame:lidar"), false);
  assert.deepEqual([...unplaced], ["frame:lidar", "frame:lost"]);
});

test("the front view puts y to the right and z up", () => {
  close(project([0, 1, 0], { yaw: 0, pitch: 0 }), [1, -0, 0]);
  close(project([0, 0, 1], { yaw: 0, pitch: 0 }), [0, -1, 0]);
});

test("a box outline is twelve edges around its origin, and a mesh draws nothing", () => {
  const edges = outline({ geometry: { type: "box", size: [2, 2, 2] }, origin: { xyz: [1, 0, 0] } });
  assert.equal(edges.length, 12);
  const xs = edges.flat().map((point) => point[0]);
  assert.equal(Math.min(...xs), 0);
  assert.equal(Math.max(...xs), 2);
  assert.equal(outline({ geometry: { type: "mesh", filename: "package://a/b.stl" } }).length, 0);
  assert.equal(outline({ geometry: { type: "box" }, unreadable: ["box.size"] }).length, 0);
});
