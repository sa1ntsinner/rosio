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

const { layout } = load("graphLayout.ts");
const { flow, shown, qosText, matches } = load("graphModel.ts");

/** Values built inside the vm context carry its prototypes. */
const plain = (value) => JSON.parse(JSON.stringify(value));
const box = (id) => ({ id, width: 100, height: 20 });

test("data flows left to right through the topic", () => {
  const placed = layout([box("node:/b"), box("topic:/t"), box("node:/a")], [
    { from: "node:/a", to: "topic:/t" },
    { from: "topic:/t", to: "node:/b" },
  ]).boxes;
  assert.ok(placed.get("node:/a").x < placed.get("topic:/t").x);
  assert.ok(placed.get("topic:/t").x < placed.get("node:/b").x);
});

test("a cycle is broken by one backward flow and every box is placed once", () => {
  const result = layout([box("a"), box("b"), box("c")], [
    { from: "a", to: "b" },
    { from: "b", to: "c" },
    { from: "c", to: "a" },
  ]);
  assert.equal(result.backward.size, 1);
  assert.deepEqual([...result.boxes.keys()].sort(), ["a", "b", "c"]);
  assert.deepEqual([...result.backward], [2]);
});

test("the same input gives the same picture whatever order it arrives in", () => {
  const boxes = ["a", "b", "c", "d", "e"].map(box);
  const flows = [
    { from: "a", to: "c" },
    { from: "b", to: "c" },
    { from: "c", to: "d" },
    { from: "c", to: "e" },
  ];
  const one = layout(boxes, flows);
  const two = layout([...boxes].reverse(), flows);
  for (const id of ["a", "b", "c", "d", "e"]) assert.deepEqual(plain(one.boxes.get(id)), plain(two.boxes.get(id)));
});

test("barycentre ordering uncrosses two parallel chains", () => {
  const result = layout(["a1", "b1", "a2", "b2"].map(box), [
    { from: "a1", to: "b2" },
    { from: "b1", to: "a2" },
  ]);
  const y = (id) => result.boxes.get(id).y;
  assert.equal(y("a1") < y("b1"), y("b2") < y("a2"));
});

test("boxes nothing connects to fill columns instead of one tall stack", () => {
  const boxes = Array.from({ length: 20 }, (_, i) => box(`node:/n${String(i).padStart(2, "0")}`));
  const placed = layout(boxes, []).boxes;
  const columns = new Set([...placed.values()].map((b) => b.x));
  assert.equal(columns.size, 3);
});

const graph = {
  profile: "profile:rover",
  nodes: [{ id: "node:/a", label: "/a" }, { id: "node:/b", label: "/b" }],
  channels: [
    { id: "topic:/scan", label: "/scan", type: "sensor_msgs/msg/LaserScan" },
    { id: "topic:/tf", label: "/tf" },
    { id: "topic:/lonely", label: "/lonely" },
  ],
  links: [
    { id: "1", src: "node:/a", dst: "topic:/scan", kind: "publishes", wrapper: false },
    { id: "2", src: "node:/b", dst: "topic:/scan", kind: "subscribes", wrapper: false, qos: { reliability: "reliable", depth: 10 } },
    { id: "3", src: "node:/b", dst: "topic:/tf", kind: "subscribes", wrapper: true },
  ],
  truncated: false,
};

test("a subscription flows from the topic to the subscriber", () => {
  assert.deepEqual(plain(flow(graph.links[1])), { from: "topic:/scan", to: "node:/b" });
  assert.deepEqual(plain(flow(graph.links[0])), { from: "node:/a", to: "topic:/scan" });
});

test("wrapper links and the channels only they use stay out unless asked for", () => {
  const quiet = shown(graph, false);
  assert.equal(quiet.hidden, 1);
  assert.ok(!quiet.ids.has("topic:/tf"));
  assert.ok(quiet.ids.has("topic:/lonely"), "a channel with no links at all is still shown");
  assert.ok(shown(graph, true).ids.has("topic:/tf"));
});

test("qos reads like a parameters file and states nothing it was not told", () => {
  assert.equal(qosText({ reliability: "reliable", depth: 10 }), "reliable, depth 10");
  assert.equal(qosText({ preset: "sensor_data" }), "sensor_data preset");
  assert.equal(qosText(undefined), "");
});

test("search matches a name or a message type", () => {
  assert.deepEqual([...matches(graph, "laserscan")], ["topic:/scan"]);
  assert.deepEqual([...matches(graph, " ")], []);
});
