// The atlas panel. It renders what the extension posts and posts back what the user chose.
// Text from the workspace is set with textContent only, never parsed as markup.
import { LAYERS, SEVERITY_ORDER, layerName, layerWords } from "../src/protocol";
import type { Candidate, Entity, Fact, Finding, Layer, Location, Relation, Severity, Summary } from "../src/protocol";
import type { ToHost, ToPanel } from "../src/panelMessages";
import { kindOf, labelOf, segments } from "../src/ids";

declare function acquireVsCodeApi(): { postMessage(message: ToHost): void };
const vscode = acquireVsCodeApi();

type Child = Node | string | null | undefined | false;

function h(tag: string, props: Record<string, string> = {}, ...children: Child[]): HTMLElement {
  const element = document.createElement(tag);
  for (const [key, value] of Object.entries(props)) element.setAttribute(key, value);
  for (const child of children) {
    if (child === null || child === undefined || child === false) continue;
    element.append(typeof child === "string" ? document.createTextNode(child) : child);
  }
  return element;
}

function link(text: string, message: ToHost, title = ""): HTMLElement {
  const anchor = h("a", { tabindex: "0", title }, text);
  const go = () => vscode.postMessage(message);
  anchor.addEventListener("click", go);
  anchor.addEventListener("keydown", (event) => {
    if (event.key === "Enter") go();
  });
  return anchor;
}

function button(text: string, message: ToHost, primary = false): HTMLElement {
  const element = h("button", primary ? { class: "primary" } : {}, text);
  element.addEventListener("click", () => vscode.postMessage(message));
  return element;
}

const SEVERITY_WORD: Record<Severity, string> = { error: "error", warn: "warn", info: "note" };
const SEVERITY_TITLE: Record<Severity, string> = { error: "Errors", warn: "Warnings", info: "Notes" };

const LAYER_WHAT: Record<Layer, { present: string; absent: string }> = {
  declared: { present: "launch, yaml, urdf, interfaces", absent: "no launch profile read" },
  coded: { present: "C++ and Python sources", absent: "no sources read" },
  observed: { present: "a live probe of the graph", absent: "run rosio probe on a running system" },
  recorded: { present: "rosbag2 recordings", absent: "add a bag root to rosio.yaml" },
};

function fileAndLine(uri: string, line: number): string {
  const name = decodeURIComponent(uri.slice(uri.lastIndexOf("/") + 1));
  return `${name}:${line + 1}`;
}

function plural(count: number, one: string, many = `${one}s`): string {
  return `${count} ${count === 1 ? one : many}`;
}

// ---------------------------------------------------------------------------
// the atlas overview

/** Rules that report on the reading of the workspace rather than on the system itself. */
const ABOUT_THE_READ = ["extract.", "resolution.", "launch.unloadable", "launch.raised"];

function aboutTheSystem(finding: Finding): boolean {
  return !ABOUT_THE_READ.some((prefix) => finding.rule.startsWith(prefix));
}

function oneFinding(summary: Summary, findings: Finding[]): string {
  const { error, warn, info } = summary.findings;
  if (!findings.length) return "The layers agree. Nothing in this atlas needs a look.";
  // The list below starts with the finding to read first, so the line above it only counts.
  return [error ? plural(error, "error") : "", warn ? plural(warn, "warning") : "", info ? plural(info, "note") : ""]
    .filter(Boolean)
    .join(", ");
}

/** What a layer that states no fact about the entity still says through a relation. */
function namedBy(entity: Entity, layer: Layer): string {
  const relation = (entity.relations ?? []).find((r) => r.layers.includes(layer));
  return relation ? relationPhrase(relation) : "says nothing about it";
}

/** The layers the whole atlas holds, which an entity card tells apart from its own. */
let atlasLayers: Layer[] = [];

/** On one entity an absent layer read the workspace and said nothing about this thing. */
function layersStrip(present: Layer[], entity?: Entity): HTMLElement {
  return h(
    "div",
    { class: "layers", role: "list", "aria-label": "Layers in this atlas" },
    ...LAYERS.map((layer) => {
      const has = present.includes(layer);
      const note = entity && atlasLayers.includes(layer) ? namedBy(entity, layer) : LAYER_WHAT[layer].absent;
      return h(
        "div",
        { class: `layer ${has ? "present" : "absent"}`, "data-layer": layer, role: "listitem", title: has ? LAYER_WHAT[layer].present : "" },
        h("span", { class: "name" }, layerName(layer)),
        !has && h("span", { class: "note" }, note),
      );
    }),
  );
}

function findingRow(finding: Finding): HTMLElement {
  const first = finding.locations[0];
  const row = h(
    "div",
    { class: "row", tabindex: "0", role: "button", "aria-expanded": "false" },
    h("span", { class: `sev ${finding.severity}` }, SEVERITY_WORD[finding.severity]),
    h(
      "div",
      { class: "what" },
      h("span", {}, layerWords(finding.title)),
      h("span", { class: "rule" }, finding.rule),
      h(
        "p",
        { class: "say" },
        layerWords(finding.say),
        " ",
        h("span", {}, `Fix: ${layerWords(finding.fix)}`),
      ),
    ),
    h(
      "span",
      { class: "end" },
      finding.fixable && h("button", { class: "fix" }, "Apply fix"),
      first && h("a", { class: "where", title: "Open the line" }, fileAndLine(first.uri, first.line)),
    ),
  );
  const open = () => first && vscode.postMessage({ type: "open", uri: first.uri, line: first.line });
  row.querySelector(".fix")?.addEventListener("click", (event) => {
    event.stopPropagation();
    vscode.postMessage({ type: "fix", finding });
  });
  row.querySelector(".where")?.addEventListener("click", (event) => {
    event.stopPropagation();
    open();
  });
  row.addEventListener("click", (event) => {
    if ((event as MouseEvent).detail > 1) return;
    row.setAttribute("aria-expanded", row.getAttribute("aria-expanded") === "true" ? "false" : "true");
  });
  row.addEventListener("dblclick", open);
  row.addEventListener("keydown", (event) => {
    if (event.key === "Enter") open();
    if (event.key === " ") {
      event.preventDefault();
      row.click();
    }
  });
  return row;
}

const ROWS_PER_GROUP = 8;
const FOLD_FROM = 4;

/** Many findings of one rule read as one line that opens, so a rule that fires often hides no other. */
function foldedRows(rows: Finding[]): HTMLElement[] {
  const byRule = new Map<string, Finding[]>();
  for (const finding of rows) byRule.set(finding.rule, [...(byRule.get(finding.rule) ?? []), finding]);
  const out: HTMLElement[] = [];
  for (const finding of rows) {
    const same = byRule.get(finding.rule) ?? [];
    if (same.length < FOLD_FROM) out.push(findingRow(finding));
    else if (same[0] === finding) {
      out.push(
        h(
          "details",
          { class: "fold" },
          h(
            "summary",
            { class: "row" },
            h("span", { class: `sev ${finding.severity}` }, SEVERITY_WORD[finding.severity]),
            h("div", { class: "what" }, h("span", {}, `${finding.title}, and ${same.length - 1} more like it`), h("span", { class: "rule" }, finding.rule)),
            h("span", {}),
          ),
          ...same.map(findingRow),
        ),
      );
    }
  }
  return out;
}

/** What the system is made of, in the order an engineer reads it. Internal kinds stay out. */
const LEDGER_KINDS: [string, string][] = [
  ["profile", "profiles"],
  ["node", "nodes"],
  ["topic", "topics"],
  ["service", "services"],
  ["action", "actions"],
  ["param", "parameters"],
  ["frame", "frames"],
  ["plugin", "plugins"],
  ["launchfile", "launch files"],
  ["pkg", "packages"],
  ["bag", "recordings"],
];

function findingGroups(findings: Finding[]): HTMLElement {
  // Inside one severity, what is wrong with the system reads before what went wrong reading it.
  const groups = SEVERITY_ORDER.map((severity) =>
    findings
      .filter((f) => f.severity === severity)
      .sort((a, b) => Number(aboutTheSystem(b)) - Number(aboutTheSystem(a))),
  );
  return h(
    "div",
    {},
    ...groups.map((rows, index) => {
      if (!rows.length) return null;
      const severity = SEVERITY_ORDER[index];
      const folded = foldedRows(rows);
      const list = h("div", { class: "rows" }, ...folded.slice(0, ROWS_PER_GROUP));
      const group = h(
        "div",
        { class: "group" },
        h("h2", {}, SEVERITY_TITLE[severity], h("span", { class: "count" }, String(rows.length))),
        list,
      );
      if (folded.length > ROWS_PER_GROUP) {
        const more = h("a", { class: "more", tabindex: "0" }, `Show all ${rows.length}`);
        more.addEventListener("click", () => {
          list.replaceChildren(...folded);
          more.remove();
        });
        group.append(more);
      }
      return group;
    }),
  );
}

/** What was read, on one line, and the way to what could not be. */
function holds(summary: Summary): HTMLElement {
  const counts = summary.counters.entities_by_kind ?? {};
  const kinds = LEDGER_KINDS.filter(([kind]) => counts[kind]).map(([kind, name]) => plural(counts[kind], name.slice(0, -1), name));
  const open = Object.values(summary.counters.unresolved ?? {}).reduce((a, b) => a + b, 0);
  return h(
    "p",
    { class: "holds" },
    kinds.join(" · "),
    open ? " · " : null,
    open ? link(`${plural(open, "name")} not resolved`, { type: "unresolved" }, "Show them in the side bar") : null,
  );
}

function readFrom(summary: Summary): string {
  const kinds = [...new Set(summary.roots.map((root) => root.kind))];
  return kinds.length ? `read from ${kinds.join(" and ")} ${kinds.length > 1 ? "trees" : "tree"}` : "";
}

function renderAtlas(summary: Summary, findings: Finding[], workspace: string): HTMLElement {
  if (!summary.loaded) {
    return h(
      "main",
      {},
      h(
        "div",
        { class: "empty" },
        h("h1", {}, "No atlas yet"),
        h("p", {}, "rosio reads the launch files, parameter yaml, urdf, interfaces and the C++ and Python sources of this workspace, and writes what it found to rosio-out/atlas.json."),
        summary.error ? h("p", { class: "muted" }, summary.error) : null,
        h("div", { class: "actions" }, button("Extract the atlas", { type: "extract" }, true)),
      ),
    );
  }
  const profiles = summary.profiles.length ? summary.profiles.join(", ") : "no profile";
  const fixable = findings.filter((finding) => finding.fixable).length;
  return h(
    "main",
    {},
    h(
      "div",
      { class: "title" },
      h(
        "div",
        {},
        h("h1", {}, workspace),
        h("p", { class: "finding" }, oneFinding(summary, findings)),
        h("p", { class: "muted" }, [profiles, readFrom(summary)].filter(Boolean).join(" · ")),
      ),
      h(
        "div",
        { class: "actions" },
        fixable > 0 && button(`Apply ${plural(fixable, "fix", "fixes")}`, { type: "fixAll" }, true),
        button("System graph", { type: "graph" }),
        button("Find", { type: "find" }),
        button("Extract again", { type: "extract" }),
      ),
    ),
    layersStrip(summary.layers),
    findings.length ? findingGroups(findings) : null,
    holds(summary),
  );
}

// ---------------------------------------------------------------------------
// one entity

const KIND_TITLE: Record<string, string> = {
  node: "Node",
  topic: "Topic",
  service: "Service",
  action: "Action",
  param: "Parameter",
  pparam: "Plugin parameter",
  frame: "Frame",
  pkg: "Package",
  file: "File",
  exe: "Executable",
  launchfile: "Launch file",
  launchent: "Launch action",
  profile: "Profile",
  plugin: "Plugin",
  pinst: "Plugin instance",
  type: "Interface type",
  bag: "Recording",
  bagtopic: "Recorded topic",
};

/** How a parameter value got where it is, in words. */
const MECHANISM: Record<string, string> = {
  launch_rewritten_yaml: "yaml rewritten by the launch file",
  launch_inline_dict: "inline parameters of the launch file",
  launch_yaml: "parameters file passed by the launch file",
  yaml: "parameters file",
  code_default: "default in the source",
  ros_args: "command line argument",
  python_override: "rewritten by a launcher script",
  param_set_runtime: "ros2 param set after start",
};

function words(text: string): string {
  return text.replace(/_/g, " ");
}

const ID_KINDS = new Set(["file", "exe", "pkg", "sym", "node", "topic", "service", "action", "frame", "type", "launchfile", "launchent", "plugin", "profile", "param"]);

/** A value as a person reads it: ids by their names, lists joined, other shapes as JSON. */
function render(value: unknown): string {
  if (value === undefined) return "";
  if (Array.isArray(value)) return value.map(render).join(", ");
  if (typeof value === "string") return ID_KINDS.has(kindOf(value)) ? readableId(value) : value;
  return JSON.stringify(value);
}

function readableId(id: string): string {
  const parts = segments(id);
  switch (kindOf(id)) {
    case "file":
    case "launchfile":
    case "exe":
      return parts.join("/").replace(/^@repo\//, "");
    case "type":
      return parts.join("/");
    default:
      return labelOf(id);
  }
}

function flatten(values: Record<string, unknown>, prefix = "", out: Record<string, unknown> = {}): Record<string, unknown> {
  for (const [key, value] of Object.entries(values)) {
    if (key === "volatile") continue;
    const path = prefix ? `${prefix}.${key}` : key;
    if (value && typeof value === "object" && !Array.isArray(value)) flatten(value as Record<string, unknown>, path, out);
    else out[path] = value;
  }
  return out;
}

/** The owner a parameter belongs to, from its id. A synthetic owner has no name to show. */
function ownerOf(entity: Entity): string {
  if (entity.kind !== "param") return "";
  const body = entity.id.slice(entity.id.indexOf(":") + 1);
  const owner = decodeURIComponent(body.slice(0, body.lastIndexOf("|")));
  return owner.startsWith("~") ? "an unnamed node" : owner;
}

function winner(entity: Entity): Candidate | undefined {
  return (entity.candidates ?? []).find((candidate) => candidate.wins);
}

function how(candidate: Candidate): string {
  if (candidate.note?.startsWith("SetParameter")) return "SetParameter for the whole launch group";
  return MECHANISM[candidate.mechanism] ?? words(candidate.mechanism);
}

function sourceText(candidate: Candidate): string {
  const source = candidate.source;
  if (!source) return "";
  const path = source.file.replace(/^file:/, "").replace("|", "/");
  return source.line_hint ? `${path}:${source.line_hint}` : path;
}

function openLink(text: string, at: Location | undefined): HTMLElement {
  if (!at) return h("span", {}, text);
  return link(text, { type: "open", uri: at.uri, line: at.line }, decodeURIComponent(at.uri));
}

function lead(entity: Entity): string {
  const chosen = winner(entity);
  if (entity.kind === "param" && chosen) {
    const losers = (entity.candidates ?? []).filter((candidate) => !candidate.wins).length;
    const shadowed = losers ? ` It wins over ${plural(losers, "other value")}.` : "";
    return `${render(chosen.value)}, from ${how(chosen)} in ${sourceText(chosen)}.${shadowed}`;
  }
  const ignored = entity.kind === "param" && !chosen && entity.candidates?.find((candidate) => candidate.note)?.note;
  if (ignored) return `No value wins: ${ignored}.`;
  if (entity.kind === "node") {
    const described = describeNode(entity);
    if (described) return described;
  }
  if (entity.conflicts.length) return `The layers disagree on ${entity.conflicts.join(", ")}.`;
  if (entity.layers.length > 1) return `${entity.layers.map(layerName).join(", ")} agree on it.`;
  if (entity.layers.length === 1) return `Only ${layerName(entity.layers[0])} states it.`;
  const relation = (entity.relations ?? []).find((r) => r.layers.length);
  if (relation) return `No layer states a fact about it. It is known by name, ${relationPhrase(relation)}.`;
  return "No layer states a fact about it, it is known by name only.";
}

function firstEffective(entity: Entity): Record<string, unknown> {
  const profile = Object.values(entity.effective ?? {})[0] ?? {};
  return Object.fromEntries(Object.entries(profile).map(([key, cell]) => [key, cell.value]));
}

/** What a node is, from its own facts and edges, in one sentence. */
function describeNode(entity: Entity): string {
  const facts = firstEffective(entity);
  const relations = entity.relations ?? [];
  const component = facts.form === "composable_node";
  const what = component
    ? `A component${facts.plugin_class ? ` (${render(facts.plugin_class)})` : ""} of ${render(facts.pkg)}`
    : facts.pkg
      ? `A process of ${render(facts.pkg)}${facts.executable ? `, executable ${render(facts.executable)}` : ""}`
      : "";
  if (!what) return "";
  const starter = relations.find((r) => r.kind === "launches" && r.direction === "in");
  const manager = relations.find((r) => r.kind === "manages_lifecycle" && r.direction === "in");
  const parts = [what];
  if (starter) parts.push(`started by ${relationLabel(starter).split(",")[0]}`);
  if (manager) parts.push(`lifecycle managed by ${manager.label || manager.id}`);
  return `${parts.join(", ")}.`;
}

function parameterChain(entity: Entity): HTMLElement | null {
  const candidates = entity.candidates ?? [];
  if (!candidates.length) return null;
  return h(
    "section",
    {},
    h("h2", {}, candidates.length > 1 ? "Every value that was set, the winner first" : "Where the value comes from"),
    h(
      "table",
      {},
      h("thead", {}, h("tr", {}, h("th", {}, "profile"), h("th", {}, "value"), h("th", {}, "set by"), h("th", {}, "where"), h("th", {}, ""))),
      h(
        "tbody",
        {},
        ...candidates
          .slice()
          .sort((a, b) => a.profile.localeCompare(b.profile) || a.rank - b.rank)
          .map((candidate) =>
            h(
              "tr",
              { class: candidate.wins ? "wins" : "loses" },
              h("td", {}, candidate.profile.replace(/^profile:/, "")),
              h("td", { class: "value" }, render(candidate.value)),
              h("td", {}, how(candidate)),
              h("td", { class: "value" }, openLink(sourceText(candidate), candidate.at)),
              h("td", {}, candidate.wins ? "wins" : LOST[candidate.shadow_kind ?? ""] ?? candidate.note ?? "loses"),
            ),
          ),
      ),
    ),
  );
}

/** Why a value lost, in the words of the person who wrote it. */
const LOST: Record<string, string> = {
  value_changed: "overridden",
  restated_identical: "same value, set again",
  type_changed: "overridden with another type",
  not_read_in_profile: "not read in this profile",
  inert_plugin_block: "plugin not selected",
};

/** Fields down the side, layers across, the cells two layers disagree on tinted. */
function layerLedger(entity: Entity): HTMLElement | null {
  const profiles = [...new Set([...entity.profiles, ...entity.facts.map((fact) => fact.profile)])]
    .filter((profile) => profile !== "*").sort();
  const sections = (profiles.length ? profiles : ["*"])
    .map((profile) => profileLedger(entity, profile, profiles.length > 1)).filter((section) => section !== null);
  return sections.length ? h("div", { class: "ledger" }, ...sections) : null;
}

function profileLedger(entity: Entity, profile: string, showProfile: boolean): HTMLElement | null {
  const byLayer = new Map<Layer, Record<string, unknown>>();
  for (const scope of profile === "*" ? ["*"] : ["*", profile]) {
    for (const fact of entity.facts.filter((fact) => fact.profile === scope && fact.confidence !== "unknown")) {
      const flat = flatten(fact.values ?? {});
      byLayer.set(fact.layer, { ...(byLayer.get(fact.layer) ?? {}), ...flat });
    }
  }
  const layers = LAYERS.filter((layer) => byLayer.has(layer));
  // A parameter's single layer says what its chain already says.
  if (entity.kind === "param" && layers.length < 2 && !showProfile) return null;
  const fields = [...new Set(layers.flatMap((layer) => Object.keys(byLayer.get(layer) ?? {})))].sort();
  if (!fields.length) return null;
  const conflicts = new Set(fields.filter((field) => {
    const values = layers.map((layer) => byLayer.get(layer)?.[field]).filter((value) => value !== undefined);
    return new Set(values.map((value) => JSON.stringify(value))).size > 1;
  }));
  // Disagreements first and open; a ledger where the layers agree is reference, closed.
  fields.sort((a, b) => Number(conflicts.has(b)) - Number(conflicts.has(a)));
  const title = showProfile ? `Layer by layer in ${profile.replace(/^profile:/, "")}` : layers.length > 1 ? "Layer by layer" : `What ${layerName(layers[0])} says`;
  const props: Record<string, string> = conflicts.size ? { open: "" } : {};
  return h(
    "details",
    props,
    h("summary", {}, h("h2", {}, title, h("span", { class: "count" }, conflicts.size ? plural(conflicts.size, "disagreement") : plural(fields.length, "field")))),
    h(
      "table",
      {},
      h("thead", {}, h("tr", {}, h("th", {}, ""), ...layers.map((layer) => h("th", { class: "layer-head", "data-layer": layer }, layerName(layer))))),
      h(
        "tbody",
        {},
        ...fields.slice(0, 60).map((field) =>
          h(
            "tr",
            {},
            h("td", {}, words(field)),
            ...layers.map((layer) => {
              const value = byLayer.get(layer)?.[field];
              if (value === undefined || value === "") return h("td", { class: "dash" }, "–");
              return h("td", { class: `value${conflicts.has(field) ? " conflict" : ""}` }, render(value));
            }),
          ),
        ),
      ),
    ),
  );
}

/** Edge kinds as a heading, by direction. Kinds not named here read as their own words. */
const RELATION_TITLE: Record<string, [string, string]> = {
  has_param: ["Parameters", "Parameter of"],
  publishes: ["Publishes", "Published by"],
  subscribes: ["Subscribes to", "Subscribed by"],
  serves: ["Serves", "Served by"],
  calls: ["Calls", "Called by"],
  launches: ["Starts", "Started by"],
  manages_lifecycle: ["Manages the lifecycle of", "Lifecycle managed by"],
  instantiates: ["Instances", "Instance of"],
  broadcasts_tf: ["Broadcasts", "Broadcast by"],
  listens_tf: ["Looks up", "Looked up by"],
  tf_parent: ["Parent frame", "Child frames"],
  remaps: ["Remaps", "Remapped by"],
  includes: ["Includes", "Included by"],
  contains: ["Contains", "Part of"],
  defines: ["Defines", "Defined in"],
  provides_plugin: ["Provides", "Provided by"],
  runs: ["Runs", "Run by"],
  gates: ["Gates", "Gated by"],
  declares_param: ["Declares", "Declared by"],
  reads_param: ["Reads", "Read by"],
  uses_type: ["Type", "Used by"],
};

function relationTitle(kind: string, direction: "in" | "out"): string {
  const named = words(kind).replace(/^./, (first) => first.toUpperCase());
  const titles = RELATION_TITLE[kind] ?? [named, `${named} (in)`];
  return direction === "out" ? titles[0] : titles[1];
}

/** One relation as a phrase: "looked up by /scan_filter". */
function relationPhrase(relation: Relation): string {
  return `${relationTitle(relation.kind, relation.direction).toLowerCase()} ${relation.label || relation.id}`;
}

const RELATION_ORDER = Object.keys(RELATION_TITLE);
const RELATIONS_SHOWN = 12;

function relationGroups(entity: Entity): HTMLElement | null {
  const relations = (entity.relations ?? []).filter((r) => !(r.kind === "has_param" && r.direction === "out"));
  if (!relations.length) return null;
  const groups = new Map<string, Relation[]>();
  for (const relation of relations) {
    const key = `${relation.kind}|${relation.direction}`;
    groups.set(key, [...(groups.get(key) ?? []), relation]);
  }
  const ordered = [...groups.entries()].sort(([a], [b]) => {
    const rank = (key: string) => RELATION_ORDER.indexOf(key.split("|")[0]);
    return rank(a) - rank(b) || a.localeCompare(b);
  });
  return h(
    "section",
    { class: "relations" },
    ...ordered.map(([key, rows]) => {
      const [kind, direction] = key.split("|");
      const title = relationTitle(kind, direction as "in" | "out");
      const sorted = rows.slice().sort((a, b) => (a.label || a.id).localeCompare(b.label || b.id));
      const list = h("div", { class: "chips" }, ...sorted.slice(0, RELATIONS_SHOWN).map(relationLink));
      const group = h("div", { class: "relation" }, h("h2", {}, title, h("span", { class: "count" }, String(rows.length))), list);
      if (rows.length > RELATIONS_SHOWN) {
        const more = h("a", { class: "more", tabindex: "0" }, `Show all ${rows.length}`);
        more.addEventListener("click", () => {
          list.replaceChildren(...sorted.map(relationLink));
          more.remove();
        });
        group.append(more);
      }
      return group;
    }),
  );
}

/** A launch action reads as its file and its last step: `navigation_launch.py, ComposableNode`. */
function relationLabel(relation: Relation): string {
  if (relation.otherKind === "launchent") {
    const parts = relation.id.slice(relation.id.indexOf(":") + 1).split("|").map(decodeURIComponent);
    const file = parts[1]?.split("/").pop() ?? "";
    const step = parts[parts.length - 1]?.replace(/\(.*$/, "") ?? "";
    return `${file}, ${step}`;
  }
  return relation.label || relation.id;
}

/** The QoS one endpoint stated, in the words `ros2 topic info -v` uses, per layer. */
function qosText(relation: Relation): string {
  const parts = new Set<string>();
  for (const fact of relation.facts ?? []) {
    const qos = fact.values?.qos as Record<string, unknown> | undefined;
    if (!qos) continue;
    const stated = ["reliability", "durability", "history", "depth"]
      .map((key) => (qos[key] === null || qos[key] === undefined ? "" : key === "depth" ? `depth ${qos[key]}` : String(qos[key])))
      .filter(Boolean);
    if (stated.length) parts.add(`${stated.join(", ")} (${layerName(fact.layer)})`);
  }
  return [...parts].join("; ");
}

function relationLink(relation: Relation): HTMLElement {
  const anchor = link(relationLabel(relation), { type: "entity", id: relation.id }, relation.id);
  const qos = qosText(relation);
  return qos ? h("div", { class: "endpoint" }, anchor, h("span", { class: "muted" }, ` ${qos}`)) : anchor;
}

/** A node's parameters with the value each one ends up with, the most useful table on its card. */
function parameterTable(rows: Relation[]): HTMLElement {
  const sorted = rows.slice().sort((a, b) => a.label.localeCompare(b.label));
  const body = (shown: Relation[]) =>
    shown.map((row) =>
      h("tr", {}, h("td", {}, relationLink(row)), h("td", { class: "value" }, row.value === undefined ? "–" : render(row.value))),
    );
  const tbody = h("tbody", {}, ...body(sorted.slice(0, PARAMETERS_SHOWN)));
  const section = h(
    "section",
    { class: "parameters" },
    h("h2", {}, "Parameters", h("span", { class: "count" }, String(rows.length))),
    h("table", {}, h("thead", {}, h("tr", {}, h("th", {}, "name"), h("th", {}, "effective value"))), tbody),
  );
  if (rows.length > PARAMETERS_SHOWN) {
    const more = h("a", { class: "more", tabindex: "0" }, `Show all ${rows.length}`);
    more.addEventListener("click", () => {
      tbody.replaceChildren(...body(sorted));
      more.remove();
    });
    section.append(more);
  }
  return section;
}

const PARAMETERS_SHOWN = 14;

function effectiveFigures(entity: Entity): HTMLElement | null {
  const effective = entity.effective ?? {};
  const profiles = Object.keys(effective);
  if (entity.kind !== "param" || !profiles.length) return null;
  return h(
    "div",
    { class: "figure" },
    ...profiles.map((profile) => {
      const cell = effective[profile]?.value as { value?: unknown } | undefined;
      const value = cell?.value ?? Object.values(effective[profile] ?? {})[0]?.value;
      return h(
        "div",
        {},
        h("div", { class: "label" }, profile === "*" ? "effective" : `effective in ${profile.replace(/^profile:/, "")}`),
        h("div", { class: "value" }, render(value)),
      );
    }),
  );
}

/** Values a layer measured over a window (rates, counts, ages). They never enter a comparison,
 * so they are shown apart from the layer by layer table, with the layer that measured them. */
function measured(entity: Entity): HTMLElement | null {
  const rows: [string, string, unknown][] = [];
  for (const fact of entity.facts as Fact[]) {
    const volatile = (fact.values as Record<string, unknown>)?.volatile;
    if (!volatile || typeof volatile !== "object") continue;
    for (const [field, value] of Object.entries(flatten(volatile as Record<string, unknown>))) {
      rows.push([fact.layer, field, value]);
    }
  }
  if (!rows.length) return null;
  rows.sort((a, b) => a[0].localeCompare(b[0]) || a[1].localeCompare(b[1]));
  return h(
    "section",
    {},
    h("h2", {}, "Measured"),
    h(
      "table",
      {},
      h("thead", {}, h("tr", {}, h("th", {}, "from"), h("th", {}, "what"), h("th", { class: "num" }, "value"))),
      h(
        "tbody",
        {},
        ...rows.slice(0, 40).map(([layer, field, value]) =>
          h("tr", {}, h("td", {}, layerName(layer)), h("td", {}, words(field)), h("td", { class: "num value" }, render(value))),
        ),
      ),
    ),
  );
}

function parametersOf(entity: Entity): HTMLElement | null {
  const rows = (entity.relations ?? []).filter((r) => r.kind === "has_param" && r.direction === "out");
  return rows.length ? parameterTable(rows) : null;
}

function renderEntity(entity: Entity, findings: Finding[]): HTMLElement {
  const sites = entity.sites ?? [];
  const about = findings.filter((finding) => entity.findings.includes(finding.id));
  const owner = ownerOf(entity);
  return h(
    "main",
    {},
    h(
      "div",
      { class: "title" },
      h(
        "div",
        {},
        h("div", { class: "kind" }, owner ? `${KIND_TITLE.param} of ${owner}` : (KIND_TITLE[entity.kind] ?? entity.kind)),
        h("h1", {}, entity.label),
        h("p", { class: "finding" }, lead(entity)),
      ),
      h(
        "div",
        { class: "actions" },
        ["node", "topic", "service", "action"].includes(entity.kind) && button("Show in graph", { type: "graph", id: entity.id }),
        button("Back to the atlas", { type: "atlas" }),
        button("Find", { type: "find" }),
      ),
    ),
    layersStrip(entity.layers, entity),
    effectiveFigures(entity),
    about.length ? h("section", {}, h("h2", {}, "Findings"), h("div", { class: "rows" }, ...about.map(findingRow))) : null,
    parameterChain(entity),
    parametersOf(entity),
    measured(entity),
    relationGroups(entity),
    layerLedger(entity),
    sites.length
      ? h(
          "section",
          { class: "sites" },
          h("h2", {}, "Where it is written"),
          ...sites.map((site) =>
            link(
              `${fileAndLine(site.uri, site.line)}${site.approximate ? " (found by name)" : ""}`,
              { type: "open", uri: site.uri, line: site.line },
              decodeURIComponent(site.uri),
            ),
          ),
        )
      : null,
  );
}

// ---------------------------------------------------------------------------

window.addEventListener("message", (event: MessageEvent<ToPanel>) => {
  const message = event.data;
  const root = document.body;
  if (message.type === "atlas") root.replaceChildren(renderAtlas(message.summary, message.findings, message.workspace));
  if (message.type === "entity") {
    atlasLayers = message.layers;
    root.replaceChildren(renderEntity(message.entity, message.findings));
  }
  root.querySelector<HTMLElement>(".row, button.primary")?.focus({ preventScroll: true });
});

document.addEventListener("keydown", (event) => {
  if (event.key !== "ArrowDown" && event.key !== "ArrowUp") return;
  const rows = [...document.querySelectorAll<HTMLElement>(".row")];
  const at = rows.indexOf(document.activeElement as HTMLElement);
  if (at < 0) return;
  event.preventDefault();
  rows[Math.max(0, Math.min(rows.length - 1, at + (event.key === "ArrowDown" ? 1 : -1)))]?.focus();
});

vscode.postMessage({ type: "ready" });
