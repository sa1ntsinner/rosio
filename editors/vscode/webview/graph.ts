// The system graph. It draws what the extension posts and posts back what the user chose.
// Text from the workspace is set with textContent only, never parsed as markup.
import type { GraphData, GraphSite } from "../src/protocol";
import type { ToGraph, ToGraphHost } from "../src/graphMessages";
import { layout, type Placed } from "./graphLayout";
import { flow, matches, qosText, shown, VERB, type Link } from "./graphModel";

declare function acquireVsCodeApi(): {
  postMessage(message: ToGraphHost): void;
  getState(): { wrappers?: boolean } | undefined;
  setState(state: { wrappers?: boolean }): void;
};
const vscode = acquireVsCodeApi();
const SVG = "http://www.w3.org/2000/svg";

type Row = GraphData["nodes"][number] | GraphData["channels"][number];

let current: ToGraph | undefined;
let wrappers = vscode.getState()?.wrappers ?? false;
let selected: string | null = null;
let query = "";
let view = { x: 0, y: 0, k: 1 };

function h(tag: string, props: Record<string, string> = {}, ...children: (Node | string | null | false)[]): HTMLElement {
  const element = document.createElement(tag);
  for (const [key, value] of Object.entries(props)) element.setAttribute(key, value);
  for (const child of children) if (child) element.append(child);
  return element;
}

function s<K extends keyof SVGElementTagNameMap>(tag: K, props: Record<string, string | number> = {}): SVGElementTagNameMap[K] {
  const element = document.createElementNS(SVG, tag);
  for (const [key, value] of Object.entries(props)) element.setAttribute(key, String(value));
  return element;
}

function post(message: ToGraphHost): void {
  vscode.postMessage(message);
}

const measure = document.createElement("canvas").getContext("2d")!;
function textWidth(text: string, font: string): number {
  measure.font = font;
  return measure.measureText(text).width;
}

const NODE_H = 34;
const CHANNEL_H = 44;
const PAD = 14;

function fonts(): { node: string; channel: string; type: string } {
  const style = getComputedStyle(document.body);
  const text = style.getPropertyValue("--vscode-font-family") || "sans-serif";
  const code = style.getPropertyValue("--vscode-editor-font-family") || "monospace";
  return { node: `600 13px ${text}`, channel: `12px ${code}`, type: `11px ${text}` };
}

function shortType(type: string | undefined): string {
  return type ? type.replace("/msg/", "/").replace("/srv/", "/").replace("/action/", "/") : "";
}

function render(): void {
  const message = current;
  const root = document.querySelector("main")!;
  root.replaceChildren();
  if (!message) return;
  const { graph, profiles, focus } = message;
  const visible = shown(graph, wrappers);
  const rows = new Map<string, Row>([...graph.nodes, ...graph.channels].map((row) => [row.id, row]));
  const ids = [...visible.ids].filter((id) => rows.has(id));

  root.append(toolbar(graph, profiles, focus, visible.hidden));
  if (!ids.length) {
    root.append(
      h("div", { class: "empty" },
        h("p", {}, focus ? "Nothing is connected to this entity in this profile." : "No nodes in this profile yet."),
        h("p", { class: "muted" }, "The graph is drawn from the atlas. Extract it, then open the graph again."),
      ),
    );
    return;
  }

  const font = fonts();
  const boxes = ids.map((id) => {
    const row = rows.get(id)!;
    if (row.kind === "node") return { id, width: Math.ceil(textWidth(row.label, font.node)) + PAD * 2 + (row.findings ? 22 : 0), height: NODE_H };
    const width = Math.max(textWidth(row.label, font.channel), textWidth(shortType(row.type), font.type));
    return { id, width: Math.ceil(width) + PAD * 2 + (row.findings ? 22 : 0), height: CHANNEL_H };
  });
  const flows = visible.links.map(flow);
  const placed = layout(boxes, flows, { column: 96, row: 18 });

  const stage = h("div", { class: "stage", tabindex: "0", "aria-label": "System graph" });
  const svg = s("svg", { class: "graph" });
  const defs = s("defs");
  for (const [name, colour] of [["arrow", "var(--edge)"], ["arrow-warn", "var(--warn)"], ["arrow-bad", "var(--bad)"]]) {
    const marker = s("marker", { id: name, viewBox: "0 0 10 10", refX: 9, refY: 5, markerWidth: 7, markerHeight: 7, orient: "auto-start-reverse" });
    marker.append(s("path", { d: "M 0 1 L 9 5 L 0 9 z", fill: colour }));
    defs.append(marker);
  }
  svg.append(defs);
  const world = s("g");
  svg.append(world);

  const edgeLayer = s("g", { class: "edges" });
  const boxLayer = s("g", { class: "boxes" });
  world.append(edgeLayer, boxLayer);

  const neighbours = new Map<string, Set<string>>();
  const touch = (a: string, b: string) => (neighbours.get(a) ?? neighbours.set(a, new Set()).get(a)!).add(b);
  const edgeOf = new Map<string, SVGPathElement[]>();

  visible.links.forEach((link, index) => {
    const { from, to } = flows[index];
    const a = placed.boxes.get(from);
    const b = placed.boxes.get(to);
    if (!a || !b) return;
    touch(from, to);
    touch(to, from);
    const tone = link.problem === "error" ? "bad" : link.problem === "warn" || link.conflicts.length > 0 || link.agreement === "differ" ? "warn" : "";
    const path = s("path", { d: route(a, b), class: `edge${tone ? ` ${tone}` : ""}${link.wrapper ? " wrapper" : ""}`, "marker-end": `url(#${tone ? `arrow-${tone}` : "arrow"})` });
    const title = s("title");
    title.textContent = [`${label(rows, link.src)} ${VERB[link.kind]} ${label(rows, link.dst)}`, qosText(link.qos)].filter(Boolean).join("\n");
    path.append(title);
    edgeLayer.append(path);
    for (const id of [from, to]) (edgeOf.get(id) ?? edgeOf.set(id, []).get(id)!).push(path);
  });

  const hits = matches(graph, query);
  const boxOf = new Map<string, SVGGElement>();
  for (const [id, at] of placed.boxes) {
    const row = rows.get(id)!;
    const group = s("g", { class: `box ${row.kind}${hits.has(id) ? " hit" : ""}${id === selected ? " selected" : ""}`, transform: `translate(${at.x} ${at.y})`, tabindex: 0, role: "button", "data-id": id });
    group.setAttribute("aria-label", `${row.kind} ${row.label}`);
    const radius = row.kind === "node" ? 6 : CHANNEL_H / 2;
    group.append(s("rect", { width: at.width, height: at.height, rx: radius, ry: radius }));
    if (row.kind === "node") {
      group.append(text(row.label, PAD, NODE_H / 2 + 4.5, "label"));
    } else {
      group.append(text(row.label, PAD, 19, "label"));
      group.append(text(shortType(row.type) || row.kind, PAD, 34, "type"));
    }
    if (row.findings) {
      const worst = current?.findings[id]?.some((finding) => finding.severity === "error") ? " error" : "";
      const badge = s("g", { class: `badge${worst}`, transform: `translate(${at.width - 20} ${at.height / 2})` });
      badge.append(s("circle", { r: 8 }));
      badge.append(text(String(row.findings), 0, 3.5, "count"));
      group.append(badge);
    }
    group.addEventListener("pointerenter", () => light(id));
    group.addEventListener("pointerleave", () => light(null));
    group.addEventListener("click", (event) => {
      event.stopPropagation();
      selected = id;
      render();
    });
    group.addEventListener("dblclick", (event) => {
      event.stopPropagation();
      post({ type: "focus", id });
    });
    group.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        selected = id;
        render();
      }
    });
    boxLayer.append(group);
    boxOf.set(id, group);
  }

  function light(id: string | null): void {
    svg.classList.toggle("lit", id !== null);
    for (const [other, group] of boxOf) group.classList.toggle("near", id !== null && (other === id || neighbours.get(id)?.has(other) === true));
    for (const path of edgeLayer.querySelectorAll("path")) path.classList.remove("near");
    if (id) for (const path of edgeOf.get(id) ?? []) path.classList.add("near");
  }

  stage.append(svg);
  root.append(stage);
  if (selected && rows.has(selected) && visible.ids.has(selected)) root.append(inspector(rows, rows.get(selected)!, visible.links));

  requestAnimationFrame(() => {
    fit(stage, placed.width, placed.height, true);
    apply(world);
  });
  wire(stage, svg, world, placed.width, placed.height);
  svg.addEventListener("click", () => {
    if (!selected) return;
    selected = null;
    render();
  });
}

function label(rows: Map<string, Row>, id: string): string {
  return rows.get(id)?.label ?? id.replace(/^[a-z]+:/, "");
}

function text(content: string, x: number, y: number, cls: string): SVGTextElement {
  const element = s("text", { x, y, class: cls });
  element.textContent = content;
  return element;
}

/** A curve out of the right side of one box into the left side of the next. */
function route(a: Placed, b: Placed): string {
  const ax = a.x + a.width;
  const ay = a.y + a.height / 2;
  const by = b.y + b.height / 2;
  if (b.x > ax) {
    const bend = Math.max(32, (b.x - ax) / 2);
    return `M ${ax} ${ay} C ${ax + bend} ${ay}, ${b.x - bend} ${by}, ${b.x - 2} ${by}`;
  }
  // A flow that closes a cycle leaves to the right and loops back under both boxes.
  const low = Math.max(a.y + a.height, b.y + b.height) + 36;
  return `M ${ax} ${ay} C ${ax + 60} ${ay}, ${ax + 60} ${low}, ${(ax + b.x) / 2} ${low} S ${b.x - 60} ${by}, ${b.x - 2} ${by}`;
}

/**
 * The inspector covers the right of the stage, so an open one shrinks the room to fit into. A
 * graph opens at a scale its labels can be read at, from its left end, and Fit shows all of it.
 */
function fit(stage: HTMLElement, width: number, height: number, readable = false): void {
  const box = stage.getBoundingClientRect();
  const covered = document.querySelector(".inspector")?.getBoundingClientRect().width ?? 0;
  const room = box.width - (covered ? covered + 32 : 0);
  const margin = 40;
  const whole = Math.min(1.25, (room - margin * 2) / Math.max(1, width), (box.height - margin * 2) / Math.max(1, height));
  const k = readable ? Math.max(whole, 0.85) : whole;
  view = { k, x: k > whole ? margin : (room - width * k) / 2, y: Math.max(margin, (box.height - height * k) / 2) };
}

function apply(world: SVGGElement): void {
  world.setAttribute("transform", `translate(${view.x} ${view.y}) scale(${view.k})`);
}

/** Drag to pan, wheel to zoom about the pointer, F or the button to fit. */
function wire(stage: HTMLElement, svg: SVGSVGElement, world: SVGGElement, width: number, height: number): void {
  let drag: { x: number; y: number } | null = null;
  svg.addEventListener("pointerdown", (event) => {
    if ((event.target as Element).closest(".box")) return;
    drag = { x: event.clientX - view.x, y: event.clientY - view.y };
    svg.setPointerCapture(event.pointerId);
    svg.classList.add("dragging");
  });
  svg.addEventListener("pointermove", (event) => {
    if (!drag) return;
    view.x = event.clientX - drag.x;
    view.y = event.clientY - drag.y;
    apply(world);
  });
  svg.addEventListener("pointerup", () => {
    drag = null;
    svg.classList.remove("dragging");
  });
  svg.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      const box = svg.getBoundingClientRect();
      const px = event.clientX - box.left;
      const py = event.clientY - box.top;
      const k = Math.min(3, Math.max(0.15, view.k * Math.exp(-event.deltaY * 0.0015)));
      view = { k, x: px - ((px - view.x) * k) / view.k, y: py - ((py - view.y) * k) / view.k };
      apply(world);
    },
    { passive: false },
  );
  stage.addEventListener("keydown", (event) => {
    if (event.key === "f" || event.key === "F") {
      fit(stage, width, height);
      apply(world);
    }
  });
  document.querySelector<HTMLButtonElement>("#fit")?.addEventListener("click", () => {
    fit(stage, width, height);
    apply(world);
  });
}

function toolbar(graph: GraphData, profiles: string[], focus: ToGraph["focus"], hidden: number): HTMLElement {
  const bar = h("header", { class: "bar" });
  const title = h("div", { class: "title" }, h("h1", {}, focus ? focus.label : "System graph"));
  const counts = `${graph.nodes.length} nodes, ${graph.channels.length} channels`;
  title.append(h("span", { class: "muted" }, focus ? `and what it talks to · ${counts}` : counts));
  bar.append(title);

  const tools = h("div", { class: "tools" });
  if (focus) {
    const all = h("button", {}, "Whole system");
    all.addEventListener("click", () => post({ type: "focus", id: null }));
    tools.append(all);
  }
  if (profiles.length > 1) {
    const select = h("select", { "aria-label": "Profile" }) as HTMLSelectElement;
    for (const profile of profiles) {
      const option = h("option", { value: profile }, profile.replace(/^profile:/, "")) as HTMLOptionElement;
      option.selected = profile === graph.profile;
      select.append(option);
    }
    select.addEventListener("change", () => post({ type: "profile", profile: select.value }));
    tools.append(select);
  }
  const search = h("input", { type: "search", placeholder: "Find a node, topic or type", "aria-label": "Find in the graph" }) as HTMLInputElement;
  search.value = query;
  search.addEventListener("input", () => {
    query = search.value;
    const hits = matches(graph, query);
    for (const group of document.querySelectorAll<SVGGElement>(".box")) {
      group.classList.toggle("hit", hits.has(group.getAttribute("data-id") ?? ""));
    }
    document.querySelector("svg.graph")?.classList.toggle("searching", hits.size > 0);
  });
  tools.append(search);
  if (hidden || wrappers) {
    const toggle = h("button", { class: wrappers ? "on" : "", "aria-pressed": String(wrappers), title: "Endpoints a library creates for the node, such as a transform listener" }, wrappers ? "Hide library endpoints" : `Show ${hidden} library endpoint${hidden === 1 ? "" : "s"}`);
    toggle.addEventListener("click", () => {
      wrappers = !wrappers;
      vscode.setState({ wrappers });
      render();
    });
    tools.append(toggle);
  }
  tools.append(h("button", { id: "fit", title: "Fit the graph to the window (F)" }, "Fit"));
  bar.append(tools);
  if (!graph.links.length && graph.nodes.length) {
    bar.append(h("p", { class: "note" }, "No endpoints yet: the atlas holds no code for these nodes, only the launch files that start them. Add their sources to the workspace and extract again to see who talks to whom."));
  }
  if (graph.truncated) {
    bar.append(h("p", { class: "note" }, "This system is large, so only part of it is drawn. Double click a node to see everything it talks to."));
  }
  return bar;
}

function inspector(rows: Map<string, Row>, row: Row, links: Link[]): HTMLElement {
  const panel = h("aside", { class: "inspector", "aria-label": `${row.label} details` });
  const close = h("button", { class: "close", "aria-label": "Close" }, "×");
  close.addEventListener("click", () => {
    selected = null;
    render();
  });
  panel.append(close, h("h2", {}, row.label));
  const kind = row.kind === "node" ? "node" : [row.kind, shortType(row.type)].filter(Boolean).join(" · ");
  panel.append(h("p", { class: "muted" }, kind));
  for (const finding of current?.findings[row.id] ?? []) {
    const note = h("div", { class: `finding ${finding.severity}` }, h("p", {}, finding.say || finding.title));
    const where = finding.locations[0];
    if (where) note.append(openButton("Go to line", { uri: where.uri, line: where.line, approximate: false }, "small"));
    panel.append(note);
  }

  const actions = h("div", { class: "actions" });
  const card = h("button", { class: "primary" }, "Open card");
  card.addEventListener("click", () => post({ type: "entity", id: row.id }));
  actions.append(card);
  if (row.at) actions.append(openButton("Go to source", row.at));
  const focus = h("button", {}, "Only its neighbours");
  focus.addEventListener("click", () => post({ type: "focus", id: row.id }));
  actions.append(focus);
  panel.append(actions);

  const mine = links.filter((link) => link.src === row.id || link.dst === row.id);
  if (mine.length) {
    const list = h("ul", { class: "links" });
    for (const link of mine.sort((a, b) => label(rows, a.src === row.id ? a.dst : a.src).localeCompare(label(rows, b.src === row.id ? b.dst : b.src)))) {
      const other = link.src === row.id ? link.dst : link.src;
      const who = h("a", { tabindex: "0" }, label(rows, other));
      const pick = () => {
        selected = other;
        render();
      };
      who.addEventListener("click", pick);
      who.addEventListener("keydown", (event) => event.key === "Enter" && pick());
      const sentence = row.kind === "node" ? h("span", {}, `${VERB[link.kind]} `, who) : h("span", {}, who, ` ${VERB[link.kind]}`);
      const item = h("li", { class: link.problem === "error" ? "bad" : link.problem || link.conflicts.length ? "warn" : "" }, sentence);
      const qos = qosText(link.qos);
      if (qos) item.append(h("span", { class: "qos" }, qos));
      const why = link.problem ? current?.findings[other]?.find((finding) => finding.severity === link.problem) : undefined;
      if (why) item.append(h("span", { class: "why" }, why.title));
      if (link.conflicts.length) item.append(h("span", { class: "qos" }, `layers disagree on ${link.conflicts.join(", ")}`));
      if (link.at) item.append(openButton("Go to line", link.at, "small"));
      list.append(item);
    }
    panel.append(h("h3", {}, "Endpoints"), list);
  }
  return panel;
}

function openButton(text: string, at: GraphSite, cls = ""): HTMLElement {
  const button = h("button", cls ? { class: cls } : {}, text);
  button.addEventListener("click", () => post({ type: "open", uri: at.uri, line: at.line }));
  return button;
}

window.addEventListener("message", (event: MessageEvent<ToGraph>) => {
  if (event.data?.type !== "graph") return;
  if (current?.graph.profile !== event.data.graph.profile || current?.focus?.id !== event.data.focus?.id) selected = event.data.focus?.id ?? null;
  current = event.data;
  render();
});
window.addEventListener("resize", () => current && render());
post({ type: "ready" });
