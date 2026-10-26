// The robot model. Links sit where the joint origins put them, exactly as the model states them.
// Text from the workspace is set with textContent only, never parsed as markup.
import type { GraphSite, MeshData, RobotModel, UrdfShape } from "../src/protocol";
import type { ToRobot, ToRobotHost } from "../src/robotMessages";
import { apply, degrees, meshCorners, outline, place, project, rotate, VIEWS, type Pose, type Vec, type View } from "./robotModel";

declare function acquireVsCodeApi(): {
  postMessage(message: ToRobotHost): void;
  getState(): Saved | undefined;
  setState(state: Saved): void;
};
const vscode = acquireVsCodeApi();
const SVG = "http://www.w3.org/2000/svg";

type Shapes = "visuals" | "collisions" | "none";
type Saved = { labels?: boolean; shapes?: Shapes };

type Joint = RobotModel["joints"][number];

let current: ToRobot | undefined;
let selected: string | null = null;
let labels = vscode.getState()?.labels ?? true;
let shapes: Shapes = vscode.getState()?.shapes ?? "visuals";
const save = () => vscode.setState({ labels, shapes });
let angle: View = { ...VIEWS["3/4"] };
let pan = { x: 0, y: 0, k: 0 };

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

function post(message: ToRobotHost): void {
  vscode.postMessage(message);
}

const plain = (id: string) => id.replace(/^frame:/, "");
const numbers = (values: number[] | undefined) => (values ? values.map((value) => String(Math.round(value * 10000) / 10000)).join("  ") : "not stated");

function render(): void {
  const root = document.querySelector("main")!;
  root.replaceChildren();
  if (!current) return;
  const { robot, profiles } = current;
  const model = robot.models.find((item) => item.id === robot.model);
  root.append(toolbar(robot, profiles));
  if (!model) {
    root.append(empty("No robot model in this atlas.", "rosio reads the urdf and xacro files a launch file loads. Extract a profile that starts robot_state_publisher, then open this view again."));
    return;
  }
  if (!robot.links.length) {
    root.append(empty(`rosio could not read ${model.label}.`, "The Unresolved view names what stopped it, such as a xacro argument with no value."));
    return;
  }

  const { poses, unplaced } = place(robot);
  if (unplaced.length) {
    root.querySelector(".bar")!.append(h("p", { class: "note" }, `Not drawn, because no joint places them with a readable origin: ${unplaced.map(plain).join(", ")}.`));
  }
  const stage = h("div", { class: "stage", tabindex: "0", "aria-label": `Robot model ${model.robotName}` });
  const svg = s("svg", { class: "robot" });
  stage.append(h("canvas", { class: "solids", "aria-hidden": "true" }), svg);
  root.append(stage);
  if (selected && robot.links.some((link) => link.id === selected)) root.append(inspector(robot, poses));
  requestAnimationFrame(() => draw(svg, robot, poses));
  wire(stage, svg, robot, poses);
}

function empty(title: string, detail: string): HTMLElement {
  return h("div", { class: "empty" }, h("p", {}, title), h("p", { class: "muted" }, detail));
}

type Drawn = { link: string; lines: Vec[][]; corners: Vec[]; rgba?: number[] };

/** The shapes of the chosen kind, already in model coordinates. */
function drawn(robot: RobotModel, poses: Map<string, Pose>): Drawn[] {
  if (shapes === "none") return [];
  const out: Drawn[] = [];
  for (const link of robot.links) {
    const pose = poses.get(link.id);
    if (!pose) continue;
    for (const shape of link[shapes] ?? []) {
      const lines = outline(shape).map((line) => line.map((point) => apply(pose, point)));
      const mesh = shape.geometry?.file ? current?.meshes[shape.geometry.file] : undefined;
      const corners = mesh && !mesh.refused ? meshCorners(shape, mesh.triangles).map((point) => apply(pose, point)) : [];
      const rgba = shape.material?.rgba?.length === 4 ? shape.material.rgba : undefined;
      if (lines.length || corners.length) out.push({ link: link.id, lines, corners, rgba });
    }
  }
  return out;
}

/** The size of the model, for the axis length and the ground grid. */
function extent(poses: Map<string, Pose>, more: Vec[] = []): { centre: Vec; size: number; floor: number } {
  const low = [Infinity, Infinity, Infinity];
  const high = [-Infinity, -Infinity, -Infinity];
  for (const point of [...[...poses.values()].map((pose) => pose.t), ...more]) {
    for (const axis of [0, 1, 2]) {
      low[axis] = Math.min(low[axis], point[axis]);
      high[axis] = Math.max(high[axis], point[axis]);
    }
  }
  return {
    centre: [0, 1, 2].map((axis) => (low[axis] + high[axis]) / 2) as Vec,
    size: Math.max(0.2, ...[0, 1, 2].map((axis) => high[axis] - low[axis])),
    floor: low[2],
  };
}

function draw(svg: SVGSVGElement, robot: RobotModel, poses: Map<string, Pose>): void {
  svg.replaceChildren();
  const box = svg.getBoundingClientRect();
  const solids = drawn(robot, poses);
  const { centre, size, floor } = extent(poses, solids.flatMap((item) => [...item.lines.flat(), ...item.corners]));
  const covered = document.querySelector(".inspector")?.getBoundingClientRect().width ?? 0;
  const room = box.width - (covered ? covered + 32 : 0);
  if (!pan.k) pan = { x: 0, y: 0, k: (Math.min(room, box.height) * 0.62) / size };
  const at = (point: Vec): [number, number, number] => {
    const [x, y, depth] = project([point[0] - centre[0], point[1] - centre[1], point[2] - centre[2]], angle);
    return [room / 2 + pan.x + x * pan.k, box.height / 2 + pan.y + y * pan.k, depth];
  };

  const canvas = svg.parentElement!.querySelector("canvas")!;
  const ratio = devicePixelRatio || 1;
  canvas.width = Math.round(box.width * ratio);
  canvas.height = Math.round(box.height * ratio);
  const paint = canvas.getContext("2d")!;
  paint.setTransform(ratio, 0, 0, ratio, 0, 0);
  const style = getComputedStyle(document.body);
  paint.strokeStyle = style.getPropertyValue("--vscode-editorWidget-border").trim() || "rgba(128, 128, 128, 0.35)";
  paint.lineWidth = 1;
  const step = 10 ** Math.floor(Math.log10(size)) / (size / 10 ** Math.floor(Math.log10(size)) < 3 ? 4 : 2);
  const reach = Math.ceil((size * 0.6) / step) * step;
  for (let offset = -reach; offset <= reach + 1e-9; offset += step) {
    for (const [a, b] of [
      [[centre[0] + offset, centre[1] - reach, floor], [centre[0] + offset, centre[1] + reach, floor]],
      [[centre[0] - reach, centre[1] + offset, floor], [centre[0] + reach, centre[1] + offset, floor]],
    ] as [Vec, Vec][]) {
      const [x1, y1] = at(a);
      const [x2, y2] = at(b);
      paint.beginPath();
      paint.moveTo(x1, y1);
      paint.lineTo(x2, y2);
      paint.stroke();
    }
  }
  const scale = s("text", { class: "scale", x: 16, y: box.height - 14 });
  scale.textContent = `grid ${step < 1 ? `${Math.round(step * 1000)} mm` : `${step} m`}`;
  svg.append(scale);
  solidMeshes(paint, solids, at, style.getPropertyValue("--vscode-foreground").trim());

  const items: { depth: number; node: SVGElement }[] = [];
  for (const solid of solids) {
    if (!solid.lines.length) continue;
    const group = s("g", { class: `shape${solid.link === selected ? " near" : ""}` });
    if (solid.rgba) group.setAttribute("style", `--tint: ${rgb(solid.rgba)}`);
    let depth = 0;
    let count = 0;
    for (const line of solid.lines) {
      const points = line.map((point) => at(point));
      for (const point of points) {
        depth += point[2];
        count++;
      }
      group.append(s("polyline", { points: points.map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(" ") }));
    }
    items.push({ depth: depth / Math.max(1, count) - 1e-3, node: group });
  }
  for (const joint of robot.joints) {
    const parent = poses.get(joint.parent);
    const child = poses.get(joint.child);
    if (!parent || !child) continue;
    const [x1, y1, d1] = at(parent.t);
    const [x2, y2, d2] = at(child.t);
    const line = s("line", { x1, y1, x2, y2, class: `joint ${joint.type}${selected === joint.child || selected === joint.parent ? " near" : ""}` });
    const title = s("title");
    title.textContent = `${joint.name}, ${joint.type}`;
    line.append(title);
    items.push({ depth: Math.min(d1, d2) - 1e-6, node: line });
  }

  const axis = size * 0.12;
  for (const link of robot.links) {
    const pose = poses.get(link.id);
    if (!pose) continue;
    const [x, y, depth] = at(pose.t);
    const group = s("g", { class: `link${link.id === selected ? " selected" : ""}${link.worst ? ` ${link.worst}` : ""}`, tabindex: 0, role: "button", "data-id": link.id });
    group.setAttribute("aria-label", `link ${link.label}`);
    for (const [name, unit] of [["x", [1, 0, 0]], ["y", [0, 1, 0]], ["z", [0, 0, 1]]] as [string, Vec][]) {
      const tip = rotate(pose.r, unit);
      const [ax, ay] = at([pose.t[0] + tip[0] * axis, pose.t[1] + tip[1] * axis, pose.t[2] + tip[2] * axis]);
      group.append(s("line", { x1: x, y1: y, x2: ax, y2: ay, class: `axis ${name}` }));
    }
    group.append(s("circle", { cx: x, cy: y, r: link.id === selected ? 6 : 4.5, class: "dot" }));
    group.append(s("circle", { cx: x, cy: y, r: 12, class: "hit" }));
    if (labels || link.id === selected || link.worst) {
      const text = s("text", { x: x - 10, y: y + 4, class: "name" });
      text.textContent = link.label;
      group.append(text);
    }
    const choose = (event: Event) => {
      event.stopPropagation();
      selected = link.id;
      render();
    };
    group.addEventListener("click", choose);
    group.addEventListener("keydown", (event) => event.key === "Enter" && choose(event));
    items.push({ depth, node: group });
  }
  for (const item of items.sort((a, b) => a.depth - b.depth)) svg.append(item.node);
}

function rgb(rgba: number[], light = 1): string {
  return `rgb(${rgba.slice(0, 3).map((value) => Math.round(Math.min(1, value * light) * 255)).join(" ")})`;
}

/** Mesh triangles far to near, each lit by how squarely it faces the viewer. */
function solidMeshes(paint: CanvasRenderingContext2D, solids: Drawn[], at: (point: Vec) => [number, number, number], ink: string): void {
  const view: Vec = [Math.cos(angle.yaw) * Math.cos(angle.pitch), Math.sin(angle.yaw) * Math.cos(angle.pitch), Math.sin(angle.pitch)];
  const fallback = ink.startsWith("#") && ink.length === 7 ? [1, 3, 5].map((i) => parseInt(ink.slice(i, i + 2), 16) / 255) : [0.6, 0.6, 0.6];
  const faces: { depth: number; points: [number, number][]; fill: string }[] = [];
  for (const solid of solids) {
    const base = solid.rgba ?? fallback;
    const alpha = solid.link === selected || !selected ? 1 : 0.35;
    for (let i = 0; i + 2 < solid.corners.length; i += 3) {
      const [a, b, c] = [solid.corners[i], solid.corners[i + 1], solid.corners[i + 2]];
      const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
      const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
      const length = Math.hypot(n[0], n[1], n[2]) || 1;
      const facing = Math.abs((n[0] * view[0] + n[1] * view[1] + n[2] * view[2]) / length);
      const projected = [at(a), at(b), at(c)];
      faces.push({
        depth: (projected[0][2] + projected[1][2] + projected[2][2]) / 3,
        points: projected.map(([x, y]) => [x, y] as [number, number]),
        fill: `${rgb(base, 0.55 + 1.1 * facing ** 1.5).slice(0, -1)} / ${alpha})`,
      });
    }
  }
  faces.sort((a, b) => a.depth - b.depth);
  for (const face of faces) {
    paint.beginPath();
    paint.moveTo(face.points[0][0], face.points[0][1]);
    paint.lineTo(face.points[1][0], face.points[1][1]);
    paint.lineTo(face.points[2][0], face.points[2][1]);
    paint.closePath();
    paint.fillStyle = face.fill;
    paint.strokeStyle = face.fill;
    paint.lineWidth = 0.6;
    paint.fill();
    paint.stroke();
  }
}

/** Drag to turn the model, shift drag to move it, wheel to zoom. */
function wire(stage: HTMLElement, svg: SVGSVGElement, robot: RobotModel, poses: Map<string, Pose>): void {
  let drag: { x: number; y: number; move: boolean } | null = null;
  let moved = false;
  svg.addEventListener("pointerdown", (event) => {
    if ((event.target as Element).closest(".link")) return;
    drag = { x: event.clientX, y: event.clientY, move: event.shiftKey || event.button === 1 };
    moved = false;
    svg.setPointerCapture(event.pointerId);
    svg.classList.add("dragging");
  });
  svg.addEventListener("pointermove", (event) => {
    if (!drag) return;
    const dx = event.clientX - drag.x;
    const dy = event.clientY - drag.y;
    drag.x = event.clientX;
    drag.y = event.clientY;
    moved ||= Math.abs(dx) + Math.abs(dy) > 2;
    if (drag.move) {
      pan.x += dx;
      pan.y += dy;
    } else {
      angle = { yaw: angle.yaw - dx * 0.01, pitch: Math.max(-Math.PI / 2, Math.min(Math.PI / 2, angle.pitch + dy * 0.01)) };
    }
    draw(svg, robot, poses);
  });
  svg.addEventListener("pointerup", () => {
    drag = null;
    svg.classList.remove("dragging");
  });
  svg.addEventListener("click", () => {
    if (moved || !selected) return;
    selected = null;
    render();
  });
  svg.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      pan.k = Math.max(1, pan.k * Math.exp(-event.deltaY * 0.0015));
      draw(svg, robot, poses);
    },
    { passive: false },
  );
  stage.addEventListener("keydown", (event) => {
    if (event.key === "f" || event.key === "F") fit(svg, robot, poses);
  });
  document.querySelector<HTMLButtonElement>("#fit")?.addEventListener("click", () => fit(svg, robot, poses));
}

function fit(svg: SVGSVGElement, robot: RobotModel, poses: Map<string, Pose>): void {
  pan = { x: 0, y: 0, k: 0 };
  draw(svg, robot, poses);
}

function toolbar(robot: RobotModel, profiles: string[]): HTMLElement {
  const bar = h("header", { class: "bar" });
  const model = robot.models.find((item) => item.id === robot.model);
  const title = h("div", { class: "title" }, h("h1", {}, model?.robotName || "Robot model"));
  if (model) title.append(h("span", { class: "muted" }, `${robot.links.length} links, ${robot.joints.length} joints · ${model.label}`));
  bar.append(title);

  const tools = h("div", { class: "tools" });
  if (robot.models.length > 1) {
    const select = h("select", { "aria-label": "Model" }) as HTMLSelectElement;
    for (const item of robot.models) {
      const option = h("option", { value: item.id }, item.label) as HTMLOptionElement;
      option.selected = item.id === robot.model;
      select.append(option);
    }
    select.addEventListener("change", () => post({ type: "model", model: select.value }));
    tools.append(select);
  }
  if (profiles.length > 1) {
    const select = h("select", { "aria-label": "Profile" }) as HTMLSelectElement;
    select.append(h("option", { value: "" }, "Any profile"));
    for (const profile of profiles) {
      const option = h("option", { value: profile }, profile.replace(/^profile:/, "")) as HTMLOptionElement;
      option.selected = profile === robot.profile;
      select.append(option);
    }
    select.addEventListener("change", () => post({ type: "profile", profile: select.value }));
    tools.append(select);
  }
  const views = h("div", { class: "segmented", role: "group", "aria-label": "View" });
  for (const [name, view] of Object.entries(VIEWS)) {
    const on = Math.abs(angle.yaw - view.yaw) < 1e-6 && Math.abs(angle.pitch - view.pitch) < 1e-6;
    const button = h("button", { class: on ? "on" : "", "aria-pressed": String(on) }, name);
    button.addEventListener("click", () => {
      angle = { ...view };
      pan = { x: 0, y: 0, k: 0 };
      render();
    });
    views.append(button);
  }
  tools.append(views);
  const kinds = h("div", { class: "segmented", role: "group", "aria-label": "Shapes" });
  for (const [name, kind] of [["Visual", "visuals"], ["Collision", "collisions"], ["Frames only", "none"]] as [string, Shapes][]) {
    const button = h("button", { class: shapes === kind ? "on" : "", "aria-pressed": String(shapes === kind) }, name);
    button.addEventListener("click", () => {
      shapes = kind;
      save();
      pan = { x: 0, y: 0, k: 0 };
      render();
    });
    kinds.append(button);
  }
  tools.append(kinds);
  const names = h("button", { class: labels ? "on" : "", "aria-pressed": String(labels) }, "Names");
  names.addEventListener("click", () => {
    labels = !labels;
    save();
    render();
  });
  tools.append(names, h("button", { id: "fit", title: "Fit the model to the window (F)" }, "Fit"));
  bar.append(tools);
  if (model && !model.profiles.length) bar.append(h("p", { class: "note" }, "No profile in the atlas loads this model, so the transforms its nodes broadcast are not shown."));
  if (model?.missingMeshes?.length) bar.append(h("p", { class: "note" }, `Meshes the model names and the package does not hold: ${model.missingMeshes.join(", ")}.`));
  if (robot.truncated) bar.append(h("p", { class: "note" }, `This model is large, so only the first ${robot.limit} links are drawn.`));
  return bar;
}

function inspector(robot: RobotModel, poses: Map<string, Pose>): HTMLElement {
  const link = robot.links.find((item) => item.id === selected)!;
  const panel = h("aside", { class: "inspector", "aria-label": `${link.label} details` });
  const close = h("button", { class: "close", "aria-label": "Close" }, "×");
  close.addEventListener("click", () => {
    selected = null;
    render();
  });
  panel.append(close, h("h2", {}, link.label));
  panel.append(h("p", { class: "muted" }, ["link", ...link.layers].join(" · ")));
  if (link.findings) panel.append(h("div", { class: `finding ${link.worst ?? "warn"}` }, h("p", {}, `${link.findings} finding${link.findings === 1 ? "" : "s"} about this frame. The card lists them.`)));

  const actions = h("div", { class: "actions" });
  const card = h("button", { class: "primary" }, "Open card");
  card.addEventListener("click", () => post({ type: "entity", id: link.id }));
  actions.append(card);
  if (link.at) actions.append(openButton("Go to source", link.at));
  panel.append(actions);

  const parent = robot.joints.find((joint) => joint.child === link.id);
  if (parent) {
    panel.append(h("h3", {}, "Joint from parent"), jointBlock(parent, plain(parent.parent), parent.parent));
  } else if (robot.roots.includes(link.id)) {
    panel.append(h("h3", {}, "Root of the model"));
    const above = robot.broadcasts.filter((cast) => cast.child === link.id);
    if (!above.length) panel.append(h("p", { class: "muted" }, "Nothing in this profile broadcasts a parent for it."));
    for (const cast of above) panel.append(broadcast(cast));
  }
  const pose = poses.get(link.id);
  if (pose && parent) {
    panel.append(h("p", { class: "muted small" }, `From ${plain(robot.roots[0] ?? "")}: ${numbers(pose.t)} m`));
  }

  for (const [title, list] of [["Visual", link.visuals], ["Collision", link.collisions]] as const) {
    if (!list?.length) continue;
    panel.append(h("h3", {}, title));
    for (const shape of list) panel.append(h("p", { class: "shape-row" }, describe(shape)));
  }

  const children = robot.joints.filter((joint) => joint.parent === link.id);
  if (children.length) {
    panel.append(h("h3", {}, "Joints to children"));
    const list = h("ul", { class: "links" });
    for (const joint of children) {
      const who = h("a", { tabindex: "0" }, plain(joint.child));
      const pick = () => {
        selected = joint.child;
        render();
      };
      who.addEventListener("click", pick);
      who.addEventListener("keydown", (event) => event.key === "Enter" && pick());
      const item = h("li", {}, h("span", {}, who, ` ${joint.type}`), h("span", { class: "qos" }, `${joint.name} · xyz ${numbers(joint.xyz)}`));
      list.append(item);
    }
    panel.append(list);
  }

  const casts = robot.broadcasts.filter((cast) => cast.parent === link.id && !robot.roots.includes(cast.child));
  if (casts.length) {
    panel.append(h("h3", {}, "Broadcast below it"));
    for (const cast of casts) panel.append(broadcast(cast));
  }
  return panel;
}

const REFUSED: Record<string, string> = {
  file_missing: "not in the workspace",
  format_not_read: "not drawn, rosio reads STL and OBJ",
  mesh_unreadable: "not drawn, the file does not read",
};

function meshNote(file: string | undefined): string {
  if (!file) return ", not in the workspace";
  const mesh: MeshData | undefined = current?.meshes[file];
  if (!mesh) return "";
  if (mesh.refused) return `, ${REFUSED[mesh.refused] ?? mesh.refused}`;
  return mesh.truncated ? `, ${mesh.triangles.length / 9} of ${mesh.triangleCount} triangles drawn` : "";
}

function describe(shape: UrdfShape): string {
  const geometry = shape.geometry;
  const what = !geometry
    ? "no geometry"
    : geometry.type === "box" && geometry.size
      ? `box ${numbers(geometry.size)} m`
      : geometry.type === "mesh"
        ? `mesh ${geometry.filename ?? ""}${geometry.scale && geometry.scale.some((value) => value !== 1) ? `, scale ${numbers(geometry.scale)}` : ""}${meshNote(geometry.file)}`
        : [geometry.type, geometry.radius !== undefined ? `radius ${numbers([geometry.radius])}` : "", geometry.length !== undefined ? `length ${numbers([geometry.length])}` : ""].filter(Boolean).join(", ");
  const at = shape.origin?.xyz ? ` at ${numbers(shape.origin.xyz)}` : "";
  const material = shape.material?.name ? `, ${shape.material.name}` : "";
  const unread = shape.unreadable?.length ? `, unreadable ${shape.unreadable.join(", ")}` : "";
  return `${shape.name ? `${shape.name}: ` : ""}${what}${at}${material}${unread}`;
}

function jointBlock(joint: Joint, parentLabel: string, parentId: string): HTMLElement {
  const block = h("dl", { class: "joint-facts" });
  const row = (term: string, value: string | Node) => block.append(h("dt", {}, term), h("dd", {}, value));
  const up = h("a", { tabindex: "0" }, parentLabel);
  const pick = () => {
    selected = parentId;
    render();
  };
  up.addEventListener("click", pick);
  up.addEventListener("keydown", (event) => event.key === "Enter" && pick());
  row("name", joint.name);
  row("type", joint.type);
  row("parent", up);
  row("xyz", joint.xyz ? `${numbers(joint.xyz)} m` : "not stated");
  row("rpy", joint.rpy ? `${numbers(joint.rpy)} rad (${joint.rpy.map(degrees).join("  ")})` : "not stated");
  if (joint.axis) row("axis", numbers(joint.axis));
  if (joint.limit) row("limit", Object.entries(joint.limit).map(([key, value]) => `${key} ${value}`).join(", "));
  if (joint.mimic) row("mimic", String(joint.mimic.joint ?? ""));
  if (joint.unreadable?.length) row("unreadable", joint.unreadable.join(", "));
  const wrap = h("div", {}, block);
  if (joint.at) wrap.append(openButton("Go to line", joint.at, "small"));
  return wrap;
}

function broadcast(cast: RobotModel["broadcasts"][number]): HTMLElement {
  const parent = cast.parent ? plain(cast.parent) : cast.parentName ?? "an unknown frame";
  const item = h("div", { class: "cast" }, h("p", {}, `${parent} → ${plain(cast.child)}`), h("p", { class: "muted" }, `${cast.static ? "static, " : ""}from ${cast.nodeLabel ?? "a node with no stable name"}${cast.condition ? ` when ${cast.condition}` : ""}`));
  if (cast.at) item.append(openButton("Go to line", cast.at, "small"));
  return item;
}

function openButton(text: string, at: GraphSite, cls = ""): HTMLElement {
  const button = h("button", cls ? { class: cls } : {}, text);
  button.addEventListener("click", () => post({ type: "open", uri: at.uri, line: at.line }));
  return button;
}

window.addEventListener("message", (event: MessageEvent<ToRobot>) => {
  if (event.data?.type !== "robot") return;
  if (current?.robot.model !== event.data.robot.model) {
    selected = null;
    pan = { x: 0, y: 0, k: 0 };
  }
  current = event.data;
  render();
});
window.addEventListener("resize", () => current && render());
post({ type: "ready" });
