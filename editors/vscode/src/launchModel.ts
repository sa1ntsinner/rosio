// The Launch and Frames views as plain data, so their shape is tested without VS Code.
import type { FrameRow, LaunchEntity, LaunchFile, LaunchProfile } from "./protocol";

/** A profile reads as the name rosio.yaml gives it; the files under it say how it starts. */
export function profileTitle(profile: LaunchProfile): string {
  return decodeURIComponent(profile.id.replace(/^profile:/, "")) || profile.label;
}

/** Distinct nodes, since one node is often started from two branches of a condition. */
export function profileDescription(profile: LaunchProfile): string {
  const nodes = new Set(nodeIds(profile.entries.flatMap((file) => file.children))).size;
  return `${nodes} ${nodes === 1 ? "node" : "nodes"}`;
}

export function fileTitle(file: LaunchFile): string {
  return file.label.split("/").pop() ?? file.label;
}

/** The package the file is launched from, then the arguments the profile passes it. */
export function fileDescription(file: LaunchFile): string {
  const pkg = decodeURIComponent(file.id.replace(/^launchfile:/, "").split("|")[0]);
  return [pkg === "@repo" ? "" : pkg, ...file.args.map(([key, value]) => `${key}:=${value}`)]
    .filter(Boolean)
    .join("  ");
}

/** `Node(robot_state_publisher@45d4c79b)` reads as `robot_state_publisher`. Takes a decoded label. */
export function actionName(label: string): string {
  const inner = /^\w+\((.*)\)$/.exec(label)?.[1] ?? label;
  return inner.split("@")[0];
}

/** A node row is named by the node it starts; an unnamed node by what it runs. */
export function entityTitle(entity: LaunchEntity): string {
  if (entity.kind === "include" || entity.kind === "group") return actionName(entity.label);
  const node = entity.nodes[0];
  if (node?.label && !node.id.startsWith("node:~")) return node.label;
  return entity.executable ?? actionName(entity.label);
}

export function entityDescription(entity: LaunchEntity): string {
  const parts: string[] = [];
  if (entity.kind !== "include" && entity.kind !== "group") {
    const runs = entity.executable ? `${entity.package}/${entity.executable}` : entity.package;
    if (runs) parts.push(runs);
    if (entity.nodes[0]?.id.startsWith("node:~")) {
      parts.push(entity.namespace && entity.namespace !== "/" ? `unnamed in ${entity.namespace}` : "unnamed");
    }
  }
  for (const condition of entity.conditions ?? []) parts.push(conditionText(condition));
  return parts.join("  ");
}

/** `IfCondition(LaunchConfiguration('use_sim_time'))` reads as `if use_sim_time`. */
export function conditionText(condition: unknown): string {
  if (typeof condition !== "string") {
    const row = condition as { kind?: string; expression?: string };
    return `${row.kind === "UnlessCondition" ? "unless" : "if"} ${row.expression ?? ""}`.trim();
  }
  const match = /^(If|Unless)Condition\((.*)\)$/.exec(condition);
  if (!match) return condition;
  const expression = match[2].replace(/LaunchConfiguration\('([^']*)'\)/g, "$1");
  return `${match[1] === "If" ? "if" : "unless"} ${expression}`;
}

function nodeIds(entities: LaunchEntity[]): string[] {
  return entities.flatMap((entity) => [
    ...(started(entity) ? entity.nodes.map((node) => node.id) : []),
    ...nodeIds(entity.children ?? []),
  ]);
}

/** False when a gate of this profile evaluated to false, so the launch never starts it. */
export function started(entity: LaunchEntity): boolean {
  return !entity.gates.some((gate) => (gate as { outcome?: string }).outcome === "false");
}

/** Why a gated entity does not start, in the words of its conditions. */
export function notStartedReason(entity: LaunchEntity): string | undefined {
  const closed = entity.gates.filter((gate) => (gate as { outcome?: string }).outcome === "false");
  if (!closed.length) return undefined;
  const conditions = closed.map((gate) => conditionText((gate as { expression?: string }).expression ?? ""));
  return `Not started in this profile: ${conditions.join(", ")} is false.`;
}

/**
 * A frame row: its joint when a model states it, else who broadcasts it, else who looks it up.
 * The model file is named only when the same frame hangs off this parent in more than one model.
 */
export function frameDescription(frame: FrameRow, siblings: FrameRow[] = []): string {
  const parts: string[] = [];
  if (frame.joint) parts.push(frame.joint_type ? `${frame.joint_type} ${frame.joint}` : frame.joint);
  const by = names(frame.broadcasters);
  if (by) parts.push(`broadcast by ${by}`);
  else if (!frame.joint && frame.listeners.length) parts.push(`looked up by ${names(frame.listeners)}`);
  if (frame.model && siblings.filter((other) => other.id === frame.id).length > 1) {
    parts.push(modelTitle(frame.model).title);
  }
  return parts.join("  ");
}

export interface ModelGroup {
  model: string | null;
  roots: FrameRow[];
}

/**
 * One tree per robot model when the atlas holds several, so three robots never share one
 * base_link. Frames no model states (broadcast or looked up in code) form the last group.
 */
export function frameGroups(roots: FrameRow[]): ModelGroup[] {
  const models = new Set<string>();
  const visit = (row: FrameRow): void => {
    if (row.model) models.add(row.model);
    row.children.forEach(visit);
  };
  roots.forEach(visit);
  if (models.size <= 1) return [{ model: null, roots }];
  const groups = [...models].sort().map((model) => ({ model, roots: within(roots, model) }));
  const loose = roots.filter(modelless);
  return loose.length ? [...groups, { model: null, roots: loose }] : groups;
}

function within(rows: FrameRow[], model: string): FrameRow[] {
  return rows.flatMap((row) => {
    const children = within(row.children, model);
    return row.model === model || children.length ? [{ ...row, children }] : [];
  });
}

function modelless(row: FrameRow): boolean {
  return !row.model && row.children.every(modelless);
}

/** `file:nav2_minimal_tb3_sim|urdf/turtlebot3_waffle.urdf` reads as the file and its package. */
export function modelTitle(model: string): { title: string; pkg: string } {
  const [pkg, path] = model.replace(/^file:/, "").split("|");
  return { title: path?.split("/").pop() ?? model, pkg: pkg === "@repo" ? "" : pkg };
}

function names(rows: { label: string | null }[]): string {
  const labels = rows.map((row) => row.label).filter((label): label is string => Boolean(label));
  return labels.length > 2 ? `${labels.slice(0, 2).join(", ")} and ${labels.length - 2} more` : labels.join(", ");
}
