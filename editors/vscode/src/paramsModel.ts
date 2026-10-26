// The Parameters view as plain data, so its wording is tested without VS Code.
import type { ParamCandidate, ParamRow, ParamTree } from "./protocol";

const MECHANISM: Record<string, string> = {
  yaml_base: "parameters file",
  yaml_overlay_merge: "merged parameters file",
  yaml_wildcard: "wildcard block",
  launch_params_file: "parameters file",
  launch_inline_dict: "launch file",
  launch_rewritten_yaml: "RewrittenYaml",
  ros_args_p: "-p argument",
  ros_args_params_file: "--params-file",
  python_override: "launcher script",
  param_set_runtime: "ros2 param set",
  code_default: "default in the source",
  observed_runtime: "running node",
  commented_out: "commented out",
};

const STATE: Record<string, string> = {
  owner_name_mismatch: "no node of this name runs",
  inert_by_design: "its plugin is not selected",
  undeclared: "never declared",
  declared_unread: "declared, never read",
  unresolved_owner: "owner unknown",
  other: "not resolved",
};

const SHADOW: Record<string, string> = {
  restated_identical: "same value, overridden",
  type_changed: "overridden, other type",
  not_read_in_profile: "not read here",
  inert_plugin_block: "plugin not selected",
};

/** A value spelled the way a parameters file would spell it. */
export function formatValue(value: unknown, type: string | null | undefined): string {
  if (value === undefined || value === null) return "";
  if (typeof value === "number") return type === "float" && Number.isInteger(value) ? value.toFixed(1) : String(value);
  if (typeof value === "string") {
    if (value.length <= 60) return JSON.stringify(value);
    if (value.trimStart().startsWith("<")) return `XML, ${(value.length / 1024).toFixed(1)} KB`;
    return `${JSON.stringify(value.slice(0, 57))}…`;
  }
  if (Array.isArray(value)) return `[${value.map((item) => formatValue(item, null)).join(", ")}]`;
  return String(value);
}

function overridden(param: ParamRow): boolean {
  return param.candidates.some((candidate) => !candidate.wins && candidate.shadowKind);
}

export function paramDescription(param: ParamRow): string {
  if (param.value !== undefined) return formatValue(param.value, param.valueType);
  return STATE[param.state] ?? "no value in this profile";
}

export function paramIcon(param: ParamRow): string {
  if (param.findings) return "warning";
  if (param.value === undefined) return "circle-slash";
  return overridden(param) ? "layers" : "symbol-constant";
}

export function where(source: ParamCandidate["source"]): string {
  const name = source.file?.split("/").pop() ?? source.locator ?? "";
  return source.at ? `${name}:${source.at.line + 1}` : name;
}

export function mechanismName(candidate: ParamCandidate): string {
  return MECHANISM[candidate.mechanism] ?? candidate.mechanism.replace(/_/g, " ");
}

/** The verdict first: the place is what a narrow side bar cuts. */
export function candidateDescription(candidate: ParamCandidate): string {
  const verdict = candidate.wins ? "wins" : (SHADOW[candidate.shadowKind ?? ""] ?? "overridden");
  const place = where(candidate.source);
  return place ? `${verdict} at ${place}` : verdict;
}

/** A node whose every parameter belongs to a block no launched node reads. */
export function notLaunched(node: ParamTree["nodes"][number]): boolean {
  return node.params.length > 0 && node.params.every((param) => param.state === "owner_name_mismatch");
}

export function nodeDescription(node: ParamTree["nodes"][number]): string {
  if (notLaunched(node)) return "configured, never launched";
  const parts = [`${node.params.length} ${node.params.length === 1 ? "parameter" : "parameters"}`];
  const beaten = node.params.filter(overridden).length;
  if (beaten) parts.push(`${beaten} overridden`);
  const findings = node.params.reduce((sum, param) => sum + param.findings, 0);
  if (findings) parts.push(`${findings} ${findings === 1 ? "finding" : "findings"}`);
  return parts.join(", ");
}

export interface ParamPick {
  label: string;
  description: string;
  detail: string;
  node: string;
  param: ParamRow;
}

/** One row per parameter for a quick pick, matched on the node and the file as well. */
export function paramPicks(tree: ParamTree): ParamPick[] {
  return tree.nodes.flatMap((node) =>
    node.params.map((param) => ({
      label: param.name,
      description: paramDescription(param),
      detail: [node.label, param.source?.file].filter(Boolean).join(" · "),
      node: node.id,
      param,
    })),
  );
}
