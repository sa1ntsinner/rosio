// The custom requests rosio-lsp answers, as the extension reads them.

export type Severity = "error" | "warn" | "info";

export const LAYERS = ["declared", "coded", "observed", "recorded"] as const;
export type Layer = (typeof LAYERS)[number];

/** What a person calls each layer: the thing it is read from. The atlas keeps its own names. */
export const LAYER_NAME: Record<Layer, string> = { declared: "config", coded: "code", observed: "live", recorded: "bag" };

export function layerName(layer: string): string {
  return LAYER_NAME[layer as Layer] ?? layer;
}

/** Prose the atlas wrote, with its layer names as a person says them. Only "the coded layer"
 * is touched: "declared" alone is a verb in half the findings. */
export function layerWords(text: string): string {
  return text.replace(/\b(declared|coded|observed|recorded) layer\b/g, (_, name: Layer) => `${LAYER_NAME[name]} layer`);
}

export interface Summary {
  loaded: boolean;
  error?: string;
  schemaVersion: string | null;
  layers: Layer[];
  profiles: string[];
  roots: { kind: string; label: string; hash: string; path?: string }[];
  findings: Record<Severity, number>;
  counters: Counters;
  atlasPath: string;
}

export interface Counters {
  resolved?: Record<string, number>;
  entities?: number;
  edges?: number;
  entities_by_kind?: Record<string, number>;
  resolution_rate?: Record<string, number>;
  unresolved?: Record<string, number>;
  passes?: { complete: boolean; refused_collectors: string[] };
}

export interface Location {
  uri: string;
  line: number;
}

export interface Finding {
  id: string;
  rule: string;
  severity: Severity;
  title: string;
  say: string;
  fix: string;
  /** The fix is an exact replacement the language server offers as a quick fix. */
  fixable?: boolean;
  locations: Location[];
}

/** A value the atlas could not resolve, and the line that asked for it. */
export interface Unresolved {
  id: string;
  want: string;
  expr: string;
  reason: string;
  owner: string;
  resolvableBy: string[] | null;
  locations: Location[];
}

export interface SearchHit {
  id: string;
  kind: string;
  label: string;
  layers: Layer[];
}

export interface Fact {
  layer: Layer;
  profile: string;
  confidence: string;
  values: Record<string, unknown>;
  evidence?: { file: string; locator?: string; line_hint?: number; root: string }[];
}

export interface Candidate {
  rank: number;
  wins: boolean;
  value?: unknown;
  mechanism: string;
  note?: string;
  profile: string;
  source?: { file: string; locator?: string; line_hint?: number; root: string };
  at?: Location & { approximate: boolean };
  shadow_kind?: string;
}

export interface Relation {
  kind: string;
  direction: "in" | "out";
  id: string;
  otherKind: string;
  label: string;
  layers: Layer[];
  value?: unknown;
  facts?: { layer: Layer; profile: string; values: Record<string, unknown> }[];
}

export interface Entity {
  id: string;
  kind: string;
  label: string;
  layers: Layer[];
  profiles: string[];
  agreement: string;
  conflicts: string[];
  attrs: Record<string, unknown>;
  facts: Fact[];
  state?: string;
  candidates?: Candidate[];
  effective?: Record<string, Record<string, { value: unknown; layer: string }>>;
  sites: (Location & { approximate: boolean })[];
  relations: Relation[];
  findings: string[];
}

/** One profile of `rosio/launchTree`: the command that starts it and the files it walks. */
export interface LaunchProfile {
  id: string;
  label: string;
  command: string | null;
  entries: LaunchFile[];
}

export interface LaunchFile {
  id: string;
  label: string;
  arg_declarations: string[];
  args: [string, string][];
  location?: Location;
  children: LaunchEntity[];
}

export interface LaunchEntity {
  id: string;
  kind: "include" | "node" | "composable" | "container" | "group" | "other";
  label: string;
  nodes: { id: string; label: string | null }[];
  package: string | null;
  executable: string | null;
  namespace: string | null;
  conditions: unknown[];
  gates: unknown[];
  parameter_inputs: unknown[];
  remaps: unknown[];
  location?: Location;
  children?: LaunchEntity[];
}

/** One frame of `rosio/frameTree`; `parent` is absent on a root. */
export interface FrameRow {
  id: string;
  label: string | null;
  parent?: string;
  joint: string | null;
  joint_type: string | null;
  model: string | null;
  xyz?: number[];
  rpy?: number[];
  broadcasters: { id: string; label: string | null; location?: Location }[];
  listeners: { id: string; label: string | null; location?: Location }[];
  findings: string[];
  location?: Location;
  children: FrameRow[];
}

export const SEVERITY_ORDER: Severity[] = ["error", "warn", "info"];

export const ATLAS_CHANGED = "rosio/atlasChanged";

export interface AtlasChanged {
  atlasPath: string;
  counters: Counters;
  findings: Record<Severity, number>;
}

export const MIGRATION_PLAN = "rosio/migrationPlan";

export interface MigrationChange {
  rule: string;
  package: string;
  file: string;
  line: number;
  column: number;
  found: string;
  /** What changed, in a few words. Older engines send none. */
  title?: string;
  effect: string;
  release: string;
  say: string;
  source: string;
  replacement?: string;
  works_from?: string;
}

export interface MigrationPlan {
  from: string;
  to: string;
  /** The directory every change's `file` is relative to. */
  root?: string;
  summary: Record<string, number>;
  changes: MigrationChange[];
  applied?: { diff: string };
}

export interface BuildError {
  /** Relative to the plan root; absent when the build output named no file of the workspace. */
  file: string | null;
  line: number | null;
  message: string;
  status: "listed" | "not_listed";
  matches: { rule: string; say: string }[];
}

export interface BuildPackage {
  /** A package name, or `@setup` and `@build` for failures outside any package. */
  name: string;
  status: "finished" | "failed" | "skipped";
  errors: BuildError[];
  all_errors_listed: boolean;
}

/** `rosio migrate verify`: the plan, plus a build of the sources in the target release. */
export interface MigrationBuild extends MigrationPlan {
  image: string;
  ok: boolean;
  packages: BuildPackage[];
}

/** Where a parameter value was written, as `rosio/paramTree` states it. */
export interface ParamSource {
  /** Package path, `rover_bringup/config/rover.yaml`. */
  file?: string;
  locator?: string;
  at?: Location & { approximate?: boolean };
}

export interface ParamCandidate {
  rank: number;
  wins: boolean;
  value: unknown;
  valueType: string | null;
  mechanism: string;
  shadowKind: string | null;
  source: ParamSource;
}

export interface ParamRow {
  id: string;
  name: string;
  state: string;
  layers: string[];
  findings: number;
  /** Absent when no candidate wins in this profile. */
  value?: unknown;
  valueType?: string | null;
  source?: ParamSource;
  candidates: ParamCandidate[];
}

/** `rosio/paramTree` for one profile: every parameter, grouped by the node that owns it. */
export interface ParamTree {
  profile: string | null;
  nodes: { id: string; label: string; params: ParamRow[] }[];
}

export interface GraphSite {
  uri: string;
  line: number;
  approximate: boolean;
}

/** `rosio/graph`: the nodes of one profile, the channels between them and every endpoint. */
export interface GraphData {
  profile: string | null;
  nodes: { id: string; label: string; kind: "node"; layers: string[]; findings: number; nameSource?: string; at?: GraphSite }[];
  channels: { id: string; label: string; kind: "topic" | "service" | "action"; type?: string; layers: string[]; findings: number; at?: GraphSite }[];
  links: {
    id: string;
    src: string;
    dst: string;
    kind: "publishes" | "subscribes" | "serves" | "calls";
    layers: string[];
    agreement: string;
    conflicts: string[];
    qos?: Record<string, string | number>;
    wrapper: boolean;
    rawName?: string;
    at?: GraphSite;
    /** The worst severity of a finding about this endpoint or citing its line. */
    problem?: "error" | "warn" | "info";
  }[];
  truncated: boolean;
}

/** One `<visual>` or `<collision>` block as the model writes it. Every number is absent unless stated. */
export interface UrdfShape {
  name?: string;
  origin?: { xyz?: number[]; rpy?: number[] };
  geometry?: {
    type: "box" | "cylinder" | "capsule" | "sphere" | "mesh" | string;
    size?: number[];
    radius?: number;
    length?: number;
    /** The mesh name as written, `package://` form kept. */
    filename?: string;
    /** File id of the mesh when the workspace holds it. */
    file?: string;
    scale?: number[];
  };
  material?: { name?: string; rgba?: number[]; texture?: string };
  /** Attributes written but not readable as numbers, such as `box.size`. */
  unreadable?: string[];
}

/** `rosio/robotModel`: one urdf or xacro model, with the TF broadcasts that publish into it. */
export interface RobotModel {
  profile: string | null;
  /** The chosen model's file id, or null when the atlas holds none or `model` was unknown. */
  model: string | null;
  models: { id: string; label: string; robotName: string; strategy: string; profiles: string[]; missingMeshes?: string[]; at?: GraphSite }[];
  links: {
    id: string;
    label: string;
    layers: string[];
    findings: number;
    worst?: "error" | "warn" | "info";
    /** Frame names code spells for this link, which no model declares. */
    misnamed?: string[];
    at?: GraphSite;
    /** Absent when the atlas states no geometry for this model, empty when the link has none. */
    visuals?: UrdfShape[];
    collisions?: UrdfShape[];
  }[];
  joints: {
    id: string;
    name: string;
    type: string;
    parent: string;
    child: string;
    /** Absent when the atlas does not state it, never zero filled. */
    xyz?: number[];
    rpy?: number[];
    axis?: number[];
    limit?: Record<string, number>;
    mimic?: Record<string, unknown>;
    unreadable?: string[];
    at?: GraphSite;
  }[];
  roots: string[];
  broadcasts: {
    id: string;
    node: string;
    nodeLabel: string | null;
    child: string;
    parent?: string;
    parentName?: string;
    layers: string[];
    condition?: string;
    static?: boolean;
    at?: GraphSite;
  }[];
  truncated: boolean;
  limit: number;
}

/** `rosio/mesh`: the triangles of one mesh file, or why it was not read. */
export type MeshData =
  | {
      file: string;
      /** The lowercase file extension. */
      format: string;
      /** Flat x y z per corner, nine per triangle, in the mesh's own units. */
      triangles: number[];
      /**
       * True when the mesh had more than 20000 triangles and was decimated: corners snapped to the
       * coarsest uniform grid that keeps at most 20000, each cell's corners replaced by their mean.
       */
      truncated: boolean;
      triangleCount: number;
      /** Over every triangle, null for a mesh with none. */
      bounds: { min: [number, number, number]; max: [number, number, number] } | null;
      refused?: undefined;
    }
  /** `no_answer` is the extension's own: the server was not running or the request failed. */
  | { file: string; format: string; refused: "file_missing" | "format_not_read" | "mesh_unreadable" | "no_answer" };
