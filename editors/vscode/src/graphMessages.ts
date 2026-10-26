// Messages between the extension and the system graph.
import type { Finding, GraphData } from "./protocol";

export type ToGraph = {
  type: "graph";
  graph: GraphData;
  profiles: string[];
  focus: { id: string; label: string } | null;
  /** The findings about each drawn entity that has any, keyed by entity id. */
  findings: Record<string, Finding[]>;
};

export type ToGraphHost =
  | { type: "ready" }
  | { type: "open"; uri: string; line: number }
  | { type: "entity"; id: string }
  | { type: "focus"; id: string | null }
  | { type: "profile"; profile: string };
