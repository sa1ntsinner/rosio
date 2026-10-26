// Messages between the extension and the atlas panel.
import type { Entity, Finding, Summary } from "./protocol";

export type ToPanel =
  | { type: "atlas"; summary: Summary; findings: Finding[]; workspace: string }
  | { type: "entity"; entity: Entity; findings: Finding[] };

export type ToHost =
  | { type: "ready" }
  | { type: "open"; uri: string; line: number }
  | { type: "extract" }
  | { type: "find" }
  | { type: "atlas" }
  | { type: "entity"; id: string }
  | { type: "graph"; id?: string };
