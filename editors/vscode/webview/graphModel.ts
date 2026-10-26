// What the system graph draws, decided from `rosio/graph` alone, so it is tested without a browser.
import type { GraphData } from "../src/protocol";

export type Link = GraphData["links"][number];

export interface Shown {
  ids: Set<string>;
  links: Link[];
  hidden: number;
}

/** Data flows from the publisher and the caller to the subscriber and the server. */
export function flow(link: Link): { from: string; to: string } {
  return link.kind === "publishes" || link.kind === "calls"
    ? { from: link.src, to: link.dst }
    : { from: link.dst, to: link.src };
}

/**
 * Links a library makes for the node (a transform listener, a diagnostic updater) are noise in
 * a picture of the system, so they stay out unless asked for, and so does a channel only they use.
 */
export function shown(graph: GraphData, wrappers: boolean): Shown {
  const links = graph.links.filter((link) => wrappers || !link.wrapper);
  const used = new Set(links.flatMap((link) => [link.src, link.dst]));
  const ids = new Set(graph.nodes.map((node) => node.id));
  for (const channel of graph.channels) {
    const own = graph.links.some((link) => link.dst === channel.id || link.src === channel.id);
    if (used.has(channel.id) || !own) ids.add(channel.id);
  }
  return { ids, links, hidden: graph.links.length - links.length };
}

/** The QoS a link states, the way a parameters file would spell it. */
export function qosText(qos: Link["qos"]): string {
  if (!qos) return "";
  if (qos.preset !== undefined) return `${qos.preset} preset`;
  return ["reliability", "durability", "history", "depth"]
    .filter((key) => qos[key] !== undefined)
    .map((key) => (key === "depth" ? `depth ${qos[key]}` : String(qos[key])))
    .join(", ");
}

export const VERB: Record<Link["kind"], string> = {
  publishes: "publishes",
  subscribes: "subscribes",
  serves: "serves",
  calls: "calls",
};

/** Every box a search matches: by name, by type, by the node's own name. */
export function matches(graph: GraphData, query: string): Set<string> {
  const q = query.trim().toLowerCase();
  if (!q) return new Set();
  return new Set(
    [...graph.nodes, ...graph.channels]
      .filter((row) => row.label.toLowerCase().includes(q) || ("type" in row && row.type?.toLowerCase().includes(q)))
      .map((row) => row.id),
  );
}
