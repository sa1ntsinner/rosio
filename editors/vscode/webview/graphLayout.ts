// Left to right layered layout for the system graph: data flows from a publisher through its
// topic to every subscriber. Pure, so it is tested without a browser.

export interface Box {
  id: string;
  width: number;
  height: number;
}

export interface Flow {
  from: string;
  to: string;
}

export interface Placed extends Box {
  x: number;
  y: number;
  rank: number;
}

export interface Layout {
  boxes: Map<string, Placed>;
  width: number;
  height: number;
  /** Flows drawn against the reading direction because they close a cycle. */
  backward: Set<number>;
}

export interface Gaps {
  column: number;
  row: number;
}

const SWEEPS = 4;
const ALONE_ROWS = 8;

export function layout(boxes: Box[], flows: Flow[], gaps: Gaps = { column: 80, row: 16 }): Layout {
  const sorted = [...boxes].sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  const known = new Set(sorted.map((box) => box.id));
  const usable = flows
    .map((flow, index) => ({ ...flow, index }))
    .filter((flow) => flow.from !== flow.to && known.has(flow.from) && known.has(flow.to));

  const backward = cycleBreakers(sorted, usable);
  const forward = usable.map((flow) => (backward.has(flow.index) ? { from: flow.to, to: flow.from } : flow));
  const rank = ranks(sorted, forward);

  const linked = new Set(forward.flatMap((flow) => [flow.from, flow.to]));
  const layers: string[][] = [];
  for (const box of sorted) if (linked.has(box.id)) (layers[rank.get(box.id) ?? 0] ??= []).push(box.id);
  order(layers, forward);
  // Boxes nothing connects to would stack into one tall column, so they fill columns of their own.
  const alone = sorted.filter((box) => !linked.has(box.id)).map((box) => box.id);
  const rows = Math.max(ALONE_ROWS, ...layers.map((layer) => layer.length));
  for (let i = 0; i < alone.length; i += rows) layers.push(alone.slice(i, i + rows));

  const size = new Map(sorted.map((box) => [box.id, box]));
  const placed = new Map<string, Placed>();
  const heights = layers.map((layer) => layer.reduce((sum, id) => sum + size.get(id)!.height, 0) + gaps.row * (layer.length - 1));
  const height = Math.max(0, ...heights);
  let x = 0;
  layers.forEach((layer, r) => {
    const column = Math.max(0, ...layer.map((id) => size.get(id)!.width));
    let y = (height - heights[r]) / 2;
    for (const id of layer) {
      const box = size.get(id)!;
      placed.set(id, { ...box, x: x + (column - box.width) / 2, y, rank: r });
      y += box.height + gaps.row;
    }
    x += column + gaps.column;
  });
  return { boxes: placed, width: Math.max(0, x - gaps.column), height, backward };
}

/** Flows that point back to a box still on the depth first stack, visited in id order. */
function cycleBreakers(boxes: Box[], flows: (Flow & { index: number })[]): Set<number> {
  const out = new Map<string, (Flow & { index: number })[]>();
  for (const flow of flows) (out.get(flow.from) ?? out.set(flow.from, []).get(flow.from)!).push(flow);
  for (const list of out.values()) list.sort((a, b) => (a.to < b.to ? -1 : a.to > b.to ? 1 : a.index - b.index));
  const state = new Map<string, "open" | "done">();
  const backward = new Set<number>();
  for (const box of boxes) {
    if (state.has(box.id)) continue;
    const stack: { id: string; next: number }[] = [{ id: box.id, next: 0 }];
    state.set(box.id, "open");
    while (stack.length) {
      const top = stack[stack.length - 1];
      const flow = out.get(top.id)?.[top.next++];
      if (!flow) {
        state.set(top.id, "done");
        stack.pop();
        continue;
      }
      const seen = state.get(flow.to);
      if (seen === "open") backward.add(flow.index);
      else if (!seen) {
        state.set(flow.to, "open");
        stack.push({ id: flow.to, next: 0 });
      }
    }
  }
  return backward;
}

/** Longest path from a source, so every flow points one or more columns to the right. */
function ranks(boxes: Box[], flows: Flow[]): Map<string, number> {
  const incoming = new Map(boxes.map((box) => [box.id, 0]));
  const out = new Map<string, string[]>();
  for (const flow of flows) {
    incoming.set(flow.to, incoming.get(flow.to)! + 1);
    (out.get(flow.from) ?? out.set(flow.from, []).get(flow.from)!).push(flow.to);
  }
  const rank = new Map(boxes.map((box) => [box.id, 0]));
  const ready = boxes.filter((box) => incoming.get(box.id) === 0).map((box) => box.id);
  while (ready.length) {
    const id = ready.shift()!;
    for (const next of out.get(id) ?? []) {
      rank.set(next, Math.max(rank.get(next)!, rank.get(id)! + 1));
      incoming.set(next, incoming.get(next)! - 1);
      if (incoming.get(next) === 0) ready.push(next);
    }
  }
  return rank;
}

/** Barycentre sweeps, down then up, to keep crossings few. Ties keep the previous order. */
function order(layers: string[][], flows: Flow[]): void {
  const before = new Map<string, string[]>();
  const after = new Map<string, string[]>();
  for (const flow of flows) {
    (before.get(flow.to) ?? before.set(flow.to, []).get(flow.to)!).push(flow.from);
    (after.get(flow.from) ?? after.set(flow.from, []).get(flow.from)!).push(flow.to);
  }
  const position = new Map<string, number>();
  const remember = () => layers.forEach((layer) => layer.forEach((id, i) => position.set(id, i / Math.max(1, layer.length - 1))));
  const sweep = (layer: string[], neighbours: Map<string, string[]>) => {
    const key = new Map(
      layer.map((id, i) => {
        const near = (neighbours.get(id) ?? []).map((other) => position.get(other)!).filter((p) => p !== undefined);
        return [id, near.length ? near.reduce((a, b) => a + b, 0) / near.length : position.get(id) ?? i];
      }),
    );
    const index = new Map(layer.map((id, i) => [id, i]));
    layer.sort((a, b) => key.get(a)! - key.get(b)! || index.get(a)! - index.get(b)!);
  };
  remember();
  for (let i = 0; i < SWEEPS; i++) {
    for (let r = 1; r < layers.length; r++) {
      sweep(layers[r], before);
      remember();
    }
    for (let r = layers.length - 2; r >= 0; r--) {
      sweep(layers[r], after);
      remember();
    }
  }
}
