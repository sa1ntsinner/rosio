// Where each link of a robot model sits, from the joint origins exactly as the model states them.
import type { RobotModel, UrdfShape } from "../src/protocol";

export type Vec = [number, number, number];
/** Row major rotation and a translation. */
export type Pose = { r: number[]; t: Vec };

const IDENTITY: Pose = { r: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] };

/** URDF rpy is fixed axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll). */
export function rpyRotation([roll, pitch, yaw]: Vec): number[] {
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(roll), Math.sin(roll), Math.cos(pitch), Math.sin(pitch), Math.cos(yaw), Math.sin(yaw)];
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

export function rotate(r: number[], [x, y, z]: Vec): Vec {
  return [r[0] * x + r[1] * y + r[2] * z, r[3] * x + r[4] * y + r[5] * z, r[6] * x + r[7] * y + r[8] * z];
}

function multiply(a: number[], b: number[]): number[] {
  const out: number[] = [];
  for (let row = 0; row < 3; row++) {
    for (let col = 0; col < 3; col++) {
      out.push(a[row * 3] * b[col] + a[row * 3 + 1] * b[3 + col] + a[row * 3 + 2] * b[6 + col]);
    }
  }
  return out;
}

function compose(parent: Pose, xyz: Vec, rpy: Vec): Pose {
  const offset = rotate(parent.r, xyz);
  return { r: multiply(parent.r, rpyRotation(rpy)), t: [parent.t[0] + offset[0], parent.t[1] + offset[1], parent.t[2] + offset[2]] };
}

export type Placement = {
  poses: Map<string, Pose>;
  /** Links below a joint whose origin the model does not state readably: drawn nowhere, listed instead. */
  unplaced: string[];
};

/** Every root at the origin, every child at its joint origin from its parent. */
export function place(robot: Pick<RobotModel, "roots" | "joints" | "links">): Placement {
  const poses = new Map<string, Pose>();
  const below = new Map<string, RobotModel["joints"]>();
  for (const joint of robot.joints) {
    if (!below.has(joint.parent)) below.set(joint.parent, []);
    below.get(joint.parent)!.push(joint);
  }
  const unplaced: string[] = [];
  const walk = (frame: string, pose: Pose) => {
    poses.set(frame, pose);
    for (const joint of below.get(frame) ?? []) {
      if (poses.has(joint.child)) continue;
      if (!joint.xyz || !joint.rpy) {
        unplaced.push(joint.child);
        continue;
      }
      walk(joint.child, compose(pose, joint.xyz as Vec, joint.rpy as Vec));
    }
  };
  for (const root of robot.roots) walk(root, IDENTITY);
  for (const link of robot.links) if (!poses.has(link.id) && !unplaced.includes(link.id)) unplaced.push(link.id);
  return { poses, unplaced: unplaced.sort() };
}

export type View = { yaw: number; pitch: number };

/** Screen x to the right, screen y down, and depth toward the viewer, for a z up world. */
export function project([x, y, z]: Vec, { yaw, pitch }: View): Vec {
  const qx = Math.cos(yaw) * x + Math.sin(yaw) * y;
  const qy = -Math.sin(yaw) * x + Math.cos(yaw) * y;
  return [qy, -(z * Math.cos(pitch) - qx * Math.sin(pitch)), qx * Math.cos(pitch) + z * Math.sin(pitch)];
}

export const VIEWS: Record<string, View> = {
  "3/4": { yaw: -0.65, pitch: 0.45 },
  Front: { yaw: 0, pitch: 0 },
  Side: { yaw: -Math.PI / 2, pitch: 0 },
  Top: { yaw: 0, pitch: Math.PI / 2 },
};

export function degrees(radians: number): string {
  return `${Math.round((radians * 180) / Math.PI * 10) / 10}°`;
}

function circle(radius: number, at: (x: number, y: number) => Vec, steps = 24): Vec[] {
  const points: Vec[] = [];
  for (let i = 0; i <= steps; i++) {
    const turn = (i / steps) * Math.PI * 2;
    points.push(at(radius * Math.cos(turn), radius * Math.sin(turn)));
  }
  return points;
}

/** The edges of a box, cylinder or sphere as polylines in its link's frame. A mesh, or a shape whose numbers did not read, has none. */
export function outline(shape: UrdfShape): Vec[][] {
  const geometry = shape.geometry;
  if (!geometry) return [];
  let lines: Vec[][] = [];
  if (geometry.type === "box" && geometry.size?.length === 3) {
    const [x, y, z] = geometry.size.map((value) => value / 2);
    const corner = (i: number): Vec => [i & 1 ? x : -x, i & 2 ? y : -y, i & 4 ? z : -z];
    for (const [a, b] of [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]]) lines.push([corner(a), corner(b)]);
  } else if ((geometry.type === "cylinder" || geometry.type === "capsule") && geometry.radius !== undefined && geometry.length !== undefined) {
    const r = geometry.radius;
    const half = geometry.length / 2;
    lines.push(circle(r, (a, b) => [a, b, -half]), circle(r, (a, b) => [a, b, half]));
    for (const [a, b] of [[r, 0], [-r, 0], [0, r], [0, -r]]) lines.push([[a, b, -half], [a, b, half]]);
  } else if (geometry.type === "sphere" && geometry.radius !== undefined) {
    const r = geometry.radius;
    lines.push(circle(r, (a, b) => [a, b, 0]), circle(r, (a, b) => [a, 0, b]), circle(r, (a, b) => [0, a, b]));
  }
  const xyz = (shape.origin?.xyz ?? [0, 0, 0]) as Vec;
  const turn = rpyRotation((shape.origin?.rpy ?? [0, 0, 0]) as Vec);
  lines = lines.map((line) => line.map((point) => {
    const moved = rotate(turn, point);
    return [moved[0] + xyz[0], moved[1] + xyz[1], moved[2] + xyz[2]] as Vec;
  }));
  return lines;
}

/** A pose applied to a point in its frame. */
export function apply(pose: Pose, point: Vec): Vec {
  const moved = rotate(pose.r, point);
  return [moved[0] + pose.t[0], moved[1] + pose.t[1], moved[2] + pose.t[2]];
}

/** A mesh's triangles in its link's frame: the stated scale, then the shape origin. Nine numbers per triangle in, one point per corner out. */
export function meshCorners(shape: UrdfShape, triangles: number[]): Vec[] {
  const scale = (shape.geometry?.scale ?? [1, 1, 1]) as Vec;
  const xyz = (shape.origin?.xyz ?? [0, 0, 0]) as Vec;
  const turn = rpyRotation((shape.origin?.rpy ?? [0, 0, 0]) as Vec);
  const corners: Vec[] = [];
  for (let i = 0; i + 2 < triangles.length; i += 3) {
    const moved = rotate(turn, [triangles[i] * scale[0], triangles[i + 1] * scale[1], triangles[i + 2] * scale[2]]);
    corners.push([moved[0] + xyz[0], moved[1] + xyz[1], moved[2] + xyz[2]]);
  }
  return corners;
}
