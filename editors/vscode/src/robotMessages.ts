// Messages between the extension and the robot model view.
import type { MeshData, RobotModel } from "./protocol";

export type ToRobot = {
  type: "robot";
  robot: RobotModel;
  profiles: string[];
  /** Every mesh the model's shapes name, by file id. */
  meshes: Record<string, MeshData>;
  /** A frame to select, when the view was opened from one. */
  select?: string;
};

export type ToRobotHost =
  | { type: "ready" }
  | { type: "open"; uri: string; line: number }
  | { type: "entity"; id: string }
  | { type: "model"; model: string }
  | { type: "profile"; profile: string };
