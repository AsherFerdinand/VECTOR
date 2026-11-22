import type { Point } from "../navigation/types";
export type ObjectClass =
  "building" | "traffic-light" | "static-obstacle" | "vehicle" | "pedestrian";
/** Exact geometry belongs to the simulator and sensor generators, never the supervisor. */
export interface WorldObject {
  id: string;
  objectClass: ObjectClass;
  polygon: Point[];
  height: number;
  velocity: Point;
  collidable: boolean;
  signal?: {
    color: import("./trafficTypes").SignalColor;
    heading: number;
    stopLine: [Point, Point];
  };
}
