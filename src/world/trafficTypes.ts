import type { Point } from "../navigation/types";
import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "./types";
export type ScenarioId =
  | "normal"
  | "following"
  | "sudden-braking"
  | "red-light"
  | "pedestrian"
  | "blocked";
export type SignalColor = "red" | "yellow" | "green";
/** Static map geometry; a light's changing colour is available only through observations. */
export interface SignalMap {
  id: string;
  position: Point;
  heading: number;
  stopLine: [Point, Point];
  crossing: [Point, Point];
}
export interface TrafficActor {
  id: string;
  kind: "vehicle" | "pedestrian";
  state: VehicleState;
  length: number;
  width: number;
  object: WorldObject;
}
export interface SignalState extends SignalMap {
  color: SignalColor;
}
export interface TrafficSnapshot {
  scenario: ScenarioId | null;
  time: number;
  actors: TrafficActor[];
  signals: SignalState[];
  staticObstacles: WorldObject[];
}
