import type { Point, Route } from "../navigation/types";
export type NavigationState =
  | "IDLE"
  | "ROUTE_READY"
  | "ROUTE_FOLLOWING"
  | "APPROACHING_TURN"
  | "DESTINATION_REACHED"
  | "ROUTE_UNAVAILABLE"
  | "SAFE_STOP";
export interface NavigationTelemetry {
  state: NavigationState;
  targetSpeed: number | null;
  steeringCommand: number | null;
  remainingDistance: number | null;
  crossTrackError: number | null;
  lookAheadDistance: number | null;
  lookAheadPoint: Point | null;
  progress: number;
}
export interface NavigationSnapshot extends NavigationTelemetry {
  mode: "MANUAL" | "AUTONOMOUS";
  route: Route | null;
  trajectory: Point[];
  message: string;
}
