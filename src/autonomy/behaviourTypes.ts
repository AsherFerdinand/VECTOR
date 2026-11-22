import type { Point, Route } from "../navigation/types";
export type DrivingState =
  | "MANUAL"
  | "CRUISE"
  | "FOLLOW_LEAD"
  | "STOP_LIGHT"
  | "YIELD_PEDESTRIAN"
  | "BLOCKED"
  | "AEB"
  | "STALE_STOP"
  | "DESTINATION_REACHED";
export interface BehaviourSnapshot {
  perceptionMode?: "raw" | "fused" | "degraded";
  state: DrivingState;
  reason: string;
  leadDistance: number | null;
  relativeSpeed: number | null;
  desiredGap: number;
  ttc: number | null;
  stoppingDistance: number;
  aeb: boolean;
  targetSpeed: number;
  acceleration: number;
  braking: number;
  steering: number;
  source?: import("../sensors/types").SensorSource | null;
  observationTime: number | null;
  timeGap: number;
}
/** Only map/localisation/route information is permitted alongside measured observations. */
export interface BehaviourContext {
  tracks?: import("../perception/types").Track[];
  route: Route | null;
  progress: number;
  timeGap: number;
  targetSpeed?: number;
  destinationReached?: boolean;
}
