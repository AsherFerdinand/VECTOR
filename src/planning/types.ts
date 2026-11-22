import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "../world/types";
import type { CorridorLane } from "./corridor";
export type ManoeuvreState =
  | "FOLLOW_LANE"
  | "FOLLOW_VEHICLE"
  | "PREPARE_LANE_CHANGE"
  | "EXECUTE_LANE_CHANGE"
  | "OVERTAKE"
  | "RETURN_TO_LANE"
  | "STOP_FOR_OBSTACLE"
  | "SAFE_STOP";
export interface TrajectoryPoint extends VehicleState {
  time: number;
  curvature: number;
}
export interface TrajectoryCandidate {
  id: string;
  targetLane: CorridorLane;
  points: TrajectoryPoint[];
  valid: boolean;
  reasons: string[];
  maxCurvature: number;
  maxLateralAcceleration: number;
  duration: number;
}
export interface ManoeuvreSnapshot {
  state: ManoeuvreState;
  reason: string;
  candidates: TrajectoryCandidate[];
  selected: TrajectoryCandidate | null;
  targetSpeed: number;
}
export interface ManoeuvreOptions {
  enabled?: boolean;
  destinationX?: number;
  staticObjects?: WorldObject[];
  desiredSpeed?: number;
  returnToLane?: boolean;
}
