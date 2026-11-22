import type { VehicleState } from "./types";
export type RoadStatus = "ON_ROAD" | "PARTIALLY_OFF_ROAD" | "FULLY_OFF_ROAD";
export interface CollisionEvent {
  time: number;
  objectId: string;
  position: { x: number; y: number };
}
export interface ClearanceResult {
  valid: boolean;
  reason: string;
  distance: number | null;
}
export interface RoadDepartureEvent {
  time: number;
  status: RoadStatus;
  position: { x: number; y: number };
}
export interface PhysicalSnapshot {
  roadStatus: RoadStatus;
  sweptRoadStatus: RoadStatus;
  containmentStop: boolean;
  collision: CollisionEvent | null;
  collisionCount: number;
  roadDepartureCount: number;
  collisionEvents: CollisionEvent[];
  roadDepartures: RoadDepartureEvent[];
  clearance: ClearanceResult | null;
}
export interface MotionResult {
  state: VehicleState;
  collided: boolean;
  objectId: string | null;
  fraction: number;
}
