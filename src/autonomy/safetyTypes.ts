import type { SensorSource } from "../sensors/types";
export interface SafetySnapshot {
  state: "MANUAL" | "CLEAR" | "PROTECTIVE_BRAKING" | "STALE_STOP";
  braking: boolean;
  stoppingDistance: number;
  obstacleDistance: number | null;
  source: SensorSource | null;
  measurementTime: number | null;
  reason: string;
}
