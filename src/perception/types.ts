import type { SensorSource } from "../sensors/types";
import type { ObjectClass } from "../world/types";

/** Estimated observed surface position, not an inferred object/body centre. SI units. */
export interface Track {
  id: string;
  x: number;
  y: number;
  vx: number;
  vy: number;
  /** State order x, y, vx, vy. */
  covariance: number[][];
  status: "tentative" | "confirmed" | "coasting";
  objectClass?: ObjectClass;
  lastMeasurementTime: number;
  age: number;
  sources: SensorSource[];
  /** Last acquisition timestamp per contributing modality. */
  sourceTimes?: Partial<Record<SensorSource, number>>;
}
export interface TrackingSnapshot {
  timestamp: number;
  tracks: Track[];
}
export interface TrackerConfig {
  accelerationNoise: number;
  gate: number;
  confirmationHits: number;
  coastingAfter: number;
  deleteAfter: number;
  maxTracks: number;
}
