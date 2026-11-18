import type { Point } from "../navigation/types";
import type { ObjectClass } from "../world/types";
export type SensorSource = "lidar" | "camera" | "radar";
/** Mount x is forward from rear axle; mount y is left. SI units and radians. */
export interface SensorConfig {
  id: string;
  source: SensorSource;
  mount: Point;
  yaw: number;
  fov: number;
  maxRange: number;
  updateHz: number;
  noise: number;
  seed: number;
  rays?: number;
}
export interface SensorPose extends Point {
  heading: number;
}
export interface LidarRay {
  angle: number;
  range: number;
  point: Point;
  hit: boolean;
}
export interface LidarScan {
  sensorId: string;
  timestamp: number;
  pose: SensorPose;
  rays: LidarRay[];
  maxRange: number;
}
/** Geometric observations are distinct from world objects: no object polygon or exact world pose. */
export interface Detection {
  observationId: string;
  source: SensorSource;
  timestamp: number;
  objectClass?: ObjectClass;
  debugObjectId?: string;
  relativePosition: Point;
  range: number;
  bearing: number;
  radialVelocity?: number;
}
export interface DetectionFrame {
  sensorId: string;
  timestamp: number;
  pose: SensorPose;
  detections: Detection[];
}
export interface SensorSnapshot {
  configs: SensorConfig[];
  lidar: LidarScan | null;
  camera: DetectionFrame | null;
  radar: DetectionFrame | null;
  counts: { lidar: number; camera: number; radar: number };
}
