import type { NavigationSnapshot } from "../autonomy/types";
import type { BehaviourSnapshot } from "../autonomy/behaviourTypes";
import type { Point } from "../navigation/types";
import type { Track } from "../perception/types";
import type { PhysicalSnapshot } from "../simulation/collisionTypes";
import type { VehicleControl, VehicleState } from "../simulation/types";
import type { ScenarioId } from "../world/trafficTypes";

export const MAX_RECORDED_FRAMES = 1201;
export interface ValidationConfig {
  id: string;
  scenario: ScenarioId;
  seed: number;
  duration: number;
  timeGap?: number;
  corridorScenario?: "lane-change" | "overtaking" | "blocked" | "cut-in";
  trafficVariant?: "intersection-conflict" | "dynamic-block";
  noise?: number;
  dropout?: { start: number; end: number };
  initial?: VehicleState;
  destination?: Point;
}
/** Truth exists only in recorded validation data, never in perception/planning inputs. */
export interface TrackingTruth {
  id: string;
  x: number;
  y: number;
  vx: number;
  vy: number;
  polygon?: Point[];
}
export interface ValidationFrame {
  time: number;
  vehicle: VehicleState;
  control: VehicleControl;
  behaviour: BehaviourSnapshot;
  physical: PhysicalSnapshot;
  navigation: NavigationSnapshot;
  tracks?: Track[];
  truth?: TrackingTruth[];
  /** Actual measured polygon clearance, not route-preflight clearance. */
  clearance?: number | null;
  laneDeparture?: boolean;
  manoeuvre?: string;
  impactSpeed?: number;
  hazardActive?: boolean;
}
export interface ValidationEvent {
  time: number;
  kind: "decision" | "collision" | "road" | "tracking" | "manoeuvre";
  message: string;
}
export interface ValidationMetrics {
  collisionCount: number;
  impactSpeed: number | null;
  minimumClearance: number | null;
  minimumTTC: number | null;
  followingDistanceRMSE: number | null;
  aebResponseTime: number | null;
  maximumDeceleration: number;
  maximumLateralAcceleration: number;
  maximumCrossTrackError: number;
  roadDepartures: number;
  laneDepartures: number;
  routeCompleted: boolean;
  trackingPositionRMSE: number | null;
  trackingVelocityRMSE: number | null;
  /** Mean position NEES divided by 2; near one under calibrated assumptions. */
  positionCovarianceConsistency: number | null;
  trackingMatches: number;
  interventions: number;
  manoeuvreOutcome: string | null;
}
export interface ValidationResult {
  config: ValidationConfig;
  frames: ValidationFrame[];
  events: ValidationEvent[];
  metrics: ValidationMetrics;
  passed: boolean;
  failures: string[];
}
