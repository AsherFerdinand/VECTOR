import type { ValidationConfig } from "./types";

export interface ValidationPreset {
  id: string;
  label: string;
  description: string;
  config: ValidationConfig;
}
const preset = (
  id: string,
  label: string,
  description: string,
  config: Omit<ValidationConfig, "id" | "seed" | "duration"> & {
    duration?: number;
  },
): ValidationPreset => ({
  id,
  label,
  description,
  config: { id, seed: 42, duration: 30, ...config },
});

/** Presets run actual supported actor scripts; unsupported manoeuvres are not relabelled. */
export const VALIDATION_SCENARIOS: ValidationPreset[] = [
  preset(
    "normal",
    "Normal traffic",
    "Baseline route and bounded background actors.",
    { scenario: "normal" },
  ),
  preset(
    "following",
    "Lead vehicle following",
    "ACC approach to a bounded lead vehicle.",
    { scenario: "following" },
  ),
  preset(
    "emergency",
    "Lead emergency braking",
    "Lead actor brakes at 6 m/s² after five simulation seconds.",
    { scenario: "sudden-braking" },
  ),
  preset(
    "red-light",
    "Red traffic light",
    "Observed red light, stop line and later green release.",
    { scenario: "red-light" },
  ),
  preset(
    "pedestrian",
    "Pedestrian crossing",
    "Observed crossing actor and safe resumption.",
    { scenario: "pedestrian" },
  ),
  preset(
    "static-block",
    "Static blocked route",
    "Static geometry rejects an infeasible destination route.",
    { scenario: "blocked" },
  ),
  preset(
    "intersection-conflict",
    "Intersection conflict",
    "Northbound vehicle crosses the ego route at the east intersection.",
    {
      scenario: "normal",
      trafficVariant: "intersection-conflict",
      destination: { x: 45, y: -29.25 },
    },
  ),
  preset(
    "dynamic-block",
    "Dynamic blocked route",
    "A stationary traffic vehicle blocks the driving corridor; observed stopping is required.",
    { scenario: "normal", trafficVariant: "dynamic-block" },
  ),
  preset(
    "lane-change",
    "Lane-change test corridor",
    "Explicit two-lane eastbound fixture with swept-body, uncertainty and physical control validation.",
    { scenario: "normal", corridorScenario: "lane-change", duration: 60 },
  ),
  preset(
    "overtaking",
    "Overtaking test corridor",
    "Pass an observed lead and return to the original lane on the independent mapped corridor.",
    { scenario: "normal", corridorScenario: "overtaking", duration: 60 },
  ),
  preset(
    "corridor-block",
    "Stationary vehicle on corridor",
    "Evaluate safe passing or stopping for an observed stationary vehicle.",
    { scenario: "normal", corridorScenario: "blocked", duration: 60 },
  ),
  preset(
    "cut-in",
    "Vehicle cut-in on test corridor",
    "A bounded lateral actor transition into the ego lane on the explicit multilane road.",
    { scenario: "normal", corridorScenario: "cut-in", duration: 60 },
  ),
  preset(
    "dropout",
    "Sensor dropout",
    "All sensor acquisitions stop from t=2 to t=3 s; timestamps remain stale.",
    { scenario: "following", dropout: { start: 2, end: 3 } },
  ),
  preset(
    "noisy-radar",
    "Noisy radar",
    "Deterministic 0.3 m radar range noise; camera and LiDAR stay at baseline noise.",
    { scenario: "following", noise: 0.3 },
  ),
  preset(
    "low-confidence",
    "Low-confidence tracking",
    "Longer sensor dropout exercises coasting, deletion and fresh reacquisition.",
    { scenario: "following", dropout: { start: 1.5, end: 4 } },
  ),
];
