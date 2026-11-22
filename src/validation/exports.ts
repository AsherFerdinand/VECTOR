import type { ValidationMetrics, ValidationResult } from "./types";

export function exportJSON(result: ValidationResult): string {
  return JSON.stringify(result, null, 2);
}
function csvCell(value: unknown): string {
  const text = value === null || value === undefined ? "" : String(value);
  // Quote every cell; protect spreadsheet formula injection from editable scenario IDs/reasons.
  return `"${(/^[=+@\-]/.test(text) && typeof value === "string" ? "'" : "") + text.replaceAll('"', '""')}"`;
}
export function exportCSV(result: ValidationResult): string {
  const header = [
    "scenario",
    "seed",
    "time_s",
    "x_m",
    "y_m",
    "speed_m_s",
    "acceleration_m_s2",
    "steering_rad",
    "state",
    "reason",
    "lead_distance_m",
    "desired_gap_m",
    "ttc_s",
    "track_count",
    "collision_count",
    "road_departures",
  ];
  return [
    header,
    ...result.frames.map((f) => [
      result.config.id,
      result.config.seed,
      f.time,
      f.vehicle.x,
      f.vehicle.y,
      f.vehicle.velocity,
      f.control.acceleration,
      f.control.steering,
      f.behaviour.state,
      f.behaviour.reason,
      f.behaviour.leadDistance,
      f.behaviour.desiredGap,
      f.behaviour.ttc,
      f.tracks?.length ?? 0,
      f.physical.collisionCount,
      f.physical.roadDepartureCount,
    ]),
  ]
    .map((row) => row.map(csvCell).join(","))
    .join("\r\n");
}
export interface MetricComparison {
  metric: keyof ValidationMetrics;
  a: number;
  b: number;
  delta: number;
}
export function compareResults(
  a: ValidationResult,
  b: ValidationResult,
): MetricComparison[] {
  return (Object.keys(a.metrics) as (keyof ValidationMetrics)[]).flatMap(
    (metric) => {
      const av = a.metrics[metric],
        bv = b.metrics[metric];
      return typeof av === "number" && typeof bv === "number"
        ? [{ metric, a: av, b: bv, delta: bv - av }]
        : [];
    },
  );
}
export function engineeringReport(result: ValidationResult): string {
  return [
    `# VECTOR validation: ${result.config.id}`,
    `Seed: ${result.config.seed}; scenario: ${result.config.scenario}; duration: ${result.config.duration} s`,
    `Result: ${result.passed ? "PASS" : "FAIL"}`,
    ...result.failures.map((reason) => `- ${reason}`),
    "",
    "| Metric | Value |",
    "| --- | --- |",
    ...Object.entries(result.metrics).map(
      ([key, value]) =>
        `| ${key} | ${value === null ? "Unavailable" : typeof value === "number" ? value.toFixed(4) : value} |`,
    ),
    "",
    "Tracking position errors compare estimated surface returns to the nearest actor polygon boundary within a 5 m gate (validation only). Corresponding scattering points are unknown, so polygon-based covariance consistency is unavailable. Nearest-boundary association has selection bias; this is a geometric residual, not an unbiased accuracy or calibration test.",
    "Playback uses bounded recorded states, not a new simulation. Missing impact-speed, clearance or hazard-onset measurements remain unavailable. Fixed-step metrics precede replay downsampling.",
  ].join("\n");
}
