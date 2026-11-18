import type {
  DetectionFrame,
  SensorSnapshot,
  SensorSource,
} from "../sensors/types";
import type { SafetySnapshot } from "./safetyTypes";
import type { VehicleControl, VehicleState } from "../simulation/types";
import { VEHICLE } from "../simulation/vehicle";

/** Deliberately limited, measurement-only protection; these are not production AEB parameters. */
export const SAFETY_LIMITS = Object.freeze({
  maximumAge: 0.3,
  processingDelay: 0.1,
  stopMargin: 1,
  lateralMargin: 0.15,
  corridorStep: 0.25,
});
const FRONT = (VEHICLE.length + VEHICLE.wheelbase) / 2;
const REAR = (VEHICLE.length - VEHICLE.wheelbase) / 2;
const HALF_WIDTH = VEHICLE.width / 2 + SAFETY_LIMITS.lateralMargin;
interface MeasurementPoint {
  x: number;
  y: number;
  source: SensorSource;
  timestamp: number;
}

/** Allowed perfect ego localisation transforms measured points, never world obstacle geometry. */
function relativeToVehicle(x: number, y: number, vehicle: VehicleState) {
  const dx = x - vehicle.x,
    dy = y - vehicle.y;
  const c = Math.cos(vehicle.heading),
    s = Math.sin(vehicle.heading);
  return { x: c * dx + s * dy, y: -s * dx + c * dy };
}
function validAge(timestamp: number, time: number): boolean {
  const age = time - timestamp;
  return (
    Number.isFinite(age) &&
    age >= -1e-9 &&
    age <= SAFETY_LIMITS.maximumAge + 1e-9
  );
}
function framePoints(
  frame: DetectionFrame | null,
  vehicle: VehicleState,
  time: number,
): MeasurementPoint[] {
  if (!frame || !validAge(frame.timestamp, time)) return [];
  const c = Math.cos(frame.pose.heading),
    s = Math.sin(frame.pose.heading);
  return frame.detections
    .filter((d) => validAge(d.timestamp, time))
    .map((d) => {
      const p = d.relativePosition;
      const x = frame.pose.x + c * p.x - s * p.y;
      const y = frame.pose.y + s * p.x + c * p.y;
      return {
        ...relativeToVehicle(x, y, vehicle),
        source: d.source,
        timestamp: d.timestamp,
      };
    });
}

/** Earliest estimated body contact along a constant-steering forward driving corridor. */
function corridorContact(
  point: MeasurementPoint,
  curvature: number,
  horizon: number,
): number | null {
  if (Math.abs(curvature) < 1e-6) {
    if (Math.abs(point.y) > HALF_WIDTH || point.x < -REAR) return null;
    const contact = Math.max(0, point.x - FRONT);
    return contact <= horizon ? contact : null;
  }
  const count = Math.ceil(horizon / SAFETY_LIMITS.corridorStep);
  for (let i = 0; i <= count; i++) {
    const distance = Math.min(horizon, i * SAFETY_LIMITS.corridorStep);
    const angle = curvature * distance;
    const c = Math.cos(angle),
      s = Math.sin(angle);
    const x = Math.sin(angle) / curvature;
    const y = (1 - Math.cos(angle)) / curvature;
    const dx = point.x - x,
      dy = point.y - y;
    const along = c * dx + s * dy,
      across = -s * dx + c * dy;
    if (along >= -REAR && along <= FRONT && Math.abs(across) <= HALF_WIDTH) {
      // Account conservatively for the finite corridor sampling interval.
      return Math.max(0, distance - SAFETY_LIMITS.corridorStep);
    }
  }
  return null;
}

/**
 * A pure assessment of permitted observations. Callers latch a triggered stop until
 * explicit reactivation/takeover so a shrinking braking envelope cannot cause creeping.
 */
export function assessSafety(
  vehicle: VehicleState,
  steering: number,
  sensors: SensorSnapshot,
  time: number,
  autonomous: boolean,
): SafetySnapshot {
  const speed = Math.max(0, vehicle.velocity);
  const lidar = sensors.lidar;
  const age = lidar ? Math.max(0, time - lidar.timestamp) : 0;
  const config = sensors.configs.find((c) => c.source === "lidar");
  const period = config && config.updateHz > 0 ? 1 / config.updateHz : 0.1;
  const stoppingDistance =
    (speed * speed) / (2 * VEHICLE.maxBraking) +
    speed * (age + period + SAFETY_LIMITS.processingDelay) +
    SAFETY_LIMITS.stopMargin;
  const base = {
    stoppingDistance,
    obstacleDistance: null,
    source: null,
    measurementTime: null,
  };
  if (!autonomous)
    return {
      ...base,
      state: "MANUAL",
      braking: false,
      reason:
        "Manual driving: sensor protection does not override keyboard commands.",
    };
  if (!lidar || !validAge(lidar.timestamp, time) || lidar.rays.length === 0) {
    return {
      ...base,
      state: "STALE_STOP",
      braking: true,
      source: "lidar",
      measurementTime: lidar?.timestamp ?? null,
      reason:
        "Primary LiDAR unavailable, invalid, or older than 0.30 s. Autonomous braking requested.",
    };
  }
  const points: MeasurementPoint[] = lidar.rays
    .filter((ray) => ray.hit)
    .map((ray) => ({
      ...relativeToVehicle(ray.point.x, ray.point.y, vehicle),
      source: "lidar",
      timestamp: lidar.timestamp,
    }));
  points.push(
    ...framePoints(sensors.camera, vehicle, time),
    ...framePoints(sensors.radar, vehicle, time),
  );
  const curvature =
    Math.tan(
      Math.max(-VEHICLE.maxSteering, Math.min(VEHICLE.maxSteering, steering)),
    ) / VEHICLE.wheelbase;
  let threat: { point: MeasurementPoint; distance: number } | null = null;
  for (const point of points) {
    if (![point.x, point.y].every(Number.isFinite)) continue;
    if (Math.hypot(point.x, point.y) > stoppingDistance + VEHICLE.length + 1)
      continue;
    const distance = corridorContact(point, curvature, stoppingDistance);
    if (distance !== null && (!threat || distance < threat.distance))
      threat = { point, distance };
  }
  if (threat)
    return {
      state: "PROTECTIVE_BRAKING",
      braking: true,
      stoppingDistance,
      obstacleDistance: threat.distance,
      source: threat.point.source,
      measurementTime: threat.point.timestamp,
      reason: `${threat.point.source.toUpperCase()} observation at ${threat.point.timestamp.toFixed(2)} s: body-corridor clearance ${threat.distance.toFixed(2)} m is within the ${stoppingDistance.toFixed(2)} m stopping envelope.`,
    };
  return {
    ...base,
    state: "CLEAR",
    braking: false,
    reason:
      "No fresh measurement intersects the immediate stopping corridor. This is not an obstacle-free-world guarantee.",
  };
}

/** Braking never reverses the vehicle. Manual mode always retains command authority. */
export function protectiveControl(
  requested: VehicleControl,
  safety: SafetySnapshot,
  vehicle: VehicleState,
  dt: number,
): VehicleControl {
  if (safety.state === "MANUAL" || !safety.braking) return { ...requested };
  if (!Number.isFinite(dt) || dt <= 0)
    throw new RangeError("Protection dt must be finite and positive");
  return {
    steering: requested.steering,
    acceleration:
      vehicle.velocity === 0
        ? 0
        : -Math.sign(vehicle.velocity) *
          Math.min(VEHICLE.maxBraking, Math.abs(vehicle.velocity) / dt),
  };
}
