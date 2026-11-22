import type { VehicleState } from "../simulation/types";
import type { Track } from "../perception/types";
import type { WorldObject } from "../world/types";
import { VEHICLE } from "../simulation/vehicle";
import { polygonsOverlap, vehicleFootprint } from "../simulation/collision";
import { corridorContains, laneY, type CorridorLane } from "./corridor";
import type { TrajectoryCandidate } from "./types";

/** Quintic lateral profile has zero lateral velocity and acceleration at both ends.
 * Constant longitudinal speed: this is a geometric candidate, not a teleport. */
export function laneChangeCandidate(
  vehicle: VehicleState,
  lane: CorridorLane,
  speed = Math.max(vehicle.velocity, 3),
  duration = 6,
): TrajectoryCandidate {
  const dy = laneY(lane) - vehicle.y,
    count = Math.ceil(duration / 0.025),
    points = [];
  for (let i = 0; i <= count; i++) {
    const t = (duration * i) / count,
      u = t / duration;
    const y = vehicle.y + dy * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5);
    const yd = (dy * (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4)) / duration;
    const ydd = (dy * (60 * u - 180 * u ** 2 + 120 * u ** 3)) / duration ** 2;
    const velocity = Math.hypot(speed, yd),
      curvature = (speed * ydd) / (speed * speed + yd * yd) ** 1.5;
    points.push({
      x: vehicle.x + speed * t,
      y,
      heading: Math.atan2(yd, speed),
      velocity,
      time: t,
      curvature,
    });
  }
  return {
    id: `to-${lane}`,
    targetLane: lane,
    points,
    valid: true,
    reasons: [],
    maxCurvature: Math.max(...points.map((p) => Math.abs(p.curvature))),
    maxLateralAcceleration: Math.max(
      ...points.map((p) => p.velocity * p.velocity * Math.abs(p.curvature)),
    ),
    duration,
  };
}

/** Inflated sample bodies bound motion between samples. Dynamic envelopes use
 * 2-sigma position+velocity covariance, plus conservative vehicle dimensions.
 * Unknown classes are vehicle-sized; no hidden object identity/geometry enters. */
export function validateCandidate(
  candidate: TrajectoryCandidate,
  tracks: Track[],
  now: number,
  staticObjects: WorldObject[] = [],
  destinationX = 150,
): TrajectoryCandidate {
  const reasons = new Set<string>(),
    points = candidate.points;
  for (let i = 0; i < points.length; i++) {
    const p = points[i],
      prev = points[Math.max(0, i - 1)];
    const boundaryTravel =
      Math.hypot(p.x - prev.x, p.y - prev.y) +
      4 * Math.abs(p.heading - prev.heading);
    const footprint = vehicleFootprint(p, 0.1 + boundaryTravel / 2);
    if (!corridorContains(p, 0.1 + boundaryTravel / 2))
      reasons.add("Swept vehicle footprint leaves eastbound road");
    if (p.x > destinationX) reasons.add("Manoeuvre exceeds destination");
    if (p.velocity > 8 || p.velocity < 0)
      reasons.add("Speed outside corridor limit");
    if (
      Math.abs(Math.atan(VEHICLE.wheelbase * p.curvature)) > VEHICLE.maxSteering
    )
      reasons.add("Steering limit");
    if (p.velocity * p.velocity * Math.abs(p.curvature) > 1.5)
      reasons.add("Lateral acceleration limit");
    if (i > 0) {
      const a = (p.velocity - prev.velocity) / (p.time - prev.time);
      if (
        a > VEHICLE.maxAcceleration + 0.001 ||
        a < -VEHICLE.maxBraking - 0.001
      )
        reasons.add("Longitudinal acceleration limit");
    }
    if (
      staticObjects.some(
        (o) => o.collidable && polygonsOverlap(footprint, o.polygon),
      )
    )
      reasons.add("Static obstacle intersects swept footprint");
    for (const track of tracks) {
      const age = Math.max(0, now - track.lastMeasurementTime),
        t = p.time + age;
      const varianceX = Math.max(
        0,
        (track.covariance[0]?.[0] ?? 1) +
          t * t * (track.covariance[2]?.[2] ?? 1),
      );
      const varianceY = Math.max(
        0,
        (track.covariance[1]?.[1] ?? 1) +
          t * t * (track.covariance[3]?.[3] ?? 1),
      );
      const halfX =
        (track.objectClass === "pedestrian" ? 0.5 : 2.5) +
        2 * Math.sqrt(varianceX);
      const halfY =
        (track.objectClass === "pedestrian" ? 0.5 : 1) +
        2 * Math.sqrt(varianceY);
      const x = track.x + track.vx * p.time,
        y = track.y + track.vy * p.time;
      if (
        polygonsOverlap(footprint, [
          { x: x - halfX, y: y - halfY },
          { x: x + halfX, y: y - halfY },
          { x: x + halfX, y: y + halfY },
          { x: x - halfX, y: y + halfY },
        ])
      )
        reasons.add(
          `Predicted uncertain track ${track.id} intersects trajectory`,
        );
    }
  }
  candidate.reasons = [...reasons];
  candidate.valid = !reasons.size;
  return candidate;
}
