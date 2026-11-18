import type { Point } from "../navigation/types";
import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "../world/types";
import type { SensorConfig, SensorPose } from "./types";

const EPSILON = 1e-9;
export function sensorPose(
  vehicle: VehicleState,
  config: SensorConfig,
): SensorPose {
  const c = Math.cos(vehicle.heading),
    s = Math.sin(vehicle.heading);
  return {
    x: vehicle.x + c * config.mount.x - s * config.mount.y,
    y: vehicle.y + s * config.mount.x + c * config.mount.y,
    heading: vehicle.heading + config.yaw,
  };
}
export function wrapAngle(angle: number): number {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}
export function relativePoint(pose: SensorPose, point: Point): Point {
  const dx = point.x - pose.x,
    dy = point.y - pose.y;
  return {
    x: Math.cos(pose.heading) * dx + Math.sin(pose.heading) * dy,
    y: -Math.sin(pose.heading) * dx + Math.cos(pose.heading) * dy,
  };
}
function inside(point: Point, polygon: Point[]): boolean {
  let result = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const a = polygon[i],
      b = polygon[j];
    if (
      a.y > point.y !== b.y > point.y &&
      point.x < ((b.x - a.x) * (point.y - a.y)) / (b.y - a.y) + a.x
    )
      result = !result;
  }
  return result;
}
function cross(a: Point, b: Point): number {
  return a.x * b.y - a.y * b.x;
}

/** Nearest opaque polygon intersection. Origin inside an object returns zero range. */
export function raycast(
  origin: Point,
  angle: number,
  objects: WorldObject[],
  maxRange: number,
): { distance: number; point: Point; object: WorldObject | null } {
  const direction = { x: Math.cos(angle), y: Math.sin(angle) };
  let distance = maxRange,
    object: WorldObject | null = null;
  for (const candidate of objects) {
    if (inside(origin, candidate.polygon))
      return { distance: 0, point: { ...origin }, object: candidate };
    for (let i = 0; i < candidate.polygon.length; i++) {
      const a = candidate.polygon[i],
        b = candidate.polygon[(i + 1) % candidate.polygon.length];
      const edge = { x: b.x - a.x, y: b.y - a.y };
      const offset = { x: a.x - origin.x, y: a.y - origin.y };
      const denominator = cross(direction, edge);
      let t: number;
      if (Math.abs(denominator) < EPSILON) {
        if (Math.abs(cross(offset, direction)) > EPSILON) continue;
        const ta = offset.x * direction.x + offset.y * direction.y;
        const tb =
          (b.x - origin.x) * direction.x + (b.y - origin.y) * direction.y;
        if (Math.max(ta, tb) < -EPSILON) continue;
        t = Math.max(0, Math.min(ta, tb));
      } else {
        t = cross(offset, edge) / denominator;
        const u = cross(offset, direction) / denominator;
        if (t < -EPSILON || u < -EPSILON || u > 1 + EPSILON) continue;
        t = Math.max(0, t);
      }
      if (t <= distance + EPSILON && (!object || t < distance - EPSILON)) {
        distance = Math.min(maxRange, t);
        object = candidate;
      }
    }
  }
  return {
    distance,
    point: {
      x: origin.x + direction.x * distance,
      y: origin.y + direction.y * distance,
    },
    object,
  };
}

/** Bounded uniform noise, hashed from simulation time, seed and measurement index. */
export function deterministicNoise(
  config: SensorConfig,
  time: number,
  index: number,
  channel = 0,
): number {
  let hash =
    (config.seed | 0) ^
    Math.imul(Math.round(time * 1e6), 374761393) ^
    Math.imul(index + 1, 668265263) ^
    Math.imul(channel + 1, 1274126177);
  hash = Math.imul(hash ^ (hash >>> 13), 1274126177);
  hash ^= hash >>> 16;
  return (((hash >>> 0) / 0xffffffff) * 2 - 1) * config.noise;
}

/** Finite polygon sampling and nearest-hit rays provide basic geometric visibility.
 * This is not image segmentation: narrow unsampled slivers can remain undetected. */
export function visibleObjects(
  pose: SensorPose,
  objects: WorldObject[],
  config: SensorConfig,
): { object: WorldObject; point: Point; range: number; bearing: number }[] {
  const results: {
    object: WorldObject;
    point: Point;
    range: number;
    bearing: number;
  }[] = [];
  for (const object of objects) {
    const samples: Point[] = [];
    for (let i = 0; i < object.polygon.length; i++) {
      const a = object.polygon[i],
        b = object.polygon[(i + 1) % object.polygon.length];
      samples.push(a, { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 });
      const dx = b.x - a.x,
        dy = b.y - a.y,
        length2 = dx * dx + dy * dy;
      const t =
        length2 > 0
          ? Math.max(
              0,
              Math.min(
                1,
                ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / length2,
              ),
            )
          : 0;
      samples.push({ x: a.x + t * dx, y: a.y + t * dy });
    }
    // Centre and FOV-boundary rays catch large surfaces spanning the entire FOV.
    for (const bearing of [0, -config.fov / 2, config.fov / 2]) {
      const hit = raycast(
        pose,
        pose.heading + bearing,
        [object],
        config.maxRange,
      );
      if (hit.object) samples.push(hit.point);
    }
    let nearest: {
      object: WorldObject;
      point: Point;
      range: number;
      bearing: number;
    } | null = null;
    for (const sample of samples) {
      const range = Math.hypot(sample.x - pose.x, sample.y - pose.y);
      const bearing = wrapAngle(
        Math.atan2(sample.y - pose.y, sample.x - pose.x) - pose.heading,
      );
      if (
        range > config.maxRange + EPSILON ||
        Math.abs(bearing) > config.fov / 2 + EPSILON
      )
        continue;
      const hit = raycast(
        pose,
        pose.heading + bearing,
        objects,
        Math.min(config.maxRange, range + 1e-6),
      );
      if (hit.object !== object) continue;
      if (!nearest || hit.distance < nearest.range)
        nearest = { object, point: hit.point, range: hit.distance, bearing };
    }
    if (nearest) results.push(nearest);
  }
  return results.sort(
    (a, b) => a.range - b.range || a.object.id.localeCompare(b.object.id),
  );
}
