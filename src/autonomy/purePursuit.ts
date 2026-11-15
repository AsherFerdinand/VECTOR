import type { PathPoint, Point, Route } from "../navigation/types";
import type { VehicleState } from "../simulation/types";
import { clamp, VEHICLE } from "../simulation/vehicle";

export interface PathProjection extends Point {
  s: number;
  index: number;
  crossTrackError: number;
}
export interface PursuitResult {
  steering: number;
  target: Point;
  lookAheadDistance: number;
}

/** Interpolate by arc length; the final point is returned at the route end. */
export function pointAtDistance(points: PathPoint[], distance: number): Point {
  if (points.length === 0)
    throw new Error("A reference path needs at least one point");
  const s = clamp(distance, 0, points[points.length - 1].s);
  for (let i = 1; i < points.length; i++) {
    if (points[i].s >= s) {
      const a = points[i - 1],
        b = points[i];
      const fraction = b.s > a.s ? (s - a.s) / (b.s - a.s) : 0;
      return {
        x: a.x + (b.x - a.x) * fraction,
        y: a.y + (b.y - a.y) * fraction,
      };
    }
  }
  const last = points[points.length - 1];
  return { x: last.x, y: last.y };
}

/** Local monotonic projection prevents jumps to another visit to the same junction. */
export function projectOnPath(
  route: Route,
  vehicle: Point,
  progress: number,
  maxAdvance: number,
): PathProjection {
  const points = route.points;
  let best: PathProjection = {
    ...points[0],
    s: progress,
    index: 0,
    crossTrackError: Infinity,
  };
  let bestDistance = Infinity;
  const lower = Math.max(0, progress - 0.2);
  const upper = Math.min(route.length, progress + maxAdvance);
  for (let i = 0; i < points.length - 1; i++) {
    const a = points[i],
      b = points[i + 1];
    if (b.s < lower || a.s > upper) continue;
    const dx = b.x - a.x,
      dy = b.y - a.y;
    const squareLength = dx * dx + dy * dy;
    if (squareLength < 1e-12) continue;
    const span = b.s - a.s;
    const lo = Math.max(0, (lower - a.s) / span);
    const hi = Math.min(1, (upper - a.s) / span);
    const t = clamp(
      ((vehicle.x - a.x) * dx + (vehicle.y - a.y) * dy) / squareLength,
      lo,
      hi,
    );
    const x = a.x + t * dx,
      y = a.y + t * dy;
    const distance = Math.hypot(vehicle.x - x, vehicle.y - y);
    if (distance < bestDistance) {
      bestDistance = distance;
      const cross = dx * (vehicle.y - y) - dy * (vehicle.x - x);
      best = {
        x,
        y,
        s: Math.max(progress, a.s + t * span),
        index: i,
        crossTrackError: cross / Math.sqrt(squareLength),
      };
    }
  }
  if (points.length === 1) {
    const a = points[0];
    return {
      x: a.x,
      y: a.y,
      s: 0,
      index: 0,
      crossTrackError: Math.hypot(vehicle.x - a.x, vehicle.y - a.y),
    };
  }
  return best;
}

export function purePursuit(
  route: Route,
  vehicle: VehicleState,
  progress: number,
): PursuitResult {
  const requestedLookAhead = 2.5 + 0.35 * Math.max(0, vehicle.velocity);
  const target = pointAtDistance(route.points, progress + requestedLookAhead);
  const dx = target.x - vehicle.x,
    dy = target.y - vehicle.y;
  const lookAheadDistance = Math.hypot(dx, dy);
  if (lookAheadDistance < 1e-6)
    return { steering: 0, target, lookAheadDistance };
  // Transform the target into the rear-axle frame. sin(alpha) = localY / distance.
  const localY =
    -Math.sin(vehicle.heading) * dx + Math.cos(vehicle.heading) * dy;
  const steering = Math.atan2(
    2 * VEHICLE.wheelbase * localY,
    lookAheadDistance * lookAheadDistance,
  );
  return {
    steering: clamp(steering, -VEHICLE.maxSteering, VEHICLE.maxSteering),
    target,
    lookAheadDistance,
  };
}
