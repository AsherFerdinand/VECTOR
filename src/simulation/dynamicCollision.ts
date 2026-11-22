import type { Point } from "../navigation/types";
import type { WorldObject } from "../world/types";
import type { VehicleState } from "./types";
import type { MotionResult } from "./collisionTypes";
import { polygonsOverlap, vehicleFootprint } from "./collision";
import { VEHICLE } from "./vehicle";

const CONTACT_TOLERANCE = 1e-8;
const egoRadius = Math.hypot(
  VEHICLE.length - (VEHICLE.length - VEHICLE.wheelbase) / 2,
  VEHICLE.width / 2,
);
function interpolateEgo(
  a: VehicleState,
  b: VehicleState,
  t: number,
): VehicleState {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
    heading: a.heading + (b.heading - a.heading) * t,
    velocity: a.velocity + (b.velocity - a.velocity) * t,
  };
}
function centre(polygon: Point[]): Point {
  return {
    x: polygon.reduce((s, p) => s + p.x, 0) / polygon.length,
    y: polygon.reduce((s, p) => s + p.y, 0) / polygon.length,
  };
}
interface MovingPolygon {
  id: string;
  previous: Point[];
  next: Point[];
  from: Point;
  to: Point;
  angle: number;
  radius: number;
  travel: number;
}
function movingPolygon(
  previous: WorldObject,
  next: WorldObject,
): MovingPolygon {
  const from = centre(previous.polygon),
    to = centre(next.polygon),
    first = previous.polygon[0],
    second = previous.polygon[1],
    nextFirst = next.polygon[0],
    nextSecond = next.polygon[1];
  const a = Math.atan2(second.y - first.y, second.x - first.x),
    b = Math.atan2(nextSecond.y - nextFirst.y, nextSecond.x - nextFirst.x),
    angle = Math.atan2(Math.sin(b - a), Math.cos(b - a));
  const radius = Math.max(
    ...previous.polygon.map((p) => Math.hypot(p.x - from.x, p.y - from.y)),
  );
  return {
    id: next.id,
    previous: previous.polygon,
    next: next.polygon,
    from,
    to,
    angle,
    radius,
    travel: Math.hypot(to.x - from.x, to.y - from.y) + Math.abs(angle) * radius,
  };
}
/** Rigid planar interpolation preserves shape while accounting for actor rotation. */
function polygonAt(actor: MovingPolygon, t: number): Point[] {
  const c = Math.cos(actor.angle * t),
    s = Math.sin(actor.angle * t),
    x = actor.from.x + (actor.to.x - actor.from.x) * t,
    y = actor.from.y + (actor.to.y - actor.from.y) * t;
  return actor.previous.map((p) => ({
    x: x + (p.x - actor.from.x) * c - (p.y - actor.from.y) * s,
    y: y + (p.x - actor.from.x) * s + (p.y - actor.from.y) * c,
  }));
}
function pointSegmentDistance(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x,
    dy = b.y - a.y,
    length = dx * dx + dy * dy,
    t = length
      ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / length))
      : 0;
  return Math.hypot(p.x - a.x - dx * t, p.y - a.y - dy * t);
}
function separation(a: Point[], b: Point[]): number {
  if (polygonsOverlap(a, b)) return 0;
  let distance = Infinity;
  for (const [points, edges] of [
    [a, b],
    [b, a],
  ])
    for (const p of points)
      for (let i = 0; i < edges.length; i++)
        distance = Math.min(
          distance,
          pointSegmentDistance(p, edges[i], edges[(i + 1) % edges.length]),
        );
  return distance;
}

/**
 * Continuous conservative advancement for convex rigid actor footprints.
 * Each snapshot interval must have fixed shape and rotation under pi; actor scripts
 * run at0.01s, far below this bound. New/disappearing IDs are treated as stationary.
 */
export function resolveDynamicMotion(
  previousEgo: VehicleState,
  proposedEgo: VehicleState,
  previousObjects: WorldObject[],
  nextObjects: WorldObject[],
): MotionResult {
  const previousById = new Map(previousObjects.map((o) => [o.id, o]));
  const nextById = new Map(nextObjects.map((o) => [o.id, o]));
  const ids = new Set([...previousById.keys(), ...nextById.keys()]);
  const actors: MovingPolygon[] = [];
  const egoTravel =
    Math.hypot(proposedEgo.x - previousEgo.x, proposedEgo.y - previousEgo.y) +
    Math.abs(proposedEgo.heading - previousEgo.heading) * egoRadius;
  const minX = Math.min(previousEgo.x, proposedEgo.x) - egoRadius,
    maxX = Math.max(previousEgo.x, proposedEgo.x) + egoRadius,
    minY = Math.min(previousEgo.y, proposedEgo.y) - egoRadius,
    maxY = Math.max(previousEgo.y, proposedEgo.y) + egoRadius;
  for (const id of ids) {
    const before = previousById.get(id) ?? nextById.get(id)!,
      after = nextById.get(id) ?? before;
    if (
      (!before.collidable && !after.collidable) ||
      before.polygon.length < 3 ||
      after.polygon.length < 3
    )
      continue;
    const actor = movingPolygon(before, after);
    if (
      Math.max(actor.from.x, actor.to.x) + actor.radius < minX ||
      Math.min(actor.from.x, actor.to.x) - actor.radius > maxX ||
      Math.max(actor.from.y, actor.to.y) + actor.radius < minY ||
      Math.min(actor.from.y, actor.to.y) - actor.radius > maxY
    )
      continue;
    actors.push(actor);
  }
  const free = (): MotionResult => ({
    state: { ...proposedEgo },
    collided: false,
    objectId: null,
    fraction: 1,
  });
  if (!actors.length) return free();
  if (
    actors.every((actor) => egoTravel + actor.travel < 1e-14) &&
    !actors.some((actor) =>
      polygonsOverlap(vehicleFootprint(previousEgo), actor.previous),
    )
  )
    return free();
  let fraction = 0,
    lastSafe = 0;
  const contact = (t: number) =>
    actors.find(
      (actor) =>
        separation(
          vehicleFootprint(interpolateEgo(previousEgo, proposedEgo, t)),
          polygonAt(actor, t),
        ) <= CONTACT_TOLERANCE,
    );
  for (let iteration = 0; iteration < 256; iteration++) {
    const body = vehicleFootprint(
      interpolateEgo(previousEgo, proposedEgo, fraction),
    );
    let safeStep = Infinity,
      nearest: MovingPolygon | null = null,
      minimum = Infinity;
    for (const actor of actors) {
      const distance = separation(body, polygonAt(actor, fraction));
      if (distance < minimum) {
        minimum = distance;
        nearest = actor;
      }
      const relativeTravel = egoTravel + actor.travel;
      if (relativeTravel > 1e-14)
        safeStep = Math.min(safeStep, (distance / relativeTravel) * 0.8);
    }
    if (minimum <= CONTACT_TOLERANCE) {
      const maximumTravel = Math.max(
        ...actors.map((a) => egoTravel + a.travel),
      );
      const probe = Math.min(
        1,
        fraction + 1e-7 / Math.max(maximumTravel, 1e-12),
      );
      if (
        fraction === 0 &&
        !actors.some((actor) => polygonsOverlap(body, polygonAt(actor, 0))) &&
        !contact(probe)
      ) {
        fraction = probe;
        lastSafe = probe;
        continue;
      }
      return {
        state: {
          ...interpolateEgo(previousEgo, proposedEgo, lastSafe),
          velocity: 0,
        },
        collided: true,
        objectId: nearest?.id ?? null,
        fraction: lastSafe,
      };
    }
    if (fraction >= 1 || !Number.isFinite(safeStep)) return free();
    lastSafe = fraction;
    fraction = Math.min(1, fraction + safeStep);
  }
  return {
    state: {
      ...interpolateEgo(previousEgo, proposedEgo, lastSafe),
      velocity: 0,
    },
    collided: true,
    objectId: actors[0].id,
    fraction: lastSafe,
  };
}
