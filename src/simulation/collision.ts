import type { Point, Route } from "../navigation/types";
import type { WorldObject } from "../world/types";
import { town } from "../world/town";
import { VEHICLE } from "./vehicle";
import type { VehicleState } from "./types";
import type {
  ClearanceResult,
  MotionResult,
  RoadStatus,
} from "./collisionTypes";

const CONTACT_TOLERANCE = 1e-8;
const AREA_TOLERANCE = 1e-8;
const cross = (a: Point, b: Point, p: Point) =>
  (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
const interpolate = (
  a: VehicleState,
  b: VehicleState,
  t: number,
): VehicleState => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
  heading: a.heading + (b.heading - a.heading) * t,
  velocity: a.velocity + (b.velocity - a.velocity) * t,
});
const rearOverhang = (VEHICLE.length - VEHICLE.wheelbase) / 2;
const bodyCenter = VEHICLE.wheelbase / 2;
const bodyRadius = Math.hypot(VEHICLE.length - rearOverhang, VEHICLE.width / 2);

/** Rear axle origin; the geometric body centre is 1.35m forward of it. */
export function vehicleFootprint(state: VehicleState, margin = 0): Point[] {
  if (!Number.isFinite(margin) || margin < 0)
    throw new RangeError("Footprint margin must be finite and nonnegative");
  const c = Math.cos(state.heading),
    s = Math.sin(state.heading),
    halfLength = VEHICLE.length / 2 + margin,
    halfWidth = VEHICLE.width / 2 + margin;
  return [
    [-halfLength, -halfWidth],
    [halfLength, -halfWidth],
    [halfLength, halfWidth],
    [-halfLength, halfWidth],
  ].map(([x, y]) => ({
    x: state.x + (bodyCenter + x) * c - y * s,
    y: state.y + (bodyCenter + x) * s + y * c,
  }));
}

/** SAT for the convex, counterclockwise rectangles/polygons used by this simulator. */
export function polygonsOverlap(a: Point[], b: Point[]): boolean {
  if (a.length < 3 || b.length < 3) return false;
  for (const polygon of [a, b])
    for (let i = 0; i < polygon.length; i++) {
      const p = polygon[i],
        q = polygon[(i + 1) % polygon.length],
        axis = { x: -(q.y - p.y), y: q.x - p.x };
      const length = Math.hypot(axis.x, axis.y);
      if (length < 1e-12) continue;
      const pa = a.map((v) => (v.x * axis.x + v.y * axis.y) / length),
        pb = b.map((v) => (v.x * axis.x + v.y * axis.y) / length);
      if (
        Math.max(...pa) <= Math.min(...pb) + CONTACT_TOLERANCE ||
        Math.max(...pb) <= Math.min(...pa) + CONTACT_TOLERANCE
      )
        return false;
    }
  return true;
}
function pointSegmentDistance(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x,
    dy = b.y - a.y,
    length2 = dx * dx + dy * dy;
  const t = length2
    ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / length2))
    : 0;
  return Math.hypot(p.x - a.x - dx * t, p.y - a.y - dy * t);
}
function polygonDistance(a: Point[], b: Point[]): number {
  if (polygonsOverlap(a, b)) return 0;
  let minimum = Infinity;
  for (const [vertices, edges] of [
    [a, b],
    [b, a],
  ])
    for (const p of vertices)
      for (let i = 0; i < edges.length; i++)
        minimum = Math.min(
          minimum,
          pointSegmentDistance(p, edges[i], edges[(i + 1) % edges.length]),
        );
  return minimum;
}
function bounds(points: Point[]) {
  return {
    minX: Math.min(...points.map((p) => p.x)),
    maxX: Math.max(...points.map((p) => p.x)),
    minY: Math.min(...points.map((p) => p.y)),
    maxY: Math.max(...points.map((p) => p.y)),
  };
}

/** Conservative advancement bounds every body point's motion, including rotation. */
export function resolveMotion(
  previous: VehicleState,
  proposed: VehicleState,
  objects: WorldObject[],
): MotionResult {
  const speedBound =
    Math.hypot(proposed.x - previous.x, proposed.y - previous.y) +
    Math.abs(proposed.heading - previous.heading) * bodyRadius;
  const swept = {
    minX: Math.min(previous.x, proposed.x) - bodyRadius,
    maxX: Math.max(previous.x, proposed.x) + bodyRadius,
    minY: Math.min(previous.y, proposed.y) - bodyRadius,
    maxY: Math.max(previous.y, proposed.y) + bodyRadius,
  };
  const candidates = objects.filter((object) => {
    if (!object.collidable) return false;
    const b = bounds(object.polygon);
    return (
      b.minX <= swept.maxX &&
      b.maxX >= swept.minX &&
      b.minY <= swept.maxY &&
      b.maxY >= swept.minY
    );
  });
  const result = (
    state: VehicleState,
    collided = false,
    objectId: string | null = null,
    fraction = 1,
  ): MotionResult => ({ state, collided, objectId, fraction });
  if (!candidates.length || speedBound < 1e-14) return result({ ...proposed });
  let fraction = 0,
    lastSafe = 0;
  for (let iteration = 0; iteration < 256; iteration++) {
    const state = interpolate(previous, proposed, fraction),
      footprint = vehicleFootprint(state);
    let separation = Infinity,
      nearest: WorldObject | null = null;
    for (const object of candidates) {
      const d = polygonDistance(footprint, object.polygon);
      if (d < separation) {
        separation = d;
        nearest = object;
      }
    }
    if (separation <= CONTACT_TOLERANCE) {
      // Exact initial contact is allowed to separate; movement into contact is stopped.
      const probe = Math.min(1, fraction + 1e-7 / Math.max(speedBound, 1e-12));
      if (
        fraction === 0 &&
        !candidates.some((o) => polygonsOverlap(footprint, o.polygon)) &&
        candidates.every(
          (o) =>
            polygonDistance(
              vehicleFootprint(interpolate(previous, proposed, probe)),
              o.polygon,
            ) > CONTACT_TOLERANCE,
        )
      ) {
        fraction = probe;
        lastSafe = probe;
        continue;
      }
      let low = lastSafe,
        high = fraction;
      for (let i = 0; i < 30 && high - low > 1e-12; i++) {
        const mid = (low + high) / 2;
        const body = vehicleFootprint(interpolate(previous, proposed, mid));
        if (candidates.some((o) => polygonsOverlap(body, o.polygon)))
          high = mid;
        else low = mid;
      }
      return result(
        { ...interpolate(previous, proposed, low), velocity: 0 },
        true,
        nearest?.id ?? null,
        low,
      );
    }
    if (fraction >= 1) return result({ ...proposed });
    lastSafe = fraction;
    // Distance / boundary-motion bound cannot cross any obstacle, however thin.
    fraction = Math.min(1, fraction + (separation / speedBound) * 0.8);
  }
  // A degenerate/tangential approach may converge slowly: stop conservatively.
  const state = interpolate(previous, proposed, lastSafe);
  return result({ ...state, velocity: 0 }, true, candidates[0].id, lastSafe);
}

function clipHalfPlane(
  subject: Point[],
  a: Point,
  b: Point,
  inside: boolean,
): Point[] {
  const result: Point[] = [];
  if (!subject.length) return result;
  for (let i = 0; i < subject.length; i++) {
    const p = subject[i],
      q = subject[(i + 1) % subject.length],
      dp = cross(a, b, p),
      dq = cross(a, b, q);
    const pIn = inside ? dp >= 0 : dp <= 0,
      qIn = inside ? dq >= 0 : dq <= 0;
    if (pIn) result.push(p);
    if (pIn !== qIn) {
      const t = dp / (dp - dq);
      result.push({ x: p.x + (q.x - p.x) * t, y: p.y + (q.y - p.y) * t });
    }
  }
  return result;
}
const polygonArea = (polygon: Point[]) =>
  Math.abs(
    polygon.reduce((sum, p, i) => {
      const q = polygon[(i + 1) % polygon.length];
      return sum + p.x * q.y - p.y * q.x;
    }, 0),
  ) / 2;
const roadPolygons: Point[][] = town.roads.map((road) => {
  const dx = road.to[0] - road.from[0],
    dy = road.to[1] - road.from[1],
    length = Math.hypot(dx, dy),
    nx = ((-dy / length) * town.roadWidth) / 2,
    ny = ((dx / length) * town.roadWidth) / 2;
  return [
    { x: road.from[0] - nx, y: road.from[1] - ny },
    { x: road.to[0] - nx, y: road.to[1] - ny },
    { x: road.to[0] + nx, y: road.to[1] + ny },
    { x: road.from[0] + nx, y: road.from[1] + ny },
  ];
});
/** Remove one convex road polygon, retaining disjoint convex outside fragments. */
function subtractRoad(subject: Point[], road: Point[]): Point[][] {
  let inside = subject;
  const outside: Point[][] = [];
  for (let i = 0; i < road.length && inside.length; i++) {
    const a = road[i],
      b = road[(i + 1) % road.length],
      fragment = clipHalfPlane(inside, a, b, false);
    if (fragment.length >= 3 && polygonArea(fragment) > AREA_TOLERANCE)
      outside.push(fragment);
    inside = clipHalfPlane(inside, a, b, true);
  }
  return outside;
}
function classifyFootprint(footprint: Point[]): RoadStatus {
  let uncovered = [footprint];
  for (const road of roadPolygons)
    uncovered = uncovered.flatMap((p) => subtractRoad(p, road));
  const outside = uncovered.reduce((sum, p) => sum + polygonArea(p), 0),
    area = polygonArea(footprint);
  if (outside <= AREA_TOLERANCE) return "ON_ROAD";
  return area - outside <= AREA_TOLERANCE
    ? "FULLY_OFF_ROAD"
    : "PARTIALLY_OFF_ROAD";
}
/** Exact polygon-area coverage by the union of map road rectangles, not centre tests. */
export function classifyRoad(state: VehicleState): RoadStatus {
  return classifyFootprint(vehicleFootprint(state));
}

/** Nominal swept-body preflight, 2cm margin by default; not a controller guarantee. */
export function validateRouteClearance(
  route: Route,
  objects: WorldObject[],
  margin = 0.02,
  checkRoad = true,
): ClearanceResult {
  if (!Number.isFinite(margin) || margin < 0)
    throw new RangeError("Route margin must be finite and nonnegative");
  if (route.points.length < 2)
    return {
      valid: false,
      reason: "Reference path has fewer than two points",
      distance: null,
    };
  const poses = route.points.map((p, i): VehicleState => {
    const before = route.points[Math.max(0, i - 1)],
      after = route.points[Math.min(route.points.length - 1, i + 1)];
    let heading =
      i === 0
        ? route.start.heading
        : i === route.points.length - 1
          ? route.destination.heading
          : Math.atan2(after.y - before.y, after.x - before.x);
    return { x: p.x, y: p.y, heading, velocity: 0 };
  });
  // Unwrap headings so crossing +/-pi does not introduce a fictitious full turn.
  for (let i = 1; i < poses.length; i++)
    poses[i].heading =
      poses[i - 1].heading +
      Math.atan2(
        Math.sin(poses[i].heading - poses[i - 1].heading),
        Math.cos(poses[i].heading - poses[i - 1].heading),
      );
  for (let i = 0; i < poses.length; i++) {
    const a = poses[Math.max(0, i - 1)],
      b = poses[i];
    const boundaryTravel =
      Math.hypot(b.x - a.x, b.y - a.y) +
      Math.abs(b.heading - a.heading) *
        Math.hypot(
          VEHICLE.length - rearOverhang + margin,
          VEHICLE.width / 2 + margin,
        );
    const count = Math.max(1, Math.ceil(boundaryTravel / 0.04));
    // Additional half-step inflation bounds all footprints between sampled poses.
    const intervalMargin = margin + boundaryTravel / count / 2;
    for (let j = 0; j <= count; j++) {
      const pose = interpolate(a, b, j / count),
        footprint = vehicleFootprint(pose, intervalMargin);
      const s =
        i === 0
          ? 0
          : route.points[i - 1].s +
            ((route.points[i].s - route.points[i - 1].s) * j) / count;
      if (checkRoad && classifyFootprint(footprint) !== "ON_ROAD")
        return {
          valid: false,
          reason: "Vehicle footprint or safety margin leaves drivable road",
          distance: s,
        };
      const obstacle = objects.find(
        (o) => o.collidable && polygonsOverlap(footprint, o.polygon),
      );
      if (obstacle)
        return {
          valid: false,
          reason: `Vehicle footprint intersects ${obstacle.id}`,
          distance: s,
        };
    }
  }
  return {
    valid: true,
    reason: "Nominal swept vehicle footprint clears roads and static obstacles",
    distance: null,
  };
}

/** Conservative swept road coverage. Half-interval inflation bounds every body
 * point between samples, including rotation; this is not a position correction. */
export function classifyRoadMotion(
  previous: VehicleState,
  proposed: VehicleState,
): RoadStatus {
  const boundaryTravel =
    Math.hypot(proposed.x - previous.x, proposed.y - previous.y) +
    Math.abs(proposed.heading - previous.heading) * bodyRadius;
  if (boundaryTravel < 1e-12) return classifyRoad(proposed);
  const count = Math.max(1, Math.ceil(boundaryTravel / 0.01));
  const margin = boundaryTravel / count / 2;
  let status: RoadStatus = "ON_ROAD";
  for (let i = 0; i <= count; i++) {
    const sample = classifyFootprint(
      vehicleFootprint(interpolate(previous, proposed, i / count), margin),
    );
    if (sample === "FULLY_OFF_ROAD") return sample;
    if (sample === "PARTIALLY_OFF_ROAD") status = sample;
  }
  return status;
}
