import { town } from "../world/town";
import { VEHICLE } from "../simulation/vehicle";
import type { LaneEdge, Point, RoadGraph, RoadPosition } from "./types";

const SAMPLE_SPACING = 0.5;
/** Trims leave room for tangent turns with a radius above the vehicle minimum. */
export const INTERSECTION_TRIM = 7.5;
const distance = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y);
const lerp = (a: Point, b: Point, t: number): Point => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
});
function line(a: Point, b: Point): Point[] {
  const count = Math.max(1, Math.ceil(distance(a, b) / SAMPLE_SPACING));
  return Array.from({ length: count + 1 }, (_, i) => lerp(a, b, i / count));
}
interface Port {
  node: string;
  point: Point;
  direction: Point;
  road: string;
}
interface Junction {
  incoming: Port[];
  outgoing: Port[];
}

/** Right-hand traffic, two directed lanes per road, no U-turn connectors. */
export function createRoadGraph(): RoadGraph {
  const graph: RoadGraph = { nodes: {}, edges: [] };
  const junctions: Junction[] = town.intersections.map(() => ({
    incoming: [],
    outgoing: [],
  }));
  const offset = town.roadWidth / 4;
  const addNode = (id: string, point: Point) => {
    graph.nodes[id] = { id, ...point };
    return id;
  };
  const addEdge = (
    id: string,
    from: string,
    to: string,
    points: Point[],
    kind: "lane" | "turn",
    heading: number,
    roadId?: string,
  ) => {
    const length = points
      .slice(1)
      .reduce((sum, p, i) => sum + distance(points[i], p), 0);
    graph.edges.push({ id, from, to, points, length, kind, heading, roadId });
  };
  for (const road of town.roads) {
    for (const direction of [1, -1]) {
      const from = direction === 1 ? road.from : road.to;
      const to = direction === 1 ? road.to : road.from;
      const length = Math.hypot(to[0] - from[0], to[1] - from[1]);
      const u = {
        x: (to[0] - from[0]) / length,
        y: (to[1] - from[1]) / length,
      };
      const pointAt = (s: number): Point => ({
        x: from[0] + u.x * s + u.y * offset,
        y: from[1] + u.y * s - u.x * offset,
      });
      const hits = town.intersections
        .flatMap(([x, y], index) => {
          const t = (x - from[0]) * u.x + (y - from[1]) * u.y;
          const cross = (x - from[0]) * u.y - (y - from[1]) * u.x;
          return Math.abs(cross) < 1e-6 &&
            t > INTERSECTION_TRIM &&
            t < length - INTERSECTION_TRIM
            ? [{ t, index }]
            : [];
        })
        .sort((a, b) => a.t - b.t);
      const prefix = `${road.id}:${direction}`;
      // Keep the complete nominal body within the rendered road at deadends.
      const rearOverhang = (VEHICLE.length - VEHICLE.wheelbase) / 2;
      const frontExtent = VEHICLE.length - rearOverhang;
      let prior = addNode(`${prefix}:start`, pointAt(rearOverhang));
      for (const hit of hits) {
        const entryPoint = pointAt(hit.t - INTERSECTION_TRIM),
          exitPoint = pointAt(hit.t + INTERSECTION_TRIM);
        const entry = addNode(`${prefix}:j${hit.index}:in`, entryPoint);
        const exit = addNode(`${prefix}:j${hit.index}:out`, exitPoint);
        addEdge(
          `${prior}->${entry}`,
          prior,
          entry,
          line(graph.nodes[prior], entryPoint),
          "lane",
          Math.atan2(u.y, u.x),
          road.id,
        );
        junctions[hit.index].incoming.push({
          node: entry,
          point: entryPoint,
          direction: u,
          road: road.id,
        });
        junctions[hit.index].outgoing.push({
          node: exit,
          point: exitPoint,
          direction: u,
          road: road.id,
        });
        prior = exit;
      }
      const end = addNode(`${prefix}:end`, pointAt(length - frontExtent));
      addEdge(
        `${prior}->${end}`,
        prior,
        end,
        line(graph.nodes[prior], graph.nodes[end]),
        "lane",
        Math.atan2(u.y, u.x),
        road.id,
      );
    }
  }
  for (const junction of junctions) {
    for (const incoming of junction.incoming) {
      for (const outgoing of junction.outgoing) {
        const dot =
          incoming.direction.x * outgoing.direction.x +
          incoming.direction.y * outgoing.direction.y;
        if (dot < -0.99) continue; // No U-turns, including a switch into the opposing lane.
        const cross =
          incoming.direction.x * outgoing.direction.y -
          incoming.direction.y * outgoing.direction.x;
        let points: Point[];
        if (dot > 0.99) points = line(incoming.point, outgoing.point);
        else {
          const sign = Math.sign(cross);
          const radius = INTERSECTION_TRIM + sign * offset;
          const normal = { x: -incoming.direction.y, y: incoming.direction.x };
          const center = {
            x: incoming.point.x + sign * radius * normal.x,
            y: incoming.point.y + sign * radius * normal.y,
          };
          const angle = Math.atan2(
            incoming.point.y - center.y,
            incoming.point.x - center.x,
          );
          const count = Math.ceil((radius * Math.PI) / 2 / 0.4);
          points = Array.from({ length: count + 1 }, (_, i) => ({
            x:
              center.x +
              radius * Math.cos(angle + (((sign * Math.PI) / 2) * i) / count),
            y:
              center.y +
              radius * Math.sin(angle + (((sign * Math.PI) / 2) * i) / count),
          }));
          points[0] = { ...incoming.point };
          points[count] = { ...outgoing.point };
        }
        addEdge(
          `${incoming.node}->${outgoing.node}`,
          incoming.node,
          outgoing.node,
          points,
          dot > 0.99 ? "lane" : "turn",
          Math.atan2(incoming.direction.y, incoming.direction.x),
          dot > 0.99 ? incoming.road : undefined,
        );
      }
    }
  }
  return graph;
}

/** Select only straight lane sections. Intersections and non-road clicks are unavailable. */
export function projectToRoad(
  graph: RoadGraph,
  point: Point,
  maxDistance = 4,
): RoadPosition | null {
  if (
    ![point.x, point.y, maxDistance].every(Number.isFinite) ||
    maxDistance < 0
  )
    return null;
  const onRoad = town.roads.some((road) => {
    const dx = road.to[0] - road.from[0],
      dy = road.to[1] - road.from[1],
      length = Math.hypot(dx, dy);
    const along =
      ((point.x - road.from[0]) * dx + (point.y - road.from[1]) * dy) / length;
    const across =
      Math.abs((point.x - road.from[0]) * dy - (point.y - road.from[1]) * dx) /
      length;
    return along >= 0 && along <= length && across <= town.roadWidth / 2;
  });
  if (!onRoad) return null;
  // Crossing centres have several valid headings: ask users to select an approach.
  if (
    town.intersections.some(
      ([x, y]) =>
        Math.abs(point.x - x) <= town.roadWidth / 2 &&
        Math.abs(point.y - y) <= town.roadWidth / 2,
    )
  )
    return null;
  let best: RoadPosition | null = null,
    bestDistance = maxDistance;
  for (const edge of graph.edges) {
    if (edge.kind !== "lane") continue;
    const a = edge.points[0],
      b = edge.points[edge.points.length - 1];
    const dx = b.x - a.x,
      dy = b.y - a.y,
      length = Math.hypot(dx, dy);
    const offset = ((point.x - a.x) * dx + (point.y - a.y) * dy) / length;
    if (offset < -1e-8 || offset > length + 1e-8) continue;
    const projected = {
      x: a.x + (dx * offset) / length,
      y: a.y + (dy * offset) / length,
    };
    const error = distance(point, projected);
    if (error <= bestDistance) {
      bestDistance = error;
      best = {
        ...projected,
        edgeId: edge.id,
        offset: Math.max(0, Math.min(edge.length, offset)),
        heading: edge.heading,
      };
    }
  }
  return best;
}
