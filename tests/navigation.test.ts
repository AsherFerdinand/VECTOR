import { describe, expect, it } from "vitest";
import {
  createRoadGraph,
  projectToRoad,
  INTERSECTION_TRIM,
} from "../src/navigation/graph";
import { aStar } from "../src/navigation/astar";
import { isRoadPosition, planRoute } from "../src/navigation/route";
import { town } from "../src/world/town";
import { VEHICLE } from "../src/simulation/vehicle";
import type { Point, RoadPosition } from "../src/navigation/types";

const graph = createRoadGraph();
const select = (x: number, y: number): RoadPosition => {
  const result = projectToRoad(graph, { x, y });
  expect(result).not.toBeNull();
  return result!;
};
const onPavement = (p: Point) =>
  town.roads.some((road) => {
    const dx = road.to[0] - road.from[0],
      dy = road.to[1] - road.from[1],
      length = Math.hypot(dx, dy);
    const along =
      ((p.x - road.from[0]) * dx + (p.y - road.from[1]) * dy) / length;
    const across =
      Math.abs((p.x - road.from[0]) * dy - (p.y - road.from[1]) * dx) / length;
    return (
      along >= -1e-6 &&
      along <= length + 1e-6 &&
      across <= town.roadWidth / 2 + 1e-6
    );
  });

describe("directed map-derived road graph", () => {
  it("builds two directed lanes per road section and 12 connectors per intersection", () => {
    expect(graph.edges.filter((e) => e.kind === "lane")).toHaveLength(
      town.roads.length * 2 * 3 + town.intersections.length * 4,
    );
    expect(graph.edges.filter((e) => e.kind === "turn")).toHaveLength(
      town.intersections.length * 8,
    );
    for (const edge of graph.edges) {
      expect(graph.nodes[edge.from]).toBeDefined();
      expect(graph.nodes[edge.to]).toBeDefined();
      expect(edge.length).toBeGreaterThan(0);
      expect(edge.points[0]).toMatchObject({
        x: graph.nodes[edge.from].x,
        y: graph.nodes[edge.from].y,
      });
      expect(edge.points.at(-1)).toMatchObject({
        x: graph.nodes[edge.to].x,
        y: graph.nodes[edge.to].y,
      });
    }
  });
  it("keeps every reference point on rendered pavement, outside building footprints", () => {
    for (const edge of graph.edges)
      for (const point of edge.points) {
        expect(onPavement(point), `${edge.id} ${JSON.stringify(point)}`).toBe(
          true,
        );
        for (const b of town.buildings)
          expect(
            Math.abs(point.x - b.x) < b.width / 2 &&
              Math.abs(point.y - b.y) < b.depth / 2,
          ).toBe(false);
      }
  });
  it("keeps the nominal full body on pavement through turns and at road ends", () => {
    // Perimeter samples at <=0.2m catch intrusion into concave sidewalk corners.
    const rear = (VEHICLE.length - VEHICLE.wheelbase) / 2;
    const front = VEHICLE.length - rear;
    const halfWidth = VEHICLE.width / 2;
    const perimeter: Point[] = [];
    const longSamples = Math.ceil(VEHICLE.length / 0.2),
      wideSamples = Math.ceil(VEHICLE.width / 0.2);
    for (let i = 0; i <= longSamples; i++)
      for (const y of [-halfWidth, halfWidth])
        perimeter.push({ x: -rear + (VEHICLE.length * i) / longSamples, y });
    for (let i = 0; i <= wideSamples; i++)
      for (const x of [-rear, front])
        perimeter.push({
          x,
          y: -halfWidth + (VEHICLE.width * i) / wideSamples,
        });
    const signedRoadDistance = (point: Point) =>
      Math.min(
        ...town.roads.map((road) => {
          const dx = road.to[0] - road.from[0],
            dy = road.to[1] - road.from[1],
            length = Math.hypot(dx, dy);
          const along =
            ((point.x - road.from[0]) * dx + (point.y - road.from[1]) * dy) /
            length;
          const across =
            ((point.x - road.from[0]) * dy - (point.y - road.from[1]) * dx) /
            length;
          const a = Math.abs(along - length / 2) - length / 2,
            b = Math.abs(across) - town.roadWidth / 2;
          return (
            Math.hypot(Math.max(a, 0), Math.max(b, 0)) +
            Math.min(Math.max(a, b), 0)
          );
        }),
      );
    let maximumOutside = 0,
      offending = "";
    for (const edge of graph.edges) {
      const outgoing = graph.edges.find(
        (e) => e.from === edge.to && e.kind === "lane",
      );
      const turn =
        edge.kind === "turn" && outgoing
          ? Math.atan2(
              Math.sin(outgoing.heading - edge.heading),
              Math.cos(outgoing.heading - edge.heading),
            )
          : 0;
      edge.points.forEach((pose, index) => {
        const heading =
            edge.heading + (turn * index) / (edge.points.length - 1),
          c = Math.cos(heading),
          s = Math.sin(heading);
        for (const local of perimeter) {
          const point = {
            x: pose.x + local.x * c - local.y * s,
            y: pose.y + local.x * s + local.y * c,
          };
          const outside = signedRoadDistance(point);
          if (outside > maximumOutside) {
            maximumOutside = outside;
            offending = JSON.stringify({ edge: edge.id, index, point });
          }
        }
      });
    }
    expect(maximumOutside, offending).toBeLessThanOrEqual(1e-7);
  });
  it("uses tangent turn geometry compatible with the steering bound and disallows U-turns", () => {
    const minRadius = VEHICLE.wheelbase / Math.tan(VEHICLE.maxSteering);
    expect(INTERSECTION_TRIM - town.roadWidth / 4).toBeGreaterThan(minRadius);
    for (const edge of graph.edges.filter((e) => e.kind === "turn")) {
      const following = graph.edges.find(
        (e) => e.from === edge.to && e.kind === "lane",
      )!;
      const dot = Math.cos(following.heading - edge.heading);
      expect(dot).toBeGreaterThan(-0.01);
      const start = edge.points[0],
        next = edge.points[1],
        last = edge.points.at(-1)!,
        before = edge.points.at(-2)!;
      const startAngle = Math.atan2(next.y - start.y, next.x - start.x),
        endAngle = Math.atan2(last.y - before.y, last.x - before.x);
      expect(Math.cos(startAngle - edge.heading)).toBeGreaterThan(0.999);
      expect(Math.cos(endAngle - following.heading)).toBeGreaterThan(0.999);
    }
  });
  it("projects onto lane centres with correct travel heading", () => {
    const east = select(-20, -30),
      west = select(-20, -25);
    expect(east.y).toBe(-29.25);
    expect(east.heading).toBe(0);
    expect(west.y).toBe(-24.75);
    expect(Math.cos(west.heading)).toBeCloseTo(-1);
    expect(isRoadPosition(graph, east)).toBe(true);
    expect(isRoadPosition(graph, { ...east, heading: Math.PI })).toBe(false);
  });
  it("rejects offroad, intersection-only, nonfinite and malformed inputs", () => {
    expect(projectToRoad(graph, { x: 0, y: 0 })).toBeNull();
    expect(projectToRoad(graph, { x: 0, y: -34 })).toBeNull();
    expect(projectToRoad(graph, { x: 27, y: 27 })).toBeNull();
    expect(projectToRoad(graph, { x: NaN, y: 27 })).toBeNull();
    expect(projectToRoad(graph, { x: 0, y: -27 }, -1)).toBeNull();
    expect(isRoadPosition(graph, { ...select(-20, -29.25), offset: -1 })).toBe(
      false,
    );
    expect(isRoadPosition(graph, { ...select(-20, -29.25), x: 999 })).toBe(
      false,
    );
  });
});

describe("A* and reference route construction", () => {
  it("finds the forward same-lane partial route without visiting an intersection", () => {
    const start = select(-15, -29.25),
      destination = select(10, -29.25);
    const route = planRoute(graph, start, destination)!;
    expect(route.length).toBeCloseTo(25);
    expect(route.edgeIds).toEqual([start.edgeId]);
    expect(route.points.every((p) => p.y === -29.25 && p.curvature === 0)).toBe(
      true,
    );
    expect(route.points[0]).toMatchObject({ x: start.x, y: start.y, s: 0 });
    expect(route.points.at(-1)).toMatchObject({
      x: destination.x,
      y: destination.y,
      s: 25,
    });
  });
  it("plans a connected tangent right turn and preserves dense arc samples", () => {
    const route = planRoute(graph, select(-20, -29.25), select(24.75, -45))!;
    expect(route).not.toBeNull();
    expect(route.points.some((p) => p.curvature < -0.1)).toBe(true);
    const maxCurvature = Math.max(
      ...route.points.map((p) => Math.abs(p.curvature)),
    );
    expect(maxCurvature).toBeLessThan(
      Math.tan(VEHICLE.maxSteering) / VEHICLE.wheelbase,
    );
    for (let i = 1; i < route.points.length; i++) {
      expect(route.points[i].s - route.points[i - 1].s).toBeLessThanOrEqual(
        0.500001,
      );
      expect(onPavement(route.points[i])).toBe(true);
    }
  });
  it("routes through several intersections instead of driving backward on a lane", () => {
    const start = select(-20, -29.25),
      destination = select(-40, -24.75);
    const route = planRoute(graph, start, destination)!;
    expect(route).not.toBeNull();
    expect(route.length).toBeGreaterThan(150);
    const turns = route.edgeIds
      .map((id) => graph.edges.find((e) => e.id === id)!)
      .filter((e) => e.kind === "turn");
    expect(turns.length).toBeGreaterThanOrEqual(3);
  });
  it("handles a destination behind the start on the same lane using a directed loop", () => {
    const route = planRoute(graph, select(0, -29.25), select(-10, -29.25))!;
    expect(route).not.toBeNull();
    expect(route.length).toBeGreaterThan(100);
    expect(route.edgeIds[0]).toBe(route.edgeIds.at(-1));
  });
  it("returns unavailable for a deadend, invalid poses, and a zero-distance selection", () => {
    expect(
      planRoute(graph, select(50, -29.25), select(-20, -29.25)),
    ).toBeNull();
    const start = select(-20, -29.25);
    expect(planRoute(graph, start, { ...start, edgeId: "missing" })).toBeNull();
    expect(
      planRoute(graph, { ...start, heading: 1 }, select(0, -29.25)),
    ).toBeNull();
    expect(planRoute(graph, start, start)).toBeNull();
    expect(aStar(graph, "missing", "missing")).toBeNull();
  });
  it("returns deterministic shortest routes and accepts a zero-edge graph query", () => {
    const start = select(-20, -29.25),
      destination = select(-40, -24.75);
    expect(planRoute(graph, start, destination)).toEqual(
      planRoute(createRoadGraph(), start, destination),
    );
    const node = Object.keys(graph.nodes)[0];
    expect(aStar(graph, node, node)).toEqual([]);
    const full = planRoute(graph, start, destination)!;
    const edges = full.edgeIds.map((id) =>
      graph.edges.find((e) => e.id === id)!,
    );
    for (let i = 1; i < edges.length; i++)
      expect(edges[i - 1].to).toBe(edges[i].from);
  });
});
