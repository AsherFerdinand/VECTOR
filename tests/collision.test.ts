import { describe, expect, it } from "vitest";
import {
  vehicleFootprint,
  polygonsOverlap,
  resolveMotion,
  classifyRoad,
  validateRouteClearance,
} from "../src/simulation/collision";
import {
  worldObjects,
  trafficLightPlacements,
  treePlacements,
} from "../src/world/objects";
import { town } from "../src/world/town";
import { createRoadGraph, projectToRoad } from "../src/navigation/graph";
import { planRoute } from "../src/navigation/route";
import type { VehicleState } from "../src/simulation/types";
import type { WorldObject } from "../src/world/types";
import type { Point } from "../src/navigation/types";

// Predefined acceptance: resolved body penetration <=1e-6 m; known contacts <=1e-6 m.
const pose = (x = 0, y = 0, heading = 0, velocity = 5): VehicleState => ({
  x,
  y,
  heading,
  velocity,
});
const box = (
  x: number,
  y: number,
  width: number,
  depth: number,
  id = "test-wall",
): WorldObject => ({
  id,
  objectClass: "building",
  polygon: [
    { x, y },
    { x: x + width, y },
    { x: x + width, y: y + depth },
    { x, y: y + depth },
  ],
  height: 5,
  velocity: { x: 0, y: 0 },
  collidable: true,
});
const graph = createRoadGraph();
const route = (a: Point, b: Point) =>
  planRoute(graph, projectToRoad(graph, a)!, projectToRoad(graph, b)!)!;

describe("shared solid world geometry", () => {
  it("uses the actual building dimensions, pole placements and tree trunks", () => {
    const objects = worldObjects();
    expect(objects).toHaveLength(
      town.buildings.length +
        trafficLightPlacements.length +
        treePlacements.length,
    );
    expect(objects.filter((o) => o.objectClass === "building")).toHaveLength(
      12,
    );
    expect(
      objects.find((o) => o.id === "traffic-light-0-a")!.polygon[0],
    ).toEqual({ x: -32.665, y: -32.665 });
    expect(objects.find((o) => o.id === "tree-0-13")!.polygon[0]).toEqual({
      x: -0.2,
      y: 12.8,
    });
    const building = objects.find((o) => o.id === "building-0")!,
      b = town.buildings[0];
    expect(building.polygon[0]).toEqual({
      x: b.x - b.width / 2,
      y: b.y - b.depth / 2,
    });
    expect(building.collidable).toBe(true);
  });
});

describe("oriented body footprint and overlap", () => {
  it("places the centre1.35m ahead of rear axle and rotates all four corners", () => {
    expect(vehicleFootprint(pose())[0].x).toBeCloseTo(-0.9);
    expect(vehicleFootprint(pose())[2].x).toBeCloseTo(3.6);
    const footprint = vehicleFootprint(pose(10, 20, Math.PI / 2));
    expect(footprint[0].x).toBeCloseTo(10.9);
    expect(footprint[0].y).toBeCloseTo(19.1);
    expect(vehicleFootprint(pose(), 0.1)[0].x).toBeCloseTo(-1);
  });
  it("detects rotated overlap while treating exact touching as nonpenetrating", () => {
    const wall = box(3.6, -2, 2, 4);
    expect(polygonsOverlap(vehicleFootprint(pose()), wall.polygon)).toBe(false);
    expect(polygonsOverlap(vehicleFootprint(pose(0.01)), wall.polygon)).toBe(
      true,
    );
    expect(
      polygonsOverlap(vehicleFootprint(pose(2, 0, Math.PI / 4)), wall.polygon),
    ).toBe(true);
    expect(polygonsOverlap(vehicleFootprint(pose(-10)), wall.polygon)).toBe(
      false,
    );
  });
});

describe("conservative swept collision response", () => {
  it("preserves collision-free proposed motion", () => {
    const proposed = pose(5, 0, 0.2, 8),
      result = resolveMotion(pose(), proposed, [box(20, 20, 10, 10)]);
    expect(result).toEqual({
      state: proposed,
      collided: false,
      objectId: null,
      fraction: 1,
    });
  });
  it("stops against actual rendered building geometry", () => {
    const building = worldObjects().find(
      (object) => object.id === "building-0",
    )!;
    const result = resolveMotion(pose(-60, -43), pose(-40, -43), [building]);
    expect(result.collided).toBe(true);
    expect(result.state.x).toBeCloseTo(-54.1, 6);
    expect(
      polygonsOverlap(vehicleFootprint(result.state), building.polygon),
    ).toBe(false);
  });
  it("prevents tunnelling through a1mm wall over a100m step", () => {
    const wall = box(20, -5, 0.001, 10),
      result = resolveMotion(pose(), pose(100, 0, 0, 50), [wall]);
    expect(result.collided).toBe(true);
    expect(result.objectId).toBe("test-wall");
    expect(result.state.x).toBeCloseTo(16.4, 6);
    expect(result.state.velocity).toBe(0);
    expect(polygonsOverlap(vehicleFootprint(result.state), wall.polygon)).toBe(
      false,
    );
    expect(result.fraction).toBeCloseTo(0.164, 6);
  });
  it("accounts for rotation even when both endpoint rectangles are free", () => {
    const small = box(2.25, 2.25, 0.05, 0.05);
    const start = pose(0, 0, 0),
      end = pose(0, 0, Math.PI / 2);
    expect(polygonsOverlap(vehicleFootprint(start), small.polygon)).toBe(false);
    expect(polygonsOverlap(vehicleFootprint(end), small.polygon)).toBe(false);
    const result = resolveMotion(start, end, [small]);
    expect(result.collided).toBe(true);
    expect(result.fraction).toBeGreaterThan(0);
    expect(result.fraction).toBeLessThan(1);
    expect(polygonsOverlap(vehicleFootprint(result.state), small.polygon)).toBe(
      false,
    );
  });
  it("allows reversing away after contact and remains deterministic", () => {
    const wall = box(10, -5, 2, 10),
      start = pose(),
      proposed = pose(20);
    const a = resolveMotion(start, proposed, [wall]),
      b = resolveMotion(start, proposed, [wall]);
    expect(a).toEqual(b);
    const away = resolveMotion(
      a.state,
      { ...a.state, x: a.state.x - 1, velocity: -2 },
      [wall],
    );
    expect(away.collided).toBe(false);
    expect(away.state.x).toBeLessThan(a.state.x);
    const touching = pose(6.4, 0, 0, 0);
    expect(
      resolveMotion(touching, { ...touching, x: 5.4, velocity: -1 }, [wall])
        .collided,
    ).toBe(false);
  });
  it("stops an inward command at exact contact and ignores noncollidable geometry", () => {
    const wall = box(10, -5, 2, 10),
      contact = pose(6.4, 0, 0, 0);
    expect(resolveMotion(contact, pose(7.4), [wall]).collided).toBe(true);
    expect(
      resolveMotion(pose(), pose(20), [{ ...wall, collidable: false }])
        .collided,
    ).toBe(false);
  });
});

describe("exact full-footprint road coverage and clearance", () => {
  it("distinguishes on, partially off and fully off the actual road union", () => {
    expect(classifyRoad(pose(0, -29.25))).toBe("ON_ROAD");
    expect(classifyRoad(pose(0, -31.2))).toBe("PARTIALLY_OFF_ROAD");
    expect(classifyRoad(pose(0, 0))).toBe("FULLY_OFF_ROAD");
    expect(classifyRoad(pose(27, -27, Math.PI / 4))).toBe("ON_ROAD");
    // Four corners can lie in the road union while an edge/interior cuts its concavity.
    const acrossCorner = pose(23.15, -32.75, (3 * Math.PI) / 4);
    expect(
      vehicleFootprint(acrossCorner).every((point) =>
        town.roads.some((road) => {
          const horizontal = road.from[1] === road.to[1];
          return horizontal
            ? Math.abs(point.y - road.from[1]) <= town.roadWidth / 2 &&
                point.x >= road.from[0] &&
                point.x <= road.to[0]
            : Math.abs(point.x - road.from[0]) <= town.roadWidth / 2 &&
                point.y >= road.from[1] &&
                point.y <= road.to[1];
        }),
      ),
    ).toBe(true);
    expect(classifyRoad(acrossCorner)).toBe("PARTIALLY_OFF_ROAD");
  });
  it("clears existing straight, right turn and multi-intersection routes", () => {
    for (const [start, end] of [
      [
        { x: -20, y: -29.25 },
        { x: 10, y: -29.25 },
      ],
      [
        { x: -20, y: -29.25 },
        { x: 24.75, y: -45 },
      ],
      [
        { x: -20, y: -29.25 },
        { x: -40, y: -24.75 },
      ],
    ])
      expect(
        validateRouteClearance(route(start, end), worldObjects()),
      ).toMatchObject({ valid: true });
  });
  it("rejects obstacle and road-edge crossings by the full swept footprint", () => {
    const straight = route({ x: -20, y: -29.25 }, { x: 10, y: -29.25 });
    expect(
      validateRouteClearance(straight, [box(-5, -30, 0.001, 2)]),
    ).toMatchObject({
      valid: false,
      reason: "Vehicle footprint intersects test-wall",
    });
    const outside = {
      ...straight,
      points: straight.points.map((p) => ({ ...p, y: p.y - 3 })),
      start: { ...straight.start, y: straight.start.y - 3 },
      destination: { ...straight.destination, y: straight.destination.y - 3 },
    };
    expect(validateRouteClearance(outside, []).valid).toBe(false);
    expect(validateRouteClearance(straight, [], 3).valid).toBe(false);
  });
});

describe("collision engineering validation measurements", () => {
  it("records contact error and bounded sweep without penetration", () => {
    const wall = box(20, -5, 0.001, 10),
      result = resolveMotion(pose(), pose(100, 0, 0, 50), [wall]);
    const error = Math.abs(result.state.x - 16.4);
    expect(error).toBeLessThanOrEqual(1e-6);
    expect(result.state.velocity).toBe(0);
    console.info(
      "COLLISION_VALIDATION",
      JSON.stringify({
        inputTranslationMetres: 100,
        obstacleThicknessMetres: 0.001,
        contactRearAxleX: result.state.x,
        contactPositionErrorMetres: error,
        finalSpeed: result.state.velocity,
        penetrating: polygonsOverlap(
          vehicleFootprint(result.state),
          wall.polygon,
        ),
      }),
    );
  });
});
