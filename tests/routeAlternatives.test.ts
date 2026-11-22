import { describe, expect, it } from "vitest";
import { createRoadGraph, projectToRoad } from "../src/navigation/graph";
import { planRoute } from "../src/navigation/route";
import { planClearedRoute } from "../src/navigation/clearedRoute";
import { validateRouteClearance } from "../src/simulation/collision";
import { worldObjects } from "../src/world/objects";
import type { WorldObject } from "../src/world/types";

const graph = createRoadGraph();
const at = (x: number, y: number) => {
  const position = projectToRoad(graph, { x, y });
  if (!position)
    throw new Error(`Fixture (${x},${y}) must project to a directed lane`);
  return position;
};
const obstacle: WorldObject = {
  id: "mapped-road-block",
  objectClass: "static-obstacle",
  height: 2,
  velocity: { x: 0, y: 0 },
  collidable: true,
  polygon: [
    { x: -1, y: -30.25 },
    { x: 1, y: -30.25 },
    { x: 1, y: -28.25 },
    { x: -1, y: -28.25 },
  ],
};

describe("static map alternatives preserve full-body clearance", () => {
  it("replans an obstacle-blocked shortest path over a valid directed alternative", () => {
    const start = at(-45, -29.25),
      destination = at(-40, -24.75),
      objects = [
        ...worldObjects(),
        {
          ...obstacle,
          polygon: [
            { x: -25.75, y: -1 },
            { x: -23.75, y: -1 },
            { x: -23.75, y: 1 },
            { x: -25.75, y: 1 },
          ],
        },
      ];
    const shortest = planRoute(graph, start, destination);
    expect(shortest).not.toBeNull();
    const invalid = validateRouteClearance(shortest!, objects, 0.02);
    console.info(
      "ALTERNATIVE_ROUTE_INITIAL",
      JSON.stringify({ length: shortest?.length, clearance: invalid }),
    );
    expect(invalid.valid).toBe(false);
    expect(invalid.reason).toContain("intersects");
    const result = planClearedRoute(graph, start, destination, objects, 0.02);
    console.info(
      "ALTERNATIVE_ROUTE_RESULT",
      JSON.stringify({
        length: result.route?.length,
        rerouted: result.rerouted,
        clearance: result.clearance,
      }),
    );
    expect(result.rerouted).toBe(true);
    expect(result.route).not.toBeNull();
    expect(result.clearance?.valid).toBe(true);
    expect(validateRouteClearance(result.route!, objects, 0.02).valid).toBe(
      true,
    );
    expect(result.route!.length).toBeGreaterThan(shortest!.length);
    expect(planClearedRoute(graph, start, destination, objects, 0.02)).toEqual(
      result,
    );
  });
  it("rejects an infeasible route when its only initial lane is blocked", () => {
    const start = at(-20, -29.25),
      destination = at(15, -29.25),
      objects = [...worldObjects(), obstacle];
    const result = planClearedRoute(graph, start, destination, objects, 0.02);
    expect(result.route).toBeNull();
    expect(result.rerouted).toBe(false);
    expect(result.clearance?.valid).toBe(false);
    expect(result.clearance?.reason).toContain("intersects");
  });
  it("retains an already-cleared route rather than inventing unnecessary rerouting", () => {
    const start = at(-20, -29.25),
      destination = at(15, -29.25),
      objects = worldObjects();
    const result = planClearedRoute(graph, start, destination, objects, 0.02);
    expect(result.route).toEqual(planRoute(graph, start, destination));
    expect(result.rerouted).toBe(false);
    expect(result.clearance?.valid).toBe(true);
  });
});
