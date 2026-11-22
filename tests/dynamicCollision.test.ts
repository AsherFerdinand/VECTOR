import { describe, expect, it } from "vitest";
import { resolveDynamicMotion } from "../src/simulation/dynamicCollision";
import { vehicleFootprint, polygonsOverlap } from "../src/simulation/collision";
import { TrafficWorld } from "../src/world/traffic";
import type { VehicleState } from "../src/simulation/types";
import type { WorldObject } from "../src/world/types";

// Fixed acceptance: no penetrating output, analytical contact fraction error<=1e-6.
const pose = (x = 0, y = 0, heading = 0): VehicleState => ({
  x,
  y,
  heading,
  velocity: 0,
});
const actor = (
  x: number,
  y: number,
  length = 0.1,
  width = 0.1,
  heading = 0,
  id = "moving",
): WorldObject => {
  const c = Math.cos(heading),
    s = Math.sin(heading);
  return {
    id,
    objectClass: "vehicle",
    height: 1,
    velocity: { x: 0, y: 0 },
    collidable: true,
    polygon: [
      [-length / 2, -width / 2],
      [length / 2, -width / 2],
      [length / 2, width / 2],
      [-length / 2, width / 2],
    ].map(([a, b]) => ({ x: x + a * c - b * s, y: y + a * s + b * c })),
  };
};

describe("relative swept collision between moving actors and ego", () => {
  it("detects a thin fast actor crossing a stationary ego between clear endpoints", () => {
    const previous = actor(1, -10, 0.001, 0.001),
      next = actor(1, 10, 0.001, 0.001),
      ego = pose();
    expect(polygonsOverlap(vehicleFootprint(ego), previous.polygon)).toBe(
      false,
    );
    expect(polygonsOverlap(vehicleFootprint(ego), next.polygon)).toBe(false);
    const result = resolveDynamicMotion(ego, ego, [previous], [next]);
    expect(result.collided).toBe(true);
    expect(result.objectId).toBe("moving");
    expect(result.fraction).toBeCloseTo((10 - 0.9 - 0.0005) / 20, 6);
    const contact = actor(1, -10 + 20 * result.fraction, 0.001, 0.001);
    expect(
      polygonsOverlap(vehicleFootprint(result.state), contact.polygon),
    ).toBe(false);
  });
  it("accounts for simultaneous opposite translation of ego and actor", () => {
    const before = actor(20, 0, 2, 2),
      after = actor(-20, 0, 2, 2);
    const result = resolveDynamicMotion(
      pose(),
      { ...pose(10), velocity: 10 },
      [before],
      [after],
    );
    expect(result.collided).toBe(true);
    expect(result.state.velocity).toBe(0);
    expect(result.fraction).toBeCloseTo((20 - 1 - 3.6) / 50, 6);
  });
  it("detects a rigid actor rotating into the vehicle with collision-free endpoints", () => {
    const ego = pose(),
      before = actor(5, 3, 8, 0.1, Math.PI / 2),
      after = actor(5, 3, 8, 0.1, 0);
    expect(polygonsOverlap(vehicleFootprint(ego), before.polygon)).toBe(false);
    expect(polygonsOverlap(vehicleFootprint(ego), after.polygon)).toBe(false);
    const result = resolveDynamicMotion(ego, ego, [before], [after]);
    expect(result.collided).toBe(true);
    expect(result.fraction).toBeGreaterThan(0);
    expect(result.fraction).toBeLessThan(1);
  });
  it("preserves safe relative motion and permits separation from exact contact", () => {
    expect(
      resolveDynamicMotion(pose(), pose(1), [actor(10, 10)], [actor(-10, 10)])
        .collided,
    ).toBe(false);
    const restingContact = actor(4.6, 0, 2, 2);
    expect(
      resolveDynamicMotion(pose(), pose(), [restingContact], [restingContact])
        .collided,
    ).toBe(false);
    const touching = actor(4.6, 0, 2, 2),
      away = actor(6.6, 0, 2, 2);
    expect(
      resolveDynamicMotion(pose(), pose(), [touching], [away]).collided,
    ).toBe(false);
  });
  it("is deterministic and handles an actor omitted from one endpoint snapshot", () => {
    const before = [actor(1, -10)],
      after = [actor(1, 10)];
    expect(resolveDynamicMotion(pose(), pose(), before, after)).toEqual(
      resolveDynamicMotion(pose(), pose(), before, after),
    );
    expect(
      resolveDynamicMotion(pose(), pose(), [], [actor(20, 20)]).collided,
    ).toBe(false);
    expect(
      resolveDynamicMotion(pose(), pose(), [actor(20, 20)], []).collided,
    ).toBe(false);
  });
  it("freezes the world actor at the matching relative collision fraction", () => {
    const world = new TrafficWorld("following"),
      before = world.objects;
    // A stationary ego blocks the lead vehicle just ahead of its existing front bumper.
    const ego = pose(3.5, -29.25, Math.PI),
      previous = world.snapshot;
    world.advance(1);
    const result = resolveDynamicMotion(ego, ego, before, world.objects);
    expect(result.collided).toBe(true);
    world.haltActor(result.objectId!, result.fraction);
    expect(
      polygonsOverlap(vehicleFootprint(ego), world.objects[0].polygon),
    ).toBe(false);
    const frozen = world.objects;
    world.advance(10);
    expect(world.objects).toEqual(frozen);
    expect(world.snapshot.time).toBeGreaterThan(previous.time);
  });
});
