import { describe, expect, it } from "vitest";
import { TrafficWorld, SCENARIOS, SIGNAL_MAP } from "../src/world/traffic";
import { trafficLightPlacements, worldObjects } from "../src/world/objects";
import { classifyRoad, polygonsOverlap } from "../src/simulation/collision";

// Acceptance fixed before execution: vehicle |a|<=6m/s², pedestrian|a|<=1.2m/s²,
// finite-difference speed agreement<=0.031m/s (half a6m/s² tick), no solid overlap.
describe("deterministic bounded traffic actors", () => {
  it("exposes six implemented scenarios and leaves baseline actor-free", () => {
    expect(SCENARIOS.map((s) => s.id)).toEqual([
      "normal",
      "following",
      "sudden-braking",
      "red-light",
      "pedestrian",
      "blocked",
    ]);
    expect(new TrafficWorld().snapshot.actors).toEqual([]);
    expect(new TrafficWorld().objects).toEqual([]);
  });
  it("starts lead scenarios already cruising and normal traffic smoothly at rest", () => {
    expect(
      new TrafficWorld("following").snapshot.actors[0].state,
    ).toMatchObject({ x: -4, y: -29.25, heading: 0, velocity: 3.5 });
    expect(
      new TrafficWorld("sudden-braking").snapshot.actors[0].state.velocity,
    ).toBe(3.5);
    const normal = new TrafficWorld("normal");
    expect(normal.snapshot.actors[0].state.velocity).toBe(0);
    normal.advance(0.01);
    expect(normal.snapshot.actors[0].state.velocity).toBeCloseTo(0.02);
  });
  it.each(["following", "sudden-braking", "normal", "pedestrian"] as const)(
    "%s has continuous bounded motion and truthful velocity",
    (scenario) => {
      const world = new TrafficWorld(scenario);
      let previous = world.snapshot;
      for (let i = 0; i < 1800; i++) {
        world.advance(0.01);
        const next = world.snapshot;
        for (const actor of next.actors) {
          const prior = previous.actors.find((a) => a.id === actor.id)!;
          const displacement = Math.hypot(
            actor.state.x - prior.state.x,
            actor.state.y - prior.state.y,
          );
          expect(
            Math.abs(displacement / 0.01 - actor.state.velocity),
          ).toBeLessThanOrEqual(0.031);
          expect(
            Math.abs(actor.state.velocity - prior.state.velocity) / 0.01,
          ).toBeLessThanOrEqual((actor.kind === "vehicle" ? 6 : 1.2) + 1e-6);
          expect(
            Math.hypot(actor.object.velocity.x, actor.object.velocity.y),
          ).toBeCloseTo(actor.state.velocity, 10);
          if (actor.kind === "vehicle")
            expect(classifyRoad(actor.state)).toBe("ON_ROAD");
        }
        previous = next;
      }
    },
  );
  it("brakes at5s, holds4s and resumes without crossing the mapped stop", () => {
    const world = new TrafficWorld("sudden-braking");
    world.advance(5);
    expect(world.snapshot.actors[0].state.velocity).toBe(3.5);
    world.advance(0.25);
    expect(world.snapshot.actors[0].state.velocity).toBeCloseTo(2);
    world.advance(0.75);
    expect(world.snapshot.actors[0].state.velocity).toBe(0);
    const stopped = world.snapshot.actors[0].state.x;
    world.advance(3);
    expect(world.snapshot.actors[0].state.x).toBe(stopped);
    world.advance(1);
    expect(world.snapshot.actors[0].state.velocity).toBeGreaterThan(0);
    world.advance(100);
    expect(world.snapshot.actors[0].state).toMatchObject({
      x: 15,
      velocity: 0,
    });
  });
  it.each(["normal", "following", "sudden-braking", "pedestrian"] as const)(
    "%s never enters rendered solid objects or teleports past an endpoint",
    (scenario) => {
      const world = new TrafficWorld(scenario),
        solids = worldObjects();
      for (let i = 0; i < 40; i++) {
        world.advance(1);
        for (const actor of world.snapshot.actors) {
          expect(
            solids.some((s) =>
              polygonsOverlap(actor.object.polygon, s.polygon),
            ),
          ).toBe(false);
          expect(
            Number.isFinite(
              actor.state.x + actor.state.y + actor.state.velocity,
            ),
          ).toBe(true);
        }
      }
      const end = world.snapshot.actors;
      world.advance(100);
      expect(world.snapshot.actors).toEqual(end);
    },
  );
  it("keeps the blocked-road solid aligned with the rendered scenario object", () => {
    const world = new TrafficWorld("blocked"),
      object = world.objects[0];
    expect(world.snapshot.staticObstacles).toEqual(world.objects);
    expect(object.id).toBe("scenario-road-block");
    expect(object.objectClass).toBe("static-obstacle");
    expect(object.polygon).toEqual([
      { x: -1, y: -30.25 },
      { x: 1, y: -30.25 },
      { x: 1, y: -28.25 },
      { x: -1, y: -28.25 },
    ]);
  });
  it("resets scripts, signals and collision freezes reproducibly", () => {
    const a = new TrafficWorld("normal"),
      b = new TrafficWorld("normal");
    for (let i = 0; i < 400; i++) {
      a.advance(0.01);
      b.advance(0.01);
    }
    expect(a.snapshot).toEqual(b.snapshot);
    a.haltActor("north-vehicle", 0.3);
    a.advance(3);
    a.reset();
    expect(a.snapshot).toEqual(new TrafficWorld("normal").snapshot);
    a.reset(null);
    expect(a.snapshot.actors).toEqual([]);
    expect(a.snapshot.scenario).toBeNull();
  });
  it("freezes a colliding actor at the contact fraction without subsequent teleportation", () => {
    const world = new TrafficWorld("following"),
      before = world.snapshot.actors[0].state;
    world.advance(0.1);
    const next = world.snapshot.actors[0].state;
    world.haltActor("lead-vehicle", 0.25);
    expect(world.snapshot.actors[0].state.x).toBeCloseTo(
      before.x + (next.x - before.x) * 0.25,
    );
    expect(world.snapshot.actors[0].object.velocity).toEqual({ x: 0, y: 0 });
    const frozen = world.snapshot.actors[0];
    world.advance(10);
    expect(world.snapshot.actors[0]).toEqual(frozen);
    expect(world.snapshot.time).toBeCloseTo(10.1);
  });
  it("returns snapshots that cannot mutate world state and rejects invalid steps", () => {
    const world = new TrafficWorld("normal"),
      snapshot = world.snapshot;
    snapshot.actors[0].state.x = 999;
    snapshot.actors[0].object.polygon[0].x = 999;
    snapshot.signals[0].position.x = 999;
    expect(world.snapshot.actors[0].state.x).toBe(-45);
    expect(world.snapshot.signals[0].position.x).not.toBe(999);
    expect(() => world.advance(-1)).toThrow(RangeError);
    expect(() => world.advance(NaN)).toThrow(RangeError);
  });
});

describe("map-based signal and crossing state", () => {
  it("maps all four approaches at all four intersections to real poles", () => {
    expect(SIGNAL_MAP).toHaveLength(16);
    expect(trafficLightPlacements).toHaveLength(16);
    for (const signal of SIGNAL_MAP) {
      const pole = trafficLightPlacements.find((p) => p.id === signal.id)!;
      expect(signal.position).toEqual({ x: pole.x, y: pole.y });
      const lineMid = {
        x: (signal.stopLine[0].x + signal.stopLine[1].x) / 2,
        y: (signal.stopLine[0].y + signal.stopLine[1].y) / 2,
      };
      const crossingMid = {
        x: (signal.crossing[0].x + signal.crossing[1].x) / 2,
        y: (signal.crossing[0].y + signal.crossing[1].y) / 2,
      };
      expect(
        (crossingMid.x - lineMid.x) * Math.cos(signal.heading) +
          (crossingMid.y - lineMid.y) * Math.sin(signal.heading),
      ).toBeCloseTo(1.5);
    }
  });
  it("runs reproducible red-green-yellow phases without conflicting greens", () => {
    const world = new TrafficWorld("red-light");
    const east = () =>
      world.snapshot.signals.find((s) => s.id === "traffic-light-2-a")!;
    expect(east().color).toBe("red");
    world.advance(10);
    expect(east().color).toBe("green");
    world.advance(14);
    expect(east().color).toBe("yellow");
    world.advance(3);
    expect(east().color).toBe("red");
    const normal = new TrafficWorld("normal");
    const seen = new Set<string>();
    for (let i = 0; i < 400; i++) {
      const signals = normal.snapshot.signals;
      seen.add(signals[0].color);
      for (let junction = 0; junction < 4; junction++) {
        const group = signals.filter((s) =>
          s.id.startsWith(`traffic-light-${junction}-`),
        );
        expect(
          group.filter((s) => s.color === "green").length,
        ).toBeLessThanOrEqual(2);
      }
      normal.advance(0.1);
    }
    expect([...seen].sort()).toEqual(["green", "red", "yellow"]);
  });
});
