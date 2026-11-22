import { describe, expect, it } from "vitest";
import { Simulation } from "../src/simulation/clock";
import { projectToRoad } from "../src/navigation/graph";
import { polygonsOverlap, vehicleFootprint } from "../src/simulation/collision";
const idle = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};
function gap(sim: Simulation) {
  return (
    Math.min(...sim.traffic.actors[0].object.polygon.map((p) => p.x)) -
    Math.max(...vehicleFootprint(sim.snapshot.vehicle).map((p) => p.x))
  );
}
function free(sim: Simulation) {
  for (const actor of sim.traffic.actors)
    expect(
      polygonsOverlap(
        vehicleFootprint(sim.snapshot.vehicle),
        actor.object.polygon,
      ),
    ).toBe(false);
}
// Deliberately aggressive but feasible takeover: >=1m minimum separation,
// AEB on the first autonomous tick, no collision, and no road/interlock failures.
describe("traffic safety and physical integration", () => {
  it("prioritises observed AEB during an urgent manual-to-autonomous takeover", () => {
    const sim = new Simulation(undefined, { scenario: "sudden-braking" });
    sim.start();
    for (let i = 0; i < 350; i++) sim.step({ ...idle, forward: true });
    expect(sim.snapshot.vehicle.velocity).toBeCloseTo(10.5);
    sim.pause();
    expect(sim.planTo(projectToRoad(sim.graph, { x: 45, y: -29.25 })!)).toBe(
      true,
    );
    expect(sim.activateAutonomy()).toBe(true);
    const activation = sim.snapshot.time;
    const activationX = sim.snapshot.vehicle.x;
    sim.step(idle);
    expect(sim.behaviour.state).toBe("AEB");
    expect(sim.snapshot.control.acceleration).toBeCloseTo(-6);
    expect(sim.behaviour.ttc).toBeGreaterThan(0);
    expect(sim.behaviour.ttc).toBeLessThan(1);
    const first = {
      time: sim.snapshot.time,
      clearance: gap(sim),
      ttc: sim.behaviour.ttc,
      braking: sim.snapshot.control.acceleration,
    };
    let minimum = gap(sim);
    let firstStop: { time: number; distance: number; speed: number } | null =
      null;
    const states = new Set([sim.behaviour.state]);
    for (let i = 0; i < 1799; i++) {
      sim.step(idle);
      free(sim);
      minimum = Math.min(minimum, gap(sim));
      states.add(sim.behaviour.state);
      if (!firstStop && sim.snapshot.vehicle.velocity <= 0.1)
        firstStop = {
          time: sim.snapshot.time - activation,
          distance: sim.snapshot.vehicle.x - activationX,
          speed: sim.snapshot.vehicle.velocity,
        };
    }
    expect(first.time - activation).toBeCloseTo(0.01);
    expect(firstStop).not.toBeNull();
    expect(minimum).toBeGreaterThanOrEqual(1);
    expect(states.has("FOLLOW_LEAD")).toBe(true);
    expect(sim.physical.collisionCount).toBe(0);
    expect(sim.physical.roadDepartureCount).toBe(0);
    expect(sim.physical.containmentStop).toBe(false);
    console.info(
      "PRODUCTION_AEB",
      JSON.stringify({
        first,
        minimumGap: minimum,
        firstStop,
        states: [...states],
        collisions: sim.physical.collisionCount,
      }),
    );
  }, 30000);
  it("stops both colliding vehicles, holds one physical contact and permits reverse escape", () => {
    const sim = new Simulation(undefined, { scenario: "following" });
    sim.start();
    for (let i = 0; i < 800; i++) {
      sim.step({ ...idle, forward: true });
      free(sim);
    }
    expect(sim.physical.collisionCount).toBe(1);
    expect(sim.physical.collision?.objectId).toBe("lead-vehicle");
    expect(sim.snapshot.vehicle.velocity).toBe(0);
    expect(sim.traffic.actors[0].state.velocity).toBe(0);
    expect(sim.behaviour.state).toBe("MANUAL");
    const position = sim.snapshot.vehicle.x;
    for (let i = 0; i < 100; i++) {
      sim.step({ ...idle, backward: true });
      free(sim);
    }
    expect(sim.snapshot.vehicle.x).toBeLessThan(position - 1);
    expect(sim.physical.collisionCount).toBe(1);
    sim.reset();
    expect(sim.physical.collisionCount).toBe(0);
    expect(sim.traffic.actors[0].state.velocity).toBe(3.5);
  });
  it("prevents a moving pedestrian from tunnelling into a stationary manual vehicle", () => {
    const sim = new Simulation(undefined, { scenario: "pedestrian" });
    expect(sim.setStart(projectToRoad(sim.graph, { x: 21, y: -29.25 })!)).toBe(
      true,
    );
    sim.start();
    const position = { ...sim.snapshot.vehicle };
    for (let i = 0; i < 850; i++) {
      sim.step(idle);
      free(sim);
    }
    expect(sim.snapshot.vehicle).toEqual(position);
    expect(sim.physical.collisionCount).toBe(1);
    expect(sim.physical.collision?.objectId).toBe("crossing-pedestrian");
    expect(sim.traffic.actors[0].state.velocity).toBe(0);
    expect(sim.safety.state).toBe("MANUAL");
  });
  it("rejects a start that would overlap an actor after scenario time resets", () => {
    const sim = new Simulation(undefined, { scenario: "following" });
    sim.start();
    for (let i = 0; i < 800; i++) sim.step(idle);
    sim.pause();
    expect(sim.traffic.actors[0].state.x).toBeGreaterThan(10);
    const before = structuredClone({
      state: sim.snapshot,
      traffic: sim.traffic,
      sensors: sim.sensors,
    });
    expect(sim.setStart(projectToRoad(sim.graph, { x: -4, y: -29.25 })!)).toBe(
      false,
    );
    expect({
      state: sim.snapshot,
      traffic: sim.traffic,
      sensors: sim.sensors,
    }).toEqual(before);
  });
});
