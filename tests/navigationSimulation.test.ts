import { describe, expect, it } from "vitest";
import { Simulation } from "../src/simulation/clock";
import { projectToRoad } from "../src/navigation/graph";
import type { DriverInput } from "../src/simulation/types";
const idle: DriverInput = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};
function prepared() {
  const sim = new Simulation();
  const destination = projectToRoad(sim.graph, { x: 15, y: -29.25 })!;
  expect(sim.planTo(destination)).toBe(true);
  return sim;
}
describe("navigation in the fixed-step simulation", () => {
  it("uses the unchanged bicycle integrator to complete a route and stop", () => {
    const sim = prepared();
    expect(sim.activateAutonomy()).toBe(true);
    for (
      let tick = 0;
      tick < 18000 && sim.navigation.state !== "DESTINATION_REACHED";
      tick++
    )
      sim.step(idle);
    expect(sim.navigation.state).toBe("DESTINATION_REACHED");
    for (let i = 0; i < 100; i++) sim.step(idle);
    expect(sim.snapshot.vehicle.velocity).toBe(0);
    expect(
      Math.hypot(sim.snapshot.vehicle.x - 15, sim.snapshot.vehicle.y + 29.25),
    ).toBeLessThan(0.8);
  });
  it("gives a manual takeover exclusive control on the same physics tick", () => {
    const sim = prepared();
    sim.activateAutonomy();
    for (let i = 0; i < 100; i++) sim.step(idle);
    const previous = sim.snapshot;
    sim.step({ ...idle, backward: true, right: true });
    expect(sim.navigation.mode).toBe("MANUAL");
    expect(sim.navigation.remainingDistance).toBeNull();
    expect(sim.snapshot.control.acceleration).toBeCloseTo(-6);
    expect(sim.snapshot.vehicle.velocity).toBeCloseTo(
      previous.vehicle.velocity - 0.06,
    );
    expect(sim.snapshot.control.steering).toBeLessThan(0);
  });
  it("pauses motion, retains route, and resumes without replanning every frame", () => {
    const sim = prepared();
    sim.activateAutonomy();
    sim.advance(0.05, idle);
    sim.pause();
    const paused = sim.snapshot,
      route = sim.navigation.route,
      nav = sim.navigation;
    sim.advance(20, idle);
    expect(sim.snapshot).toEqual(paused);
    expect(sim.navigation).toEqual(nav);
    sim.start();
    sim.advance(0.01, idle);
    expect(sim.snapshot.time).toBe(0.06);
    expect(sim.navigation.route).toBe(route);
  });
  it("recalculates selected endpoints on reset, remains manual and paused", () => {
    const sim = prepared();
    const original = sim.navigation.route;
    sim.activateAutonomy();
    sim.advance(0.1, idle);
    sim.reset();
    expect(sim.navigation.mode).toBe("MANUAL");
    expect(sim.navigation.state).toBe("ROUTE_READY");
    expect(sim.navigation.route).toEqual(original);
    expect(sim.snapshot.time).toBe(0);
    expect(sim.snapshot.running).toBe(false);
    expect(sim.navigation.trajectory).toEqual([]);
  });
  it("guards start relocation and never teleports on destination selection", () => {
    const sim = prepared(),
      before = sim.snapshot;
    const start = projectToRoad(sim.graph, { x: 0, y: 29.25 })!;
    expect(sim.snapshot).toEqual(before);
    sim.start();
    expect(sim.setStart(start)).toBe(false);
    expect(sim.snapshot.vehicle).toEqual(before.vehicle);
    sim.advance(0.1, { ...idle, forward: true });
    sim.pause();
    expect(sim.setStart(start)).toBe(false);
    sim.reset();
    expect(sim.setStart(start)).toBe(true);
    expect(sim.snapshot.vehicle.heading).toBeCloseTo(Math.PI);
    expect(sim.snapshot.vehicle.velocity).toBe(0);
    expect(sim.snapshot.running).toBe(false);
    sim.start();
    expect(sim.planTo(start)).toBe(false);
  });
  it("is deterministic across identical input sequences and different frame partitions", () => {
    const a = prepared(),
      b = prepared();
    a.activateAutonomy();
    b.activateAutonomy();
    for (let i = 0; i < 100; i++) a.advance(0.01, idle);
    for (let i = 0; i < 20; i++) {
      b.advance(0.013, idle);
      b.advance(0.037, idle);
    }
    expect(a.snapshot).toEqual(b.snapshot);
    expect(a.navigation).toEqual(b.navigation);
  });
  it("retains pose and speed when navigation is cancelled or cleared", () => {
    const sim = prepared();
    sim.activateAutonomy();
    sim.advance(0.1, idle);
    const before = sim.snapshot;
    sim.cancelAutonomy();
    expect(sim.snapshot).toEqual(before);
    expect(sim.navigation.mode).toBe("MANUAL");
    sim.clearRoute();
    expect(sim.snapshot).toEqual(before);
    expect(sim.navigation.route).toBeNull();
    expect(sim.navigation.state).toBe("IDLE");
  });
  it("does not let malformed start poses mutate vehicle state", () => {
    const sim = prepared(),
      before = sim.snapshot;
    const p = sim.navigation.route!.start;
    expect(sim.setStart({ ...p, heading: Number.NaN })).toBe(false);
    expect(sim.snapshot).toEqual(before);
  });
});
