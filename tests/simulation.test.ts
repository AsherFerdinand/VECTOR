import { describe, expect, it } from "vitest";
import { Simulation } from "../src/simulation/clock";
import { INITIAL_STATE, VEHICLE } from "../src/simulation/vehicle";
import type { DriverInput } from "../src/simulation/types";

const idle: DriverInput = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};
const accelerate = { ...idle, forward: true };

describe("fixed-step simulation", () => {
  it("produces identical results for identical control sequences", () => {
    const a = new Simulation();
    const b = new Simulation();
    a.start();
    b.start();
    for (let i = 0; i < 400; i++) {
      const input = { ...accelerate, left: i > 100 && i < 300, brake: i > 350 };
      a.advance(0.017, input);
      b.advance(0.017, input);
    }
    expect(a.snapshot).toEqual(b.snapshot);
  });
  it("produces identical motion for equivalent time under different rendering frame intervals", () => {
    const a = new Simulation();
    const b = new Simulation();
    a.start();
    b.start();
    for (let i = 0; i < 100; i++) a.advance(0.01, accelerate);
    for (let i = 0; i < 20; i++) {
      b.advance(0.013, accelerate);
      b.advance(0.037, accelerate);
    }
    expect(a.snapshot).toEqual(b.snapshot);
    expect(a.snapshot.time).toBe(1);
    expect(a.snapshot.vehicle.velocity).toBeCloseTo(3);
  });
  it("pauses time and motion and resumes without accumulating paused time", () => {
    const sim = new Simulation();
    sim.start();
    sim.advance(0.03, accelerate);
    sim.pause();
    const paused = sim.snapshot;
    sim.advance(20, accelerate);
    expect(sim.snapshot).toEqual(paused);
    sim.start();
    sim.advance(0.01, accelerate);
    expect(sim.snapshot.time).toBe(0.04);
  });
  it("resets all state deterministically and remains paused", () => {
    const sim = new Simulation();
    sim.start();
    sim.advance(2, { ...accelerate, left: true });
    sim.reset();
    expect(sim.snapshot).toEqual(new Simulation().snapshot);
    expect(sim.snapshot.vehicle).toEqual(INITIAL_STATE);
  });
  it("bounds slow-frame work and reports discarded wall time", () => {
    const sim = new Simulation();
    sim.start();
    sim.advance(2, accelerate);
    expect(sim.snapshot.time).toBe(0.1);
    expect(sim.snapshot.droppedTime).toBeCloseTo(1.9);
    sim.advance(0, accelerate);
    expect(sim.snapshot.time).toBe(0.1);
  });
  it("holds speed limits and reports achieved acceleration at saturation", () => {
    const sim = new Simulation();
    sim.start();
    for (let i = 0; i < 600; i++) sim.advance(0.01, accelerate);
    expect(sim.snapshot.vehicle.velocity).toBe(VEHICLE.maxSpeed);
    expect(sim.snapshot.control.acceleration).toBe(0);
  });
  it("returns snapshots that cannot mutate internal physics", () => {
    const sim = new Simulation();
    const snapshot = sim.snapshot;
    snapshot.vehicle.x = 999;
    snapshot.control.steering = 999;
    expect(sim.snapshot.vehicle).toEqual(INITIAL_STATE);
    expect(sim.snapshot.control.steering).toBe(0);
  });
  it("rejects invalid rendering intervals", () => {
    const sim = new Simulation();
    sim.start();
    expect(() => sim.advance(Number.NaN, idle)).toThrow(RangeError);
    expect(() => sim.advance(-1, idle)).toThrow(RangeError);
  });
});
