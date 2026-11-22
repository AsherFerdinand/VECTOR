import { describe, expect, it } from "vitest";
import { FIXED_DT, Simulation } from "../src/simulation/clock";
import type { DriverInput } from "../src/simulation/types";
import { BehaviourPlanner } from "../src/autonomy/behaviourPlanner";
import type { Track } from "../src/perception/types";
import { runScenario } from "../src/validation/runner";
import { projectToRoad } from "../src/navigation/graph";

const IDLE: DriverInput = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};
const capture = (sim: Simulation) =>
  structuredClone({
    snapshot: sim.snapshot,
    tracking: sim.tracking,
    behaviour: sim.behaviour,
    physical: sim.physical,
  });

describe("production fusion integration", () => {
  it("yields to a measured crossing vehicle before overlap and completes the route", () => {
    const result = runScenario({
      id: "crossing-regression",
      scenario: "normal",
      trafficVariant: "intersection-conflict",
      seed: 42,
      duration: 30,
      destination: { x: 45, y: -29.25 },
    });
    expect(result.metrics.collisionCount).toBe(0);
    expect(result.metrics.roadDepartures).toBe(0);
    expect(result.metrics.routeCompleted).toBe(true);
    expect(
      result.events.some(
        (event) =>
          event.kind === "decision" &&
          event.message.includes("predicts a crossing conflict"),
      ),
    ).toBe(true);
    expect(result.passed).toBe(true);
  }, 30000);
  it("uses eligible fused velocity for ACC and rejects uncertain tracks", () => {
    const sim = new Simulation(undefined, { scenario: "following" });
    const track: Track = {
      id: "estimated-1",
      x: -10,
      y: -29.25,
      vx: 2,
      vy: 0,
      covariance: [
        [0.1, 0, 0, 0],
        [0, 0.1, 0, 0],
        [0, 0, 0.2, 0],
        [0, 0, 0, 0.2],
      ],
      status: "confirmed",
      objectClass: "vehicle",
      lastMeasurementTime: 0,
      age: 1,
      sources: ["radar", "camera"],
    };
    const evaluate = (tracks: Track[]) => {
      const planner = new BehaviourPlanner();
      planner.update(
        sim.snapshot.vehicle,
        { steering: 0, acceleration: 1 },
        sim.sensors,
        0,
        true,
        {
          route: sim.navigation.route,
          progress: 0,
          timeGap: 1.5,
          targetSpeed: 6,
          tracks,
        },
      );
      return planner.snapshot;
    };
    const fused = evaluate([track]);
    expect(fused.leadDistance).toBeCloseTo(6.4, 6);
    expect(fused.relativeSpeed).toBeCloseTo(2, 6);
    const uncertain = structuredClone(track);
    uncertain.covariance[0][0] = 10;
    expect(evaluate([uncertain]).leadDistance).toEqual(
      evaluate([]).leadDistance,
    );
    expect(evaluate([{ ...track, status: "tentative" }]).leadDistance).toEqual(
      evaluate([]).leadDistance,
    );
  });
  it("gives measured-track emergency braking priority without braking for an adjacent corridor", () => {
    const sim = new Simulation(undefined, { scenario: "following" });
    const track: Track = {
      id: "estimated-hazard",
      x: -14,
      y: -29.25,
      vx: 0,
      vy: 0,
      covariance: [
        [0.1, 0, 0, 0],
        [0, 0.1, 0, 0],
        [0, 0, 0.2, 0],
        [0, 0, 0, 0.2],
      ],
      status: "confirmed",
      objectClass: "vehicle",
      lastMeasurementTime: 0,
      age: 1,
      sources: ["radar"],
    };
    const evaluate = (tracked: Track) => {
      const planner = new BehaviourPlanner();
      const control = planner.update(
        { ...sim.snapshot.vehicle, velocity: 6 },
        { steering: 0, acceleration: 3 },
        sim.sensors,
        0,
        true,
        {
          route: sim.navigation.route,
          progress: 0,
          timeGap: 1.5,
          targetSpeed: 6,
          tracks: [tracked],
        },
      );
      return { control, data: planner.snapshot };
    };
    const urgent = evaluate(track);
    expect(urgent.data.state).toBe("AEB");
    expect(urgent.control.acceleration).toBe(-6);
    expect(evaluate({ ...track, y: -24.75 }).data.aeb).toBe(false);
  });
  it("predicts lateral arrivals while ignoring a vehicle moving away from the route", () => {
    const sim = new Simulation(undefined, { scenario: "following" });
    const track: Track = {
      id: "crossing-estimate",
      x: 0,
      y: -34.25,
      vx: 0,
      vy: 2,
      covariance: [
        [0.1, 0, 0, 0],
        [0, 0.1, 0, 0],
        [0, 0, 0.2, 0],
        [0, 0, 0, 0.2],
      ],
      status: "confirmed",
      objectClass: "vehicle",
      lastMeasurementTime: 0,
      age: 1,
      sources: ["radar"],
    };
    const evaluate = (tracked: Track) => {
      const planner = new BehaviourPlanner();
      planner.update(
        { ...sim.snapshot.vehicle, velocity: 6 },
        { steering: 0, acceleration: 3 },
        sim.sensors,
        0,
        true,
        {
          route: sim.navigation.route,
          progress: 0,
          timeGap: 1.5,
          targetSpeed: 6,
          tracks: [tracked],
        },
      );
      return planner.snapshot;
    };
    expect(evaluate(track).reason).toContain("predicts a crossing conflict");
    expect(evaluate({ ...track, vy: -2 }).reason).not.toContain(
      "predicts a crossing conflict",
    );
  });
  it("uses fresh confirmed observations while following and preserves deterministic reset", () => {
    const sim = new Simulation(undefined, {
      scenario: "following",
      fusedControl: true,
    });
    expect(sim.activateAutonomy()).toBe(true);
    let confirmedVehicle = false,
      followed = false;
    for (let tick = 0; tick < 350; tick++) {
      sim.step(IDLE);
      confirmedVehicle ||= sim.tracking.tracks.some(
        (t) => t.objectClass === "vehicle" && t.status === "confirmed",
      );
      followed ||= sim.behaviour.state === "FOLLOW_LEAD";
      expect(sim.physical.collisionCount).toBe(0);
      expect(sim.physical.roadDepartureCount).toBe(0);
    }
    expect(confirmedVehicle).toBe(true);
    expect(followed).toBe(true);
    const first = capture(sim);
    sim.reset();
    expect(sim.activateAutonomy()).toBe(true);
    for (let tick = 0; tick < 350; tick++) sim.step(IDLE);
    expect(capture(sim)).toEqual(first);
  }, 30000);

  it("brakes on sensor dropout instead of treating predicted tracks as fresh measurements", () => {
    const sim = new Simulation(undefined, {
      scenario: "following",
      fusedControl: true,
      sensorDropout: { start: 1, end: 3 },
    });
    sim.activateAutonomy();
    for (let tick = 0; tick < 160; tick++) sim.step(IDLE);
    expect(sim.behaviour.state).toBe("STALE_STOP");
    expect(sim.snapshot.control.acceleration).toBeLessThanOrEqual(0);
    expect(sim.sensors.lidar!.timestamp).toBeLessThan(1.01);
    expect(sim.tracking.tracks.every((t) => t.lastMeasurementTime < 1.01)).toBe(
      true,
    );
    expect(sim.physical.collisionCount).toBe(0);
    expect(sim.physical.roadDepartureCount).toBe(0);
  }, 30000);

  it("keeps pause and manual takeover authoritative with fusion enabled", () => {
    const sim = new Simulation(undefined, {
      scenario: "following",
      fusedControl: true,
    });
    sim.activateAutonomy();
    for (let tick = 0; tick < 100; tick++) sim.step(IDLE);
    sim.pause();
    const paused = capture(sim);
    for (let tick = 0; tick < 20; tick++) sim.step(IDLE);
    expect(capture(sim)).toEqual(paused);
    sim.start();
    sim.step({ ...IDLE, left: true, brake: true });
    expect(sim.navigation.mode).toBe("MANUAL");
    expect(sim.behaviour.state).toBe("MANUAL");
    expect(sim.snapshot.time).toBeCloseTo(paused.snapshot.time + FIXED_DT, 10);
    expect(sim.snapshot.control.acceleration).toBeLessThanOrEqual(0);
  }, 30000);
});

// Start validation uses reset-time variant geometry, just as the reset world does.
it("rejects a start occupied by a configured dynamic blocker", () => {
  const simulation = new Simulation(undefined, {
    scenario: "normal",
    trafficVariant: "dynamic-block",
    fusedControl: true,
  });
  const position = projectToRoad(simulation.graph, { x: 0, y: -29.25 })!;
  const ordinary = new Simulation(undefined, { scenario: "normal" });
  expect(ordinary.setStart(position)).toBe(true);
  expect(simulation.setStart(position)).toBe(false);
  expect(simulation.snapshot.vehicle.x).toBe(-20);
});
