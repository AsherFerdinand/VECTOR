import { describe, expect, it } from "vitest";
import { Simulation, FIXED_DT } from "../src/simulation/clock";
import { projectToRoad } from "../src/navigation/graph";
import { polygonsOverlap, vehicleFootprint } from "../src/simulation/collision";
import type { DriverInput } from "../src/simulation/types";
import type { ScenarioId } from "../src/world/trafficTypes";

// Declared before running production scenarios. 1 m lead clearance reserves body
// separation; 1 m observation tolerance covers a 10 Hz sample plus one physics step.
const LIMITS = {
  seconds: 180,
  minimumLeadGap: 1,
  maximumGapError: 1,
  lateralAcceleration: 1.500001,
  stopLineTolerance: 0.05,
  greenResumeBy: 20,
};
const IDLE: DriverInput = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};
const timeout = 60000;
function diagnostic(sim: Simulation) {
  return {
    time: sim.snapshot.time,
    vehicle: sim.snapshot.vehicle,
    behaviour: sim.behaviour.state,
    reason: sim.behaviour.reason,
    navigation: sim.navigation.state,
    physical: sim.physical,
  };
}
function requireSafe(sim: Simulation) {
  const p = sim.physical;
  if (
    p.collisionCount !== 0 ||
    p.roadDepartureCount !== 0 ||
    p.containmentStop ||
    p.roadStatus !== "ON_ROAD" ||
    p.sweptRoadStatus !== "ON_ROAD"
  ) {
    console.info("SCENARIO_FAILURE", JSON.stringify(diagnostic(sim)));
    throw new Error(
      "Production scenario violated collision or swept-road acceptance",
    );
  }
}
function longitudinalGap(sim: Simulation): number | null {
  const lead = sim.traffic.actors.find(
    (a) => a.kind === "vehicle" && a.id === "lead-vehicle",
  );
  if (!lead) return null;
  return (
    Math.min(...lead.object.polygon.map((p) => p.x)) -
    Math.max(...vehicleFootprint(sim.snapshot.vehicle).map((p) => p.x))
  );
}
function capture(sim: Simulation) {
  return structuredClone({
    snapshot: sim.snapshot,
    traffic: sim.traffic,
    sensors: sim.sensors,
    behaviour: sim.behaviour,
    navigation: sim.navigation,
    physical: sim.physical,
  });
}
function runScenario(
  id: Exclude<ScenarioId, "blocked">,
  destination?: { x: number; y: number },
) {
  const sim = new Simulation(undefined, { scenario: id });
  if (destination) {
    const point = projectToRoad(sim.graph, destination);
    expect(point).not.toBeNull();
    expect(sim.planTo(point!)).toBe(true);
  }
  expect(sim.navigation.route).not.toBeNull();
  expect(sim.activateAutonomy()).toBe(true);
  const states = new Map<string, number>();
  let minimumGap = Infinity,
    maxGapError = 0,
    gapSamples = 0,
    maximumLateral = 0;
  let stoppedBeforeGreen = false,
    resumedAfterGreen = false,
    firstYieldFront: number | null = null;
  let suddenBrakeResponse: number | null = null,
    leadBraked = false;
  let maximumRedFront = -Infinity;
  let priorLeadSpeed =
    sim.traffic.actors.find((a) => a.id === "lead-vehicle")?.state.velocity ??
    0;
  let terminal = false;
  for (let tick = 0; tick < LIMITS.seconds / FIXED_DT; tick++) {
    sim.step(IDLE);
    requireSafe(sim);
    const snapshot = sim.snapshot,
      data = sim.behaviour;
    states.set(data.state, (states.get(data.state) ?? 0) + 1);
    maximumLateral = Math.max(
      maximumLateral,
      (snapshot.vehicle.velocity ** 2 *
        Math.abs(Math.tan(snapshot.control.steering))) /
        2.7,
    );
    const gap = longitudinalGap(sim);
    if (gap !== null) minimumGap = Math.min(minimumGap, gap);
    if (
      gap !== null &&
      data.leadDistance !== null &&
      data.state === "FOLLOW_LEAD"
    ) {
      maxGapError = Math.max(maxGapError, Math.abs(gap - data.leadDistance));
      gapSamples++;
    }
    const front = Math.max(
      ...vehicleFootprint(snapshot.vehicle).map((p) => p.x),
    );
    if (id === "red-light") {
      const signal = sim.traffic.signals.find(
        (s) =>
          Math.abs(s.heading) < 1e-8 &&
          Math.abs(s.stopLine[0].x - 19.5) < 1e-8 &&
          s.stopLine[0].y < 0,
      )!;
      expect(signal).toBeDefined();
      if (snapshot.time < 10 && signal.color === "red") {
        maximumRedFront = Math.max(maximumRedFront, front);
        if (
          snapshot.time > 1 &&
          data.state === "STOP_LIGHT" &&
          snapshot.vehicle.velocity < 0.1
        )
          stoppedBeforeGreen = true;
      }
      if (
        snapshot.time >= 10 &&
        snapshot.time <= LIMITS.greenResumeBy &&
        signal.color === "green" &&
        snapshot.vehicle.velocity > 0.5 &&
        front > 19.5
      )
        resumedAfterGreen = true;
    }
    if (id === "pedestrian") {
      const pedestrian = sim.traffic.actors.find(
        (a) => a.kind === "pedestrian",
      )!;
      if (
        polygonsOverlap(
          vehicleFootprint(snapshot.vehicle),
          pedestrian.object.polygon,
        )
      ) {
        console.info("PEDESTRIAN_OVERLAP", JSON.stringify(diagnostic(sim)));
        throw new Error("Vehicle body intersects pedestrian");
      }
      if (data.state === "YIELD_PEDESTRIAN" && firstYieldFront === null)
        firstYieldFront = front;
    }
    if (id === "sudden-braking") {
      const leadSpeed = sim.traffic.actors.find((a) => a.id === "lead-vehicle")!
        .state.velocity;
      if (snapshot.time >= 5 && leadSpeed < priorLeadSpeed - 0.01)
        leadBraked = true;
      if (
        leadBraked &&
        snapshot.time <= 6 &&
        snapshot.control.acceleration < -0.1 &&
        suddenBrakeResponse === null
      )
        suddenBrakeResponse = snapshot.time - 5;
      priorLeadSpeed = leadSpeed;
    }
    if (
      sim.navigation.state === "DESTINATION_REACHED" &&
      Math.abs(snapshot.vehicle.velocity) <= 1e-9
    ) {
      terminal = true;
      break;
    }
  }
  const result = {
    scenario: id,
    destination,
    terminal,
    seconds: sim.snapshot.time,
    states: Object.fromEntries(states),
    minimumGap: Number.isFinite(minimumGap) ? minimumGap : null,
    maxGapError,
    gapSamples,
    maximumLateral,
    maximumRedFront: Number.isFinite(maximumRedFront) ? maximumRedFront : null,
    stoppedBeforeGreen,
    resumedAfterGreen,
    firstYieldFront,
    suddenBrakeResponse,
    collisionCount: sim.physical.collisionCount,
    roadDepartureCount: sim.physical.roadDepartureCount,
    sensorFrames: sim.sensors.counts,
    finalPosition: sim.snapshot.vehicle,
    finalBehaviour: sim.behaviour.state,
    reason: sim.behaviour.reason,
  };
  console.info("PRODUCTION_SCENARIO", JSON.stringify(result));
  expect(terminal).toBe(true);
  expect(sim.snapshot.time).toBeLessThanOrEqual(LIMITS.seconds);
  expect(maximumLateral).toBeLessThanOrEqual(LIMITS.lateralAcceleration);
  if (id === "following" || id === "sudden-braking") {
    expect(states.get("FOLLOW_LEAD") ?? 0).toBeGreaterThanOrEqual(10);
    expect(gapSamples).toBeGreaterThanOrEqual(10);
    expect(minimumGap).toBeGreaterThanOrEqual(LIMITS.minimumLeadGap);
    expect(maxGapError).toBeLessThanOrEqual(LIMITS.maximumGapError);
  }
  if (id === "sudden-braking") {
    expect(leadBraked).toBe(true);
    expect(suddenBrakeResponse).not.toBeNull();
    expect(suddenBrakeResponse!).toBeLessThanOrEqual(1);
  }
  if (id === "red-light") {
    expect(maximumRedFront).toBeLessThanOrEqual(
      19.5 + LIMITS.stopLineTolerance,
    );
    expect(stoppedBeforeGreen).toBe(true);
    expect(resumedAfterGreen).toBe(true);
  }
  if (id === "pedestrian") {
    expect(firstYieldFront).not.toBeNull();
    expect(firstYieldFront!).toBeLessThan(21);
  }
  return sim;
}

describe("production scenario integration at fixed100Hz", () => {
  it.each([
    "normal",
    "following",
    "sudden-braking",
    "red-light",
    "pedestrian",
  ] as const)(
    "%s completes with no collisions, road departures, or containment interlock",
    (id) => {
      runScenario(id);
    },
    timeout,
  );
  it("rejects the blocked default route without moving or disguising a simulator interlock", () => {
    const sim = new Simulation(undefined, { scenario: "blocked" });
    const initial = sim.snapshot;
    expect(sim.navigation.state).toBe("ROUTE_UNAVAILABLE");
    expect(sim.physical.clearance?.valid).toBe(false);
    expect(sim.behaviour.state).toBe("BLOCKED");
    expect(sim.activateAutonomy()).toBe(false);
    sim.step(IDLE);
    expect(sim.snapshot).toEqual(initial);
    expect(sim.physical.containmentStop).toBe(false);
    expect(sim.physical.collisionCount).toBe(0);
    console.info(
      "PRODUCTION_SCENARIO",
      JSON.stringify({ scenario: "blocked", reason: sim.navigation.message }),
    );
  });
  it.each([
    ["right turn", { x: 24.75, y: -45 }],
    ["multiple intersections", { x: -40, y: -24.75 }],
  ] as const)(
    "normal scenario %s keeps the actual swept body on road",
    (_name, destination) => {
      runScenario("normal", destination);
    },
    timeout,
  );
  it(
    "generates actual moving-object observations and replays all world/sensor timestamps after reset",
    () => {
      const sim = new Simulation(undefined, { scenario: "following" });
      expect(sim.activateAutonomy()).toBe(true);
      let observedVehicle = false,
        observedRangeRate = false;
      for (let tick = 0; tick < 350; tick++) {
        sim.step(IDLE);
        observedVehicle ||=
          sim.sensors.camera?.detections.some(
            (d) => d.objectClass === "vehicle",
          ) ?? false;
        observedRangeRate ||=
          sim.sensors.radar?.detections.some(
            (d) =>
              d.debugObjectId === "lead-vehicle" &&
              Number.isFinite(d.radialVelocity),
          ) ?? false;
      }
      const first = capture(sim);
      expect(observedVehicle).toBe(true);
      expect(observedRangeRate).toBe(true);
      expect(sim.traffic.actors[0].state.x).toBeGreaterThan(-4);
      sim.reset();
      expect(sim.activateAutonomy()).toBe(true);
      for (let tick = 0; tick < 350; tick++) sim.step(IDLE);
      expect(capture(sim)).toEqual(first);
      expect(sim.sensors.lidar?.timestamp).toBeCloseTo(sim.traffic.time, 8);
    },
    timeout,
  );
  it(
    "pauses ego, actors, and sensors together and transfers manual authority without a reset",
    () => {
      const sim = new Simulation(undefined, { scenario: "following" });
      sim.activateAutonomy();
      for (let tick = 0; tick < 100; tick++) sim.step(IDLE);
      sim.pause();
      const paused = capture(sim);
      for (let tick = 0; tick < 200; tick++) sim.step(IDLE);
      sim.advance(2, IDLE);
      expect(capture(sim)).toEqual(paused);
      sim.start();
      const before = sim.snapshot;
      sim.step({ ...IDLE, left: true, brake: true });
      expect(sim.navigation.mode).toBe("MANUAL");
      expect(sim.behaviour.state).toBe("MANUAL");
      expect(sim.snapshot.time).toBeCloseTo(before.time + FIXED_DT);
      expect(sim.snapshot.control.acceleration).toBeLessThanOrEqual(0);
      expect(sim.snapshot.control.steering).toBeGreaterThan(
        before.control.steering,
      );
      expect(
        Math.hypot(
          sim.snapshot.vehicle.x - before.vehicle.x,
          sim.snapshot.vehicle.y - before.vehicle.y,
        ),
      ).toBeLessThan(0.2);
      expect(sim.traffic.time).toBeGreaterThan(paused.traffic.time);
    },
    timeout,
  );
  it(
    "uses the configured time gap in production ACC's desired body separation",
    () => {
      const outcomes = [1, 2.5].map((timeGap) => {
        const sim = new Simulation(undefined, {
          scenario: "following",
          timeGap,
        });
        sim.activateAutonomy();
        for (let tick = 0; tick < 200; tick++) sim.step(IDLE);
        const data = sim.behaviour,
          velocityBeforeLastStep =
            sim.snapshot.vehicle.velocity -
            sim.snapshot.control.acceleration * FIXED_DT;
        expect(data.timeGap).toBe(timeGap);
        expect(data.desiredGap).toBeCloseTo(
          3 + timeGap * Math.max(0, velocityBeforeLastStep),
          8,
        );
        return {
          timeGap,
          desiredGap: data.desiredGap,
          speed: velocityBeforeLastStep,
        };
      });
      console.info("TIME_GAP_VALIDATION", JSON.stringify(outcomes));
      expect(outcomes[1].desiredGap).toBeGreaterThan(outcomes[0].desiredGap);
    },
    timeout,
  );
});
