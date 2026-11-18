import { describe, expect, it } from "vitest";
import { Simulation } from "../src/simulation/clock";
import { integrateVehicle, INITIAL_STATE } from "../src/simulation/vehicle";
import {
  polygonsOverlap,
  resolveMotion,
  vehicleFootprint,
} from "../src/simulation/collision";
import { projectToRoad } from "../src/navigation/graph";
import { SensorSuite, DEFAULT_SENSOR_CONFIGS } from "../src/sensors/suite";
import {
  assessSafety,
  protectiveControl,
} from "../src/autonomy/safetySupervisor";
import type { DriverInput, VehicleState } from "../src/simulation/types";
import type { WorldObject } from "../src/world/types";
import type { SafetySnapshot } from "../src/autonomy/safetyTypes";

const idle: DriverInput = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};
const forward = { ...idle, forward: true };
function box(
  x: number,
  y: number,
  width: number,
  depth: number,
  id = "test-wall",
): WorldObject {
  return {
    id,
    objectClass: "static-obstacle",
    polygon: [
      { x, y },
      { x: x + width, y },
      { x: x + width, y: y + depth },
      { x, y: y + depth },
    ],
    height: 3,
    velocity: { x: 0, y: 0 },
    collidable: true,
  };
}
function assertFree(simulation: Simulation) {
  expect(
    simulation.objects.some(
      (object) =>
        object.collidable &&
        polygonsOverlap(
          vehicleFootprint(simulation.snapshot.vehicle),
          object.polygon,
        ),
    ),
  ).toBe(false);
}

// Acceptance declared before evaluation: contact position error <=1e-6 m; no
// penetration at any resolved pose; <=one event while holding the same contact;
// original routes complete within180 simulation seconds and <=0.8 m endpoint error.
// Real10Hz sensor protection at6 m/s must stop >=1 m before a known wide obstacle.
describe("production collision, sensor and navigation integration", () => {
  it("A: prevents building penetration, records one contact and permits reverse escape", () => {
    const sim = new Simulation({ x: -25, y: -43, heading: 0, velocity: 0 });
    sim.start();
    for (let tick = 0; tick < 450; tick++) {
      sim.step(forward);
      assertFree(sim);
      if (sim.physical.collisionCount > 0) {
        expect(sim.snapshot.vehicle.velocity).toBe(0);
        expect(Math.abs(sim.snapshot.vehicle.x + 19.6)).toBeLessThanOrEqual(
          1e-6,
        );
      }
    }
    expect(sim.physical.collisionCount).toBe(1);
    expect(sim.physical.collision?.objectId).toBe("building-1");
    expect(Math.abs(sim.snapshot.vehicle.x - -19.6)).toBeLessThanOrEqual(1e-6);
    expect(sim.snapshot.vehicle.velocity).toBe(0);
    const contactX = sim.snapshot.vehicle.x;
    const firstContact = sim.physical.collision;
    for (let tick = 0; tick < 100; tick++)
      sim.step({ ...idle, backward: true });
    expect(sim.snapshot.vehicle.velocity).toBeLessThan(0);
    expect(sim.snapshot.vehicle.x).toBeLessThan(contactX - 1);
    expect(sim.physical.collisionCount).toBe(1);
    expect(sim.safety.state).toBe("MANUAL");
    assertFree(sim);
    const reverseX = sim.snapshot.vehicle.x;
    sim.reset();
    expect(sim.physical.collisionCount).toBe(0);
    expect(sim.physical.collisionEvents).toEqual([]);
    expect(sim.sensors.counts).toEqual({ lidar: 1, camera: 1, radar: 1 });
    sim.start();
    for (let tick = 0; tick < 450; tick++) sim.step(forward);
    expect(sim.physical.collision).toEqual(firstContact);
    expect(sim.physical.collisionCount).toBe(1);
    expect(sim.snapshot.vehicle.velocity).toBe(0);
    console.info(
      "INTEGRATION_A",
      JSON.stringify({
        contactRearAxleX: contactX,
        contactError: Math.abs(contactX + 19.6),
        collisionCount: sim.physical.collisionCount,
        reverseRearAxleX: reverseX,
      }),
    );
  });

  it("B: resolves an elevated-speed first tick against a1mm wall without tunnelling", () => {
    const wall = box(4, -2, 0.001, 4);
    // Deliberately outside normal speed limits to stress the sweep, not a supported drive setting.
    const sim = new Simulation(
      { x: 0, y: 0, heading: 0, velocity: 100 },
      { objects: [wall] },
    );
    sim.start();
    sim.step(idle);
    expect(sim.physical.collisionCount).toBe(1);
    expect(sim.snapshot.vehicle.x).toBeCloseTo(0.4, 6);
    expect(sim.snapshot.vehicle.velocity).toBe(0);
    assertFree(sim);
    console.info(
      "INTEGRATION_B",
      JSON.stringify({
        initialSpeed: 100,
        firstStepContactX: sim.snapshot.vehicle.x,
        finalSpeed: sim.snapshot.vehicle.velocity,
        penetrating: polygonsOverlap(
          vehicleFootprint(sim.snapshot.vehicle),
          wall.polygon,
        ),
      }),
    );
  });

  it("C: records partial then full road departure without treating grass as a collision", () => {
    const sim = new Simulation({
      x: -10,
      y: -29.25,
      heading: Math.PI / 2,
      velocity: 0,
    });
    expect(sim.physical.roadStatus).toBe("ON_ROAD");
    sim.start();
    const observed = new Set<string>([sim.physical.roadStatus]);
    for (
      let tick = 0;
      tick < 300 && sim.physical.roadStatus !== "FULLY_OFF_ROAD";
      tick++
    ) {
      sim.step(forward);
      observed.add(sim.physical.roadStatus);
    }
    expect([...observed]).toEqual([
      "ON_ROAD",
      "PARTIALLY_OFF_ROAD",
      "FULLY_OFF_ROAD",
    ]);
    expect(sim.physical.roadDepartureCount).toBe(2);
    expect(sim.physical.roadDepartures.map((event) => event.status)).toEqual([
      "PARTIALLY_OFF_ROAD",
      "FULLY_OFF_ROAD",
    ]);
    expect(sim.physical.collisionCount).toBe(0);
    expect(sim.snapshot.vehicle.velocity).toBeGreaterThan(0);
    console.info(
      "INTEGRATION_C",
      JSON.stringify({
        statuses: [...observed],
        departureEvents: sim.physical.roadDepartureCount,
        collisions: sim.physical.collisionCount,
        time: sim.snapshot.time,
        rearAxleY: sim.snapshot.vehicle.y,
      }),
    );
  });

  it("generates11 frames per sensor at1 s, freezes on pause, and exactly resets/replays", () => {
    const sim = new Simulation(),
      initialSensors = sim.sensors;
    sim.start();
    for (let tick = 0; tick < 100; tick++) sim.step(forward);
    expect(sim.snapshot.time).toBe(1);
    expect(sim.sensors.counts).toEqual({ lidar: 11, camera: 11, radar: 11 });
    expect(sim.sensors.lidar?.timestamp).toBe(1);
    expect(sim.sensors.lidar?.pose.x).toBeGreaterThan(
      initialSensors.lidar!.pose.x,
    );
    const firstRun = {
      state: sim.snapshot.vehicle,
      sensors: sim.sensors,
      physical: sim.physical,
    };
    sim.pause();
    const paused = sim.snapshot,
      pausedSensors = sim.sensors;
    sim.advance(20, forward);
    sim.step(forward);
    expect(sim.snapshot).toEqual(paused);
    expect(sim.sensors).toBe(pausedSensors);
    sim.reset();
    expect(sim.sensors).toEqual(initialSensors);
    expect(sim.snapshot.vehicle).toEqual(INITIAL_STATE);
    expect(sim.snapshot.time).toBe(0);
    expect(sim.physical.collisionEvents).toEqual([]);
    expect(sim.physical.roadDepartures).toEqual([]);
    sim.start();
    for (let tick = 0; tick < 100; tick++) sim.step(forward);
    expect({
      state: sim.snapshot.vehicle,
      sensors: sim.sensors,
      physical: sim.physical,
    }).toEqual(firstRun);
  });

  it("rejects known static route obstruction in preflight rather than silently following it", () => {
    const wall = box(-5, -30.25, 0.1, 2, "route-blocker");
    const sim = new Simulation(INITIAL_STATE, { objects: [wall] });
    const before = sim.snapshot;
    const destination = projectToRoad(sim.graph, { x: 10, y: -29.25 })!;
    expect(sim.planTo(destination)).toBe(false);
    expect(sim.physical.clearance).toMatchObject({
      valid: false,
      reason: "Vehicle footprint intersects route-blocker",
    });
    expect(sim.navigation.state).toBe("ROUTE_UNAVAILABLE");
    expect(sim.navigation.route).toBeNull();
    expect(sim.snapshot).toEqual(before);
    expect(sim.activateAutonomy()).toBe(false);
    expect(sim.navigation.mode).toBe("MANUAL");
  });

  it("H: preserves right-turn and multi-intersection completion with collision and sensor layers enabled", () => {
    const measurements = [];
    for (const [name, destination] of [
      ["right", { x: 24.75, y: -45 }],
      ["multi", { x: -40, y: -24.75 }],
    ] as const) {
      const sim = new Simulation();
      expect(sim.planTo(projectToRoad(sim.graph, destination)!)).toBe(true);
      expect(sim.physical.clearance?.valid).toBe(true);
      expect(sim.activateAutonomy()).toBe(true);
      for (
        let tick = 0;
        tick < 18000 && sim.navigation.state !== "DESTINATION_REACHED";
        tick++
      ) {
        sim.step(idle);
        if (sim.navigation.state === "SAFE_STOP") break;
      }
      expect(sim.navigation.state, `${name}: ${sim.safety.reason}`).toBe(
        "DESTINATION_REACHED",
      );
      sim.step(idle);
      const error = Math.hypot(
        sim.snapshot.vehicle.x - destination.x,
        sim.snapshot.vehicle.y - destination.y,
      );
      expect(error).toBeLessThanOrEqual(0.8);
      expect(sim.snapshot.vehicle.velocity).toBe(0);
      expect(sim.physical.collisionCount).toBe(0);
      expect(sim.safety.braking).toBe(false);
      assertFree(sim);
      expect(sim.sensors.counts.lidar).toBe(
        Math.floor(sim.snapshot.time * 10 + 1e-9) + 1,
      );
      measurements.push({
        name,
        endpointError: error,
        time: sim.snapshot.time,
        collisions: sim.physical.collisionCount,
        roadDepartureEvents: sim.physical.roadDepartureCount,
        lidarScans: sim.sensors.counts.lidar,
      });
    }
    console.info("INTEGRATION_H", JSON.stringify(measurements));
  }, 30000);

  it("latches stale-sensor stopping, preserves it on resume, and restores manual authority on takeover", () => {
    const configs = DEFAULT_SENSOR_CONFIGS.map((config) => ({
      ...config,
      mount: { ...config.mount },
      updateHz: config.source === "lidar" ? 1 : config.updateHz,
    }));
    const sim = new Simulation(INITIAL_STATE, { sensorConfigs: configs });
    expect(sim.planTo(projectToRoad(sim.graph, { x: 10, y: -29.25 })!)).toBe(
      true,
    );
    expect(sim.activateAutonomy()).toBe(true);
    for (let tick = 0; tick < 35; tick++) sim.step(idle);
    expect(sim.safety.state).toBe("STALE_STOP");
    expect(sim.navigation.state).toBe("SAFE_STOP");
    for (let tick = 0; tick < 200; tick++) sim.step(idle);
    expect(sim.snapshot.vehicle.velocity).toBe(0);
    expect(sim.sensors.lidar!.timestamp).toBeGreaterThan(0);
    expect(sim.safety.state).toBe("STALE_STOP");
    sim.pause();
    const frozen = sim.snapshot;
    sim.advance(2, idle);
    expect(sim.snapshot).toEqual(frozen);
    sim.start();
    sim.step(idle);
    expect(sim.safety.state).toBe("STALE_STOP");
    sim.cancelAutonomy();
    expect(sim.safety.state).toBe("MANUAL");
    expect(sim.safety.braking).toBe(false);
    sim.step(forward);
    expect(sim.snapshot.vehicle.velocity).toBeGreaterThan(0);
    expect(sim.navigation.mode).toBe("MANUAL");
    sim.reset();
    expect(sim.safety.state).toBe("MANUAL");
    expect(sim.safety.braking).toBe(false);
    expect(sim.sensors.lidar?.timestamp).toBe(0);
    expect(sim.navigation.state).toBe("ROUTE_READY");
  });

  it("G: stops before a wall using real10Hz sensor scans and never relies on collision response", () => {
    const wall = box(23.6, -10, 0.1, 20),
      suite = new SensorSuite([wall]);
    let vehicle: VehicleState = { x: 0, y: 0, heading: 0, velocity: 6 };
    let latch: SafetySnapshot | null = null,
      collisionCount = 0,
      triggerGap: number | null = null,
      stopTime = 0;
    for (let tick = 0; tick < 1000; tick++) {
      const time = tick * 0.01;
      suite.update(time, vehicle);
      const assessment = assessSafety(vehicle, 0, suite.snapshot, time, true);
      if (assessment.braking && !latch) {
        latch = assessment;
        triggerGap = 23.6 - vehicle.x - 3.6;
      }
      const requested = protectiveControl(
        { steering: 0, acceleration: 0 },
        latch ?? assessment,
        vehicle,
        0.01,
      );
      const motion = resolveMotion(
        vehicle,
        integrateVehicle(vehicle, requested, 0.01),
        [wall],
      );
      collisionCount += Number(motion.collided);
      vehicle = motion.state;
      if (latch && vehicle.velocity === 0) {
        stopTime = (tick + 1) * 0.01;
        break;
      }
    }
    const finalGap = 23.6 - vehicle.x - 3.6;
    expect(latch?.state).toBe("PROTECTIVE_BRAKING");
    expect(triggerGap).not.toBeNull();
    expect(collisionCount).toBe(0);
    expect(vehicle.velocity).toBe(0);
    expect(finalGap).toBeGreaterThanOrEqual(1);
    expect(polygonsOverlap(vehicleFootprint(vehicle), wall.polygon)).toBe(
      false,
    );
    console.info(
      "INTEGRATION_G",
      JSON.stringify({
        triggerFrontClearance: triggerGap,
        stoppedFrontClearance: finalGap,
        stopTime,
        speed: vehicle.velocity,
        collisionCount,
        scanCounts: suite.snapshot.counts,
      }),
    );
  });
});
