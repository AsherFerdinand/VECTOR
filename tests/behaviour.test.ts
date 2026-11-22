import { describe, expect, it } from "vitest";
import {
  BehaviourPlanner,
  BEHAVIOUR_LIMITS,
} from "../src/autonomy/behaviourPlanner";
import { integrateVehicle, clamp } from "../src/simulation/vehicle";
import type { VehicleState, VehicleControl } from "../src/simulation/types";
import type { BehaviourContext } from "../src/autonomy/behaviourTypes";
import type {
  Detection,
  DetectionFrame,
  SensorSnapshot,
} from "../src/sensors/types";
import { DEFAULT_SENSOR_CONFIGS } from "../src/sensors/suite";
import { generateCamera } from "../src/sensors/camera";
import { generateRadar } from "../src/sensors/radar";
import type { WorldObject } from "../src/world/types";

const origin: VehicleState = { x: 0, y: 0, heading: 0, velocity: 6 };
const context: BehaviourContext = {
  route: null,
  progress: 0,
  timeGap: 1.5,
  targetSpeed: 6,
};
const nav = (v: number): VehicleControl => ({
  steering: 0,
  acceleration: clamp(2.5 * (6 - v), -6, 3),
});
type Observation = {
  x: number;
  y: number;
  objectClass: Detection["objectClass"];
  speed?: number;
  signal?: Detection["signal"];
};
function measurements(
  vehicle: VehicleState,
  time: number,
  observations: Observation[] = [],
  lidarHit?: { x: number; y: number },
): SensorSnapshot {
  const pose = { x: vehicle.x, y: vehicle.y, heading: vehicle.heading };
  function frame(source: "camera" | "radar"): DetectionFrame {
    return {
      sensorId: source,
      timestamp: time,
      pose,
      detections: observations.map((o, i) => {
        const dx = o.x - vehicle.x,
          dy = o.y - vehicle.y,
          c = Math.cos(vehicle.heading),
          s = Math.sin(vehicle.heading);
        const x = c * dx + s * dy,
          y = -s * dx + c * dy;
        return {
          observationId: `${source}-${i}`,
          source,
          timestamp: time,
          objectClass: o.objectClass,
          relativePosition: { x, y },
          range: Math.hypot(x, y),
          bearing: Math.atan2(y, x),
          radialVelocity:
            source === "radar"
              ? ((o.speed ?? 0) - vehicle.velocity) * Math.cos(Math.atan2(y, x))
              : undefined,
          signal: source === "camera" ? o.signal : undefined,
        };
      }),
    };
  }
  return {
    configs: DEFAULT_SENSOR_CONFIGS,
    lidar: {
      sensorId: "lidar",
      timestamp: time,
      pose,
      maxRange: 40,
      rays: [
        {
          angle: 0,
          range: lidarHit
            ? Math.hypot(lidarHit.x - vehicle.x, lidarHit.y - vehicle.y)
            : 40,
          point: lidarHit ?? { x: vehicle.x + 40, y: vehicle.y },
          hit: !!lidarHit,
        },
      ],
    },
    camera: frame("camera"),
    radar: frame("radar"),
    counts: { lidar: 1, camera: 1, radar: 1 },
  };
}
function signal(
  x: number,
  color: "red" | "yellow" | "green",
  heading = 0,
): Observation {
  return {
    x: x + 4,
    y: -3.5,
    objectClass: "traffic-light",
    signal: {
      color,
      heading,
      stopLine: [
        { x, y: -2.25 },
        { x, y: 2.25 },
      ],
    },
  };
}

// Acceptance defined before running: no normal jerk step exceeds3m/s³; following
// gap stays>=3m in these feasible cases; red/pedestrian stop leaves>=0.2m clearance;
// urgent braking uses at most6m/s²; fresh-clear observations resume after0.50s.
describe("observed behaviour and command authority", () => {
  it("never overrides manual driving, even with missing sensors and an immediate hazard", () => {
    const planner = new BehaviourPlanner(),
      snapshots = measurements(origin, 0, [signal(4, "red")], { x: 4, y: 0 });
    snapshots.lidar = null;
    expect(
      planner.update(
        origin,
        { steering: 0.2, acceleration: 3 },
        snapshots,
        0,
        false,
        context,
      ),
    ).toEqual({ steering: 0.2, acceleration: 3 });
    expect(planner.snapshot.state).toBe("MANUAL");
    expect(planner.snapshot.aeb).toBe(false);
  });
  it("prioritizes stale data over AEB and AEB over pedestrian/signal responses", () => {
    const planner = new BehaviourPlanner(),
      sensors = measurements(
        origin,
        0,
        [signal(8, "red"), { x: 9, y: 2, objectClass: "pedestrian" }],
        { x: 5, y: 0 },
      );
    planner.update(origin, nav(6), sensors, 0, true, context);
    expect(planner.snapshot.state).toBe("AEB");
    expect(planner.snapshot.acceleration).toBe(-6);
    planner.update(origin, nav(6), sensors, 0.31, true, context);
    expect(planner.snapshot.state).toBe("STALE_STOP");
  });
  it("ignores irrelevant roadside detections and radar signal semantics", () => {
    const planner = new BehaviourPlanner();
    const sensors = measurements(origin, 0, [
      { x: 9, y: 4, objectClass: "building" },
      { x: 12, y: 5, objectClass: "vehicle", speed: 0 },
    ]);
    sensors.radar!.detections.push({
      observationId: "not-camera",
      source: "radar",
      timestamp: 0,
      range: 20,
      bearing: 0,
      relativePosition: { x: 20, y: 0 },
      objectClass: "traffic-light",
      signal: signal(15, "red").signal,
    });
    planner.update(origin, nav(6), sensors, 0, true, context);
    expect(planner.snapshot.state).toBe("CRUISE");
    expect(planner.snapshot.acceleration).toBe(0);
  });
  it("uses configured time gap and correct radar closing-speed sign", () => {
    const planner = new BehaviourPlanner({ dMin: 3, timeGap: 1.5 });
    planner.update(
      origin,
      nav(6),
      measurements(origin, 0, [
        { x: 15.6, y: 0, objectClass: "vehicle", speed: 4 },
      ]),
      0,
      true,
      { ...context, timeGap: 2 },
    );
    expect(planner.snapshot.state).toBe("FOLLOW_LEAD");
    expect(planner.snapshot.leadDistance).toBeCloseTo(12);
    expect(planner.snapshot.relativeSpeed).toBeCloseTo(-2);
    expect(planner.snapshot.desiredGap).toBe(15);
    expect(planner.snapshot.ttc).toBeCloseTo(6);
    expect(planner.snapshot.acceleration).toBeLessThanOrEqual(0);
  });
  it("does not AEB for a speed-matched lead while ACC opens a short gap", () => {
    const planner = new BehaviourPlanner();
    planner.update(
      origin,
      nav(6),
      measurements(
        origin,
        0,
        [{ x: 7.6, y: 0, objectClass: "vehicle", speed: 6 }],
        { x: 7.6, y: 0 },
      ),
      0,
      true,
      context,
    );
    expect(planner.snapshot.state).toBe("FOLLOW_LEAD");
    expect(planner.snapshot.aeb).toBe(false);
    expect(planner.snapshot.ttc).toBeNull();
    expect(planner.snapshot.acceleration).toBeLessThan(0);
  });
  it("reports destination state and resets deterministically", () => {
    const planner = new BehaviourPlanner();
    planner.update(
      { ...origin, velocity: 0 },
      nav(0),
      measurements(origin, 0),
      0,
      true,
      { ...context, targetSpeed: 0, destinationReached: true },
    );
    expect(planner.snapshot.state).toBe("DESTINATION_REACHED");
    expect(planner.snapshot.acceleration).toBe(0);
    planner.reset();
    expect(planner.snapshot).toEqual(new BehaviourPlanner().snapshot);
  });
  it("waits for stable fresh observations before automatically releasing stale stop", () => {
    const planner = new BehaviourPlanner(),
      stopped = { ...origin, velocity: 0 };
    planner.update(
      stopped,
      nav(0),
      measurements(stopped, 0),
      0.4,
      true,
      context,
    );
    expect(planner.snapshot.state).toBe("STALE_STOP");
    planner.update(
      stopped,
      nav(0),
      measurements(stopped, 0.5),
      0.5,
      true,
      context,
    );
    expect(planner.snapshot.state).toBe("STALE_STOP");
    planner.update(stopped, nav(0), measurements(stopped, 1), 1, true, context);
    expect(planner.snapshot.state).toBe("CRUISE");
    expect(planner.snapshot.acceleration).toBeGreaterThan(0);
  });
});

describe("lead-following engineering scenarios", () => {
  it("converges to a moving lead speed while respecting minimum gap and normal jerk", () => {
    const planner = new BehaviourPlanner();
    let vehicle = { ...origin, velocity: 0 },
      leadX = 14.6,
      previousAcceleration = 0,
      minimumGap = Infinity;
    for (let tick = 0; tick < 2500; tick++) {
      const now = tick * 0.01;
      const command = planner.update(
        vehicle,
        nav(vehicle.velocity),
        measurements(vehicle, now, [
          { x: leadX, y: 0, objectClass: "vehicle", speed: 2.8 },
        ]),
        now,
        true,
        context,
      );
      expect(
        Math.abs(command.acceleration - previousAcceleration),
      ).toBeLessThanOrEqual(BEHAVIOUR_LIMITS.jerk * 0.01 + 1e-8);
      expect(planner.snapshot.aeb).toBe(false);
      vehicle = integrateVehicle(vehicle, command, 0.01);
      leadX += 2.8 * 0.01;
      minimumGap = Math.min(minimumGap, leadX - vehicle.x - 3.6);
      previousAcceleration = command.acceleration;
    }
    expect(minimumGap).toBeGreaterThanOrEqual(3);
    expect(vehicle.velocity).toBeCloseTo(2.8, 1);
    const finalGap = leadX - vehicle.x - 3.6;
    expect(Math.abs(finalGap - (3 + 1.5 * vehicle.velocity))).toBeLessThan(0.5);
    console.info(
      "BEHAVIOUR_FOLLOW",
      JSON.stringify({ minimumGap, finalGap, finalSpeed: vehicle.velocity }),
    );
  });
  it("stops behind a suddenly braking lead without violating the minimum gap", () => {
    const planner = new BehaviourPlanner();
    let vehicle = { ...origin, velocity: 3 },
      leadX = 15.6,
      leadSpeed = 3,
      minimumGap = Infinity;
    for (let tick = 0; tick < 1800; tick++) {
      const now = tick * 0.01;
      if (now >= 5) leadSpeed = Math.max(0, leadSpeed - 6 * 0.01);
      const command = planner.update(
        vehicle,
        nav(vehicle.velocity),
        measurements(vehicle, now, [
          { x: leadX, y: 0, objectClass: "vehicle", speed: leadSpeed },
        ]),
        now,
        true,
        context,
      );
      vehicle = integrateVehicle(vehicle, command, 0.01);
      leadX += leadSpeed * 0.01;
      minimumGap = Math.min(minimumGap, leadX - vehicle.x - 3.6);
    }
    expect(minimumGap).toBeGreaterThanOrEqual(3);
    expect(vehicle.velocity).toBeLessThan(0.05);
    console.info(
      "BEHAVIOUR_SUDDEN_BRAKE",
      JSON.stringify({
        minimumGap,
        finalSpeed: vehicle.velocity,
        state: planner.snapshot.state,
      }),
    );
  });
});

describe("observed signals and pedestrians", () => {
  it("stops before a red line and automatically resumes after observed green", () => {
    const planner = new BehaviourPlanner();
    let vehicle = { ...origin },
      minimumRedGap = Infinity,
      stopped = false;
    for (let tick = 0; tick < 1500; tick++) {
      const now = tick * 0.01,
        color = now < 12 ? "red" : "green";
      const command = planner.update(
        vehicle,
        nav(vehicle.velocity),
        measurements(vehicle, now, [signal(25, color)]),
        now,
        true,
        context,
      );
      vehicle = integrateVehicle(vehicle, command, 0.01);
      if (color === "red") {
        minimumRedGap = Math.min(minimumRedGap, 25 - vehicle.x - 3.6);
        stopped ||= vehicle.velocity < 0.05;
      }
    }
    expect(minimumRedGap).toBeGreaterThanOrEqual(0.2);
    expect(stopped).toBe(true);
    expect(vehicle.velocity).toBeGreaterThan(1);
    expect(planner.snapshot.state).toBe("CRUISE");
    console.info(
      "BEHAVIOUR_RED_GREEN",
      JSON.stringify({ minimumRedGap, resumedSpeed: vehicle.velocity }),
    );
  });
  it("proceeds through a late yellow instead of harsh braking but stops for an early yellow", () => {
    const late = new BehaviourPlanner();
    late.update(
      origin,
      nav(6),
      measurements(origin, 0, [signal(7, "yellow")]),
      0,
      true,
      context,
    );
    expect(late.snapshot.state).toBe("CRUISE");
    expect(late.snapshot.acceleration).toBe(0);
    late.update(
      origin,
      nav(6),
      measurements(origin, 0.1, [signal(7, "red")]),
      0.1,
      true,
      context,
    );
    expect(late.snapshot.state).toBe("CRUISE");
    const early = new BehaviourPlanner();
    early.update(
      origin,
      nav(6),
      measurements(origin, 0, [signal(35, "yellow")]),
      0,
      true,
      context,
    );
    expect(early.snapshot.state).toBe("STOP_LIGHT");
  });
  it("ignores a red signal assigned to the opposite direction", () => {
    const planner = new BehaviourPlanner();
    planner.update(
      origin,
      nav(6),
      measurements(origin, 0, [signal(20, "red", Math.PI)]),
      0,
      true,
      context,
    );
    expect(planner.snapshot.state).toBe("CRUISE");
  });
  it("holds an observed red if the light becomes occluded, without claiming a new red measurement", () => {
    const planner = new BehaviourPlanner(),
      stopped = { ...origin, velocity: 0 };
    planner.update(
      stopped,
      nav(0),
      measurements(stopped, 0, [signal(10, "red")]),
      0,
      true,
      context,
    );
    planner.update(
      stopped,
      nav(0),
      measurements(stopped, 0.2),
      0.2,
      true,
      context,
    );
    expect(planner.snapshot.state).toBe("STOP_LIGHT");
    expect(planner.snapshot.observationTime).toBe(0);
    expect(planner.snapshot.reason).toContain("Last observed");
  });
  it("yields to a near-crossing pedestrian then resumes after the crossing clears", () => {
    const planner = new BehaviourPlanner();
    let vehicle = { ...origin, velocity: 0 },
      minimumYieldGap = Infinity,
      yielded = false;
    for (let tick = 0; tick < 1100; tick++) {
      const now = tick * 0.01,
        present = now < 7;
      const observations: Observation[] = present
        ? [{ x: 16, y: 3, objectClass: "pedestrian", speed: 0 }]
        : [];
      const command = planner.update(
        vehicle,
        nav(vehicle.velocity),
        measurements(vehicle, now, observations),
        now,
        true,
        context,
      );
      vehicle = integrateVehicle(vehicle, command, 0.01);
      if (present) {
        yielded ||= planner.snapshot.state === "YIELD_PEDESTRIAN";
        minimumYieldGap = Math.min(minimumYieldGap, 16 - vehicle.x - 3.6);
      }
    }
    expect(yielded).toBe(true);
    expect(minimumYieldGap).toBeGreaterThanOrEqual(0.2);
    expect(planner.snapshot.state).toBe("CRUISE");
    expect(vehicle.velocity).toBeGreaterThan(1);
    console.info(
      "BEHAVIOUR_PEDESTRIAN",
      JSON.stringify({ minimumYieldGap, resumedSpeed: vehicle.velocity }),
    );
  });
});

describe("sensor semantic observations", () => {
  it("copies only visible camera signal semantics and preserves measurement snapshots", () => {
    const pole: WorldObject = {
      id: "pole",
      objectClass: "traffic-light",
      height: 4,
      collidable: true,
      velocity: { x: 0, y: 0 },
      polygon: [
        { x: 20, y: -3 },
        { x: 20.2, y: -3 },
        { x: 20.2, y: -2.8 },
        { x: 20, y: -2.8 },
      ],
      signal: signal(18, "red").signal,
    };
    const camera = DEFAULT_SENSOR_CONFIGS.find((c) => c.source === "camera")!;
    const observed = generateCamera(origin, [pole], camera, 0);
    expect(observed.detections[0].signal?.color).toBe("red");
    pole.signal!.color = "green";
    expect(observed.detections[0].signal?.color).toBe("red");
    expect(
      generateCamera(origin, [pole], camera, 0.1).detections[0].signal?.color,
    ).toBe("green");
    const behind = {
      ...pole,
      polygon: pole.polygon.map((p) => ({ x: -p.x, y: p.y })),
    };
    expect(
      generateCamera(origin, [behind], camera, 0.2).detections,
    ).toHaveLength(0);
  });
  it("associates camera class with radar range-rate without adding ideal radar semantics", () => {
    const car: WorldObject = {
      id: "car",
      objectClass: "vehicle",
      height: 1.5,
      collidable: true,
      velocity: { x: 3, y: 0 },
      polygon: [
        { x: 20, y: -0.9 },
        { x: 24.5, y: -0.9 },
        { x: 24.5, y: 0.9 },
        { x: 20, y: 0.9 },
      ],
    };
    const config = DEFAULT_SENSOR_CONFIGS.find((c) => c.source === "radar")!;
    const output = generateRadar(origin, [car], config, 0);
    expect(output.detections[0].objectClass).toBeUndefined();
    expect(output.detections[0].radialVelocity).toBeLessThan(0);
    expect(output.detections[0].signal).toBeUndefined();
    const cameraConfig = DEFAULT_SENSOR_CONFIGS.find(
      (c) => c.source === "camera",
    )!;
    const sensors = measurements(origin, 0);
    sensors.camera = generateCamera(origin, [car], cameraConfig, 0);
    sensors.radar = output;
    const planner = new BehaviourPlanner();
    planner.update(origin, nav(6), sensors, 0, true, context);
    expect(planner.snapshot.state).toBe("FOLLOW_LEAD");
    expect(planner.snapshot.relativeSpeed).toBeLessThan(0);
    expect(planner.snapshot.source).toBe("radar");
  });
});
