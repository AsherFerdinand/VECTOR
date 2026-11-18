import { describe, expect, it } from "vitest";
import {
  assessSafety,
  protectiveControl,
  SAFETY_LIMITS,
} from "../src/autonomy/safetySupervisor";
import { integrateVehicle } from "../src/simulation/vehicle";
import type { SensorSnapshot } from "../src/sensors/types";
import type { VehicleState } from "../src/simulation/types";

const vehicle: VehicleState = { x: 0, y: 0, heading: 0, velocity: 6 };
function scan(
  points: { x: number; y: number }[],
  timestamp = 0,
): SensorSnapshot {
  return {
    configs: [
      {
        id: "lidar",
        source: "lidar",
        mount: { x: 1.35, y: 0 },
        yaw: 0,
        fov: 2 * Math.PI,
        maxRange: 40,
        updateHz: 10,
        noise: 0,
        seed: 1,
        rays: 180,
      },
    ],
    lidar: {
      sensorId: "lidar",
      timestamp,
      pose: { x: 1.35, y: 0, heading: 0 },
      maxRange: 40,
      rays: points.length
        ? points.map((point) => ({
            angle: Math.atan2(point.y, point.x - 1.35),
            range: Math.hypot(point.x - 1.35, point.y),
            point,
            hit: true,
          }))
        : [{ angle: 0, range: 40, point: { x: 41.35, y: 0 }, hit: false }],
    },
    camera: null,
    radar: null,
    counts: { lidar: 1, camera: 0, radar: 0 },
  };
}

// Acceptance fixed before evaluation: primary freshness <=0.30 s, manual commands
// never overridden, and a feasible 6 m/s approach must stop >=1 m before the obstacle.
describe("measurement-only protective supervisor", () => {
  it("brakes for a measured obstacle inside the stopping envelope", () => {
    const safety = assessSafety(vehicle, 0, scan([{ x: 8, y: 0 }]), 0, true);
    expect(safety.state).toBe("PROTECTIVE_BRAKING");
    expect(safety.obstacleDistance).toBeCloseTo(4.4);
    expect(safety.stoppingDistance).toBeCloseTo(5.2);
    expect(safety.source).toBe("lidar");
    expect(safety.measurementTime).toBe(0);
    expect(safety.reason).toContain("4.40 m");
    expect(
      protectiveControl(
        { steering: 0.1, acceleration: 3 },
        safety,
        vehicle,
        0.01,
      ),
    ).toEqual({ steering: 0.1, acceleration: -6 });
  });
  it("does not brake for roadside or behind-vehicle measurements", () => {
    expect(
      assessSafety(
        vehicle,
        0,
        scan([
          { x: 7, y: 4 },
          { x: -5, y: 0 },
        ]),
        0,
        true,
      ).state,
    ).toBe("CLEAR");
    expect(
      assessSafety(vehicle, 0, scan([{ x: 30, y: 0 }]), 0, true).state,
    ).toBe("CLEAR");
  });
  it("assesses a curved body corridor instead of treating every nearby point as a threat", () => {
    const steering = 0.4,
      curvature = Math.tan(steering) / 2.7,
      distance = 6;
    const point = {
      x: Math.sin(curvature * distance) / curvature,
      y: (1 - Math.cos(curvature * distance)) / curvature,
    };
    const curved = assessSafety(vehicle, steering, scan([point]), 0, true);
    expect(curved.state).toBe("PROTECTIVE_BRAKING");
    expect(assessSafety(vehicle, -steering, scan([point]), 0, true).state).toBe(
      "CLEAR",
    );
  });
  it("uses current ego localisation to account for movement after acquisition", () => {
    const sensors = scan([{ x: 9, y: 0 }]);
    const advanced = { ...vehicle, x: 1 };
    expect(
      assessSafety(advanced, 0, sensors, 0.1, true).obstacleDistance,
    ).toBeCloseTo(4.4);
    expect(
      assessSafety(advanced, 0, sensors, 0.1, true).stoppingDistance,
    ).toBeGreaterThan(5.2);
  });
  it("stops on missing, stale or future-dated primary measurements", () => {
    const sensors = scan([]);
    expect(
      assessSafety(vehicle, 0, sensors, SAFETY_LIMITS.maximumAge, true).state,
    ).toBe("CLEAR");
    expect(assessSafety(vehicle, 0, sensors, 0.301, true).state).toBe(
      "STALE_STOP",
    );
    expect(
      assessSafety(vehicle, 0, { ...sensors, lidar: null }, 0, true).state,
    ).toBe("STALE_STOP");
    expect(assessSafety(vehicle, 0, scan([], 1), 0, true).state).toBe(
      "STALE_STOP",
    );
  });
  it("does not let radar replace a stale primary LiDAR scan", () => {
    const sensors = scan([], 0);
    sensors.radar = {
      sensorId: "radar",
      timestamp: 1,
      pose: { x: 0, y: 0, heading: 0 },
      detections: [],
    };
    expect(assessSafety(vehicle, 0, sensors, 1, true).state).toBe("STALE_STOP");
  });
  it("accepts a fresh camera observation without reading debug object identity", () => {
    const sensors = scan([]);
    sensors.camera = {
      sensorId: "camera",
      timestamp: 0,
      pose: { x: 2, y: 0, heading: Math.PI / 2 },
      detections: [
        {
          observationId: "measurement-1",
          debugObjectId: "not-used",
          source: "camera",
          timestamp: 0,
          range: 6,
          bearing: -Math.PI / 2,
          relativePosition: { x: 0, y: -6 },
          objectClass: "building",
        },
      ],
    };
    expect(assessSafety(vehicle, 0, sensors, 0, true).source).toBe("camera");
    sensors.camera.detections[0].debugObjectId = "different-unseen-identifier";
    expect(
      assessSafety(vehicle, 0, sensors, 0, true).obstacleDistance,
    ).toBeCloseTo(4.4);
  });
  it("never overrides manual acceleration, even when sensors are stale or an obstacle is near", () => {
    for (const sensors of [
      scan([{ x: 4, y: 0 }]),
      { ...scan([]), lidar: null },
    ]) {
      const safety = assessSafety(vehicle, 0, sensors, 1, false);
      expect(safety.state).toBe("MANUAL");
      expect(safety.braking).toBe(false);
      expect(
        protectiveControl(
          { steering: 0.2, acceleration: 3 },
          safety,
          vehicle,
          0.01,
        ),
      ).toEqual({ steering: 0.2, acceleration: 3 });
    }
  });
  it("can release an assessment after a fresh clear scan; the simulation owns the safe-stop latch", () => {
    expect(
      assessSafety(vehicle, 0, scan([{ x: 5, y: 0 }]), 0, true).braking,
    ).toBe(true);
    expect(assessSafety(vehicle, 0, scan([], 0.1), 0.1, true).state).toBe(
      "CLEAR",
    );
  });
  it("clips final braking to rest, including reverse motion", () => {
    const safety = assessSafety(vehicle, 0, scan([{ x: 5, y: 0 }]), 0, true);
    for (const velocity of [0.01, -0.01, 0]) {
      const state = { ...vehicle, velocity };
      const result = integrateVehicle(
        state,
        protectiveControl(
          { steering: 0, acceleration: 3 },
          safety,
          state,
          0.01,
        ),
        0.01,
      );
      expect(result.velocity).toBe(0);
    }
  });
  it("stops a feasible repeated-scan approach before contact using only measurements", () => {
    let state = { ...vehicle },
      latched = false;
    const obstacleX = 23.6;
    let triggerGap: number | null = null;
    for (let tick = 0; tick < 1000; tick++) {
      const time = tick * 0.01;
      const assessment = assessSafety(
        state,
        0,
        scan([{ x: obstacleX, y: 0 }], time),
        time,
        true,
      );
      if (assessment.braking && !latched)
        triggerGap = obstacleX - state.x - 3.6;
      latched ||= assessment.braking;
      const safety = latched
        ? { ...assessment, braking: true, state: "PROTECTIVE_BRAKING" as const }
        : assessment;
      state = integrateVehicle(
        state,
        protectiveControl(
          { steering: 0, acceleration: 0 },
          safety,
          state,
          0.01,
        ),
        0.01,
      );
      if (latched && state.velocity === 0) break;
    }
    const gap = obstacleX - state.x - 3.6;
    expect(triggerGap).not.toBeNull();
    expect(state.velocity).toBe(0);
    expect(gap).toBeGreaterThanOrEqual(1);
    console.info(
      `SAFETY VALIDATION: trigger gap=${triggerGap?.toFixed(6)} m, stopped gap=${gap.toFixed(6)} m, speed=${state.velocity} m/s`,
    );
  });
});
