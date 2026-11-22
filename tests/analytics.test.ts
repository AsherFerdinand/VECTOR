import { describe, expect, it } from "vitest";
import { Simulation } from "../src/simulation/clock";
import {
  buildTimeline,
  calculateMetrics,
  createValidationResult,
} from "../src/validation/metrics";
import {
  compareResults,
  engineeringReport,
  exportCSV,
  exportJSON,
} from "../src/validation/exports";
import { ReplayPlayer } from "../src/validation/replay";
import {
  MAX_RECORDED_FRAMES,
  type ValidationConfig,
  type ValidationFrame,
} from "../src/validation/types";

const config: ValidationConfig = {
  id: "fixture",
  scenario: "normal",
  seed: 7,
  duration: 1,
};
function frame(time: number): ValidationFrame {
  const sim = new Simulation(undefined, { scenario: "normal" });
  return structuredClone({
    time,
    vehicle: sim.snapshot.vehicle,
    control: sim.snapshot.control,
    behaviour: sim.behaviour,
    physical: sim.physical,
    navigation: sim.navigation,
  });
}

describe("validation analytics", () => {
  it("measures physical/controller peaks and following RMSE without inventing unavailable measurements", () => {
    const a = frame(0),
      b = frame(0.1);
    a.control.acceleration = -4;
    b.vehicle.velocity = 4.6;
    a.vehicle.velocity = 5;
    a.control.steering = 0.1;
    a.behaviour.state = "FOLLOW_LEAD";
    a.behaviour.leadDistance = 8;
    a.behaviour.desiredGap = 5;
    a.behaviour.ttc = 2;
    b.behaviour.ttc = 1;
    b.navigation.crossTrackError = -0.4;
    b.navigation.state = "DESTINATION_REACHED";
    const metrics = calculateMetrics([a, b]);
    expect(metrics.maximumDeceleration).toBeCloseTo(4);
    expect(metrics.maximumLateralAcceleration).toBeCloseTo(
      (25 * Math.tan(0.1)) / 2.7,
    );
    expect(metrics.maximumCrossTrackError).toBe(0.4);
    expect(metrics.followingDistanceRMSE).toBe(3);
    expect(metrics.minimumTTC).toBe(1);
    expect(metrics.routeCompleted).toBe(true);
    expect(metrics.impactSpeed).toBeNull();
    expect(metrics.aebResponseTime).toBeNull();
    expect(metrics.minimumClearance).toBeNull();
  });
  it("matches truth one-to-one only for validation and computes normalized covariance residual", () => {
    const a = frame(0);
    a.truth = [{ id: "hidden", x: 0, y: 0, vx: 2, vy: 0 }];
    a.tracks = [0, 1].map((index) => ({
      id: `track${index}`,
      x: 1 + index,
      y: 1,
      vx: 1,
      vy: 0,
      covariance: [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
      ],
      status: "confirmed",
      lastMeasurementTime: 0,
      age: 0,
      sources: ["radar"],
    }));
    const metrics = calculateMetrics([a]);
    expect(metrics.trackingMatches).toBe(1);
    expect(metrics.trackingPositionRMSE).toBeCloseTo(Math.sqrt(2));
    expect(metrics.trackingVelocityRMSE).toBe(1);
    expect(metrics.positionCovarianceConsistency).toBe(1);
  });
  it("compares surface tracks against polygon boundary without claiming calibrated covariance", () => {
    const a = frame(0);
    a.truth = [
      {
        id: "body",
        x: 2,
        y: 0,
        vx: 0,
        vy: 0,
        polygon: [
          { x: 0, y: -1 },
          { x: 4, y: -1 },
          { x: 4, y: 1 },
          { x: 0, y: 1 },
        ],
      },
    ];
    a.tracks = [
      {
        id: "surface",
        x: 0.1,
        y: 0,
        vx: 0,
        vy: 0,
        covariance: [
          [1, 0],
          [0, 1],
        ],
        status: "confirmed",
        lastMeasurementTime: 0,
        age: 0,
        sources: ["lidar"],
      },
    ];
    expect(calculateMetrics([a]).trackingPositionRMSE).toBeCloseTo(0.1);
    expect(calculateMetrics([a]).positionCovarianceConsistency).toBeNull();
  });
  it("does not interpret omitted tracking samples as track deletion", () => {
    const a = frame(0),
      b = frame(0.01),
      c = frame(0.1);
    a.tracks = [
      {
        id: "stable",
        x: 0,
        y: 0,
        vx: 0,
        vy: 0,
        covariance: [
          [1, 0],
          [0, 1],
        ],
        status: "confirmed",
        lastMeasurementTime: 0,
        age: 0,
        sources: ["lidar"],
      },
    ];
    c.tracks = structuredClone(a.tracks);
    expect(
      buildTimeline([a, b, c]).filter((event) => event.kind === "tracking"),
    ).toHaveLength(1);
    c.tracks = [];
    expect(
      buildTimeline([a, b, c]).filter((event) => event.kind === "tracking"),
    ).toHaveLength(2);
  });
  it("accepts an explicit blocked-corridor stop but fails a containment interlock", () => {
    const a = frame(0);
    a.manoeuvre = "STOP_FOR_OBSTACLE";
    const stopped = { ...config, corridorScenario: "blocked" as const };
    expect(createValidationResult(stopped, [a]).passed).toBe(true);
    a.physical.containmentStop = true;
    expect(createValidationResult(stopped, [a]).passed).toBe(false);
  });
  it("records actual hazard response and rejects unsafe or incomplete runs", () => {
    const a = frame(1),
      b = frame(1.2);
    a.hazardActive = true;
    b.hazardActive = true;
    b.behaviour.aeb = true;
    b.behaviour.state = "AEB";
    b.physical.collisionCount = 1;
    b.impactSpeed = 3;
    expect(calculateMetrics([a, b]).aebResponseTime).toBeCloseTo(0.2);
    const result = createValidationResult(config, [a, b]);
    expect(result.passed).toBe(false);
    expect(result.metrics.impactSpeed).toBe(3);
    expect(result.events.some((e) => e.kind === "collision")).toBe(true);
    expect(result.failures).toContain(
      "Route not completed within configured duration",
    );
  });
  it("bounds replay storage while retaining full-resolution event/metric evaluation", () => {
    const template = frame(0);
    const frames = Array.from({ length: MAX_RECORDED_FRAMES + 11 }, (_, i) => ({
      ...template,
      time: i / 100,
      control: { steering: 0, acceleration: i === 7 ? -5 : 0 },
    }));
    const result = createValidationResult(config, frames);
    expect(result.frames).toHaveLength(MAX_RECORDED_FRAMES);
    expect(result.frames[0].time).toBe(0);
    expect(result.frames.at(-1)?.time).toBe(frames.at(-1)?.time);
    expect(result.metrics.maximumDeceleration).toBe(0);
    frames[0].vehicle.x = 999;
    expect(result.frames[0].vehicle.x).not.toBe(999);
  });
  it("plays exact recorded snapshots with pause, nonuniform seek and finite guards", () => {
    const frames = [frame(0), frame(0.1), frame(0.7)];
    frames[1].vehicle.x = 123;
    const player = new ReplayPlayer(frames);
    expect(player.seek(0.4)?.vehicle.x).toBe(123);
    player.play();
    expect(player.advance(0.3)?.time).toBe(0.7);
    expect(player.playing).toBe(false);
    expect(player.seekIndex(0)?.time).toBe(0);
    player.play();
    player.pause();
    player.advance(1);
    expect(player.index).toBe(0);
    expect(() => player.seek(NaN)).toThrow();
    expect(() => new ReplayPlayer([frame(1), frame(0)])).toThrow();
  });
  it("exports actual metrics, quotes CSV strings and compares numeric A/B values", () => {
    const a = createValidationResult({ ...config, id: '=bad,"name"' }, [
      frame(0),
    ]);
    const b = structuredClone(a);
    b.metrics.maximumDeceleration = 2;
    expect(JSON.parse(exportJSON(a)).config.seed).toBe(7);
    expect(exportCSV(a)).toContain('"\'=bad,""name"""');
    expect(
      compareResults(a, b).find((row) => row.metric === "maximumDeceleration")
        ?.delta,
    ).toBe(2);
    expect(engineeringReport(a)).toContain("Unavailable");
    expect(engineeringReport(a)).toContain("nearest actor polygon boundary");
  });
});
