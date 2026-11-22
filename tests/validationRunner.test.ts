import { describe, expect, it } from "vitest";
import { runScenario, runBatch } from "../src/validation/runner";
import { VALIDATION_SCENARIOS } from "../src/validation/scenarios";

const normal = {
  id: "repeatable",
  scenario: "normal" as const,
  seed: 73,
  duration: 1,
};
describe("headless validation runner", () => {
  it("runs fixed steps without rendering and reproduces recorded states", () => {
    const a = runScenario(normal);
    const b = runScenario(normal);
    expect(a.frames).toEqual(b.frames);
    expect(a.metrics).toEqual(b.metrics);
    expect(a.frames.at(-1)?.time).toBe(1);
    expect(a.frames.at(-1)?.vehicle.x).toBeGreaterThan(a.frames[0].vehicle.x);
  });
  it("uses isolated simulations for each batch seed/configuration", () => {
    const results = runBatch([
      normal,
      { ...normal, id: "alternate", timeGap: 2 },
    ]);
    expect(results).toHaveLength(2);
    expect(results[0].config.id).toBe("repeatable");
    expect(results[1].config.timeGap).toBe(2);
    expect(results[0].frames[0].vehicle).toEqual(results[1].frames[0].vehicle);
  });
  it("records a real sensor dropout degraded-mode stop and reacquisition", () => {
    const result = runScenario({
      ...normal,
      scenario: "following",
      duration: 3,
      dropout: { start: 0.5, end: 1.5 },
    });
    expect(
      result.frames.some((frame) => frame.behaviour.state === "STALE_STOP"),
    ).toBe(true);
    expect(result.metrics.collisionCount).toBe(0);
    expect(
      result.frames
        .at(-1)
        ?.tracks?.some((track) => track.lastMeasurementTime >= 1.5),
    ).toBe(true);
  });
  it("runs fused perception, manoeuvre planning and physical lane changes through completion", () => {
    const result = runScenario({
      ...normal,
      corridorScenario: "overtaking",
      duration: 50,
    });
    expect(result.passed).toBe(true);
    expect(result.metrics.routeCompleted).toBe(true);
    expect(result.metrics.manoeuvreOutcome).toBe("COMPLETED");
    expect(result.metrics.collisionCount).toBe(0);
    expect(result.metrics.roadDepartures).toBe(0);
    expect(
      result.frames.some((frame) => frame.manoeuvre === "EXECUTE_LANE_CHANGE"),
    ).toBe(true);
    expect(
      result.frames.some((frame) => frame.manoeuvre === "RETURN_TO_LANE"),
    ).toBe(true);
    expect(result.metrics.maximumLateralAcceleration).toBeLessThanOrEqual(
      1.500001,
    );
    expect(result.frames.at(-1)?.vehicle.y).toBeCloseTo(-90.25, 1);
  });
  it("validates the bounded cut-in actor through fused avoidance and route completion", () => {
    const result = runScenario({
      ...normal,
      seed: 42,
      corridorScenario: "cut-in",
      duration: 60,
    });
    expect(result.passed).toBe(true);
    expect(result.metrics.collisionCount).toBe(0);
    expect(result.metrics.roadDepartures).toBe(0);
    expect(result.metrics.routeCompleted).toBe(true);
    expect(result.metrics.minimumClearance).toBeGreaterThan(1);
    const moving = result.frames.find(
      (frame) =>
        frame.time > 3 && frame.time < 6 && (frame.truth?.[0].vy ?? 0) < -0.1,
    );
    expect(moving).toBeDefined();
  });
  it("rejects unbounded, malformed and unsupported initial conditions", () => {
    expect(() => runScenario({ ...normal, duration: 121 })).toThrow(RangeError);
    expect(() => runBatch([])).toThrow(RangeError);
    expect(() => runBatch(Array.from({ length: 13 }, () => normal))).toThrow(
      RangeError,
    );
    expect(() =>
      runScenario({ ...normal, dropout: { start: 0.8, end: 0.7 } }),
    ).toThrow(RangeError);
    expect(() =>
      runScenario({
        ...normal,
        initial: { x: -20, y: -29.25, heading: 0, velocity: 1 },
      }),
    ).toThrow(/stationary/);
  });
  it("has actual supported scenario presets and reports static route rejection", () => {
    const blocked = VALIDATION_SCENARIOS.find(
      (preset) => preset.id === "static-block",
    )!;
    const result = runScenario({ ...blocked.config, duration: 0.1 });
    expect(result.frames.at(-1)?.navigation.state).toBe("ROUTE_UNAVAILABLE");
    expect(result.metrics.collisionCount).toBe(0);
    expect(result.frames.at(-1)?.vehicle.velocity).toBe(0);
  });
  it("records covariance and actual geometric clearance while keeping truth in validation only", () => {
    const result = runScenario({
      ...normal,
      scenario: "following",
      duration: 0.5,
    });
    const observed = result.frames.find((frame) => frame.tracks?.length);
    expect(observed?.tracks?.[0].covariance).toHaveLength(4);
    expect(observed?.truth?.[0].id).toBeDefined();
    expect(result.metrics.minimumClearance).toBeGreaterThan(0);
  });
});
