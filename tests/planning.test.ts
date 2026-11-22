import { describe, it, expect } from "vitest";
import {
  CORRIDOR,
  corridorContains,
  corridorRoute,
  laneY,
} from "../src/planning/corridor";
import {
  laneChangeCandidate,
  validateCandidate,
} from "../src/planning/trajectory";
import { ManoeuvrePlanner } from "../src/planning/manoeuvrePlanner";
import { CorridorSimulation } from "../src/planning/corridorSimulation";
import type { Track } from "../src/perception/types";
const vehicle = { x: -45, y: laneY("east-outer"), heading: 0, velocity: 4 };
const track = (x: number, y: number, vx = 0): Track => ({
  id: "inferred-1",
  x,
  y,
  vx,
  vy: 0,
  covariance: [
    [0.01, 0, 0, 0],
    [0, 0.01, 0, 0],
    [0, 0, 0.01, 0],
    [0, 0, 0, 0.01],
  ],
  status: "confirmed",
  lastMeasurementTime: 0,
  age: 1,
  sources: ["radar"],
});
describe("explicit mapped manoeuvre corridor", () => {
  it("defines four explicit lanes and retains direction-specific body containment", () => {
    expect(CORRIDOR.lanes).toHaveLength(4);
    expect(corridorContains(vehicle)).toBe(true);
    expect(corridorContains({ ...vehicle, y: -85 })).toBe(false);
    expect(corridorContains({ ...vehicle, y: -83.25 }, 0, false)).toBe(true);
    const route = corridorRoute(vehicle);
    expect(route.points.at(-1)?.x).toBe(150);
    expect(() => corridorRoute(vehicle, -50)).toThrow();
  });
  it("quintic transition respects body, steering, acceleration and destination constraints", () => {
    const candidate = validateCandidate(
      laneChangeCandidate(vehicle, "east-inner", 4, 6),
      [],
      0,
    );
    expect(candidate.valid).toBe(true);
    expect(candidate.points.at(-1)?.y).toBe(laneY("east-inner"));
    expect(candidate.points[0].heading).toBe(0);
    expect(candidate.points.at(-1)?.heading).toBeCloseTo(0, 12);
    expect(candidate.maxLateralAcceleration).toBeLessThan(1.5);
    expect(candidate.points.every((p) => corridorContains(p, 0.1))).toBe(true);
  });
  it("rejects excessive speed, aggressive curvature and destination overshoot", () => {
    expect(
      validateCandidate(laneChangeCandidate(vehicle, "east-inner", 9, 2), [], 0)
        .valid,
    ).toBe(false);
    expect(
      validateCandidate(
        laneChangeCandidate(vehicle, "east-inner", 4, 6),
        [],
        0,
        [],
        -30,
      ).reasons,
    ).toContain("Manoeuvre exceeds destination");
  });
  it("rejects swept static obstacles and future moving uncertainty envelopes", () => {
    const candidate = laneChangeCandidate(vehicle, "east-inner", 4, 6);
    const p = candidate.points[120];
    const obstacle = {
      id: "box",
      objectClass: "static-obstacle" as const,
      height: 1,
      collidable: true,
      velocity: { x: 0, y: 0 },
      polygon: [
        { x: p.x - 1, y: p.y - 1 },
        { x: p.x + 1, y: p.y - 1 },
        { x: p.x + 1, y: p.y + 1 },
        { x: p.x - 1, y: p.y + 1 },
      ],
    };
    expect(validateCandidate(candidate, [], 0, [obstacle]).valid).toBe(false);
    const crossing = track(-20, -86.75, -4);
    expect(
      validateCandidate(
        laneChangeCandidate(vehicle, "east-inner", 4, 6),
        [crossing],
        0,
      ).valid,
    ).toBe(false);
  });
  it("requires a stable feasible gap before committing, and blocks an occupied adjacent lane", () => {
    const planner = new ManoeuvrePlanner(),
      lead = track(-15, laneY("east-outer"), 1);
    expect(planner.update(vehicle, [lead], 0).state).toBe(
      "PREPARE_LANE_CHANGE",
    );
    expect(
      planner.update(vehicle, [{ ...lead, lastMeasurementTime: 0.6 }], 0.6)
        .state,
    ).toBe("EXECUTE_LANE_CHANGE");
    const blocked = new ManoeuvrePlanner();
    const result = blocked.update(
      vehicle,
      [lead, track(-43, laneY("east-inner"), 4)],
      0,
    );
    expect(result.state).toBe("STOP_FOR_OBSTACLE");
    expect(result.candidates.every((c) => !c.valid)).toBe(true);
  });
  it("aborts for stale corridor estimates and refuses coretown fake lane changes", () => {
    const planner = new ManoeuvrePlanner();
    expect(
      planner.update(
        vehicle,
        [{ ...track(-35, vehicle.y), lastMeasurementTime: -1 }],
        0,
      ).state,
    ).toBe("SAFE_STOP");
    expect(planner.update({ ...vehicle, y: -29.25 }, [], 1).state).toBe(
      "SAFE_STOP",
    );
  });
  it("completes sensors→fusion→planning→control→dynamics overtaking and return", () => {
    const sim = new CorridorSimulation({ scenario: "overtaking", seed: 12 });
    const result = sim.run(60);
    expect(result.completed).toBe(true);
    expect(result.laneChanges).toBe(2);
    expect(result.collisions).toBe(0);
    expect(result.roadDepartures).toBe(0);
    expect(result.minimumGap).toBeGreaterThan(1);
    expect(result.maximumLateralAcceleration).toBeLessThanOrEqual(1.501);
    expect(result.maximumDeceleration).toBeLessThanOrEqual(6 + 1e-9);
    expect(
      sim.recording.some(
        (f) =>
          f.manoeuvre.state === "EXECUTE_LANE_CHANGE" &&
          f.tracking.tracks.some((t) => t.status === "confirmed"),
      ),
    ).toBe(true);
    expect(
      sim.recording.some((f) => f.manoeuvre.state === "RETURN_TO_LANE"),
    ).toBe(true);
    expect(sim.recording.length).toBeLessThanOrEqual(1200);
  });
  it("performs a single lane change without returning and safely stops when both lanes are blocked", () => {
    const single = new CorridorSimulation({ scenario: "lane-change" }),
      blocked = new CorridorSimulation({ scenario: "blocked" });
    const changed = single.run(60),
      stopped = blocked.run(20);
    expect(changed.completed).toBe(true);
    expect(changed.laneChanges).toBe(1);
    expect(single.snapshot.vehicle.y).toBeCloseTo(laneY("east-inner"), 1);
    expect(stopped.completed).toBe(false);
    expect(stopped.collisions).toBe(0);
    expect(stopped.roadDepartures).toBe(0);
    expect(blocked.snapshot.vehicle.velocity).toBe(0);
    expect(blocked.snapshot.manoeuvre.state).toBe("STOP_FOR_OBSTACLE");
  });
  it("handles a genuine same-direction cut-in through measured tracking and bounded control", () => {
    const sim = new CorridorSimulation({ scenario: "cut-in", seed: 42 });
    const result = sim.run(60);
    expect(result.completed).toBe(true);
    expect(result.collisions).toBe(0);
    expect(result.roadDepartures).toBe(0);
    expect(result.minimumGap).toBeGreaterThan(1);
    expect(result.maximumLateralAcceleration).toBeLessThanOrEqual(1.501);
    const crossing = sim.recording.filter((f) => f.time > 2 && f.time < 8);
    expect(
      crossing.some(
        (f) =>
          f.actor.y > laneY("east-outer") + 0.5 &&
          f.actor.y < laneY("east-inner") - 0.5,
      ),
    ).toBe(true);
    expect(crossing.some((f) => f.actor.heading < -0.1)).toBe(true);
    expect(
      sim.recording.some((f) =>
        f.tracking.tracks.some((t) => t.status === "confirmed"),
      ),
    ).toBe(true);
  });
  it("reproduces recorded states from identical seed and fixed-step inputs", () => {
    const a = new CorridorSimulation({ seed: 77, noise: 0.01 }),
      b = new CorridorSimulation({ seed: 77, noise: 0.01 });
    for (let i = 0; i < 250; i++) {
      a.step();
      b.step();
    }
    expect(a.snapshot).toEqual(b.snapshot);
    expect(a.recording).toEqual(b.recording);
    expect(() => a.step(0.02)).toThrow();
  });
});
