import { describe, expect, it } from "vitest";
import {
  predictCV,
  updateScalar,
  positionGate,
} from "../src/perception/kalman";
import { MultiObjectTracker } from "../src/perception/tracker";
import type {
  SensorSnapshot,
  DetectionFrame,
  SensorSource,
} from "../src/sensors/types";
const vehicle = { x: 0, y: 0, heading: 0, velocity: 0 };
function snapshot(
  t: number,
  x = 10,
  source: SensorSource = "camera",
  id = "hidden-a",
): SensorSnapshot {
  const frame: DetectionFrame = {
    sensorId: source,
    timestamp: t,
    pose: { x: 0, y: 0, heading: 0 },
    detections: [
      {
        observationId: "observation",
        source,
        timestamp: t,
        relativePosition: { x, y: 0 },
        range: x,
        bearing: 0,
        objectClass: source === "camera" ? "vehicle" : undefined,
        debugObjectId: id,
        ...(source === "radar" ? { radialVelocity: 2 } : {}),
      },
    ],
  };
  return {
    configs: [],
    lidar: null,
    camera: source === "camera" ? frame : null,
    radar: source === "radar" ? frame : null,
    counts: { lidar: 0, camera: 1, radar: source === "radar" ? 1 : 0 },
  };
}
const empty: SensorSnapshot = {
  configs: [],
  lidar: null,
  camera: null,
  radar: null,
  counts: { lidar: 0, camera: 0, radar: 0 },
};
describe("constant velocity estimation", () => {
  it("propagates mean and covariance with continuous acceleration noise", () => {
    const initial = {
      state: [1, 2, 3, -2],
      covariance: [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
      ],
    };
    const result = predictCV(initial, 2, 1);
    expect(result.state).toEqual([7, -2, 3, -2]);
    expect(result.covariance[0][0]).toBeCloseTo(1 + 4 + 8 / 3);
    const halves = predictCV(predictCV(initial, 1, 1), 1, 1);
    for (let i = 0; i < 4; i++)
      for (let j = 0; j < 4; j++)
        expect(halves.covariance[i][j]).toBeCloseTo(
          result.covariance[i][j],
          12,
        );
  });
  it("reduces measured uncertainty without inventing exact unknown velocity", () => {
    const e = {
      state: [0, 0, 0, 0],
      covariance: [
        [4, 0, 0, 0],
        [0, 4, 0, 0],
        [0, 0, 25, 0],
        [0, 0, 0, 25],
      ],
    };
    const r = updateScalar(e, [1, 0, 0, 0], 2, 1);
    expect(r.state[0]).toBeCloseTo(1.6);
    expect(r.covariance[0][0]).toBeCloseTo(0.8);
    expect(r.covariance[2][2]).toBe(25);
    expect(positionGate(e, 100, 0, 1)).toBeGreaterThan(9.21);
  });
  it("keeps covariance symmetric and positive under repeated updates", () => {
    let e = {
      state: [0, 0, 0, 0],
      covariance: [
        [4, 0, 0, 0],
        [0, 4, 0, 0],
        [0, 0, 25, 0],
        [0, 0, 0, 25],
      ],
    };
    for (let i = 0; i < 200; i++) {
      e = predictCV(e, 0.1);
      e = updateScalar(e, [1, 0, 0, 0], i * 0.2, 0.3);
      e = updateScalar(e, [0, 1, 0, 0], 0, 0.3);
    }
    for (let i = 0; i < 4; i++) {
      expect(e.covariance[i][i]).toBeGreaterThan(0);
      for (let j = 0; j < 4; j++)
        expect(e.covariance[i][j]).toBeCloseTo(e.covariance[j][i], 10);
    }
  });
});
describe("measurement association and lifecycle", () => {
  it("confirms on distinct acquisition times and estimates moving velocity", () => {
    const tracker = new MultiObjectTracker();
    let result = tracker.update(snapshot(0), vehicle, 0);
    expect(result.tracks[0].status).toBe("tentative");
    for (let i = 1; i <= 30; i++)
      result = tracker.update(
        snapshot(i * 0.1, 10 + i * 0.2),
        vehicle,
        i * 0.1,
      );
    expect(result.tracks).toHaveLength(1);
    expect(result.tracks[0].status).toBe("confirmed");
    expect(result.tracks[0].vx).toBeCloseTo(2, 1);
  });
  it("does not count repeated stale frames as confirmation", () => {
    const tracker = new MultiObjectTracker();
    const frame = snapshot(0);
    tracker.update(frame, vehicle, 0);
    const result = tracker.update(frame, vehicle, 0.2);
    expect(result.tracks[0].status).toBe("tentative");
    expect(result.tracks[0].lastMeasurementTime).toBe(0);
  });
  it("fuses asynchronous camera and radar without hidden ID association", () => {
    const tracker = new MultiObjectTracker();
    tracker.update(snapshot(0, 10, "camera", "hidden-a"), vehicle, 0);
    let result = tracker.update(
      snapshot(0.05, 10.1, "radar", "totally-different"),
      vehicle,
      0.05,
    );
    expect(result.tracks).toHaveLength(1);
    expect(result.tracks[0].sources).toEqual(["camera", "radar"]);
    expect(result.tracks[0].vx).toBeGreaterThan(1.5);
    result = tracker.update(
      snapshot(0.1, 10.2, "camera", "again"),
      vehicle,
      0.1,
    );
    expect(result.tracks[0].status).toBe("confirmed");
  });
  it("coasts, grows uncertainty, deletes and resets reproducibly", () => {
    const tracker = new MultiObjectTracker();
    const initial = tracker.update(snapshot(0), vehicle, 0);
    const coast = tracker.update(empty, vehicle, 0.5);
    expect(coast.tracks[0].status).toBe("coasting");
    expect(coast.tracks[0].covariance[0][0]).toBeGreaterThan(
      initial.tracks[0].covariance[0][0],
    );
    expect(tracker.update(empty, vehicle, 1.6).tracks).toHaveLength(0);
    tracker.reset();
    expect(tracker.update(snapshot(0), vehicle, 0)).toEqual(initial);
  });
  it("rejects distant statistical mismatches and preserves two object identities", () => {
    const tracker = new MultiObjectTracker();
    const first = snapshot(0);
    first.camera!.detections.push({
      ...first.camera!.detections[0],
      relativePosition: { x: 25, y: 0 },
      range: 25,
    });
    const result = tracker.update(first, vehicle, 0);
    expect(result.tracks).toHaveLength(2);
    const next = snapshot(0.1, 10.2);
    next.camera!.detections.push({
      ...next.camera!.detections[0],
      relativePosition: { x: 25.1, y: 0 },
      range: 25.1,
    });
    expect(tracker.update(next, vehicle, 0.1).tracks.map((t) => t.id)).toEqual([
      "track-1",
      "track-2",
    ]);
  });
  it("uses lidar clusters as position measurements with nonzero uncertainty", () => {
    const tracker = new MultiObjectTracker();
    const scan: SensorSnapshot = {
      ...empty,
      lidar: {
        sensorId: "lidar",
        timestamp: 0,
        pose: { x: 0, y: 0, heading: 0 },
        maxRange: 40,
        rays: [
          { angle: 0, range: 10, hit: true, point: { x: 10, y: 0 } },
          { angle: 0.02, range: 10.1, hit: true, point: { x: 10, y: 0.2 } },
        ],
      },
    };
    const result = tracker.update(scan, vehicle, 0);
    expect(result.tracks).toHaveLength(1);
    expect(result.tracks[0].sources).toEqual(["lidar"]);
    expect(result.tracks[0].covariance[0][0]).toBeGreaterThan(0);
  });
  it("compensates ego velocity in radial measurements without pretending tangential velocity is measured", () => {
    const tracker = new MultiObjectTracker();
    const scan = snapshot(0, 10, "radar");
    scan.radar!.detections[0].radialVelocity = -3;
    const track = tracker.update(scan, { ...vehicle, velocity: 5 }, 0)
      .tracks[0];
    expect(track.vx).toBeCloseTo(2, 1);
    expect(track.covariance[3][3]).toBe(25);
  });
});

describe("asynchronous sensor robustness", () => {
  it("records separate sensor contribution acquisition times", () => {
    const tracker = new MultiObjectTracker();
    tracker.update(snapshot(0), vehicle, 0);
    tracker.update(snapshot(0.05, 10.1, "radar"), vehicle, 0.05);
    const s = snapshot(0.2, 10.4);
    s.radar = snapshot(0.05, 10.1, "radar").radar;
    const t = tracker.update(s, vehicle, 0.2).tracks[0];
    expect(t.sourceTimes).toEqual({ camera: 0.2, radar: 0.05 });
    expect(t.lastMeasurementTime).toBe(0.2);
  });
  it("is independent of debug ground truth IDs", () => {
    const a = new MultiObjectTracker(),
      b = new MultiObjectTracker();
    for (let i = 0; i < 12; i++)
      expect(
        a.update(
          snapshot(i * 0.1, 10 + i * 0.1, "camera", `truth-${i}`),
          vehicle,
          i * 0.1,
        ),
      ).toEqual(
        b.update(
          snapshot(i * 0.1, 10 + i * 0.1, "camera", "unrelated"),
          vehicle,
          i * 0.1,
        ),
      );
  });
  it("rejects invalid configuration/time and ignores malformed measurements", () => {
    expect(() => new MultiObjectTracker({ deleteAfter: -1 })).toThrow(
      RangeError,
    );
    expect(() => new MultiObjectTracker({ gate: NaN })).toThrow(RangeError);
    const tracker = new MultiObjectTracker();
    expect(() => tracker.update(empty, vehicle, NaN)).toThrow(RangeError);
    expect(tracker.update(snapshot(0, NaN), vehicle, 0).tracks).toEqual([]);
  });
  it("reacquires an expired target as a new identity rather than reviving it", () => {
    const tracker = new MultiObjectTracker();
    tracker.update(snapshot(0), vehicle, 0);
    expect(tracker.update(snapshot(3), vehicle, 3).tracks[0].id).toBe(
      "track-2",
    );
  });
  it("bounds estimates under deterministic small measurement noise", () => {
    const tracker = new MultiObjectTracker();
    let output = tracker.update(snapshot(0), vehicle, 0);
    for (let i = 1; i <= 60; i++)
      output = tracker.update(
        snapshot(i * 0.1, 10 + i * 0.2 + 0.25 * Math.sin(i * 2.3)),
        vehicle,
        i * 0.1,
      );
    expect(output.tracks).toHaveLength(1);
    expect(output.tracks[0].vx).toBeGreaterThan(1);
    expect(output.tracks[0].vx).toBeLessThan(3);
  });
});
