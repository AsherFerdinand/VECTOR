import { describe, expect, it } from "vitest";
import { raycast, sensorPose } from "../src/sensors/geometry";
import { generateLidar } from "../src/sensors/lidar";
import { generateCamera } from "../src/sensors/camera";
import { generateRadar } from "../src/sensors/radar";
import { DEFAULT_SENSOR_CONFIGS, SensorSuite } from "../src/sensors/suite";
import type { SensorConfig } from "../src/sensors/types";
import type { WorldObject } from "../src/world/types";

const vehicle = { x: 0, y: 0, heading: 0, velocity: 0 };
const config = (
  source: SensorConfig["source"],
  changes: Partial<SensorConfig> = {},
): SensorConfig => ({
  id: source,
  source,
  mount: { x: 0, y: 0 },
  yaw: 0,
  fov: Math.PI / 2,
  maxRange: 40,
  updateHz: 10,
  noise: 0,
  seed: 12,
  rays: 1,
  ...changes,
});
const box = (
  id: string,
  x: number,
  y: number,
  width = 2,
  depth = 2,
): WorldObject => ({
  id,
  objectClass: "building",
  polygon: [
    { x, y },
    { x: x + width, y },
    { x: x + width, y: y + depth },
    { x, y: y + depth },
  ],
  height: 5,
  velocity: { x: 0, y: 0 },
  collidable: true,
});
const obstacle = box("front", 10, -1);

describe("sensor geometry and mounting", () => {
  it("transforms a sensor mount from the rear axle into world coordinates", () => {
    const pose = sensorPose(
      { ...vehicle, x: 5, y: 2, heading: Math.PI / 2 },
      config("camera", { mount: { x: 2, y: 1 }, yaw: Math.PI / 4 }),
    );
    expect(pose.x).toBeCloseTo(4);
    expect(pose.y).toBeCloseTo(4);
    expect(pose.heading).toBeCloseTo((Math.PI * 3) / 4);
  });
  it("returns the nearest known intersection and maximum range on a miss", () => {
    const hit = raycast(vehicle, 0, [box("behind", 20, -1), obstacle], 40);
    expect(hit.distance).toBeCloseTo(10);
    expect(hit.object?.id).toBe("front");
    expect(hit.point).toEqual({ x: 10, y: 0 });
    const miss = raycast(vehicle, Math.PI, [obstacle], 40);
    expect(miss.object).toBeNull();
    expect(miss.distance).toBe(40);
    expect(raycast(vehicle, 0, [obstacle], 5).object).toBeNull();
  });
  it("handles an origin inside geometry and collinear polygon edges", () => {
    expect(raycast({ x: 11, y: 0 }, 0, [obstacle], 40).distance).toBe(0);
    expect(
      raycast(vehicle, 0, [box("collinear", 10, 0)], 40).distance,
    ).toBeCloseTo(10);
  });
});

describe("synthetic LiDAR", () => {
  it("produces a geometric range, hit point, local ray angle and timestamp", () => {
    const scan = generateLidar(vehicle, [obstacle], config("lidar"), 0.3);
    expect(scan.timestamp).toBe(0.3);
    expect(scan.rays).toHaveLength(1);
    expect(scan.rays[0]).toEqual({
      angle: 0,
      range: 10,
      hit: true,
      point: { x: 10, y: 0 },
    });
  });
  it("uses vehicle and sensor orientation and occludes farther obstacles", () => {
    const cfg = config("lidar", { yaw: Math.PI / 2 });
    expect(
      generateLidar(vehicle, [box("north", -1, 10)], cfg, 0).rays[0].range,
    ).toBeCloseTo(10);
    expect(
      generateLidar(vehicle, [obstacle, box("far", 20, -1)], config("lidar"), 0)
        .rays[0].range,
    ).toBe(10);
    expect(
      generateLidar(
        { ...vehicle, heading: Math.PI },
        [obstacle],
        config("lidar"),
        0,
      ).rays[0].hit,
    ).toBe(false);
  });
  it("samples full coverage without a duplicate end ray and bounds deterministic noise", () => {
    const full = generateLidar(
      vehicle,
      [],
      config("lidar", { fov: 2 * Math.PI, rays: 180 }),
      0,
    );
    expect(full.rays).toHaveLength(180);
    expect(full.rays[0].angle).toBe(-Math.PI);
    expect(full.rays.at(-1)!.angle).toBeLessThan(Math.PI);
    const cfg = config("lidar", { noise: 0.2 });
    const a = generateLidar(vehicle, [obstacle], cfg, 0.4),
      b = generateLidar(vehicle, [obstacle], cfg, 0.4);
    expect(a).toEqual(b);
    expect(Math.abs(a.rays[0].range - 10)).toBeLessThanOrEqual(0.2);
    expect(generateLidar(vehicle, [obstacle], cfg, 0.5).rays[0].range).not.toBe(
      a.rays[0].range,
    );
  });
});

describe("synthetic camera visibility", () => {
  it("classifies only visible in-range objects inside its field of view", () => {
    const objects = [
      obstacle,
      box("behind", -10, -1),
      box("outside-fov", 0, 10),
      box("too-far", 45, -1),
    ];
    const frame = generateCamera(vehicle, objects, config("camera"), 0.2);
    expect(frame.timestamp).toBe(0.2);
    expect(frame.detections).toHaveLength(1);
    expect(frame.detections[0]).toMatchObject({
      source: "camera",
      objectClass: "building",
      timestamp: 0.2,
      debugObjectId: "front",
      range: 10,
      relativePosition: { x: 10, y: 0 },
    });
    expect(frame.detections[0]).not.toHaveProperty("confidence");
  });
  it("does not report a fully occluded object but sees a partially exposed one", () => {
    const hidden = box("hidden", 20, -0.5, 2, 1);
    const exposed = box("exposed", 20, 2, 2, 3);
    const frame = generateCamera(
      vehicle,
      [box("blocking", 10, -2, 2, 4), hidden, exposed],
      config("camera"),
      0,
    );
    expect(frame.detections.map((d) => d.debugObjectId)).not.toContain(
      "hidden",
    );
    expect(frame.detections.map((d) => d.debugObjectId)).toContain("exposed");
  });
  it("detects a large wall crossing the FOV even when its vertices are outside", () => {
    expect(
      generateCamera(
        vehicle,
        [box("wide", 10, -100, 2, 200)],
        config("camera"),
        0,
      ).detections,
    ).toHaveLength(1);
    const turned = generateCamera(
      { ...vehicle, heading: Math.PI / 2 },
      [box("north", -1, 10)],
      config("camera"),
      0,
    );
    expect(turned.detections[0].bearing).toBeCloseTo(0);
  });
});

describe("simplified radar", () => {
  it("measures range, bearing and negative radial velocity toward a static object", () => {
    const result = generateRadar(
      { ...vehicle, velocity: 5 },
      [obstacle],
      config("radar"),
      0.5,
    ).detections[0];
    expect(result.range).toBeCloseTo(10);
    expect(result.bearing).toBeCloseTo(0);
    expect(result.radialVelocity).toBeCloseTo(-5);
    expect(result.timestamp).toBe(0.5);
    expect(result).not.toHaveProperty("objectClass");
  });
  it("uses actual object velocity and mount rotational velocity", () => {
    const moving = { ...obstacle, velocity: { x: 2, y: 0 } };
    expect(
      generateRadar({ ...vehicle, velocity: 5 }, [moving], config("radar"), 0)
        .detections[0].radialVelocity,
    ).toBeCloseTo(-3);
    const rotating = generateRadar(
      vehicle,
      [obstacle],
      config("radar", { mount: { x: 0, y: 1 } }),
      0,
      2,
    ).detections[0];
    expect(rotating.radialVelocity).toBeCloseTo(2); // omega cross mount = (-2,0).
  });
  it("respects sensor orientation, range, FOV and line-of-sight", () => {
    expect(
      generateRadar(vehicle, [obstacle], config("radar", { yaw: Math.PI }), 0)
        .detections,
    ).toHaveLength(0);
    expect(
      generateRadar(vehicle, [obstacle], config("radar", { maxRange: 5 }), 0)
        .detections,
    ).toHaveLength(0);
    const frame = generateRadar(
      vehicle,
      [obstacle, box("far", 20, -0.5, 2, 1)],
      config("radar"),
      0,
    );
    expect(frame.detections).toHaveLength(1);
  });
  it("replays configured range, bearing and velocity noise deterministically", () => {
    const cfg = config("radar", { noise: 0.1 });
    const a = generateRadar({ ...vehicle, velocity: 5 }, [obstacle], cfg, 0.2);
    expect(a).toEqual(
      generateRadar({ ...vehicle, velocity: 5 }, [obstacle], cfg, 0.2),
    );
    expect(Math.abs(a.detections[0].radialVelocity! + 5)).toBeLessThanOrEqual(
      0.1,
    );
  });
});

describe("simulation-time sensor scheduling", () => {
  it("samples at 10 Hz, does not update while simulation time is unchanged, and resets replay", () => {
    const suite = new SensorSuite([obstacle]);
    suite.update(0, vehicle);
    for (let tick = 1; tick <= 25; tick++) suite.update(tick * 0.01, vehicle);
    expect(suite.snapshot.counts).toEqual({ lidar: 3, camera: 3, radar: 3 });
    expect(suite.snapshot.lidar?.timestamp).toBeCloseTo(0.2);
    const paused = suite.snapshot;
    suite.update(0.25, { ...vehicle, x: 1 });
    expect(suite.snapshot).toEqual(paused);
    suite.reset();
    suite.update(0, vehicle);
    const initial = suite.snapshot;
    const replay = new SensorSuite([obstacle]);
    replay.update(0, vehicle);
    expect(initial).toEqual(replay.snapshot);
  });
  it("drops skipped periods instead of unbounded catch-up or fabricated old poses", () => {
    const suite = new SensorSuite([obstacle]);
    suite.update(0, vehicle);
    suite.update(1000, vehicle);
    expect(suite.snapshot.counts.lidar).toBe(2);
    expect(suite.snapshot.lidar?.timestamp).toBe(1000);
  });
  it("supports distinct update rates and protects internal output from caller mutation", () => {
    const configs = DEFAULT_SENSOR_CONFIGS.map((c) => ({
      ...c,
      updateHz: c.source === "radar" ? 5 : 10,
    }));
    const suite = new SensorSuite([obstacle], configs);
    suite.update(0, vehicle);
    suite.update(0.1, vehicle);
    expect(suite.snapshot.counts).toEqual({ lidar: 2, camera: 2, radar: 1 });
    const snapshot = suite.snapshot;
    expect(() => {
      snapshot.configs[0].mount.x = 999;
    }).toThrow(TypeError);
    expect(() => {
      snapshot.lidar!.rays[0].range = 999;
    }).toThrow(TypeError);
    expect(suite.snapshot).toBe(snapshot);
    expect(suite.snapshot.configs[0].mount.x).toBe(1.35);
    expect(suite.snapshot.lidar!.rays[0].range).not.toBe(999);
  });
  it("rejects invalid sensor configuration and backward simulation time", () => {
    expect(
      () => new SensorSuite([], [config("radar", { updateHz: 0 })]),
    ).toThrow(RangeError);
    const suite = new SensorSuite([]);
    suite.update(0.2, vehicle);
    expect(() => suite.update(0.1, vehicle)).toThrow(RangeError);
  });
});
