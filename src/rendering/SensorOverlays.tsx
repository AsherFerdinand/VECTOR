import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import {
  BufferAttribute,
  BufferGeometry,
  Group,
  LineBasicMaterial,
  LineSegments,
  Points,
  PointsMaterial,
} from "three";
import { vehicleFootprint } from "../simulation/collision";
import type { Simulation } from "../simulation/clock";
import type { Point } from "../navigation/types";
import type {
  DetectionFrame,
  LidarScan,
  SensorPose,
  SensorSource,
} from "../sensors/types";
import { town } from "../world/town";

const MAX_POINTS = 1024;
function makeLines(color: string, capacity: number, opacity = 1) {
  const geometry = new BufferGeometry();
  geometry.setAttribute(
    "position",
    new BufferAttribute(new Float32Array(capacity * 3), 3),
  );
  geometry.setDrawRange(0, 0);
  const line = new LineSegments(
    geometry,
    new LineBasicMaterial({ color, transparent: opacity < 1, opacity }),
  );
  line.frustumCulled = false;
  return line;
}
function writeLines(line: LineSegments, segments: Point[][], height: number) {
  const attribute = line.geometry.getAttribute("position") as BufferAttribute;
  let count = 0;
  for (const segment of segments) {
    if (count + 2 > attribute.count) break;
    attribute.setXYZ(count++, segment[0].x, height, -segment[0].y);
    attribute.setXYZ(count++, segment[1].x, height, -segment[1].y);
  }
  attribute.needsUpdate = true;
  line.geometry.setDrawRange(0, count);
}
const outline = (polygon: Point[]) =>
  polygon.map((point, i) => [point, polygon[(i + 1) % polygon.length]]);
const toWorld = (point: Point, pose: SensorPose): Point => ({
  x:
    pose.x +
    point.x * Math.cos(pose.heading) -
    point.y * Math.sin(pose.heading),
  y:
    pose.y +
    point.x * Math.sin(pose.heading) +
    point.y * Math.cos(pose.heading),
});

/** Exact boundary of the union of the axis-aligned road rectangles in town.json. */
function roadBoundaries(): Point[][] {
  const rectangles = town.roads.map((road) => {
    const horizontal = road.from[1] === road.to[1];
    return {
      x0:
        Math.min(road.from[0], road.to[0]) -
        (horizontal ? 0 : town.roadWidth / 2),
      x1:
        Math.max(road.from[0], road.to[0]) +
        (horizontal ? 0 : town.roadWidth / 2),
      y0:
        Math.min(road.from[1], road.to[1]) -
        (horizontal ? town.roadWidth / 2 : 0),
      y1:
        Math.max(road.from[1], road.to[1]) +
        (horizontal ? town.roadWidth / 2 : 0),
    };
  });
  const xs = [...new Set(rectangles.flatMap((r) => [r.x0, r.x1]))].sort(
    (a, b) => a - b,
  );
  const ys = [...new Set(rectangles.flatMap((r) => [r.y0, r.y1]))].sort(
    (a, b) => a - b,
  );
  const inside = (i: number, j: number) =>
    i >= 0 &&
    j >= 0 &&
    i < xs.length - 1 &&
    j < ys.length - 1 &&
    rectangles.some(
      (r) =>
        (xs[i] + xs[i + 1]) / 2 > r.x0 &&
        (xs[i] + xs[i + 1]) / 2 < r.x1 &&
        (ys[j] + ys[j + 1]) / 2 > r.y0 &&
        (ys[j] + ys[j + 1]) / 2 < r.y1,
    );
  const segments: Point[][] = [];
  for (let i = 0; i < xs.length - 1; i++)
    for (let j = 0; j < ys.length - 1; j++)
      if (inside(i, j)) {
        if (!inside(i - 1, j))
          segments.push([
            { x: xs[i], y: ys[j] },
            { x: xs[i], y: ys[j + 1] },
          ]);
        if (!inside(i + 1, j))
          segments.push([
            { x: xs[i + 1], y: ys[j] },
            { x: xs[i + 1], y: ys[j + 1] },
          ]);
        if (!inside(i, j - 1))
          segments.push([
            { x: xs[i], y: ys[j] },
            { x: xs[i + 1], y: ys[j] },
          ]);
        if (!inside(i, j + 1))
          segments.push([
            { x: xs[i], y: ys[j + 1] },
            { x: xs[i + 1], y: ys[j + 1] },
          ]);
      }
  return segments;
}

/** Batched measurement geometry is kept distinct from labelled collision ground truth. */
export function SensorOverlays({
  simulation,
  source,
  enabled,
  collisionEnabled,
}: {
  simulation: Simulation;
  source: SensorSource | "compute";
  enabled: boolean;
  collisionEnabled: boolean;
}) {
  const latest = useRef<{
    source: string;
    frame: DetectionFrame | LidarScan | null;
  }>({ source: "", frame: null });
  const geometry = useMemo(() => {
    const group = new Group();
    group.userData.sensorDebug = true;
    const pointsGeometry = new BufferGeometry();
    pointsGeometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array(MAX_POINTS * 3), 3),
    );
    pointsGeometry.setDrawRange(0, 0);
    const points = new Points(
      pointsGeometry,
      new PointsMaterial({
        color: "#74c7f6",
        size: 0.32,
        sizeAttenuation: true,
      }),
    );
    points.frustumCulled = false;
    const rays = makeLines("#4d9fd0", MAX_POINTS * 2, 0.3),
      fov = makeLines("#74c7f6", 256, 0.65);
    const footprint = makeLines("#a5e8cd", 8),
      objects = makeLines("#dc8c7b", 4096, 0.7),
      road = makeLines("#9fb1be", 1024, 0.8);
    writeLines(road, roadBoundaries(), 0.25);
    group.add(points, rays, fov, footprint, objects, road);
    return { group, points, rays, fov, footprint, objects, road };
  }, []);
  useEffect(
    () => () => {
      for (const object of [
        geometry.points,
        geometry.rays,
        geometry.fov,
        geometry.footprint,
        geometry.objects,
        geometry.road,
      ]) {
        object.geometry.dispose();
        object.material.dispose();
      }
    },
    [geometry],
  );
  useFrame(() => {
    geometry.group.visible = enabled || collisionEnabled;
    geometry.points.visible =
      geometry.rays.visible =
      geometry.fov.visible =
        enabled && source !== "compute";
    geometry.footprint.visible =
      geometry.objects.visible =
      geometry.road.visible =
        collisionEnabled;
    if (collisionEnabled) {
      writeLines(
        geometry.footprint,
        outline(vehicleFootprint(simulation.snapshot.vehicle)),
        0.28,
      );
      const hazardous =
        simulation.physical.roadStatus !== "ON_ROAD" ||
        simulation.safety.braking;
      geometry.footprint.material.color.set(hazardous ? "#ef7069" : "#a5e8cd");
      writeLines(
        geometry.objects,
        simulation.objects.flatMap((object) =>
          object.collidable ? outline(object.polygon) : [],
        ),
        0.28,
      );
    }
    if (!enabled || source === "compute") return;
    const sensors = simulation.sensors;
    const config = sensors.configs.find((c) => c.source === source);
    const frame = sensors[source];
    if (!config || !frame) return;
    if (latest.current.source === source && latest.current.frame === frame)
      return;
    latest.current = { source, frame };
    const pose = frame.pose;
    const positions = geometry.points.geometry.getAttribute(
      "position",
    ) as BufferAttribute;
    let count = 0;
    const raySegments: Point[][] = [];
    if (source === "lidar") {
      const scan = sensors.lidar!;
      for (let i = 0; i < scan.rays.length; i++) {
        const ray = scan.rays[i];
        if (ray.hit && count < MAX_POINTS)
          positions.setXYZ(count++, ray.point.x, 0.4, -ray.point.y);
        if (i % 12 === 0) raySegments.push([pose, ray.point]);
      }
    } else {
      const detections =
        source === "camera"
          ? sensors.camera!.detections
          : sensors.radar!.detections;
      for (const detection of detections) {
        const point = toWorld(detection.relativePosition, pose);
        if (count < MAX_POINTS)
          positions.setXYZ(count++, point.x, 0.6, -point.y);
        raySegments.push([pose, point]);
      }
    }
    positions.needsUpdate = true;
    geometry.points.geometry.setDrawRange(0, count);
    geometry.points.material.color.set(
      simulation.safety.braking && simulation.safety.source === source
        ? "#ef7069"
        : "#74c7f6",
    );
    writeLines(geometry.rays, raySegments, 0.35);
    const arc: Point[] = [];
    for (let i = 0; i <= 64; i++) {
      const angle = pose.heading - config.fov / 2 + (config.fov * i) / 64;
      arc.push({
        x: pose.x + config.maxRange * Math.cos(angle),
        y: pose.y + config.maxRange * Math.sin(angle),
      });
    }
    const segments = arc.slice(1).map((point, i) => [arc[i], point]);
    if (config.fov < Math.PI * 2 - 1e-6)
      segments.push([pose, arc[0]], [pose, arc[arc.length - 1]]);
    // A small cross marks the sensor origin even when there are no detections.
    segments.push(
      [
        { x: pose.x - 0.3, y: pose.y },
        { x: pose.x + 0.3, y: pose.y },
      ],
      [
        { x: pose.x, y: pose.y - 0.3 },
        { x: pose.x, y: pose.y + 0.3 },
      ],
    );
    writeLines(geometry.fov, segments, 0.32);
  });
  return <primitive object={geometry.group} />;
}
