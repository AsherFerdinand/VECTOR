import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import {
  BufferAttribute,
  BufferGeometry,
  Group,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshBasicMaterial,
  SphereGeometry,
} from "three";
import type { Simulation } from "../simulation/clock";
import type { Route } from "../navigation/types";

const MAX_TRAJECTORY_POINTS = 600;

/** Flat ribbon avoids implementation-dependent WebGL line width for the planned path. */
function routeRibbon(route: Route | null): BufferGeometry {
  const geometry = new BufferGeometry();
  if (!route || route.points.length < 2) return geometry;
  const vertices = new Float32Array((route.points.length - 1) * 18);
  let index = 0;
  const halfWidth = 0.17;
  for (let i = 1; i < route.points.length; i++) {
    const a = route.points[i - 1],
      b = route.points[i];
    const dx = b.x - a.x,
      dy = b.y - a.y;
    const length = Math.hypot(dx, dy);
    if (length < 1e-9) continue;
    const nx = (-dy / length) * halfWidth,
      ny = (dx / length) * halfWidth;
    // Winding faces upward in the world(x,height,-y) coordinate convention.
    for (const [x, y] of [
      [a.x + nx, a.y + ny],
      [a.x - nx, a.y - ny],
      [b.x + nx, b.y + ny],
      [b.x + nx, b.y + ny],
      [a.x - nx, a.y - ny],
      [b.x - nx, b.y - ny],
    ]) {
      vertices[index++] = x;
      vertices[index++] = 0.2;
      vertices[index++] = -y;
    }
  }
  geometry.setAttribute("position", new BufferAttribute(vertices, 3));
  geometry.setDrawRange(0, index / 3);
  geometry.computeBoundingSphere();
  return geometry;
}

/** One planned ribbon, one bounded actual-path buffer, and two controller indicators. */
export function RouteOverlay({
  simulation,
  engineering = true,
}: {
  simulation: Simulation;
  engineering?: boolean;
}) {
  const previousRoute = useRef<Route | null | undefined>(undefined);
  const overlay = useMemo(() => {
    const group = new Group();
    const planned = new Mesh(
      new BufferGeometry(),
      new MeshBasicMaterial({ color: "#58dfcc" }),
    );
    const actualGeometry = new BufferGeometry();
    actualGeometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array(MAX_TRAJECTORY_POINTS * 3), 3),
    );
    actualGeometry.setDrawRange(0, 0);
    const actual = new Line(
      actualGeometry,
      new LineBasicMaterial({ color: "#efb18e" }),
    );
    actual.frustumCulled = false;
    const headingGeometry = new BufferGeometry();
    headingGeometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array(6), 3),
    );
    const heading = new Line(
      headingGeometry,
      new LineBasicMaterial({ color: "#d6e9af" }),
    );
    heading.frustumCulled = false;
    const target = new Mesh(
      new SphereGeometry(0.35, 8, 6),
      new MeshBasicMaterial({ color: "#d6e9af" }),
    );
    group.add(planned, actual, heading, target);
    return { group, planned, actual, heading, target };
  }, []);

  useEffect(
    () => () => {
      for (const object of [
        overlay.planned,
        overlay.actual,
        overlay.heading,
        overlay.target,
      ]) {
        object.geometry.dispose();
        object.material.dispose();
      }
    },
    [overlay],
  );

  useFrame(() => {
    const navigation = simulation.navigation;
    const snapshot = simulation.snapshot;
    if (previousRoute.current !== navigation.route) {
      overlay.planned.geometry.dispose();
      overlay.planned.geometry = routeRibbon(navigation.route);
      previousRoute.current = navigation.route;
    }
    const points = navigation.trajectory;
    const count = Math.min(points.length, MAX_TRAJECTORY_POINTS);
    const positions = overlay.actual.geometry.getAttribute(
      "position",
    ) as BufferAttribute;
    for (let i = 0; i < count; i++) {
      const point = points[points.length - count + i];
      positions.setXYZ(i, point.x, 0.24, -point.y);
    }
    positions.needsUpdate = true;
    overlay.actual.geometry.setDrawRange(0, count);
    overlay.actual.visible = engineering && count > 1;
    const lookAhead = navigation.lookAheadPoint;
    overlay.target.visible = engineering && lookAhead !== null;
    if (lookAhead) overlay.target.position.set(lookAhead.x, 0.48, -lookAhead.y);
    overlay.heading.visible = engineering && navigation.route !== null;
    const vehicle = snapshot.vehicle;
    const angle = vehicle.heading + snapshot.control.steering;
    const heading = overlay.heading.geometry.getAttribute(
      "position",
    ) as BufferAttribute;
    heading.setXYZ(0, vehicle.x, 0.4, -vehicle.y);
    heading.setXYZ(
      1,
      vehicle.x + 4 * Math.cos(angle),
      0.4,
      -vehicle.y - 4 * Math.sin(angle),
    );
    heading.needsUpdate = true;
  });
  return <primitive object={overlay.group} />;
}
