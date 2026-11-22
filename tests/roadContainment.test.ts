import { describe, expect, it } from "vitest";
import { NavigationController } from "../src/autonomy/navigationController";
import { createRoadGraph, projectToRoad } from "../src/navigation/graph";
import { planRoute } from "../src/navigation/route";
import { classifyRoad } from "../src/simulation/collision";
import { integrateVehicle, VEHICLE } from "../src/simulation/vehicle";
import type { Point } from "../src/navigation/types";

// Acceptance fixed before tuning: complete full-body road containment at each
// physics endpoint AND 0.002 s interpolated sweep samples; no position correction.
const ACCEPTANCE = {
  maxCTE: 1,
  rmsCTE: 0.5,
  endpoint: 0.8,
  seconds: 180,
  lateral: 1.500001,
};
const graph = createRoadGraph();
function rotate(p: Point, angle: number): Point {
  return {
    x: p.x * Math.cos(angle) - p.y * Math.sin(angle),
    y: p.x * Math.sin(angle) + p.y * Math.cos(angle),
  };
}
function evaluate(start: Point, end: Point, stopBeforeTurn = false) {
  const a = projectToRoad(graph, start),
    b = projectToRoad(graph, end);
  if (!a || !b) throw Error("Invalid fixture");
  const route = planRoute(graph, a, b);
  if (!route) throw Error("Unreachable fixture");
  const controller = new NavigationController(route);
  let vehicle = { x: a.x, y: a.y, heading: a.heading, velocity: 0 },
    steering = 0,
    ticks = 0;
  let departures = 0,
    maxCTE = 0,
    errorSquares = 0,
    maxLateral = 0,
    firstDeparture: unknown = null;
  const firstCurve = route.points.find((p) => Math.abs(p.curvature) > 0.05);
  let approachStop: "waiting" | "braking" | "complete" = "waiting";
  for (; ticks < ACCEPTANCE.seconds / 0.01; ticks++) {
    const previous = vehicle;
    const command = controller.update(vehicle, steering, 0.01);
    steering = command.steering;
    if (
      stopBeforeTurn &&
      firstCurve &&
      approachStop === "waiting" &&
      controller.telemetry.progress * route.length >= firstCurve.s - 6
    )
      approachStop = "braking";
    if (approachStop === "braking") {
      command.acceleration = -Math.min(
        VEHICLE.maxBraking,
        Math.max(0, vehicle.velocity) / 0.01,
      );
      if (vehicle.velocity <= 1e-12) approachStop = "complete";
    }
    vehicle = integrateVehicle(previous, command, 0.01);
    maxLateral = Math.max(
      maxLateral,
      (Math.max(previous.velocity ** 2, vehicle.velocity ** 2) *
        Math.abs(Math.tan(steering))) /
        VEHICLE.wheelbase,
    );
    const cte = Math.abs(controller.telemetry.crossTrackError ?? 0);
    maxCTE = Math.max(maxCTE, cte);
    errorSquares += cte ** 2;
    for (let sample = 1; sample <= 5; sample++) {
      const t = sample / 5;
      const pose = {
        ...vehicle,
        x: previous.x + (vehicle.x - previous.x) * t,
        y: previous.y + (vehicle.y - previous.y) * t,
        heading: previous.heading + (vehicle.heading - previous.heading) * t,
      };
      if (classifyRoad(pose) !== "ON_ROAD") {
        departures++;
        if (!firstDeparture)
          firstDeparture = {
            time: ticks * 0.01 + t * 0.01,
            pose,
            steering,
            cte,
            status: classifyRoad(pose),
          };
      }
    }
    if (
      controller.telemetry.state === "SAFE_STOP" ||
      (controller.telemetry.state === "DESTINATION_REACHED" &&
        Math.abs(vehicle.velocity) < 1e-12)
    )
      break;
  }
  return {
    departures,
    maxCTE,
    rmsCTE: Math.sqrt(errorSquares / (ticks + 1)),
    maxLateral,
    endpointError: Math.hypot(vehicle.x - b.x, vehicle.y - b.y),
    seconds: (ticks + 1) * 0.01,
    state: controller.telemetry.state,
    firstDeparture,
  };
}
const fixtures = [
  ["straight", { x: 10, y: -29.25 }],
  ["right", { x: 24.75, y: -45 }],
  ["left", { x: 29.25, y: 0 }],
  ["multiple turns", { x: -40, y: -24.75 }],
] as const;
describe("actual autonomous body and swept road containment", () => {
  for (const orientation of [0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2]) {
    it.each(fixtures)(
      `%s orientation ${orientation} stays wholly on pavement`,
      (name, destination) => {
        const result = evaluate(
          rotate({ x: -20, y: -29.25 }, orientation),
          rotate(destination, orientation),
        );
        console.info(
          `CONTAINMENT ${name} ${orientation}: ${JSON.stringify(result)}`,
        );
        expect(result.state).toBe("DESTINATION_REACHED");
        expect(result.departures).toBe(0);
        expect(result.maxCTE).toBeLessThan(ACCEPTANCE.maxCTE);
        expect(result.rmsCTE).toBeLessThan(ACCEPTANCE.rmsCTE);
        expect(result.endpointError).toBeLessThan(ACCEPTANCE.endpoint);
        expect(result.seconds).toBeLessThan(ACCEPTANCE.seconds);
        expect(result.maxLateral).toBeLessThanOrEqual(ACCEPTANCE.lateral);
      },
    );
  }
});

describe("road containment after a measured approach stop", () => {
  it.each(fixtures.filter(([name]) => name !== "straight"))(
    "%s remains on road when restarting before a turn",
    (name, destination) => {
      const result = evaluate({ x: -20, y: -29.25 }, destination, true);
      console.info(
        `CONTAINMENT approach stop ${name}: ${JSON.stringify(result)}`,
      );
      expect(result.state).toBe("DESTINATION_REACHED");
      expect(result.departures).toBe(0);
      expect(result.maxLateral).toBeLessThanOrEqual(ACCEPTANCE.lateral);
      expect(result.maxCTE).toBeLessThan(ACCEPTANCE.maxCTE);
      expect(result.rmsCTE).toBeLessThan(ACCEPTANCE.rmsCTE);
      expect(result.endpointError).toBeLessThan(ACCEPTANCE.endpoint);
      expect(result.seconds).toBeLessThan(ACCEPTANCE.seconds);
    },
  );
});
