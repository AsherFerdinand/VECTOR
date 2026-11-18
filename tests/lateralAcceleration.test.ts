import { describe, expect, it } from "vitest";
import { NavigationController } from "../src/autonomy/navigationController";
import { createRoadGraph, projectToRoad } from "../src/navigation/graph";
import { planRoute } from "../src/navigation/route";
import { integrateVehicle, VEHICLE } from "../src/simulation/vehicle";
import type { Point } from "../src/navigation/types";

// Predeclared acceptance: measured bicycle acceleration must obey the 1.5 m/s²
// budget, including speed-controller lag and actual applied steering. Preserve
// the existing tracking/arrival criteria; no physical state is clamped for tests.
const LIMITS = {
  lateral: 1.5,
  numericTolerance: 1e-6,
  maxError: 1,
  rmsError: 0.5,
  endpointError: 0.8,
  stoppingSpeed: 0.1,
  seconds: 180,
};
const graph = createRoadGraph();
function run(start: Point, destination: Point) {
  const a = projectToRoad(graph, start),
    b = projectToRoad(graph, destination);
  if (!a || !b) throw new Error("Invalid regression scenario");
  const route = planRoute(graph, a, b);
  if (!route) throw new Error("Unreachable regression scenario");
  const controller = new NavigationController(route);
  let vehicle = { x: a.x, y: a.y, heading: a.heading, velocity: 0 },
    steering = 0;
  let maxLateral = 0,
    maxError = 0,
    squareError = 0,
    maxSpeed = 0,
    ticks = 0;
  for (; ticks < LIMITS.seconds / 0.01; ticks++) {
    const command = controller.update(vehicle, steering, 0.01);
    steering = command.steering;
    // Euler yaw uses pre-step speed; check both ends of the applied control step.
    const curvature = Math.abs(Math.tan(steering)) / VEHICLE.wheelbase;
    maxLateral = Math.max(maxLateral, vehicle.velocity ** 2 * curvature);
    vehicle = integrateVehicle(vehicle, command, 0.01);
    maxLateral = Math.max(maxLateral, vehicle.velocity ** 2 * curvature);
    const telemetry = controller.telemetry;
    const error = Math.abs(telemetry.crossTrackError ?? 0);
    maxError = Math.max(maxError, error);
    squareError += error * error;
    maxSpeed = Math.max(maxSpeed, vehicle.velocity);
    if (
      telemetry.state === "SAFE_STOP" ||
      (telemetry.state === "DESTINATION_REACHED" &&
        Math.abs(vehicle.velocity) < 1e-12)
    )
      break;
  }
  return {
    maxLateral,
    maxError,
    rmsError: Math.sqrt(squareError / (ticks + 1)),
    maxSpeed,
    endpointError: Math.hypot(vehicle.x - b.x, vehicle.y - b.y),
    stoppingSpeed: Math.abs(vehicle.velocity),
    seconds: (ticks + 1) * 0.01,
    state: controller.telemetry.state,
  };
}

describe("realised lateral-acceleration regression", () => {
  const scenarios = [
    ["straight", { x: -20, y: -29.25 }, { x: 10, y: -29.25 }],
    ["right turn", { x: -20, y: -29.25 }, { x: 24.75, y: -45 }],
    ["left turn", { x: -20, y: -29.25 }, { x: 29.25, y: 0 }],
    ["multiple intersections", { x: -20, y: -29.25 }, { x: -40, y: -24.75 }],
    [
      "opposite-direction right turn",
      { x: 20, y: 29.25 },
      { x: -24.75, y: 45 },
    ],
  ] as const;
  it.each(scenarios)(
    "%s stays within the measured budget without losing route completion",
    (name, start, end) => {
      const result = run(start, end);
      console.info(`LATERAL ${name}: ${JSON.stringify(result)}`);
      expect(result.state).toBe("DESTINATION_REACHED");
      expect(result.maxLateral).toBeLessThanOrEqual(
        LIMITS.lateral + LIMITS.numericTolerance,
      );
      expect(result.maxError).toBeLessThanOrEqual(LIMITS.maxError);
      expect(result.rmsError).toBeLessThanOrEqual(LIMITS.rmsError);
      expect(result.endpointError).toBeLessThanOrEqual(LIMITS.endpointError);
      expect(result.stoppingSpeed).toBeLessThanOrEqual(LIMITS.stoppingSpeed);
      expect(result.seconds).toBeLessThan(LIMITS.seconds);
      if (name === "straight") expect(result.maxSpeed).toBeGreaterThan(5.99);
    },
  );
});
