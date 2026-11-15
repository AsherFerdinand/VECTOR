import { describe, expect, it } from "vitest";
import { createRoadGraph, projectToRoad } from "../src/navigation/graph";
import { planRoute } from "../src/navigation/route";
import { NavigationController } from "../src/autonomy/navigationController";
import { integrateVehicle } from "../src/simulation/vehicle";
import type { Point, Route } from "../src/navigation/types";

// Declared before evaluation: lane-centre half-width is 2.25 m. These tighter
// tracking bounds reserve >1 m to lane edges for model/discretisation error.
export const ACCEPTANCE = {
  maxCrossTrack: 1,
  rmsCrossTrack: 0.5,
  endpointError: 0.8,
  stoppingSpeed: 0.1,
  maxProgressStep: 0.3,
  completionSeconds: 180,
  cornerOverspeedTolerance: 0.25,
};
const graph = createRoadGraph();
function makeRoute(start: Point, destination: Point): Route {
  const a = projectToRoad(graph, start),
    b = projectToRoad(graph, destination);
  if (!a || !b)
    throw new Error("Validation points must project to valid lanes");
  const route = planRoute(graph, a, b);
  if (!route) throw new Error("Validation route must be connected");
  return route;
}
export function evaluate(route: Route) {
  const controller = new NavigationController(route);
  let vehicle = {
    x: route.start.x,
    y: route.start.y,
    heading: route.start.heading,
    velocity: 0,
  };
  let steering = 0,
    maximum = 0,
    squares = 0,
    ticks = 0,
    previousProgress = 0,
    maxProgressStep = 0;
  let maximumSteering = 0,
    maxSpeed = 0,
    minTurnSpeed = Infinity,
    maximumLateralAcceleration = 0;
  const firstTurn = route.points.find((p) => Math.abs(p.curvature) > 0.05);
  let firstTurnEntrySpeed: number | null = null;
  const states = new Set<string>();
  for (; ticks < ACCEPTANCE.completionSeconds / 0.01; ticks++) {
    const control = controller.update(vehicle, steering, 0.01);
    steering = control.steering;
    const data = controller.telemetry;
    const error = Math.abs(data.crossTrackError ?? 0);
    maximum = Math.max(maximum, error);
    squares += error * error;
    maxProgressStep = Math.max(
      maxProgressStep,
      (data.progress - previousProgress) * route.length,
    );
    previousProgress = data.progress;
    states.add(data.state);
    maximumSteering = Math.max(maximumSteering, Math.abs(steering));
    maximumLateralAcceleration = Math.max(
      maximumLateralAcceleration,
      (vehicle.velocity ** 2 * Math.abs(Math.tan(steering))) / 2.7,
    );
    if (
      firstTurn &&
      firstTurnEntrySpeed === null &&
      data.progress * route.length >= firstTurn.s
    )
      firstTurnEntrySpeed = vehicle.velocity;
    maxSpeed = Math.max(maxSpeed, vehicle.velocity);
    if (data.state === "APPROACHING_TURN")
      minTurnSpeed = Math.min(minTurnSpeed, vehicle.velocity);
    vehicle = integrateVehicle(vehicle, control, 0.01);
    if (
      data.state === "SAFE_STOP" ||
      (data.state === "DESTINATION_REACHED" &&
        Math.abs(vehicle.velocity) < 1e-12)
    )
      break;
  }
  const endpoint = route.points[route.points.length - 1];
  return {
    state: controller.telemetry.state,
    maximum,
    rms: Math.sqrt(squares / (ticks + 1)),
    endpointError: Math.hypot(vehicle.x - endpoint.x, vehicle.y - endpoint.y),
    stoppingSpeed: Math.abs(vehicle.velocity),
    seconds: (ticks + 1) * 0.01,
    maxProgressStep,
    maximumSteering,
    maxSpeed,
    firstTurnEntrySpeed,
    maximumLateralAcceleration,
    minTurnSpeed: Number.isFinite(minTurnSpeed) ? minTurnSpeed : null,
    states: [...states],
    vehicle,
  };
}

describe("deterministic engineering validation", () => {
  const scenarios = [
    [
      "A straight / D destination stop",
      { x: -20, y: -29.25 },
      { x: 10, y: -29.25 },
    ],
    ["B 90-degree right turn", { x: -20, y: -29.25 }, { x: 24.75, y: -45 }],
    ["C multiple intersections", { x: -20, y: -29.25 }, { x: -40, y: -24.75 }],
  ] as const;
  it.each(scenarios)(
    "%s satisfies the predefined tracking and stopping thresholds",
    (name, start, end) => {
      const result = evaluate(makeRoute(start, end));
      console.info(`VALIDATION ${name}: ${JSON.stringify(result)}`);
      expect(result.state).toBe("DESTINATION_REACHED");
      expect(result.maximum).toBeLessThanOrEqual(ACCEPTANCE.maxCrossTrack);
      expect(result.rms).toBeLessThanOrEqual(ACCEPTANCE.rmsCrossTrack);
      expect(result.endpointError).toBeLessThanOrEqual(
        ACCEPTANCE.endpointError,
      );
      expect(result.stoppingSpeed).toBeLessThanOrEqual(
        ACCEPTANCE.stoppingSpeed,
      );
      expect(result.maxProgressStep).toBeLessThanOrEqual(
        ACCEPTANCE.maxProgressStep,
      );
      expect(result.seconds).toBeLessThan(ACCEPTANCE.completionSeconds);
      if (result.firstTurnEntrySpeed !== null) {
        const route = makeRoute(start, end);
        const firstTurn = route.points.find(
          (p) => Math.abs(p.curvature) > 0.05,
        )!;
        const curvature = Math.max(
          ...route.points
            .filter((p) => p.s >= firstTurn.s && p.s <= firstTurn.s + 3)
            .map((p) => Math.abs(p.curvature)),
        );
        expect(result.firstTurnEntrySpeed).toBeLessThanOrEqual(
          Math.sqrt(1.5 / curvature) + ACCEPTANCE.cornerOverspeedTolerance,
        );
      }
    },
  );
  it("replays exactly with the same fixed-step conditions", () => {
    const route = makeRoute({ x: -20, y: -29.25 }, { x: 24.75, y: -45 });
    expect(evaluate(route)).toEqual(evaluate(route));
  });
});
