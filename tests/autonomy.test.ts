import { describe, expect, it } from "vitest";
import type { Route, Point } from "../src/navigation/types";
import {
  purePursuit,
  pointAtDistance,
  projectOnPath,
} from "../src/autonomy/purePursuit";
import { speedControl, targetSpeed } from "../src/autonomy/speedController";
import { NavigationController } from "../src/autonomy/navigationController";
import { VEHICLE } from "../src/simulation/vehicle";

function line(points: Point[]): Route {
  let s = 0;
  return {
    points: points.map((point, index) => {
      if (index)
        s += Math.hypot(
          point.x - points[index - 1].x,
          point.y - points[index - 1].y,
        );
      return { ...point, s, curvature: 0 };
    }),
    get length() {
      return s;
    },
    edgeIds: [],
    start: { ...points[0], edgeId: "start", offset: 0, heading: 0 },
    destination: {
      ...points[points.length - 1],
      edgeId: "end",
      offset: 0,
      heading: 0,
    },
  };
}
const straight = line([
  { x: 0, y: 0 },
  { x: 10, y: 0 },
  { x: 30, y: 0 },
]);
const origin = { x: 0, y: 0, heading: 0, velocity: 0 };

describe("Pure Pursuit geometry", () => {
  it("commands zero steering on a straight path and interpolates look-ahead", () => {
    const result = purePursuit(straight, origin, 0);
    expect(result.steering).toBe(0);
    expect(result.target).toEqual({ x: 2.5, y: 0 });
    expect(pointAtDistance(straight.points, 15)).toEqual({ x: 15, y: 0 });
  });
  it("turns left and right with correctly signed steering", () => {
    expect(
      purePursuit(
        line([
          { x: 0, y: 0 },
          { x: 4, y: 4 },
        ]),
        origin,
        0,
      ).steering,
    ).toBeGreaterThan(0);
    expect(
      purePursuit(
        line([
          { x: 0, y: 0 },
          { x: 4, y: -4 },
        ]),
        origin,
        0,
      ).steering,
    ).toBeLessThan(0);
  });
  it("saturates at the existing steering limit", () => {
    expect(
      purePursuit(
        line([
          { x: 0, y: 0 },
          { x: 0, y: 1 },
        ]),
        origin,
        0,
      ).steering,
    ).toBe(VEHICLE.maxSteering);
  });
  it("projects signed cross-track error and prevents backward progress", () => {
    const projection = projectOnPath(straight, { x: 5, y: 1 }, 5, 0.2);
    expect(projection.s).toBe(5);
    expect(projection.crossTrackError).toBeCloseTo(1);
    expect(projectOnPath(straight, { x: 4.9, y: -1 }, 5, 0.2).s).toBe(5);
  });
  it("bounds progress even when a later section passes near the vehicle", () => {
    const loop = line([
      { x: 0, y: 0 },
      { x: 10, y: 0 },
      { x: 10, y: 10 },
      { x: 0, y: 10 },
      { x: 0, y: 0 },
    ]);
    expect(projectOnPath(loop, { x: 0, y: 0 }, 0, 0.2).s).toBe(0);
    expect(projectOnPath(straight, { x: 20, y: 0 }, 0, 0.2).s).toBeCloseTo(0.2);
  });
});

describe("longitudinal control", () => {
  it("accelerates toward the target and respects configured acceleration bounds", () => {
    expect(speedControl(0, 6, 0.01)).toBe(3);
    expect(speedControl(6, 0, 0.01)).toBe(-6);
    expect(speedControl(0.01, 0, 0.01)).toBeGreaterThanOrEqual(-1);
  });
  it("limits straight speed and progressively brakes for the destination", () => {
    expect(targetSpeed(straight, 0).speed).toBe(6);
    expect(targetSpeed(straight, 29).speed).toBeLessThan(2);
    expect(targetSpeed(straight, 30).speed).toBe(0);
  });
  it("anticipates a corner before reaching it", () => {
    const curved = {
      ...straight,
      points: straight.points.map((p) => ({
        ...p,
        curvature: p.s === 10 ? 1 / 6.75 : 0,
      })),
    };
    expect(targetSpeed(curved, 8).speed).toBeLessThan(6);
    expect(targetSpeed(curved, 10).speed).toBeCloseTo(Math.sqrt(1.5 * 6.75));
  });
});

describe("navigation state and command safety", () => {
  it("begins ready then follows and rate-limits steering", () => {
    const controller = new NavigationController(
      line([
        { x: 0, y: 0 },
        { x: 10, y: 10 },
      ]),
    );
    expect(controller.telemetry.state).toBe("ROUTE_READY");
    const control = controller.update(origin, 0, 0.01);
    expect(controller.telemetry.state).toBe("ROUTE_FOLLOWING");
    expect(Math.abs(control.steering)).toBeLessThanOrEqual(
      (Math.PI / 3) * 0.01,
    );
  });
  it("holds a terminal safe-stop instead of steering back to a lost path", () => {
    const controller = new NavigationController(straight);
    const command = controller.update(
      { ...origin, y: 4, velocity: 2 },
      0,
      0.01,
    );
    expect(controller.telemetry.state).toBe("SAFE_STOP");
    expect(command.acceleration).toBe(-6);
    controller.update(origin, 0, 0.01);
    expect(controller.telemetry.state).toBe("SAFE_STOP");
  });
  it("safe-stops for a longitudinally lost pose or a look-ahead point behind", () => {
    const lost = new NavigationController(straight);
    lost.update({ ...origin, x: 20, velocity: 2 }, 0, 0.01);
    expect(lost.telemetry.state).toBe("SAFE_STOP");
    const wrongHeading = new NavigationController(straight);
    wrongHeading.update({ ...origin, heading: Math.PI, velocity: 1 }, 0, 0.01);
    expect(wrongHeading.telemetry.state).toBe("SAFE_STOP");
  });
  it("reaches a stationary nearby endpoint and never requests reverse", () => {
    const controller = new NavigationController(
      line([
        { x: 0, y: 0 },
        { x: 0.2, y: 0 },
      ]),
    );
    const command = controller.update(origin, 0, 0.01);
    expect(controller.telemetry.state).toBe("DESTINATION_REACHED");
    expect(command.acceleration).toBe(0);
  });
});
