import { describe, expect, it } from "vitest";
import { integrateVehicle, VEHICLE } from "../src/simulation/vehicle";
import {
  driverControl,
  STEERING_RATE,
  CENTERING_RATE,
} from "../src/simulation/controller";
import type { DriverInput, VehicleState } from "../src/simulation/types";

const origin: VehicleState = { x: 0, y: 0, heading: 0, velocity: 5 };
const idle: DriverInput = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};

describe("kinematic bicycle model", () => {
  it("moves straight without changing heading at zero steering", () => {
    let state = { ...origin, heading: Math.PI / 4 };
    for (let i = 0; i < 100; i++)
      state = integrateVehicle(state, { steering: 0, acceleration: 0 }, 0.01);
    expect(state.heading).toBe(Math.PI / 4);
    expect(state.x).toBeCloseTo(5 / Math.sqrt(2), 10);
    expect(state.y).toBeCloseTo(5 / Math.sqrt(2), 10);
  });
  it("accelerates according to dv/dt = a", () => {
    const state = integrateVehicle(
      origin,
      { steering: 0, acceleration: 3 },
      0.1,
    );
    expect(state.velocity).toBeCloseTo(5.3);
    expect(state.x).toBeCloseTo(0.5); // Explicit Euler uses the previous velocity.
  });
  it("clamps excessive acceleration, steering and both speed limits", () => {
    const fast = integrateVehicle(
      { ...origin, velocity: VEHICLE.maxSpeed },
      { steering: 50, acceleration: 999 },
      1,
    );
    expect(fast.velocity).toBe(VEHICLE.maxSpeed);
    expect(fast.heading).toBeCloseTo(
      (VEHICLE.maxSpeed / VEHICLE.wheelbase) * Math.tan(VEHICLE.maxSteering),
    );
    const reverse = integrateVehicle(
      origin,
      { steering: 0, acceleration: -999 },
      10,
    );
    expect(reverse.velocity).toBe(-VEHICLE.reverseSpeed);
  });
  it.each([0.1, 0.3, -0.3])(
    "matches turning radius L/tan(delta) at steering %s",
    (steering) => {
      const next = integrateVehicle(
        origin,
        { steering, acceleration: 0 },
        0.01,
      );
      const yawRate = next.heading / 0.01;
      expect(origin.velocity / yawRate).toBeCloseTo(
        VEHICLE.wheelbase / Math.tan(steering),
        10,
      );
      expect(Math.sign(yawRate)).toBe(Math.sign(steering));
    },
  );
  it("reverses yaw direction when driving backward", () => {
    expect(
      integrateVehicle(
        { ...origin, velocity: -2 },
        { steering: 0.2, acceleration: 0 },
        0.01,
      ).heading,
    ).toBeLessThan(0);
  });
  it("rejects invalid integration intervals", () => {
    expect(() =>
      integrateVehicle(origin, { steering: 0, acceleration: 0 }, 0),
    ).toThrow(RangeError);
  });
});

describe("driver controls", () => {
  it("limits steering rate and returns to centre without overshoot", () => {
    expect(
      driverControl({ ...idle, left: true }, 0, 0, 0.01).steering,
    ).toBeCloseTo(STEERING_RATE * 0.01);
    expect(driverControl(idle, 0, 0.2, 0.01).steering).toBeCloseTo(
      0.2 - CENTERING_RATE * 0.01,
    );
    expect(driverControl(idle, 0, 0.001, 0.01).steering).toBe(0);
  });
  it("brakes to rest without crossing zero in either direction", () => {
    for (const velocity of [0.01, -0.01, 3, -2]) {
      const control = driverControl(
        { ...idle, brake: true },
        velocity,
        0,
        0.01,
      );
      const next = integrateVehicle({ ...origin, velocity }, control, 0.01);
      expect(Math.abs(next.velocity)).toBeLessThanOrEqual(Math.abs(velocity));
      expect(next.velocity * velocity).toBeGreaterThanOrEqual(0);
      expect(Math.abs(control.acceleration)).toBeLessThanOrEqual(
        VEHICLE.maxBraking,
      );
    }
  });
  it("stops before changing propulsion direction", () => {
    const control = driverControl({ ...idle, backward: true }, 0.02, 0, 0.01);
    expect(
      integrateVehicle({ ...origin, velocity: 0.02 }, control, 0.01).velocity,
    ).toBe(0);
    expect(
      driverControl({ ...idle, backward: true }, 0, 0, 0.01).acceleration,
    ).toBe(-3);
    expect(
      driverControl({ ...idle, forward: true }, -1, 0, 0.01).acceleration,
    ).toBe(6);
  });
  it("resolves conflicting inputs with braking and centred steering", () => {
    const control = driverControl(
      { forward: true, backward: true, left: true, right: true, brake: false },
      2,
      0,
      0.01,
    );
    expect(control).toEqual({ steering: 0, acceleration: -6 });
    expect(
      driverControl({ ...idle, forward: true, brake: true }, 0, 0, 0.01)
        .acceleration,
    ).toBe(0);
  });
});
