import type { VehicleControl, VehicleState } from "./types";

/** Geometric dimensions in m, speeds in m/s, angles in rad, acceleration in m/s². */
export const VEHICLE = Object.freeze({
  length: 4.5,
  width: 1.8,
  wheelbase: 2.7,
  maxSteering: Math.PI / 6,
  maxAcceleration: 3,
  maxBraking: 6,
  maxSpeed: 50 / 3.6,
  reverseSpeed: 10 / 3.6,
});

export const INITIAL_STATE: Readonly<VehicleState> = Object.freeze({
  x: -20,
  y: -29.25,
  heading: 0,
  velocity: 0,
});

export function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}

/** Explicit Euler; x/y locate the rear axle, heading is counterclockwise from +x. */
export function integrateVehicle(
  state: VehicleState,
  control: VehicleControl,
  dt: number,
): VehicleState {
  if (!Number.isFinite(dt) || dt <= 0)
    throw new RangeError("dt must be finite and positive");
  const steering = clamp(
    control.steering,
    -VEHICLE.maxSteering,
    VEHICLE.maxSteering,
  );
  const acceleration =
    state.velocity < 0
      ? clamp(
          control.acceleration,
          -VEHICLE.maxAcceleration,
          VEHICLE.maxBraking,
        )
      : clamp(
          control.acceleration,
          -VEHICLE.maxBraking,
          VEHICLE.maxAcceleration,
        );
  const velocity = clamp(
    state.velocity + acceleration * dt,
    -VEHICLE.reverseSpeed,
    VEHICLE.maxSpeed,
  );
  return {
    x: state.x + state.velocity * Math.cos(state.heading) * dt,
    y: state.y + state.velocity * Math.sin(state.heading) * dt,
    heading:
      state.heading +
      (state.velocity / VEHICLE.wheelbase) * Math.tan(steering) * dt,
    // Remove only floating-point residue at rest; do not snap meaningful motion.
    velocity: Math.abs(velocity) < 1e-12 ? 0 : velocity,
  };
}
