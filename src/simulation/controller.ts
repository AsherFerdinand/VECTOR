import type { DriverInput, VehicleControl } from "./types";
import { clamp, VEHICLE } from "./vehicle";

/** Road-wheel steering moves at 60°/s, returning to centre at 90°/s. */
export const STEERING_RATE = Math.PI / 3;
export const CENTERING_RATE = Math.PI / 2;

/** Brake commands stop at zero; a held direction can propel on the following step. */
export function driverControl(
  input: DriverInput,
  velocity: number,
  steering: number,
  dt: number,
): VehicleControl {
  const direction = Number(input.left) - Number(input.right);
  const target = direction * VEHICLE.maxSteering;
  const rate = direction === 0 ? CENTERING_RATE : STEERING_RATE;
  const nextSteering =
    steering + clamp(target - steering, -rate * dt, rate * dt);
  let acceleration = 0;
  const stopping = () =>
    velocity === 0
      ? 0
      : -Math.sign(velocity) *
        Math.min(VEHICLE.maxBraking, Math.abs(velocity) / dt);
  if (input.brake || (input.forward && input.backward)) {
    acceleration = stopping();
  } else if (input.forward) {
    acceleration = velocity < 0 ? stopping() : VEHICLE.maxAcceleration;
  } else if (input.backward) {
    acceleration = velocity > 0 ? stopping() : -VEHICLE.maxAcceleration;
  }
  return { steering: nextSteering, acceleration };
}
