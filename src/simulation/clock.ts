import type { DriverInput, SimulationSnapshot, VehicleState } from "./types";
import { driverControl } from "./controller";
import { INITIAL_STATE, integrateVehicle } from "./vehicle";

export const FIXED_DT = 0.01;
export const MAX_FRAME_TIME = 0.1;
export const MAX_STEPS_PER_FRAME = 10;

/** Rendering contributes elapsed seconds only. Physics runs exclusively in fixed steps. */
export class Simulation {
  private state: VehicleState;
  private initial: VehicleState;
  private control = { steering: 0, acceleration: 0 };
  private ticks = 0;
  private accumulator = 0;
  private running = false;
  private distance = 0;
  private droppedTime = 0;

  constructor(initial: VehicleState = INITIAL_STATE) {
    this.initial = { ...initial };
    this.state = { ...initial };
  }

  get snapshot(): SimulationSnapshot {
    return {
      vehicle: { ...this.state },
      control: { ...this.control },
      time: this.ticks * FIXED_DT,
      running: this.running,
      distance: this.distance,
      droppedTime: this.droppedTime,
    };
  }

  start(): void {
    this.running = true;
  }
  pause(): void {
    this.running = false;
  }
  /** Reset restores the original state and leaves the simulation paused. */
  reset(): void {
    this.state = { ...this.initial };
    this.control = { steering: 0, acceleration: 0 };
    this.ticks = 0;
    this.accumulator = 0;
    this.running = false;
    this.distance = 0;
    this.droppedTime = 0;
  }

  advance(frameSeconds: number, input: DriverInput): void {
    if (!this.running) return;
    if (!Number.isFinite(frameSeconds) || frameSeconds < 0)
      throw new RangeError("frameSeconds must be finite and nonnegative");
    const accepted = Math.min(frameSeconds, MAX_FRAME_TIME);
    this.droppedTime += frameSeconds - accepted;
    this.accumulator += accepted;
    let steps = 0;
    // Epsilon protects equivalent frame partitions against binary roundoff.
    while (
      this.accumulator + 1e-12 >= FIXED_DT &&
      steps < MAX_STEPS_PER_FRAME
    ) {
      const requested = driverControl(
        input,
        this.state.velocity,
        this.control.steering,
        FIXED_DT,
      );
      const previous = this.state;
      this.state = integrateVehicle(previous, requested, FIXED_DT);
      this.distance += previous.velocity * FIXED_DT;
      this.control = {
        steering: requested.steering,
        acceleration: (this.state.velocity - previous.velocity) / FIXED_DT,
      };
      this.accumulator = Math.max(0, this.accumulator - FIXED_DT);
      this.ticks += 1;
      steps += 1;
    }
  }
}
