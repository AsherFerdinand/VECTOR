import type { DriverInput, SimulationSnapshot, VehicleState } from "./types";
import { driverControl } from "./controller";
import { INITIAL_STATE, integrateVehicle } from "./vehicle";
import { createRoadGraph, projectToRoad } from "../navigation/graph";
import { planRoute, isRoadPosition } from "../navigation/route";
import type { Point, RoadPosition, Route } from "../navigation/types";
import { NavigationController } from "../autonomy/navigationController";
import type { NavigationSnapshot, NavigationState } from "../autonomy/types";

export const FIXED_DT = 0.01;
export const MAX_FRAME_TIME = 0.1;
export const MAX_STEPS_PER_FRAME = 10;
const activeInput = (input: DriverInput) => Object.values(input).some(Boolean);
const angleDifference = (a: number, b: number) =>
  Math.atan2(Math.sin(a - b), Math.cos(a - b));

/** Fixed-step ground truth. Both driving modes use the same vehicle integrator. */
export class Simulation {
  readonly graph = createRoadGraph();
  private state: VehicleState;
  private initial: VehicleState;
  private control = { steering: 0, acceleration: 0 };
  private ticks = 0;
  private accumulator = 0;
  private running = false;
  private distance = 0;
  private droppedTime = 0;
  private mode: NavigationSnapshot["mode"] = "MANUAL";
  private route: Route | null = null;
  private destination: RoadPosition | null = null;
  private navigator: NavigationController | null = null;
  private idleState: NavigationState = "IDLE";
  private trajectory: Point[] = [];
  private message =
    "Select a destination on the map, or choose a new starting lane.";

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

  get navigation(): NavigationSnapshot {
    const telemetry = this.navigator?.telemetry;
    const hasCurrentProgress =
      this.mode === "AUTONOMOUS" || telemetry?.state === "ROUTE_READY";
    return {
      mode: this.mode,
      route: this.route,
      trajectory: this.trajectory,
      state:
        this.mode === "AUTONOMOUS" && telemetry
          ? telemetry.state
          : this.idleState,
      targetSpeed:
        this.mode === "AUTONOMOUS" ? (telemetry?.targetSpeed ?? null) : null,
      steeringCommand:
        this.mode === "AUTONOMOUS"
          ? (telemetry?.steeringCommand ?? null)
          : null,
      remainingDistance: hasCurrentProgress
        ? (telemetry?.remainingDistance ?? this.route?.length ?? null)
        : null,
      crossTrackError:
        this.mode === "AUTONOMOUS"
          ? (telemetry?.crossTrackError ?? null)
          : null,
      lookAheadDistance:
        this.mode === "AUTONOMOUS"
          ? (telemetry?.lookAheadDistance ?? null)
          : null,
      lookAheadPoint:
        this.mode === "AUTONOMOUS" ? (telemetry?.lookAheadPoint ?? null) : null,
      progress: telemetry?.progress ?? 0,
      message: this.message,
    };
  }

  start(): void {
    this.running = true;
  }
  pause(): void {
    this.running = false;
  }

  /** Relocation is explicit, stationary and paused; it also becomes the reset pose. */
  setStart(position: RoadPosition): boolean {
    if (this.running || Math.abs(this.state.velocity) > 0.01) {
      this.message =
        "Start selection requires a stationary vehicle and a paused simulation. Reset or brake first.";
      return false;
    }
    if (!isRoadPosition(this.graph, position)) {
      this.message = "Choose a valid directed lane for the start.";
      return false;
    }
    this.initial = {
      x: position.x,
      y: position.y,
      heading: position.heading,
      velocity: 0,
    };
    this.reset();
    this.message = this.route
      ? "Start updated. Route recalculated."
      : "Start selected. Choose a destination.";
    return true;
  }

  private currentRoadPosition(): RoadPosition | null {
    const position = projectToRoad(this.graph, this.state, 1.0);
    return position &&
      Math.abs(angleDifference(this.state.heading, position.heading)) < 0.35
      ? position
      : null;
  }

  /** Planning never teleports the vehicle. Pausing prevents replacing an active route. */
  planTo(destination: RoadPosition): boolean {
    if (this.running) {
      this.message = "Pause before selecting a new destination.";
      return false;
    }
    if (!isRoadPosition(this.graph, destination)) {
      this.message = "Choose a valid destination on a directed lane.";
      return false;
    }
    this.destination = { ...destination };
    return this.prepareRoute();
  }

  private prepareRoute(): boolean {
    const start = this.currentRoadPosition();
    this.route =
      start && this.destination
        ? planRoute(this.graph, start, this.destination)
        : null;
    this.navigator = this.route ? new NavigationController(this.route) : null;
    this.mode = "MANUAL";
    this.idleState = this.route ? "ROUTE_READY" : "ROUTE_UNAVAILABLE";
    this.trajectory = [];
    this.message = this.route
      ? "Route ready. Start autonomous drive when ready."
      : "Route unavailable. Use a road-aligned start and a reachable destination at least 1 m away.";
    return this.route !== null;
  }

  activateAutonomy(): boolean {
    if (this.state.velocity < 0) {
      this.message = "Stop reversing before activating autonomous driving.";
      return false;
    }
    if (!this.destination) {
      this.message =
        "Select a valid route before activating autonomous driving.";
      return false;
    }
    if (!this.prepareRoute()) return false;
    this.mode = "AUTONOMOUS";
    this.running = true;
    this.message =
      "Map-based path following · perfect localisation. Driving keys take over immediately.";
    return true;
  }

  cancelAutonomy(): void {
    this.mode = "MANUAL";
    this.idleState = this.route ? "ROUTE_READY" : "IDLE";
    this.message =
      "Manual control. Vehicle motion is preserved; brake to stop.";
  }

  clearRoute(): void {
    this.cancelAutonomy();
    this.route = null;
    this.destination = null;
    this.navigator = null;
    this.idleState = "IDLE";
    this.trajectory = [];
    this.message = "Route cleared. Select a destination.";
  }

  /** Reset retains the selected start and destination and recalculates their route. */
  reset(): void {
    this.state = { ...this.initial };
    this.control = { steering: 0, acceleration: 0 };
    this.ticks = 0;
    this.accumulator = 0;
    this.running = false;
    this.distance = 0;
    this.droppedTime = 0;
    this.mode = "MANUAL";
    this.trajectory = [];
    if (this.destination) this.prepareRoute();
    else {
      this.route = null;
      this.navigator = null;
      this.idleState = "IDLE";
      this.message =
        "Simulation reset. Select a destination or drive manually.";
    }
  }

  /** Explicit deterministic tick for engineering tests; obeys pause and input arbitration. */
  step(input: DriverInput): void {
    if (this.mode === "AUTONOMOUS" && activeInput(input)) this.cancelAutonomy();
    if (!this.running) return;
    const requested =
      this.mode === "AUTONOMOUS" && this.navigator
        ? this.navigator.update(this.state, this.control.steering, FIXED_DT)
        : driverControl(
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
    this.ticks += 1;
    const last = this.trajectory.at(-1);
    if (
      this.route &&
      (!last || Math.hypot(this.state.x - last.x, this.state.y - last.y) >= 0.3)
    ) {
      this.trajectory = [
        ...this.trajectory.slice(-599),
        { x: this.state.x, y: this.state.y },
      ];
    }
    if (this.mode === "AUTONOMOUS") {
      const status = this.navigator!.telemetry.state;
      if (status === "DESTINATION_REACHED")
        this.message = "Destination reached. Vehicle stopped.";
      if (status === "SAFE_STOP")
        this.message =
          "Tracking tolerance exceeded. Braking to a safe stop; reset or take manual control.";
    }
  }

  advance(frameSeconds: number, input: DriverInput): void {
    if (this.mode === "AUTONOMOUS" && activeInput(input)) this.cancelAutonomy();
    if (!this.running) return;
    if (!Number.isFinite(frameSeconds) || frameSeconds < 0)
      throw new RangeError("frameSeconds must be finite and nonnegative");
    const accepted = Math.min(frameSeconds, MAX_FRAME_TIME);
    this.droppedTime += frameSeconds - accepted;
    this.accumulator += accepted;
    let steps = 0;
    while (
      this.accumulator + 1e-12 >= FIXED_DT &&
      steps < MAX_STEPS_PER_FRAME
    ) {
      this.step(input);
      this.accumulator = Math.max(0, this.accumulator - FIXED_DT);
      steps += 1;
    }
  }
}
