import type { Route } from "../navigation/types";
import type { VehicleControl, VehicleState } from "../simulation/types";
import { clamp, VEHICLE } from "../simulation/vehicle";
import { STEERING_RATE } from "../simulation/controller";
import type { NavigationTelemetry } from "./types";
import { projectOnPath, purePursuit } from "./purePursuit";
import { speedControl, targetSpeed } from "./speedController";

/** Map-based path following with perfect localisation; no dynamic-world observations. */
export class NavigationController {
  private progress = 0;
  private data: NavigationTelemetry;

  constructor(private readonly route: Route) {
    if (route.points.length === 0)
      throw new Error("Cannot follow an empty reference path");
    this.data = {
      state: "ROUTE_READY",
      targetSpeed: 0,
      steeringCommand: 0,
      remainingDistance: route.length,
      crossTrackError: 0,
      lookAheadDistance: null,
      lookAheadPoint: null,
      progress: 0,
    };
  }

  get telemetry(): NavigationTelemetry {
    return {
      ...this.data,
      lookAheadPoint: this.data.lookAheadPoint
        ? { ...this.data.lookAheadPoint }
        : null,
    };
  }

  update(
    vehicle: VehicleState,
    actualSteering: number,
    dt: number,
  ): VehicleControl {
    if (!Number.isFinite(dt) || dt <= 0)
      throw new RangeError("Controller dt must be finite and positive");
    const stop = () => {
      this.data.steeringCommand = actualSteering;
      return {
        steering: actualSteering,
        acceleration:
          vehicle.velocity === 0
            ? 0
            : vehicle.velocity > 0
              ? -Math.min(VEHICLE.maxBraking, vehicle.velocity / dt)
              : Math.min(VEHICLE.maxBraking, -vehicle.velocity / dt),
      };
    };
    if (
      this.data.state === "DESTINATION_REACHED" ||
      this.data.state === "SAFE_STOP"
    )
      return stop();
    const projection = projectOnPath(
      this.route,
      vehicle,
      this.progress,
      Math.max(0.2, Math.abs(vehicle.velocity) * dt * 2),
    );
    this.progress = projection.s;
    const remaining = Math.max(0, this.route.length - this.progress);
    const endpoint = this.route.points[this.route.points.length - 1];
    const endpointError = Math.hypot(
      vehicle.x - endpoint.x,
      vehicle.y - endpoint.y,
    );
    const pursuit = purePursuit(this.route, vehicle, this.progress);
    const target = targetSpeed(this.route, this.progress);
    this.data = {
      state: target.approachingTurn ? "APPROACHING_TURN" : "ROUTE_FOLLOWING",
      targetSpeed: target.speed,
      steeringCommand: pursuit.steering,
      remainingDistance: remaining,
      crossTrackError: projection.crossTrackError,
      lookAheadDistance: pursuit.lookAheadDistance,
      lookAheadPoint: pursuit.target,
      progress: this.route.length > 0 ? this.progress / this.route.length : 1,
    };
    if (
      Math.hypot(vehicle.x - projection.x, vehicle.y - projection.y) > 3 ||
      vehicle.velocity < -0.1
    ) {
      this.data.state = "SAFE_STOP";
      this.data.targetSpeed = 0;
      return stop();
    }
    if (
      remaining <= 0.8 &&
      endpointError <= 0.8 &&
      Math.abs(vehicle.velocity) <= 0.1
    ) {
      this.data.state = "DESTINATION_REACHED";
      this.data.targetSpeed = 0;
      return stop();
    }
    const localTargetX =
      Math.cos(vehicle.heading) * (pursuit.target.x - vehicle.x) +
      Math.sin(vehicle.heading) * (pursuit.target.y - vehicle.y);
    if (localTargetX < -0.1) {
      this.data.targetSpeed = 0;
      // At the destination tolerate a small integration overshoot, braking in place.
      // A behind-target elsewhere indicates the pose no longer matches this route.
      if (remaining > 0.8 || endpointError > 0.8) this.data.state = "SAFE_STOP";
      return stop();
    }
    // Beyond the endpoint, a target behind the vehicle must never trigger a U-turn.
    const lastSegment =
      this.route.points[Math.max(0, this.route.points.length - 2)];
    const dx = endpoint.x - lastSegment.x,
      dy = endpoint.y - lastSegment.y;
    if (
      remaining < 0.1 &&
      (vehicle.x - endpoint.x) * dx + (vehicle.y - endpoint.y) * dy >
        0.8 * Math.hypot(dx, dy)
    ) {
      this.data.state = "SAFE_STOP";
      this.data.targetSpeed = 0;
      return stop();
    }
    return {
      steering:
        actualSteering +
        clamp(
          pursuit.steering - actualSteering,
          -STEERING_RATE * dt,
          STEERING_RATE * dt,
        ),
      acceleration: speedControl(vehicle.velocity, target.speed, dt),
    };
  }
}
