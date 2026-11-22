import type { VehicleState, VehicleControl } from "../simulation/types";
import type { Track } from "../perception/types";
import { clamp, VEHICLE } from "../simulation/vehicle";
import { corridorContains, corridorLane, laneY } from "./corridor";
import type {
  ManoeuvreSnapshot,
  ManoeuvreOptions,
  TrajectoryCandidate,
} from "./types";
import { laneChangeCandidate, validateCandidate } from "./trajectory";

/** Bounded planner for the explicitly mapped two-eastbound-lane corridor.
 * Core town routes retain their existing safe-stop/static reroute behaviour. */
export class ManoeuvrePlanner {
  private value: ManoeuvreSnapshot = {
    state: "FOLLOW_LANE",
    reason: "Corridor lane following",
    candidates: [],
    selected: null,
    targetSpeed: 6,
  };
  private active: TrajectoryCandidate | null = null;
  private started = 0;
  private clearSince: number | null = null;
  private lastChange = -Infinity;
  private overtaking = false;
  get snapshot(): ManoeuvreSnapshot {
    return this.value;
  }
  reset(): void {
    this.active = null;
    this.started = 0;
    this.clearSince = null;
    this.lastChange = -Infinity;
    this.overtaking = false;
    this.value = {
      state: "FOLLOW_LANE",
      reason: "Planner reset",
      candidates: [],
      selected: null,
      targetSpeed: 6,
    };
  }
  update(
    vehicle: VehicleState,
    tracks: Track[],
    now: number,
    options: ManoeuvreOptions = {},
  ): ManoeuvreSnapshot {
    const speed = clamp(options.desiredSpeed ?? 6, 0, 8),
      destination = options.destinationX ?? 150;
    const stop = (reason: string): ManoeuvreSnapshot => {
      this.active = null;
      this.clearSince = null;
      return (this.value = {
        state: "SAFE_STOP",
        reason,
        candidates: [],
        selected: null,
        targetSpeed: 0,
      });
    };
    if (options.enabled === false) {
      this.reset();
      return this.value;
    }
    if (!corridorContains(vehicle, 0.02) || Math.abs(vehicle.heading) > 0.7)
      return stop("Vehicle is outside the mapped eastbound manoeuvre corridor");
    const relevant = tracks.filter(
      (t) => t.status !== "tentative" && now - t.lastMeasurementTime <= 0.6,
    );
    const untrusted = tracks.some(
      (t) =>
        t.x > vehicle.x &&
        t.x < vehicle.x + 20 &&
        Math.abs(t.y - vehicle.y) < 2 &&
        (t.status === "tentative" || now - t.lastMeasurementTime > 0.6),
    );
    if (untrusted)
      return stop(
        "Unconfirmed or stale driving-corridor estimate: manoeuvre inhibited",
      );
    if (this.active) {
      const elapsed = now - this.started;
      const remainder: TrajectoryCandidate = {
        ...this.active,
        points: this.active.points
          .filter((p) => p.time >= elapsed)
          .map((p) => ({ ...p, time: p.time - elapsed })),
        reasons: [],
      };
      const checked = validateCandidate(
        remainder,
        relevant,
        now,
        options.staticObjects,
        destination,
      );
      if (!checked.valid) {
        this.active = null;
        this.clearSince = null;
        return (this.value = {
          state: "SAFE_STOP",
          reason: `Abort: ${checked.reasons.join("; ")}`,
          candidates: [checked],
          selected: null,
          targetSpeed: 0,
        });
      }
      const atLane =
        Math.abs(vehicle.y - laneY(this.active.targetLane)) < 0.25 &&
        Math.abs(vehicle.heading) < 0.08;
      if (elapsed >= this.active.duration && atLane) {
        this.lastChange = now;
        this.overtaking =
          this.active.targetLane === "east-inner" &&
          options.returnToLane !== false;
        this.active = null;
        this.clearSince = null;
      } else
        return (this.value = {
          state:
            this.active.targetLane === "east-inner"
              ? "EXECUTE_LANE_CHANGE"
              : "RETURN_TO_LANE",
          reason:
            "Committed feasible trajectory; emergency braking retains control priority",
          candidates: [checked],
          selected: this.active,
          targetSpeed: this.active.points[0].velocity,
        });
    }
    const lane = corridorLane(vehicle.y);
    const lead = relevant
      .filter(
        (t) =>
          t.x > vehicle.x &&
          t.x - vehicle.x < 35 &&
          Math.abs(t.y - vehicle.y) < 2.3,
      )
      .sort((a, b) => a.x - b.x)[0];
    const blocked = !!lead && lead.vx < speed - 1;
    const wantReturn =
      options.returnToLane !== false &&
      this.overtaking &&
      lane === "east-inner" &&
      !relevant.some(
        (t) =>
          Math.abs(t.y - laneY("east-outer")) < 2.3 &&
          t.x > vehicle.x - 12 &&
          t.x < vehicle.x + 30,
      );
    if (!blocked && !wantReturn) {
      this.clearSince = null;
      return (this.value = {
        state: this.overtaking ? "OVERTAKE" : "FOLLOW_LANE",
        reason: this.overtaking
          ? "Passing observed slow vehicle in adjacent eastbound lane"
          : "No slower lead in driving lane",
        candidates: [],
        selected: null,
        targetSpeed: speed,
      });
    }
    const target = wantReturn
      ? "east-outer"
      : lane === "east-outer"
        ? "east-inner"
        : null;
    if (!target)
      return (this.value = {
        state: "FOLLOW_VEHICLE",
        reason: "No additional same-direction lane available",
        candidates: [],
        selected: null,
        targetSpeed: Math.max(0, lead?.vx ?? 0),
      });
    // A longitudinal vehicle speed is needed to execute a smooth profile. At rest,
    // retain ACC/stop rather than discontinuously assuming a moving vehicle.
    if (vehicle.velocity < 2)
      return (this.value = {
        state: "STOP_FOR_OBSTACLE",
        reason: "Insufficient forward speed for bounded lane-change profile",
        candidates: [],
        selected: null,
        targetSpeed: 0,
      });
    const candidates = [5, 6, 7].map((duration) =>
      validateCandidate(
        laneChangeCandidate(
          vehicle,
          target,
          Math.min(speed, vehicle.velocity),
          duration,
        ),
        relevant,
        now,
        options.staticObjects,
        destination,
      ),
    );
    const selected = candidates.find((c) => c.valid);
    if (!selected) {
      this.clearSince = null;
      return (this.value = {
        state: "STOP_FOR_OBSTACLE",
        reason: "All same-direction alternatives rejected",
        candidates,
        selected: null,
        targetSpeed: 0,
      });
    }
    if (this.clearSince === null) this.clearSince = now;
    if (now - this.clearSince < 0.5 || now - this.lastChange < 2)
      return (this.value = {
        state: "PREPARE_LANE_CHANGE",
        reason:
          "Require 0.5s continuously feasible gap and 2s manoeuvre cooldown",
        candidates,
        selected: null,
        targetSpeed: Math.min(speed, vehicle.velocity),
      });
    this.active = selected;
    this.started = now;
    this.clearSince = null;
    return (this.value = {
      state: wantReturn ? "RETURN_TO_LANE" : "EXECUTE_LANE_CHANGE",
      reason:
        "Validated road-contained candidate and predicted uncertainty envelope",
      candidates,
      selected,
      targetSpeed: Math.min(speed, vehicle.velocity),
    });
  }
  /** Steering controller tracks the selected geometry through the same bicycle
   * model. Caller combines acceleration with ACC/AEB by choosing stricter brake. */
  control(
    vehicle: VehicleState,
    now: number,
    dt: number,
    previousSteering = 0,
  ): VehicleControl {
    void now;
    let target;
    if (this.active) {
      const points = this.active.points;
      let nearest = 0;
      for (let i = 1; i < points.length; i++)
        if (
          Math.hypot(points[i].x - vehicle.x, points[i].y - vehicle.y) <
          Math.hypot(
            points[nearest].x - vehicle.x,
            points[nearest].y - vehicle.y,
          )
        )
          nearest = i;
      const lookahead = 1.2 + 0.15 * vehicle.velocity;
      target =
        points
          .slice(nearest)
          .find(
            (p) => Math.hypot(p.x - vehicle.x, p.y - vehicle.y) >= lookahead,
          ) ?? points[points.length - 1];
    }
    const aim = target ?? {
      x: vehicle.x + 8,
      y: laneY(corridorLane(vehicle.y)),
    };
    const dx = aim.x - vehicle.x,
      dy = aim.y - vehicle.y;
    const lateral =
      -Math.sin(vehicle.heading) * dx + Math.cos(vehicle.heading) * dy;
    const desired = Math.atan2(
      2 * VEHICLE.wheelbase * lateral,
      Math.max(dx * dx + dy * dy, 0.1),
    );
    const steering = clamp(
      desired,
      Math.max(-VEHICLE.maxSteering, previousSteering - (dt * Math.PI) / 3),
      Math.min(VEHICLE.maxSteering, previousSteering + (dt * Math.PI) / 3),
    );
    return {
      steering,
      acceleration: clamp(
        1.5 * (this.value.targetSpeed - vehicle.velocity),
        -VEHICLE.maxBraking,
        VEHICLE.maxAcceleration,
      ),
    };
  }
}
