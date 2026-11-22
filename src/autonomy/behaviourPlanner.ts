import type { Point } from "../navigation/types";
import type {
  Detection,
  DetectionFrame,
  SensorSnapshot,
} from "../sensors/types";
import type { VehicleControl, VehicleState } from "../simulation/types";
import { VEHICLE, clamp } from "../simulation/vehicle";
import type {
  BehaviourContext,
  BehaviourSnapshot,
  DrivingState,
} from "./behaviourTypes";

export const BEHAVIOUR_LIMITS = Object.freeze({
  maximumAge: 0.3,
  clearHold: 0.5,
  jerk: 3,
  stopBuffer: 0.8,
  comfortableBraking: 2.5,
  maximumCruise: 6,
  lateralMargin: 0.2,
});
const FRONT = (VEHICLE.length + VEHICLE.wheelbase) / 2;
const HALF_WIDTH = VEHICLE.width / 2 + BEHAVIOUR_LIMITS.lateralMargin;
interface ObservedPoint extends Point {
  world: Point;
  source: "lidar" | "camera" | "radar";
  timestamp: number;
  objectClass?: Detection["objectClass"];
  relativeSpeed: number | null;
  signal?: Detection["signal"];
}
interface Projection {
  gap: number;
  lateral: number;
}
interface Hazard {
  state: DrivingState;
  reason: string;
  gap: number | null;
  timestamp: number | null;
  emergency: boolean;
  source: "lidar" | "camera" | "radar" | null;
}
const fresh = (timestamp: number, now: number) =>
  Number.isFinite(timestamp) &&
  now - timestamp >= -1e-9 &&
  now - timestamp <= BEHAVIOUR_LIMITS.maximumAge + 1e-9;
const localPoint = (point: Point, vehicle: VehicleState): Point => {
  const dx = point.x - vehicle.x,
    dy = point.y - vehicle.y;
  const c = Math.cos(vehicle.heading),
    s = Math.sin(vehicle.heading);
  return { x: c * dx + s * dy, y: -s * dx + c * dy };
};
function observedFrame(
  frame: DetectionFrame | null,
  vehicle: VehicleState,
  now: number,
): ObservedPoint[] {
  if (!frame || !fresh(frame.timestamp, now)) return [];
  const c = Math.cos(frame.pose.heading),
    s = Math.sin(frame.pose.heading);
  return frame.detections
    .filter((d) => fresh(d.timestamp, now))
    .map((d) => {
      const bearingWorld = frame.pose.heading + d.bearing;
      // Only radial velocity is observed. A radial constant-velocity projection is an
      // explicit short-age approximation, not access to the actor's hidden velocity.
      const radial = d.radialVelocity;
      const objectRadial =
        radial === undefined
          ? 0
          : radial +
            vehicle.velocity * Math.cos(vehicle.heading - bearingWorld);
      const age = Math.max(0, now - d.timestamp);
      const world = {
        x:
          frame.pose.x +
          c * d.relativePosition.x -
          s * d.relativePosition.y +
          Math.cos(bearingWorld) * objectRadial * age,
        y:
          frame.pose.y +
          s * d.relativePosition.x +
          c * d.relativePosition.y +
          Math.sin(bearingWorld) * objectRadial * age,
      };
      const angle = bearingWorld - vehicle.heading;
      return {
        ...localPoint(world, vehicle),
        world,
        source: d.source,
        timestamp: d.timestamp,
        objectClass: d.objectClass,
        signal: d.signal,
        relativeSpeed:
          radial === undefined
            ? null
            : radial / Math.max(0.35, Math.cos(angle)),
      };
    });
}

/** Project only the immediate route neighbourhood; unrelated later route visits are excluded. */
function pathProjection(
  point: ObservedPoint,
  vehicle: VehicleState,
  steering: number,
  context: BehaviourContext,
): Projection {
  const route = context.route;
  if (route && route.points.length > 1) {
    const progress = clamp(context.progress, 0, 1) * route.length;
    let nearest = Infinity,
      result = { gap: point.x - FRONT, lateral: point.y };
    for (let i = 1; i < route.points.length; i++) {
      const a = route.points[i - 1],
        b = route.points[i];
      if (b.s < progress - 1 || a.s > progress + 45) continue;
      const dx = b.x - a.x,
        dy = b.y - a.y,
        square = dx * dx + dy * dy;
      if (square < 1e-12) continue;
      const fraction = clamp(
        ((point.world.x - a.x) * dx + (point.world.y - a.y) * dy) / square,
        0,
        i === route.points.length - 1 ? Number.POSITIVE_INFINITY : 1,
      );
      const px = a.x + fraction * dx,
        py = a.y + fraction * dy;
      const distance = Math.hypot(point.world.x - px, point.world.y - py);
      if (distance < nearest) {
        nearest = distance;
        const cross = dx * (point.world.y - py) - dy * (point.world.x - px);
        result = {
          gap: a.s + fraction * (b.s - a.s) - progress - FRONT,
          lateral: (Math.sign(cross) || 1) * distance,
        };
      }
    }
    return result;
  }
  const curvature =
    Math.tan(clamp(steering, -VEHICLE.maxSteering, VEHICLE.maxSteering)) /
    VEHICLE.wheelbase;
  if (Math.abs(curvature) < 1e-6)
    return { gap: point.x - FRONT, lateral: point.y };
  let nearest = Infinity,
    result = { gap: point.x - FRONT, lateral: point.y };
  for (let distance = 0; distance <= 45; distance += 0.25) {
    const angle = curvature * distance;
    const x = Math.sin(angle) / curvature,
      y = (1 - Math.cos(angle)) / curvature;
    const error = Math.hypot(point.x - x, point.y - y);
    if (error < nearest) {
      nearest = error;
      result = { gap: distance - FRONT, lateral: error };
    }
  }
  return result;
}
function signalKey(signal: NonNullable<Detection["signal"]>): string {
  return signal.stopLine
    .map((p) => `${p.x.toFixed(2)},${p.y.toFixed(2)}`)
    .join(";");
}
function signalGap(
  signal: NonNullable<Detection["signal"]>,
  vehicle: VehicleState,
): number | null {
  if (Math.cos(signal.heading - vehicle.heading) < 0.8) return null;
  const [a, b] = signal.stopLine;
  const ux = Math.cos(vehicle.heading),
    uy = Math.sin(vehicle.heading);
  const vx = b.x - a.x,
    vy = b.y - a.y;
  const denominator = ux * vy - uy * vx;
  if (Math.abs(denominator) < 1e-9) return null;
  const dx = a.x - vehicle.x,
    dy = a.y - vehicle.y;
  const distance = (dx * vy - dy * vx) / denominator;
  const across = (dx * uy - dy * ux) / denominator;
  return across >= -0.05 && across <= 1.05 ? distance - FRONT : null;
}

/** Measurement-only ACC, observed signal/pedestrian response and automatically releasable AEB. */
export class BehaviourPlanner {
  private readonly defaultTimeGap: number;
  private readonly dMin: number;
  private previousTime: number | null = null;
  private previousAcceleration = 0;
  private hold: Hazard | null = null;
  private clearSince: number | null = null;
  private pendingSignal: {
    signal: NonNullable<Detection["signal"]>;
    timestamp: number;
  } | null = null;
  private yellowCommit: string | null = null;
  private data: BehaviourSnapshot;

  constructor(options: { timeGap?: number; dMin?: number } = {}) {
    this.defaultTimeGap = options.timeGap ?? 1.5;
    this.dMin = options.dMin ?? 3;
    if (
      ![this.defaultTimeGap, this.dMin].every(
        (v) => Number.isFinite(v) && v > 0,
      )
    )
      throw new RangeError("Invalid following gap configuration");
    this.data = this.empty();
  }
  private empty(): BehaviourSnapshot {
    return {
      state: "MANUAL",
      reason: "Manual driving; behaviour control is inactive.",
      leadDistance: null,
      relativeSpeed: null,
      desiredGap: this.dMin,
      ttc: null,
      stoppingDistance: 0,
      aeb: false,
      targetSpeed: 0,
      acceleration: 0,
      braking: 0,
      steering: 0,
      observationTime: null,
      timeGap: this.defaultTimeGap,
    };
  }
  get snapshot(): BehaviourSnapshot {
    return { ...this.data };
  }
  reset(): void {
    this.previousTime = null;
    this.previousAcceleration = 0;
    this.hold = null;
    this.clearSince = null;
    this.pendingSignal = null;
    this.yellowCommit = null;
    this.data = this.empty();
  }

  update(
    vehicle: VehicleState,
    requested: VehicleControl,
    sensors: SensorSnapshot,
    now: number,
    autonomous: boolean,
    context: BehaviourContext,
  ): VehicleControl {
    if (
      !Number.isFinite(now) ||
      (this.previousTime !== null && now < this.previousTime - 1e-9)
    )
      throw new RangeError(
        "Behaviour time must be finite and monotonic; reset before replay",
      );
    const dt =
      this.previousTime === null
        ? 0.01
        : clamp(now - this.previousTime, 0, 0.1);
    this.previousTime = now;
    const timeGap =
      Number.isFinite(context.timeGap) && context.timeGap > 0
        ? clamp(context.timeGap, 0.5, 4)
        : this.defaultTimeGap;
    const speed = Math.max(0, vehicle.velocity),
      desiredGap = this.dMin + timeGap * speed;
    const stoppingDistance =
      (speed * speed) / (2 * VEHICLE.maxBraking) + speed * 0.3 + 1;
    if (!autonomous) {
      this.hold = null;
      this.clearSince = null;
      this.pendingSignal = null;
      this.yellowCommit = null;
      this.previousAcceleration = requested.acceleration;
      this.data = {
        ...this.empty(),
        desiredGap,
        timeGap,
        acceleration: requested.acceleration,
        braking: Math.max(0, -requested.acceleration),
        steering: requested.steering,
        targetSpeed: Math.max(0, context.targetSpeed ?? speed),
      };
      return { ...requested };
    }
    const lidar = sensors.lidar;
    const camera = observedFrame(sensors.camera, vehicle, now),
      radar = observedFrame(sensors.radar, vehicle, now);
    // Radar has range/range-rate but no semantic classifier. Associate nearby
    // measured camera points, never hidden object IDs, for a vehicle/pedestrian label.
    for (const observation of radar) {
      const semantic = camera
        .filter(
          (p) => p.objectClass === "vehicle" || p.objectClass === "pedestrian",
        )
        .map((p) => ({
          point: p,
          distance: Math.hypot(p.x - observation.x, p.y - observation.y),
        }))
        .filter((p) => p.distance <= 2.5)
        .sort((a, b) => a.distance - b.distance)[0];
      if (semantic) observation.objectClass = semantic.point.objectClass;
    }
    const points: ObservedPoint[] = [...camera, ...radar];
    if (lidar && fresh(lidar.timestamp, now))
      for (const ray of lidar.rays)
        if (ray.hit)
          points.push({
            ...localPoint(ray.point, vehicle),
            world: ray.point,
            source: "lidar",
            timestamp: lidar.timestamp,
            relativeSpeed: null,
          });
    const projected = points
      .map((point) => ({
        point,
        projection: pathProjection(point, vehicle, requested.steering, context),
      }))
      .filter(
        (item) =>
          Number.isFinite(item.projection.gap) &&
          item.point.x >= -1 &&
          item.projection.gap >= -FRONT - 1,
      );
    const vehicles = projected.filter(
      (item) =>
        item.point.objectClass === "vehicle" &&
        Math.abs(item.projection.lateral) <= HALF_WIDTH + 0.2,
    );
    // Prefer range-rate observations; no simulator identity or exact actor state is used.
    const lead =
      vehicles
        .filter((item) => item.point.source === "radar")
        .sort((a, b) => a.projection.gap - b.projection.gap)[0] ??
      vehicles.sort((a, b) => a.projection.gap - b.projection.gap)[0];
    const gap = lead ? Math.max(0, lead.projection.gap) : null;
    const relativeSpeed = lead ? (lead.point.relativeSpeed ?? -speed) : null;
    const closing = Math.max(0, -(relativeSpeed ?? 0));
    const ttc = gap !== null && closing > 0.05 ? gap / closing : null;
    let hazard: Hazard | null = null;
    if (!lidar || !fresh(lidar.timestamp, now) || lidar.rays.length === 0)
      hazard = {
        state: "STALE_STOP",
        reason:
          "Primary LiDAR missing or older than 0.30 s; braking until fresh observations remain clear.",
        gap: null,
        timestamp: lidar?.timestamp ?? null,
        emergency: true,
        source: "lidar",
      };
    if (!hazard) {
      for (const item of projected) {
        if (Math.abs(item.projection.lateral) > HALF_WIDTH) continue;
        const nearLead =
          lead &&
          Math.hypot(item.point.x - lead.point.x, item.point.y - lead.point.y) <
            5;
        const rate =
          item.point.relativeSpeed ??
          (nearLead ? (relativeSpeed ?? -speed) : -speed);
        const approach = Math.max(0, -rate);
        const delay = Math.max(0, now - item.point.timestamp) + 0.2;
        const emergencyDistance =
          (approach * approach) / (2 * VEHICLE.maxBraking) +
          approach * delay +
          0.7;
        const contactTime =
          approach > 0.05
            ? Math.max(0, item.projection.gap) / approach
            : Infinity;
        if (
          item.projection.gap < 0.5 ||
          (approach > 0.05 &&
            (item.projection.gap <= emergencyDistance || contactTime < 1))
        ) {
          hazard = {
            state: "AEB",
            reason: `${item.point.source.toUpperCase()} measured immediate corridor clearance ${item.projection.gap.toFixed(2)} m, closing ${approach.toFixed(2)} m/s; protective braking.`,
            gap: Math.max(0, item.projection.gap),
            timestamp: item.point.timestamp,
            emergency: true,
            source: item.point.source,
          };
          break;
        }
      }
    }
    if (!hazard) {
      const pedestrian = projected
        .filter(
          (item) =>
            item.point.objectClass === "pedestrian" &&
            Math.abs(item.projection.lateral) <= 4.5 &&
            item.projection.gap <= Math.max(20, speed * 3 + 10),
        )
        .sort((a, b) => a.projection.gap - b.projection.gap)[0];
      if (pedestrian)
        hazard = {
          state: "YIELD_PEDESTRIAN",
          reason:
            "Fresh pedestrian observation near the planned crossing corridor; yielding without hidden motion prediction.",
          gap: Math.max(0, pedestrian.projection.gap),
          timestamp: pedestrian.point.timestamp,
          emergency: false,
          source: pedestrian.point.source,
        };
    }
    if (!hazard) {
      let currentSignal: ObservedPoint | null = null,
        currentGap: number | null = null;
      for (const observation of camera) {
        if (!observation.signal) continue;
        const distance = signalGap(observation.signal, vehicle);
        if (
          distance !== null &&
          distance >= -0.5 &&
          distance < 45 &&
          (currentGap === null || distance < currentGap)
        ) {
          currentSignal = observation;
          currentGap = distance;
        }
      }
      if (currentSignal?.signal && currentGap !== null) {
        const signal = currentSignal.signal,
          key = signalKey(signal);
        if (signal.color === "green") this.pendingSignal = null;
        else if (this.yellowCommit !== key) {
          const comfortableDistance =
            (speed * speed) / (2 * BEHAVIOUR_LIMITS.comfortableBraking) +
            speed * 0.7 +
            BEHAVIOUR_LIMITS.stopBuffer;
          if (signal.color === "yellow" && currentGap < comfortableDistance)
            this.yellowCommit = key;
          else
            this.pendingSignal = {
              signal: {
                ...signal,
                stopLine: [
                  { ...signal.stopLine[0] },
                  { ...signal.stopLine[1] },
                ],
              },
              timestamp: currentSignal.timestamp,
            };
        }
      }
      if (this.pendingSignal) {
        const distance = signalGap(this.pendingSignal.signal, vehicle);
        if (distance !== null && distance >= -0.5)
          hazard = {
            state: "STOP_LIGHT",
            reason: currentSignal
              ? `Observed ${this.pendingSignal.signal.color} signal on this approach; stop before the measured stop line.`
              : "Last observed stop signal is no longer visible; hold until a fresh green observation.",
            gap: Math.max(0, distance),
            timestamp: this.pendingSignal.timestamp,
            emergency: false,
            source: "camera",
          };
        else this.pendingSignal = null;
      }
    }
    if (!hazard) {
      const blocker = projected
        .filter(
          (item) =>
            item.point.objectClass !== "vehicle" &&
            item.point.objectClass !== "pedestrian" &&
            item.point.objectClass !== "traffic-light" &&
            item.point.source !== "lidar" &&
            Math.abs(item.projection.lateral) <= HALF_WIDTH &&
            item.projection.gap < 35,
        )
        .sort((a, b) => a.projection.gap - b.projection.gap)[0];
      if (blocker)
        hazard = {
          state: "BLOCKED",
          reason:
            "Fresh geometric obstacle observation occupies the planned corridor; stopping without rerouting.",
          gap: Math.max(0, blocker.projection.gap),
          timestamp: blocker.point.timestamp,
          emergency: false,
          source: blocker.point.source,
        };
    }
    if (hazard) {
      this.hold = hazard;
      this.clearSince = null;
    } else if (this.hold) {
      this.clearSince ??= now;
      if (now - this.clearSince >= BEHAVIOUR_LIMITS.clearHold - 1e-9) {
        this.hold = null;
        this.clearSince = null;
      }
    }
    const active = hazard ?? this.hold;
    const navTarget = clamp(
      context.targetSpeed ?? speed + requested.acceleration / 2.5,
      0,
      BEHAVIOUR_LIMITS.maximumCruise,
    );
    let state: DrivingState =
      active?.state ??
      (context.destinationReached
        ? "DESTINATION_REACHED"
        : lead
          ? "FOLLOW_LEAD"
          : "CRUISE");
    let reason =
      active?.reason ??
      (context.destinationReached
        ? "Navigation reports destination reached."
        : lead
          ? "Fresh lead-vehicle range/range-rate regulate the configured time gap."
          : "No relevant sensed hazard; following the navigation command.");
    let target = navTarget,
      acceleration = requested.acceleration;
    if (active) {
      if (!hazard) {
        target = 0;
        acceleration = -VEHICLE.maxBraking;
        reason += " Waiting 0.50 s of clear observations before resuming.";
      } else if (active.emergency) {
        target = 0;
        acceleration = -VEHICLE.maxBraking;
      } else {
        const distance = Math.max(
          0,
          (active.gap ?? 0) - BEHAVIOUR_LIMITS.stopBuffer,
        );
        const anticipation =
          speed *
          (1.2 +
            Math.max(0, this.previousAcceleration) / BEHAVIOUR_LIMITS.jerk);
        target = Math.min(
          navTarget,
          Math.sqrt(
            2 *
              BEHAVIOUR_LIMITS.comfortableBraking *
              Math.max(0, distance - anticipation),
          ),
        );
        const feedforward = (speed * speed) / (2 * Math.max(0.5, distance));
        acceleration = Math.min(
          requested.acceleration,
          1.8 * (target - speed) - feedforward,
        );
        if (distance <= 1.5 && speed <= 1.5) {
          target = 0;
          acceleration = -VEHICLE.maxBraking;
        }
      }
    } else if (context.destinationReached) {
      target = 0;
      acceleration = -VEHICLE.maxBraking;
    } else if (lead && gap !== null && relativeSpeed !== null) {
      const leadSpeed = Math.max(0, speed + relativeSpeed);
      target = Math.min(
        navTarget,
        Math.max(0, leadSpeed + 0.35 * (gap - desiredGap)),
      );
      if (leadSpeed < 0.1 && gap <= this.dMin + 0.5) target = 0;
      acceleration = Math.min(
        requested.acceleration,
        1.4 * (target - speed) + 0.4 * relativeSpeed,
      );
    } else if (context.destinationReached) {
      target = 0;
      acceleration = -VEHICLE.maxBraking;
    }
    // A traffic-control reason does not remove the nearer lead constraint.
    // Combine normal longitudinal limits, then let AEB/stale handling dominate.
    if (
      active &&
      !active.emergency &&
      lead &&
      gap !== null &&
      relativeSpeed !== null &&
      !context.destinationReached
    ) {
      const leadSpeed = Math.max(0, speed + relativeSpeed);
      let leadTarget = Math.max(0, leadSpeed + 0.35 * (gap - desiredGap));
      if (leadSpeed < 0.1 && gap <= this.dMin + 0.5) leadTarget = 0;
      target = Math.min(target, leadTarget);
      acceleration = Math.min(
        acceleration,
        1.4 * (leadTarget - speed) + 0.4 * relativeSpeed,
      );
    }
    const emergency =
      active?.emergency === true || (active !== null && !hazard);
    acceleration = clamp(
      acceleration,
      -VEHICLE.maxBraking,
      VEHICLE.maxAcceleration,
    );
    if (
      !emergency &&
      acceleration > 0 &&
      this.previousAcceleration > 0 &&
      speed + this.previousAcceleration ** 2 / (2 * BEHAVIOUR_LIMITS.jerk) >=
        target - 0.015
    )
      acceleration = 0;
    if (!emergency)
      acceleration = clamp(
        acceleration,
        this.previousAcceleration - BEHAVIOUR_LIMITS.jerk * dt,
        this.previousAcceleration + BEHAVIOUR_LIMITS.jerk * dt,
      );
    // Geometry/destination braking has priority over comfort smoothing. Delaying
    // the navigation request can overshoot a stop or enter a turn too quickly.
    if (requested.acceleration <= 0)
      acceleration = Math.min(acceleration, requested.acceleration);
    if (vehicle.velocity >= 0 && acceleration < 0 && dt > 0)
      acceleration = Math.max(acceleration, -vehicle.velocity / dt);
    if (speed === 0 && acceleration < 0) acceleration = 0;
    if (Math.abs(acceleration) < 1e-12) acceleration = 0;
    this.previousAcceleration = acceleration;
    this.data = {
      state,
      reason,
      leadDistance: gap,
      relativeSpeed,
      desiredGap,
      ttc,
      stoppingDistance,
      aeb: state === "AEB",
      targetSpeed: target,
      acceleration,
      braking: Math.max(0, -acceleration),
      steering: requested.steering,
      observationTime:
        active?.timestamp ?? lead?.point.timestamp ?? lidar?.timestamp ?? null,
      source: active?.source ?? lead?.point.source ?? "lidar",
      timeGap,
    };
    return { steering: requested.steering, acceleration };
  }
}
