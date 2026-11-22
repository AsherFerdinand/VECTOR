import type { VehicleControl, VehicleState } from "../simulation/types";
import { integrateVehicle, clamp } from "../simulation/vehicle";
import { vehicleFootprint, polygonsOverlap } from "../simulation/collision";
import { resolveDynamicMotion } from "../simulation/dynamicCollision";
import { SensorSuite, DEFAULT_SENSOR_CONFIGS } from "../sensors/suite";
import { MultiObjectTracker } from "../perception/tracker";
import type { TrackingSnapshot } from "../perception/types";
import type { WorldObject } from "../world/types";
import { BehaviourPlanner } from "../autonomy/behaviourPlanner";
import type { BehaviourSnapshot } from "../autonomy/behaviourTypes";
import { ManoeuvrePlanner } from "./manoeuvrePlanner";
import { corridorContains, corridorRoute, laneY } from "./corridor";
import type { ManoeuvreSnapshot } from "./types";
import type { Route } from "../navigation/types";
export interface CorridorMetrics {
  collisions: number;
  roadDepartures: number;
  minimumGap: number;
  maximumLateralAcceleration: number;
  maximumDeceleration: number;
  laneChanges: number;
  completed: boolean;
  safeStops: number;
}
export interface CorridorFrame {
  time: number;
  vehicle: VehicleState;
  actor: VehicleState;
  control: VehicleControl;
  tracking: TrackingSnapshot;
  manoeuvre: ManoeuvreSnapshot;
  behaviour: BehaviourSnapshot;
}
export interface CorridorOptions {
  scenario?: "lane-change" | "overtaking" | "blocked" | "cut-in";
  seed?: number;
  noise?: number;
}
/** Standalone explicitly mapped-road validation fixture. Same physics and sensors
 * as Drive; actor truth is used only in generators, collision and recorded debug. */
export class CorridorSimulation {
  private vehicle: VehicleState = {
    x: -45,
    y: laneY("east-outer"),
    heading: 0,
    velocity: 4,
  };
  private actor: VehicleState = {
    x: -10,
    y: laneY("east-outer"),
    heading: 0,
    velocity: 2,
  };
  private actorVX = 2;
  private actorVY = 0;
  private actorHalted = false;
  private time = 0;
  private objects: WorldObject[] = [];
  private sensors: SensorSuite;
  private tracker = new MultiObjectTracker();
  private tracking: TrackingSnapshot = { timestamp: 0, tracks: [] };
  private planner = new ManoeuvrePlanner();
  private behaviour = new BehaviourPlanner();
  private control: VehicleControl = { steering: 0, acceleration: 0 };
  private frames: CorridorFrame[] = [];
  private metrics: CorridorMetrics = {
    collisions: 0,
    roadDepartures: 0,
    minimumGap: Infinity,
    maximumLateralAcceleration: 0,
    maximumDeceleration: 0,
    laneChanges: 0,
    completed: false,
    safeStops: 0,
  };
  private collisionActive = false;
  private departureActive = false;
  private previousLane = "east-outer";
  private previousState = "FOLLOW_LANE";
  constructor(private options: CorridorOptions = {}) {
    if (options.scenario === "blocked") {
      this.actor.velocity = 0;
      this.actorVX = 0;
    }
    if (options.scenario === "cut-in") this.actor.y = laneY("east-inner");
    this.objects.push(this.actorObject());
    if (options.scenario === "blocked")
      this.objects.push({
        id: "mapped-adjacent-block",
        objectClass: "static-obstacle",
        polygon: [
          { x: -13, y: -88.4 },
          { x: -7, y: -88.4 },
          { x: -7, y: -85.1 },
          { x: -13, y: -85.1 },
        ],
        height: 1.5,
        velocity: { x: 0, y: 0 },
        collidable: true,
      });
    this.sensors = new SensorSuite(
      this.objects,
      DEFAULT_SENSOR_CONFIGS.map((c) => ({
        ...c,
        seed: (options.seed ?? 1) + c.seed,
        noise: options.noise ?? 0,
      })),
    );
    this.sample();
  }
  private actorObject(): WorldObject {
    return {
      id: "corridor-scripted-lead",
      objectClass: "vehicle",
      polygon: vehicleFootprint(this.actor),
      height: 1.5,
      velocity: { x: this.actorVX, y: this.actorVY },
      collidable: true,
    };
  }
  private sample(): void {
    this.sensors.update(
      this.time,
      this.vehicle,
      (this.vehicle.velocity / 2.7) * Math.tan(this.control.steering),
    );
    this.tracking = this.tracker.update(
      this.sensors.snapshot,
      this.vehicle,
      this.time,
    );
  }
  get snapshot(): CorridorFrame {
    return {
      time: this.time,
      vehicle: { ...this.vehicle },
      actor: { ...this.actor },
      control: { ...this.control },
      tracking: this.tracking,
      manoeuvre: this.planner.snapshot,
      behaviour: this.behaviour.snapshot,
    };
  }
  get results(): CorridorMetrics {
    return { ...this.metrics };
  }
  get recording(): readonly CorridorFrame[] {
    return this.frames;
  }
  step(dt = 0.01): CorridorFrame {
    if (Math.abs(dt - 0.01) > 1e-9)
      throw new RangeError("Corridor simulation uses fixed 0.01s steps");
    if (this.metrics.completed) return this.snapshot;
    const oldObjects = this.objects.map((o) => ({
      ...o,
      polygon: o.polygon.map((p) => ({ ...p })),
    }));
    const tracks = this.tracking.tracks;
    const plan = this.planner.update(this.vehicle, tracks, this.time, {
      desiredSpeed: 6,
      destinationX: 150,
      staticObjects: this.objects.filter(
        (o) => o.objectClass === "static-obstacle",
      ),
      returnToLane: this.options.scenario !== "lane-change",
    });
    const requested = this.planner.control(
      this.vehicle,
      this.time,
      dt,
      this.control.steering,
    );
    let route: Route = corridorRoute(this.vehicle, 150);
    if (plan.selected && plan.selected.points.length > 1) {
      const points = plan.selected.points;
      let s = 0;
      route = {
        ...route,
        points: points.map((p, i) => {
          if (i) s += Math.hypot(p.x - points[i - 1].x, p.y - points[i - 1].y);
          return { x: p.x, y: p.y, curvature: p.curvature, s };
        }),
        length: 0,
      };
      route.length = s;
    }
    const destinationLane =
      this.options.scenario === "lane-change" ? "east-inner" : "east-outer";
    const destinationReached =
      this.vehicle.x >= 145 &&
      Math.abs(this.vehicle.y - laneY(destinationLane)) < 0.35;
    const stoppingTarget = Math.min(
      plan.targetSpeed,
      Math.sqrt(2 * 2 * Math.max(0, 148 - this.vehicle.x)),
    );
    const controlled = this.behaviour.update(
      this.vehicle,
      {
        ...requested,
        acceleration: Math.min(
          requested.acceleration,
          1.5 * (stoppingTarget - this.vehicle.velocity),
        ),
      },
      this.sensors.snapshot,
      this.time,
      true,
      {
        route,
        progress: 0,
        timeGap: 1.5,
        targetSpeed: stoppingTarget,
        tracks,
        destinationReached,
      },
    );
    controlled.acceleration = clamp(controlled.acceleration, -6, 3);
    const ayLimitSteering = Math.atan(
      (1.5 * 2.7) / Math.max(this.vehicle.velocity ** 2, 0.1),
    );
    controlled.steering = clamp(
      controlled.steering,
      -ayLimitSteering,
      ayLimitSteering,
    );
    // No hidden actor geometry enters planner/control. Collision resolves afterward.
    const proposed = integrateVehicle(this.vehicle, controlled, dt);
    const oldActor = { ...this.actor };
    const actorAcceleration = this.actor.x > 165 ? -2 : 0;
    this.actor = {
      ...this.actor,
      x: this.actor.x + this.actorVX * dt,
      velocity: Math.max(0, this.actorVX + actorAcceleration * dt),
    };
    this.actorVX = this.actor.velocity;
    if (this.options.scenario === "cut-in" && !this.actorHalted) {
      const u = clamp((this.time + dt - 2) / 6, 0, 1),
        dy = -3.5;
      this.actor.y =
        laneY("east-inner") + dy * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5);
      this.actorVY = (dy * (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4)) / 6;
      this.actor.heading = Math.atan2(this.actorVY, this.actorVX);
      this.actor.velocity = Math.hypot(this.actorVX, this.actorVY);
    }
    this.objects[0] = this.actorObject();
    const resolved = resolveDynamicMotion(
      this.vehicle,
      proposed,
      oldObjects,
      this.objects,
    );
    if (resolved.collided) {
      if (!this.collisionActive) this.metrics.collisions++;
      if (resolved.objectId === "corridor-scripted-lead") {
        this.actor = {
          ...oldActor,
          x: oldActor.x + (this.actor.x - oldActor.x) * resolved.fraction,
          y: oldActor.y + (this.actor.y - oldActor.y) * resolved.fraction,
          heading:
            oldActor.heading +
            (this.actor.heading - oldActor.heading) * resolved.fraction,
          velocity: 0,
        };
        this.actorVX = 0;
        this.actorVY = 0;
        this.actorHalted = true;
        this.objects[0] = this.actorObject();
      }
    }
    this.collisionActive = resolved.collided;
    const boundaryTravel =
      Math.hypot(proposed.x - this.vehicle.x, proposed.y - this.vehicle.y) +
      4 * Math.abs(proposed.heading - this.vehicle.heading);
    const departed =
      !corridorContains(this.vehicle, boundaryTravel / 2) ||
      !corridorContains(proposed, boundaryTravel / 2);
    if (departed && !this.departureActive) this.metrics.roadDepartures++;
    this.departureActive = departed;
    const actualDeceleration =
      (this.vehicle.velocity - resolved.state.velocity) / dt;
    this.vehicle = resolved.state;
    this.control = controlled;
    this.time = Number((this.time + dt).toFixed(8));
    this.sample();
    this.metrics.minimumGap = Math.min(
      this.metrics.minimumGap,
      polygonDistance(
        vehicleFootprint(this.vehicle),
        vehicleFootprint(this.actor),
      ),
    );
    this.metrics.maximumLateralAcceleration = Math.max(
      this.metrics.maximumLateralAcceleration,
      Math.abs(
        (this.vehicle.velocity ** 2 * Math.tan(controlled.steering)) / 2.7,
      ),
    );
    this.metrics.maximumDeceleration = Math.max(
      this.metrics.maximumDeceleration,
      actualDeceleration,
    );
    if (
      Math.abs(this.vehicle.y - laneY("east-inner")) < 0.25 &&
      this.previousLane === "east-outer"
    ) {
      this.metrics.laneChanges++;
      this.previousLane = "east-inner";
    }
    if (
      Math.abs(this.vehicle.y - laneY("east-outer")) < 0.25 &&
      this.previousLane === "east-inner"
    ) {
      this.metrics.laneChanges++;
      this.previousLane = "east-outer";
    }
    if (plan.state === "SAFE_STOP" && this.previousState !== "SAFE_STOP")
      this.metrics.safeStops++;
    this.previousState = plan.state;
    this.metrics.completed =
      destinationReached && this.vehicle.velocity <= 0.15;
    if (Math.round(this.time * 100) % 10 === 0 && this.frames.length < 1200)
      this.frames.push(this.snapshot);
    return this.snapshot;
  }
  run(duration = 60): CorridorMetrics {
    for (
      let i = 0;
      i < Math.min(duration, 120) * 100 && !this.metrics.completed;
      i++
    )
      this.step();
    return this.results;
  }
}

/** Exact convex polygon boundary distance for debug/validation metrics only. */
function polygonDistance(
  a: { x: number; y: number }[],
  b: { x: number; y: number }[],
): number {
  if (polygonsOverlap(a, b)) return 0;
  let distance = Infinity;
  for (const [vertices, edges] of [
    [a, b],
    [b, a],
  ])
    for (const p of vertices)
      for (let i = 0; i < edges.length; i++) {
        const u = edges[i],
          v = edges[(i + 1) % edges.length],
          dx = v.x - u.x,
          dy = v.y - u.y;
        const t = clamp(
          ((p.x - u.x) * dx + (p.y - u.y) * dy) / (dx * dx + dy * dy),
          0,
          1,
        );
        distance = Math.min(
          distance,
          Math.hypot(p.x - u.x - t * dx, p.y - u.y - t * dy),
        );
      }
  return distance;
}
