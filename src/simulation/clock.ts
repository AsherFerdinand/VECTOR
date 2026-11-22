import type { DriverInput, SimulationSnapshot, VehicleState } from "./types";
import { driverControl } from "./controller";
import { INITIAL_STATE, integrateVehicle, VEHICLE } from "./vehicle";
import { createRoadGraph, projectToRoad } from "../navigation/graph";
import { isRoadPosition } from "../navigation/route";
import type { Point, RoadPosition, Route } from "../navigation/types";
import { NavigationController } from "../autonomy/navigationController";
import type { NavigationSnapshot, NavigationState } from "../autonomy/types";

import { worldObjects } from "../world/objects";
import type { WorldObject } from "../world/types";
import {
  vehicleFootprint,
  polygonsOverlap,
  resolveMotion,
  classifyRoad,
  classifyRoadMotion,
} from "./collision";
import type {
  PhysicalSnapshot,
  CollisionEvent,
  RoadDepartureEvent,
  RoadStatus,
  ClearanceResult,
} from "./collisionTypes";
import { SensorSuite, DEFAULT_SENSOR_CONFIGS } from "../sensors/suite";
import type { SensorConfig, SensorSnapshot } from "../sensors/types";
import { assessSafety, protectiveControl } from "../autonomy/safetySupervisor";
import type { SafetySnapshot } from "../autonomy/safetyTypes";

import { TrafficWorld, SCENARIOS } from "../world/traffic";
import type { ScenarioId, TrafficSnapshot } from "../world/trafficTypes";
import { resolveDynamicMotion } from "./dynamicCollision";
import { BehaviourPlanner } from "../autonomy/behaviourPlanner";
import type { BehaviourSnapshot } from "../autonomy/behaviourTypes";
import { planClearedRoute } from "../navigation/clearedRoute";

import { MultiObjectTracker } from "../perception/tracker";
import type { TrackingSnapshot } from "../perception/types";

export interface SimulationOptions {
  sensorDropout?: { start: number; end: number };
  fusedControl?: boolean;
  scenario?: ScenarioId;
  trafficVariant?: import("../world/trafficTypes").TrafficVariant;
  timeGap?: number;
  objects?: WorldObject[];
  sensorConfigs?: SensorConfig[];
  routeMargin?: number;
}

export const FIXED_DT = 0.01;
export const MAX_FRAME_TIME = 0.1;
export const MAX_STEPS_PER_FRAME = 10;
const activeInput = (input: DriverInput) => Object.values(input).some(Boolean);
const angleDifference = (a: number, b: number) =>
  Math.atan2(Math.sin(a - b), Math.cos(a - b));

/** Fixed-step ground truth. Both driving modes use the same vehicle integrator. */
export class Simulation {
  readonly graph = createRoadGraph();
  readonly objects: WorldObject[];
  readonly routeMargin: number;
  private staticObjects: WorldObject[];
  private trafficWorld: TrafficWorld;
  private trafficVariant?: import("../world/trafficTypes").TrafficVariant;
  private trafficData: TrafficSnapshot;
  private cameraTrafficData: TrafficSnapshot;
  private planner: BehaviourPlanner;
  private timeGapValue = 1.5;
  private sweptRoadStatus: RoadStatus;
  private containmentStop = false;
  private tracker = new MultiObjectTracker();
  private trackingData!: TrackingSnapshot;
  private sensorDropout?: { start: number; end: number };
  private fusedControl: boolean;
  private sensorSuite: SensorSuite;
  private sensorData: SensorSnapshot;
  private safetyData: SafetySnapshot;
  private safetyLatch: SafetySnapshot | null = null;
  private physicalStop = false;
  private roadStatus: RoadStatus;
  private collisionEvents: CollisionEvent[] = [];
  private roadDepartures: RoadDepartureEvent[] = [];
  private collisionCount = 0;
  private roadDepartureCount = 0;
  private contact: { id: string; x: number; y: number } | null = null;
  private clearance: ClearanceResult | null = null;
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

  constructor(
    initial: VehicleState = INITIAL_STATE,
    options: SimulationOptions = {},
  ) {
    if (!Object.values(initial).every(Number.isFinite))
      throw new RangeError("Vehicle state must be finite");
    this.sensorDropout = options.sensorDropout;
    this.fusedControl = options.fusedControl ?? false;
    if (
      this.sensorDropout &&
      (!Number.isFinite(this.sensorDropout.start) ||
        !Number.isFinite(this.sensorDropout.end) ||
        this.sensorDropout.start < 0 ||
        this.sensorDropout.end <= this.sensorDropout.start)
    )
      throw new RangeError("Invalid dropout interval");
    this.staticObjects = options.objects ?? worldObjects();
    this.objects = [...this.staticObjects];
    this.trafficVariant = options.trafficVariant;
    this.trafficWorld = new TrafficWorld(
      options.scenario ?? null,
      options.trafficVariant,
    );
    this.trafficData = this.trafficWorld.snapshot;
    this.cameraTrafficData = this.trafficData;
    this.timeGapValue = options.timeGap ?? 1.5;
    this.planner = new BehaviourPlanner({ timeGap: this.timeGapValue });
    this.syncTrafficGeometry();
    this.routeMargin = options.routeMargin ?? 0.02;
    if (!Number.isFinite(this.routeMargin) || this.routeMargin < 0)
      throw new RangeError("Invalid route clearance margin");
    if (
      this.objects.some(
        (o) =>
          o.collidable && polygonsOverlap(vehicleFootprint(initial), o.polygon),
      )
    )
      throw new RangeError(
        "Initial vehicle footprint overlaps a solid obstacle",
      );
    this.initial = { ...initial };
    this.state = { ...initial };
    this.roadStatus = classifyRoad(this.state);
    this.sweptRoadStatus = this.roadStatus;
    this.sensorSuite = new SensorSuite(
      this.objects,
      options.sensorConfigs ?? DEFAULT_SENSOR_CONFIGS,
    );
    this.sensorSuite.update(0, this.state);
    this.sensorData = this.sensorSuite.snapshot;
    this.trackingData = this.tracker.update(this.sensorData, this.state, 0);
    this.cameraTrafficData = this.trafficData;
    this.safetyData = assessSafety(this.state, 0, this.sensorData, 0, false);
    if (options.scenario) {
      this.selectScenario(options.scenario);
      if (
        Object.keys(initial).some(
          (key) =>
            initial[key as keyof VehicleState] !==
            INITIAL_STATE[key as keyof VehicleState],
        )
      ) {
        this.initial = { ...initial };
        this.reset();
      }
    }
  }

  /** Geometry is used by physics/rendering/generators only, not the behaviour planner. */
  private syncTrafficGeometry(): void {
    this.trafficData = this.trafficWorld.snapshot;
    if (this.trafficData.scenario)
      for (const object of this.staticObjects) {
        const signal = this.trafficData.signals.find((s) => s.id === object.id);
        if (signal)
          object.signal = {
            color: signal.color,
            heading: signal.heading,
            stopLine: signal.stopLine,
          };
      }
    this.objects.splice(
      0,
      this.objects.length,
      ...this.staticObjects,
      ...this.trafficData.staticObstacles,
      ...this.trafficData.actors.map((a) => a.object),
    );
  }
  get tracking(): TrackingSnapshot {
    return this.trackingData;
  }
  get cameraTraffic(): TrafficSnapshot {
    return this.cameraTrafficData;
  }
  get traffic(): TrafficSnapshot {
    return this.trafficData;
  }
  get timeGap(): number {
    return this.timeGapValue;
  }
  setTimeGap(value: number): void {
    if (!Number.isFinite(value) || value < 1 || value > 3)
      throw new RangeError("Time gap must be between 1 and 3 seconds");
    this.timeGapValue = value;
  }
  get behaviour(): BehaviourSnapshot {
    const data = this.planner.snapshot;
    if (this.containmentStop || this.physicalStop)
      return {
        ...data,
        state: "BLOCKED",
        targetSpeed: 0,
        acceleration: this.control.acceleration,
        braking: Math.max(0, -this.control.acceleration),
        steering: this.control.steering,
        reason: this.containmentStop
          ? "SIMULATOR INTERLOCK: proposed swept body leaves the road; autonomous motion stopped."
          : "SIMULATOR INTERLOCK: physical contact stopped autonomous motion.",
      };
    if (this.trafficData.scenario && this.idleState === "ROUTE_UNAVAILABLE")
      return {
        ...data,
        state: "BLOCKED",
        targetSpeed: 0,
        reason: this.message,
      };
    return data;
  }
  selectScenario(id: ScenarioId): void {
    if (!SCENARIOS.some((s) => s.id === id))
      throw new RangeError("Unknown driving scenario");
    this.initial = { ...INITIAL_STATE };
    this.destination = null;
    this.trafficWorld.reset(id);
    this.reset();
    const x =
      id === "red-light" || id === "pedestrian"
        ? 45
        : id === "following" || id === "sudden-braking"
          ? 5
          : 15;
    const destination = projectToRoad(this.graph, { x, y: INITIAL_STATE.y });
    if (destination) this.planTo(destination);
  }

  get sensors(): SensorSnapshot {
    return this.sensorData;
  }
  get safety(): SafetySnapshot {
    return { ...this.safetyData };
  }
  get physical(): PhysicalSnapshot {
    return {
      roadStatus: this.roadStatus,
      sweptRoadStatus: this.sweptRoadStatus,
      containmentStop: this.containmentStop,
      collision: this.collisionEvents.at(-1) ?? null,
      collisionCount: this.collisionCount,
      roadDepartureCount: this.roadDepartureCount,
      collisionEvents: this.collisionEvents,
      roadDepartures: this.roadDepartures,
      clearance: this.clearance,
    };
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
        this.mode === "AUTONOMOUS" &&
        (this.safetyLatch || this.physicalStop || this.containmentStop)
          ? "SAFE_STOP"
          : this.mode === "AUTONOMOUS" && telemetry
            ? telemetry.state
            : this.idleState,
      targetSpeed:
        this.mode === "AUTONOMOUS"
          ? this.safetyLatch || this.physicalStop || this.containmentStop
            ? 0
            : this.trafficData.scenario
              ? this.planner.snapshot.targetSpeed
              : (telemetry?.targetSpeed ?? null)
          : null,
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
    const candidate = {
      x: position.x,
      y: position.y,
      heading: position.heading,
      velocity: 0,
    };
    // Selecting a start also resets scenario time. Validate against the reset
    // actor poses, not the poses from the experiment being replaced.
    const resetObjects = [
      ...this.staticObjects,
      ...new TrafficWorld(this.trafficData.scenario, this.trafficVariant)
        .objects,
    ];
    if (
      resetObjects.some(
        (o) =>
          o.collidable &&
          polygonsOverlap(vehicleFootprint(candidate), o.polygon),
      )
    ) {
      this.message =
        "Start rejected: vehicle footprint overlaps a solid object.";
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
      Math.abs(angleDifference(this.state.heading, position.heading)) < 0.35 &&
      classifyRoad(this.state) === "ON_ROAD"
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
    const planned =
      start && this.destination
        ? planClearedRoute(
            this.graph,
            start,
            this.destination,
            [...this.staticObjects, ...this.trafficData.staticObstacles],
            this.routeMargin,
          )
        : null;
    this.route = planned?.route ?? null;
    this.clearance = planned?.clearance ?? null;
    this.navigator = this.route ? new NavigationController(this.route) : null;
    this.safetyLatch = null;
    this.physicalStop = false;
    this.containmentStop = false;
    this.planner.reset();
    this.mode = "MANUAL";
    this.idleState = this.route ? "ROUTE_READY" : "ROUTE_UNAVAILABLE";
    this.trajectory = [];
    this.message = this.route
      ? "Route ready. Start autonomous drive when ready."
      : this.clearance && !this.clearance.valid
        ? `Route rejected: ${this.clearance.reason} at ${this.clearance.distance?.toFixed(1) ?? "unknown"} m.`
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
    this.safetyData = assessSafety(
      this.state,
      this.control.steering,
      this.sensorData,
      this.ticks * FIXED_DT,
      true,
    );
    this.safetyLatch =
      !this.trafficData.scenario && this.safetyData.braking
        ? this.safetyData
        : null;
    this.running = true;
    this.message =
      "Map-based path following · perfect localisation · sensor-based protective braking. Driving keys take over immediately.";
    return true;
  }

  cancelAutonomy(): void {
    this.mode = "MANUAL";
    this.safetyLatch = null;
    this.physicalStop = false;
    this.containmentStop = false;
    this.planner.reset();
    this.safetyData = assessSafety(
      this.state,
      this.control.steering,
      this.sensorData,
      this.ticks * FIXED_DT,
      false,
    );
    this.idleState = this.route ? "ROUTE_READY" : "IDLE";
    this.message =
      "Manual control. Vehicle motion is preserved; brake to stop.";
  }

  clearRoute(): void {
    this.cancelAutonomy();
    this.route = null;
    this.clearance = null;
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
    this.collisionEvents = [];
    this.roadDepartures = [];
    this.collisionCount = 0;
    this.roadDepartureCount = 0;
    this.contact = null;
    this.roadStatus = classifyRoad(this.state);
    this.sweptRoadStatus = this.roadStatus;
    this.trafficWorld.reset();
    this.syncTrafficGeometry();
    this.planner.reset();
    this.containmentStop = false;
    this.tracker.reset();
    this.sensorSuite.reset();
    this.sensorSuite.update(0, this.state);
    this.sensorData = this.sensorSuite.snapshot;
    this.trackingData = this.tracker.update(this.sensorData, this.state, 0);
    this.cameraTrafficData = this.trafficData;
    this.safetyLatch = null;
    this.physicalStop = false;
    this.safetyData = assessSafety(this.state, 0, this.sensorData, 0, false);
    this.clearance = null;
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
    let requested =
      this.mode === "AUTONOMOUS" && this.navigator
        ? this.navigator.update(this.state, this.control.steering, FIXED_DT)
        : driverControl(
            input,
            this.state.velocity,
            this.control.steering,
            FIXED_DT,
          );
    const autonomous = this.mode === "AUTONOMOUS";
    if (this.trafficData.scenario) {
      requested = this.planner.update(
        this.state,
        requested,
        this.sensorData,
        this.ticks * FIXED_DT,
        autonomous,
        {
          tracks: this.fusedControl ? this.trackingData.tracks : undefined,
          route: this.route,
          progress: this.navigator?.telemetry.progress ?? 0,
          timeGap: this.timeGapValue,
          targetSpeed: this.navigator?.telemetry.targetSpeed ?? 0,
          destinationReached:
            this.navigator?.telemetry.state === "DESTINATION_REACHED",
        },
      );
      const decision = this.planner.snapshot;
      this.safetyData = {
        state: !autonomous
          ? "MANUAL"
          : decision.state === "STALE_STOP"
            ? "STALE_STOP"
            : decision.aeb
              ? "PROTECTIVE_BRAKING"
              : "CLEAR",
        braking:
          autonomous && (decision.aeb || decision.state === "STALE_STOP"),
        stoppingDistance: decision.stoppingDistance,
        obstacleDistance: decision.leadDistance,
        source: decision.source ?? null,
        measurementTime: decision.observationTime,
        reason: decision.reason,
      };
    } else {
      const assessment = assessSafety(
        this.state,
        requested.steering,
        this.sensorData,
        this.ticks * FIXED_DT,
        autonomous,
      );
      if (autonomous && assessment.braking && !this.safetyLatch)
        this.safetyLatch = { ...assessment };
      this.safetyData =
        autonomous && this.safetyLatch
          ? {
              ...this.safetyLatch,
              reason:
                this.safetyLatch.reason +
                " Stop held until manual takeover, reset, or explicit reactivation.",
            }
          : assessment;
      requested = protectiveControl(
        requested,
        this.safetyData,
        this.state,
        FIXED_DT,
      );
    }
    if (autonomous && (this.physicalStop || this.containmentStop))
      requested = {
        steering: this.control.steering,
        acceleration:
          -Math.sign(this.state.velocity) *
          Math.min(
            VEHICLE.maxBraking,
            Math.abs(this.state.velocity) / FIXED_DT,
          ),
      };
    const previous = this.state;
    let proposed = integrateVehicle(previous, requested, FIXED_DT);
    this.sweptRoadStatus = classifyRoadMotion(previous, proposed);
    if (autonomous && this.sweptRoadStatus !== "ON_ROAD") {
      this.containmentStop = true;
      proposed = { ...previous, velocity: 0 };
    }
    const beforeActors = this.trafficData.actors.map((a) => a.object);
    this.trafficWorld.advance(FIXED_DT);
    const afterActors = this.trafficWorld.snapshot.actors.map((a) => a.object);
    const staticGeometry = [
      ...this.staticObjects,
      ...this.trafficWorld.snapshot.staticObstacles,
    ];
    let motion = resolveMotion(previous, proposed, staticGeometry);
    const dynamic = resolveDynamicMotion(
      previous,
      proposed,
      beforeActors,
      afterActors,
    );
    if (dynamic.collided && dynamic.fraction <= motion.fraction) {
      motion = dynamic;
      if (dynamic.objectId)
        this.trafficWorld.haltActor(dynamic.objectId, dynamic.fraction);
    }
    this.syncTrafficGeometry();
    // Explicit Euler first changes velocity at rest. While already in contact,
    // inhibit an inward launch so held throttle cannot alternate between rest
    // and a small velocity. A separating reverse launch remains available.
    if (
      this.contact &&
      Math.abs(previous.velocity) < 1e-12 &&
      proposed.velocity !== 0 &&
      !motion.collided
    ) {
      const probe = {
        ...proposed,
        x:
          proposed.x +
          proposed.velocity * Math.cos(proposed.heading) * FIXED_DT,
        y:
          proposed.y +
          proposed.velocity * Math.sin(proposed.heading) * FIXED_DT,
      };
      const launch = resolveMotion(proposed, probe, this.objects);
      if (launch.collided)
        motion = {
          ...motion,
          state: { ...motion.state, velocity: 0 },
          collided: true,
          objectId: launch.objectId,
        };
    }
    this.state = motion.state;
    this.distance += previous.velocity * FIXED_DT * motion.fraction;
    this.control = {
      steering: requested.steering,
      acceleration: (this.state.velocity - previous.velocity) / FIXED_DT,
    };
    this.ticks += 1;
    const now = this.ticks * FIXED_DT;
    if (motion.collided && motion.objectId) {
      if (!this.contact || this.contact.id !== motion.objectId) {
        const event = {
          time: now,
          objectId: motion.objectId,
          position: { x: this.state.x, y: this.state.y },
        };
        this.collisionEvents = [...this.collisionEvents.slice(-63), event];
        this.collisionCount += 1;
        this.contact = {
          id: motion.objectId,
          x: this.state.x,
          y: this.state.y,
        };
      }
      if (autonomous) this.physicalStop = true;
    } else if (
      this.contact &&
      Math.hypot(this.state.x - this.contact.x, this.state.y - this.contact.y) >
        0.05
    )
      this.contact = null;
    const road = classifyRoad(this.state);
    if (road !== this.roadStatus && road !== "ON_ROAD") {
      this.roadDepartureCount += 1;
      this.roadDepartures = [
        ...this.roadDepartures.slice(-63),
        {
          time: now,
          status: road,
          position: { x: this.state.x, y: this.state.y },
        },
      ];
    }
    this.roadStatus = road;
    const previousCameraFrame = this.sensorData.camera;
    if (
      !this.sensorDropout ||
      now < this.sensorDropout.start ||
      now >= this.sensorDropout.end
    )
      this.sensorSuite.update(
        now,
        this.state,
        (this.state.heading - previous.heading) / FIXED_DT,
      );
    this.sensorData = this.sensorSuite.snapshot;
    this.trackingData = this.tracker.update(this.sensorData, this.state, now);
    if (this.sensorData.camera !== previousCameraFrame)
      this.cameraTrafficData = this.trafficData;
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
          "Tracking tolerance exceeded. Braking; reset or take manual control.";
      if (this.safetyLatch) this.message = this.safetyData.reason;
      if (
        this.trafficData.scenario &&
        !this.physicalStop &&
        !this.containmentStop
      )
        this.message = this.planner.snapshot.reason;
      if (this.containmentStop)
        this.message =
          "Actual swept footprint validation failed. Autonomous motion stopped; reset or take manual control.";
      if (this.physicalStop)
        this.message =
          "Physical contact inhibited autonomous motion. This is a collision-engine stop, not a perception decision. Take manual control or reset.";
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
