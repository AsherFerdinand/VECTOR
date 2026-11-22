import { Simulation, FIXED_DT } from "../simulation/clock";
import type { SimulationOptions } from "../simulation/clock";
import { INITIAL_STATE } from "../simulation/vehicle";
import type { DriverInput } from "../simulation/types";
import { projectToRoad } from "../navigation/graph";
import { DEFAULT_SENSOR_CONFIGS } from "../sensors/suite";
import { polygonsOverlap, vehicleFootprint } from "../simulation/collision";
import type { Point } from "../navigation/types";
import { CorridorSimulation } from "../planning/corridorSimulation";
import { corridorContains } from "../planning/corridor";
import { createValidationResult } from "./metrics";
import type {
  ValidationConfig,
  ValidationFrame,
  ValidationResult,
} from "./types";

export const MAX_VALIDATION_SECONDS = 120;
export const MAX_BATCH_RUNS = 12;
const IDLE: DriverInput = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
};

function validateConfig(config: ValidationConfig): void {
  if (
    !config.id ||
    !Number.isInteger(config.seed) ||
    !Number.isFinite(config.duration) ||
    config.duration <= 0 ||
    config.duration > MAX_VALIDATION_SECONDS
  )
    throw new RangeError(
      "Validation requires an id, integer seed and duration in (0, 120] seconds",
    );
  if (
    config.timeGap !== undefined &&
    (!Number.isFinite(config.timeGap) ||
      config.timeGap < 1 ||
      config.timeGap > 3)
  )
    throw new RangeError("Time gap must be between 1 and 3 seconds");
  if (
    config.destination &&
    !Object.values(config.destination).every(Number.isFinite)
  )
    throw new RangeError("Destination must be finite");
  if (
    config.noise !== undefined &&
    (!Number.isFinite(config.noise) || config.noise < 0 || config.noise > 10)
  )
    throw new RangeError("Radar range noise must be between 0 and 10 metres");
  if (
    config.dropout &&
    (!Number.isFinite(config.dropout.start) ||
      !Number.isFinite(config.dropout.end) ||
      config.dropout.start < 0 ||
      config.dropout.end <= config.dropout.start ||
      config.dropout.end > config.duration)
  )
    throw new RangeError("Dropout must be a finite interval within the run");
  if (
    config.initial &&
    (!Object.values(config.initial).every(Number.isFinite) ||
      config.initial.velocity !== 0)
  )
    throw new RangeError(
      "Configurable validation starts currently require finite, stationary lane poses",
    );
}
function pointSegmentDistance(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x,
    dy = b.y - a.y;
  const l2 = dx * dx + dy * dy;
  const t = l2
    ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2))
    : 0;
  return Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
/** Validation-only polygon distance; hidden geometry never enters online control. */
function clearance(sim: Simulation): number | null {
  const footprint = vehicleFootprint(sim.snapshot.vehicle);
  let nearest = Infinity;
  for (const object of sim.objects) {
    if (!object.collidable) continue;
    if (polygonsOverlap(footprint, object.polygon)) return 0;
    for (let i = 0; i < footprint.length; i++)
      for (let j = 0; j < object.polygon.length; j++) {
        nearest = Math.min(
          nearest,
          pointSegmentDistance(
            footprint[i],
            object.polygon[j],
            object.polygon[(j + 1) % object.polygon.length],
          ),
          pointSegmentDistance(
            object.polygon[j],
            footprint[i],
            footprint[(i + 1) % footprint.length],
          ),
        );
      }
  }
  return Number.isFinite(nearest) ? nearest : null;
}

/** A synchronous, renderer-independent fixed-step run with bounded duration and recording. */
export function runScenario(config: ValidationConfig): ValidationResult {
  validateConfig(config);
  if (config.corridorScenario) return runCorridorScenario(config);
  const sensorConfigs = DEFAULT_SENSOR_CONFIGS.map((sensor, index) => ({
    ...sensor,
    mount: { ...sensor.mount },
    seed: config.seed + index * 1009,
    noise: sensor.source === "radar" ? (config.noise ?? 0) : sensor.noise,
  }));
  const options: SimulationOptions = {
    scenario: config.scenario,
    trafficVariant: config.trafficVariant,
    timeGap: config.timeGap,
    sensorConfigs,
    sensorDropout: config.dropout,
    fusedControl: true,
  };
  const sim = new Simulation(INITIAL_STATE, options);
  if (config.initial) {
    const position = projectToRoad(sim.graph, config.initial, 0.1);
    if (
      !position ||
      Math.abs(
        Math.atan2(
          Math.sin(position.heading - config.initial.heading),
          Math.cos(position.heading - config.initial.heading),
        ),
      ) > 0.05 ||
      !sim.setStart(position)
    )
      throw new RangeError(
        "Initial pose must align with a valid, unoccupied directed lane",
      );
  }
  if (config.destination) {
    const destination = projectToRoad(sim.graph, config.destination, 0.1);
    if (!destination)
      throw new RangeError("Destination must lie on a directed lane");
    sim.planTo(destination);
  }
  // setStart resets route selection; restore the scenario's ordinary target when needed.
  if (
    !sim.navigation.route &&
    config.scenario !== "blocked" &&
    !config.destination
  ) {
    const x =
      config.scenario === "red-light" || config.scenario === "pedestrian"
        ? 45
        : config.scenario === "following" ||
            config.scenario === "sudden-braking"
          ? 5
          : 15;
    const destination = projectToRoad(sim.graph, { x, y: INITIAL_STATE.y });
    if (destination) sim.planTo(destination);
  }
  sim.activateAutonomy();
  // A rejected static route still advances the deterministic world while the ego remains manual/still.
  sim.start();
  const frames: ValidationFrame[] = [];
  const ticks = Math.floor(config.duration / FIXED_DT + 1e-9);
  let lastCollisionCount = 0;
  let previousSpeed = sim.snapshot.vehicle.velocity;
  let latestClearance: number | null = null;
  for (let tick = 0; tick <= ticks; tick++) {
    if (tick) sim.step(IDLE);
    const snapshot = sim.snapshot;
    const physical = sim.physical;
    const navigation = sim.navigation;
    const sample = tick % 10 === 0 || tick === ticks;
    if (sample) latestClearance = clearance(sim);
    const collided = physical.collisionCount > lastCollisionCount;
    const frame: ValidationFrame = {
      time: snapshot.time,
      vehicle: snapshot.vehicle,
      control: snapshot.control,
      behaviour: { ...sim.behaviour },
      physical: {
        ...physical,
        collisionEvents: [...physical.collisionEvents],
        roadDepartures: [...physical.roadDepartures],
      },
      navigation: { ...navigation, route: null, trajectory: [] },
      clearance: sample ? latestClearance : undefined,
      impactSpeed: collided ? Math.abs(previousSpeed) : undefined,
      // Script timing is validation truth only, not used by any controller.
      hazardActive:
        config.scenario === "sudden-braking" &&
        snapshot.time >= 5 &&
        snapshot.time <= 9,
    };
    if (sample) {
      frame.tracks = sim.tracking.tracks.map((track) => ({
        ...track,
        covariance: track.covariance.map((row) => [...row]),
        sources: [...track.sources],
      }));
      frame.truth = sim.traffic.actors.map((actor) => {
        const polygon = actor.object.polygon;
        return {
          id: actor.id,
          polygon: polygon.map((point) => ({ ...point })),
          x: polygon.reduce((sum, p) => sum + p.x, 0) / polygon.length,
          y: polygon.reduce((sum, p) => sum + p.y, 0) / polygon.length,
          vx: actor.state.velocity * Math.cos(actor.state.heading),
          vy: actor.state.velocity * Math.sin(actor.state.heading),
        };
      });
    }
    frames.push(frame);
    previousSpeed = snapshot.vehicle.velocity;
    lastCollisionCount = physical.collisionCount;
  }
  return createValidationResult(
    {
      ...config,
      initial: config.initial && { ...config.initial },
      destination: config.destination && { ...config.destination },
      dropout: config.dropout && { ...config.dropout },
    },
    frames,
  );
}

/** Each entry receives an isolated simulation; output order equals configuration order. */
export function runBatch(configs: ValidationConfig[]): ValidationResult[] {
  if (configs.length === 0 || configs.length > MAX_BATCH_RUNS)
    throw new RangeError("Batch must contain between 1 and 12 runs");
  configs.forEach(validateConfig);
  return configs.map(runScenario);
}

/** Same sensors, fusion and vehicle model on an explicitly mapped multilane fixture. */
function runCorridorScenario(config: ValidationConfig): ValidationResult {
  if (
    config.initial ||
    config.destination ||
    config.dropout ||
    config.trafficVariant
  )
    throw new RangeError(
      "Corridor presets currently use their declared initial conditions and actor script",
    );
  const sim = new CorridorSimulation({
    scenario: config.corridorScenario,
    seed: config.seed,
    noise: config.noise,
  });
  const frames: ValidationFrame[] = [];
  const ticks = Math.floor(config.duration / FIXED_DT + 1e-9);
  let previousVelocity = sim.snapshot.vehicle.velocity;
  let previousCollisionCount = 0;
  for (let tick = 0; tick <= ticks; tick++) {
    const data = tick ? sim.step() : sim.snapshot;
    const metrics = sim.results;
    const onRoad = corridorContains(data.vehicle);
    const target = data.manoeuvre.selected;
    const cte =
      target && target.points.length > 1
        ? Math.min(
            ...target.points
              .slice(1)
              .map((point, i) =>
                pointSegmentDistance(data.vehicle, target.points[i], point),
              ),
          )
        : 0;
    const frame: ValidationFrame = {
      time: data.time,
      vehicle: { ...data.vehicle },
      control: { ...data.control },
      behaviour: { ...data.behaviour },
      physical: {
        roadStatus: onRoad ? "ON_ROAD" : "PARTIALLY_OFF_ROAD",
        sweptRoadStatus: onRoad ? "ON_ROAD" : "PARTIALLY_OFF_ROAD",
        containmentStop: false,
        collision: null,
        collisionCount: metrics.collisions,
        roadDepartureCount: metrics.roadDepartures,
        collisionEvents: [],
        roadDepartures: [],
        clearance: null,
      },
      navigation: {
        mode: "AUTONOMOUS",
        state: metrics.completed ? "DESTINATION_REACHED" : "ROUTE_FOLLOWING",
        route: null,
        trajectory: [],
        message: data.manoeuvre.reason,
        targetSpeed: data.manoeuvre.targetSpeed,
        steeringCommand: data.control.steering,
        remainingDistance: Math.max(0, 145 - data.vehicle.x),
        crossTrackError: cte,
        lookAheadDistance: null,
        lookAheadPoint: null,
        progress: Math.max(0, Math.min(1, (data.vehicle.x + 45) / 190)),
      },
      clearance: metrics.minimumGap,
      impactSpeed:
        metrics.collisions > previousCollisionCount
          ? Math.abs(previousVelocity)
          : undefined,
      laneDeparture: !onRoad,
      manoeuvre: metrics.completed ? "COMPLETED" : data.manoeuvre.state,
    };
    if (tick % 10 === 0 || metrics.completed || tick === ticks) {
      frame.tracks = data.tracking.tracks.map((track) => ({
        ...track,
        covariance: track.covariance.map((row) => [...row]),
        sources: [...track.sources],
      }));
      const polygon = vehicleFootprint(data.actor);
      frame.truth = [
        {
          id: "corridor-validation-lead",
          x: data.actor.x + 1.35 * Math.cos(data.actor.heading),
          y: data.actor.y + 1.35 * Math.sin(data.actor.heading),
          vx: data.actor.velocity * Math.cos(data.actor.heading),
          vy: data.actor.velocity * Math.sin(data.actor.heading),
          polygon,
        },
      ];
    }
    frames.push(frame);
    previousVelocity = data.vehicle.velocity;
    previousCollisionCount = metrics.collisions;
    if (metrics.completed) break;
  }
  return createValidationResult(config, frames);
}
