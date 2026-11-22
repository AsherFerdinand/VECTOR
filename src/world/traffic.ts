import { town } from "./town";
import { trafficLightPlacements } from "./objects";
import { vehicleFootprint } from "../simulation/collision";
import { VEHICLE } from "../simulation/vehicle";
import type { Point } from "../navigation/types";
import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "./types";
import type {
  ScenarioId,
  SignalColor,
  SignalMap,
  TrafficActor,
  TrafficSnapshot,
} from "./trafficTypes";

export const SCENARIOS: {
  id: ScenarioId;
  label: string;
  description: string;
}[] = [
  {
    id: "normal",
    label: "Normal traffic",
    description: "A bounded vehicle and pedestrian on the northern streets.",
  },
  {
    id: "following",
    label: "Following a lead vehicle",
    description:
      "Follow a lead EV cruising at 3.5 m/s before its mapped lane-end stop.",
  },
  {
    id: "sudden-braking",
    label: "Sudden lead braking",
    description:
      "The lead EV brakes at 5 s, holds 4 s, then resumes within its lane.",
  },
  {
    id: "red-light",
    label: "Red-light stop",
    description: "The eastern junction is red for 10 s, then green for 14 s.",
  },
  {
    id: "pedestrian",
    label: "Pedestrian crossing",
    description: "A pedestrian enters the western marked crossing at 3 s.",
  },
  {
    id: "blocked",
    label: "Blocked road",
    description:
      "A solid 2 m square obstacle blocks the initial eastbound lane.",
  },
];

const laneOffset = town.roadWidth / 4;
const southRoad = town.roads.find((road) => road.id === "south")!;
const northRoad = town.roads.find((road) => road.id === "north")!;
/** Every approach has one physical pole and a stop line before its marked crossing. */
export const SIGNAL_MAP: SignalMap[] = town.intersections.flatMap(
  ([x, y], junction) => {
    const approaches = [
      {
        suffix: "a",
        heading: 0,
        stopLine: [
          { x: x - 7.5, y: y - town.roadWidth / 2 },
          { x: x - 7.5, y },
        ],
        crossing: [
          { x: x - 6, y: y - 6 },
          { x: x - 6, y: y + 6 },
        ],
      },
      {
        suffix: "b",
        heading: Math.PI,
        stopLine: [
          { x: x + 7.5, y },
          { x: x + 7.5, y: y + town.roadWidth / 2 },
        ],
        crossing: [
          { x: x + 6, y: y - 6 },
          { x: x + 6, y: y + 6 },
        ],
      },
      {
        suffix: "c",
        heading: Math.PI / 2,
        stopLine: [
          { x, y: y - 7.5 },
          { x: x + town.roadWidth / 2, y: y - 7.5 },
        ],
        crossing: [
          { x: x - 6, y: y - 6 },
          { x: x + 6, y: y - 6 },
        ],
      },
      {
        suffix: "d",
        heading: -Math.PI / 2,
        stopLine: [
          { x: x - town.roadWidth / 2, y: y + 7.5 },
          { x, y: y + 7.5 },
        ],
        crossing: [
          { x: x - 6, y: y + 6 },
          { x: x + 6, y: y + 6 },
        ],
      },
    ];
    return approaches.map((approach) => {
      const id = `traffic-light-${junction}-${approach.suffix}`,
        pole = trafficLightPlacements.find((p) => p.id === id)!;
      return {
        id,
        position: { x: pole.x, y: pole.y },
        heading: approach.heading,
        stopLine: approach.stopLine as [Point, Point],
        crossing: approach.crossing as [Point, Point],
      };
    });
  },
);

const eastApproach = SIGNAL_MAP.find(
  (signal) =>
    signal.heading === 0 && signal.position.x > 0 && signal.position.y < 0,
)!;
const northCrossing = SIGNAL_MAP.find(
  (signal) =>
    signal.heading === Math.PI &&
    signal.position.x > 0 &&
    signal.position.y > 0,
)!.crossing;
const leadStartX = -4;
const vehicleFrontExtent = (VEHICLE.length + VEHICLE.wheelbase) / 2;
const leadStopX = eastApproach.stopLine[0].x - vehicleFrontExtent - 0.9;

function rectangle(
  x: number,
  y: number,
  length: number,
  width: number,
  heading = 0,
): Point[] {
  const c = Math.cos(heading),
    s = Math.sin(heading);
  return [
    [-length / 2, -width / 2],
    [length / 2, -width / 2],
    [length / 2, width / 2],
    [-length / 2, width / 2],
  ].map(([a, b]) => ({ x: x + a * c - b * s, y: y + a * s + b * c }));
}
function signalColor(
  time: number,
  heading: number,
  redScenario: boolean,
): SignalColor {
  const horizontal = Math.abs(Math.sin(heading)) < 0.5;
  if (redScenario) {
    const phase = time % 40;
    if (horizontal)
      return phase < 10
        ? "red"
        : phase < 24
          ? "green"
          : phase < 27
            ? "yellow"
            : "red";
    return phase < 7
      ? "green"
      : phase < 10
        ? "yellow"
        : phase < 27
          ? "red"
          : phase < 37
            ? "green"
            : "yellow";
  }
  const phase = time % 30;
  if (horizontal) return phase < 12 ? "green" : phase < 15 ? "yellow" : "red";
  return phase < 15 ? "red" : phase < 27 ? "green" : "yellow";
}
interface Motion {
  distance: number;
  speed: number;
}
/** Constant-speed travel followed by bounded2m/s² braking to a geometric stop. */
function cruiseToStop(time: number, speed: number, available: number): Motion {
  if (time < 0 || available <= 0) return { distance: 0, speed: 0 };
  const braking = 2,
    brakeDistance = (speed * speed) / (2 * braking),
    cruiseTime = Math.max(0, (available - brakeDistance) / speed),
    brakeTime = speed / braking;
  if (time < cruiseTime) return { distance: time * speed, speed };
  if (time < cruiseTime + brakeTime) {
    const t = time - cruiseTime;
    return {
      distance: speed * cruiseTime + speed * t - 0.5 * braking * t * t,
      speed: speed - braking * t,
    };
  }
  return { distance: available, speed: 0 };
}
/** A bounded trapezoidal profile starting and finishing at rest. */
function travelFromRest(
  time: number,
  speed: number,
  available: number,
  acceleration = 2,
): Motion {
  if (time <= 0) return { distance: 0, speed: 0 };
  const peak = Math.min(speed, Math.sqrt(available * acceleration));
  const rampTime = peak / acceleration,
    rampDistance = (peak * peak) / (2 * acceleration),
    cruiseTime = (available - 2 * rampDistance) / peak;
  if (time < rampTime)
    return {
      distance: 0.5 * acceleration * time * time,
      speed: acceleration * time,
    };
  if (time < rampTime + cruiseTime)
    return { distance: rampDistance + peak * (time - rampTime), speed: peak };
  if (time < 2 * rampTime + cruiseTime) {
    const t = time - rampTime - cruiseTime;
    return {
      distance:
        available - rampDistance + peak * t - 0.5 * acceleration * t * t,
      speed: peak - acceleration * t,
    };
  }
  return { distance: available, speed: 0 };
}
function suddenLead(time: number): Motion {
  const speed = 3.5,
    brakeStart = 5,
    braking = 6,
    brakeDuration = speed / braking,
    stopDistance = speed * brakeStart + (speed * speed) / (2 * braking),
    resume = brakeStart + brakeDuration + 4,
    available = leadStopX - leadStartX;
  if (time < brakeStart) return { distance: time * speed, speed };
  if (time < brakeStart + brakeDuration) {
    const t = time - brakeStart;
    return {
      distance: brakeStart * speed + speed * t - 0.5 * braking * t * t,
      speed: speed - braking * t,
    };
  }
  if (time < resume) return { distance: stopDistance, speed: 0 };
  // A short triangular2m/s² profile resumes without crossing the stop before the junction.
  const remaining = available - stopDistance,
    acceleration = 2,
    halfDuration = Math.sqrt(remaining / acceleration),
    t = time - resume;
  if (t < halfDuration)
    return {
      distance: stopDistance + 0.5 * acceleration * t * t,
      speed: acceleration * t,
    };
  if (t < 2 * halfDuration) {
    const q = t - halfDuration;
    return {
      distance:
        stopDistance +
        remaining / 2 +
        acceleration * halfDuration * q -
        0.5 * acceleration * q * q,
      speed: acceleration * (halfDuration - q),
    };
  }
  return { distance: available, speed: 0 };
}
function actor(
  id: string,
  kind: TrafficActor["kind"],
  state: VehicleState,
): TrafficActor {
  const length = kind === "vehicle" ? 4.5 : 0.5,
    width = kind === "vehicle" ? 1.8 : 0.5;
  const object: WorldObject = {
    id,
    objectClass: kind,
    polygon:
      kind === "vehicle"
        ? vehicleFootprint(state)
        : rectangle(state.x, state.y, length, width, state.heading),
    height: kind === "vehicle" ? 1.75 : 1.7,
    velocity: {
      x: Math.cos(state.heading) * state.velocity,
      y: Math.sin(state.heading) * state.velocity,
    },
    collidable: true,
  };
  return { id, kind, state: { ...state }, length, width, object };
}

/** Analytic bounded scripts run from simulation time; they never teleport or loop at endpoints. */
export class TrafficWorld {
  private scenario: ScenarioId | null;
  private time = 0;
  private halted = new Map<string, TrafficActor>();
  private previousActors: TrafficActor[] = [];
  constructor(scenario: ScenarioId | null = null) {
    this.scenario = scenario;
  }
  reset(scenario: ScenarioId | null = this.scenario): void {
    this.scenario = scenario;
    this.time = 0;
    this.halted.clear();
    this.previousActors = [];
  }
  advance(dt: number): void {
    if (!Number.isFinite(dt) || dt < 0)
      throw new RangeError("Traffic step must be finite and nonnegative");
    this.previousActors = this.actors();
    this.time += dt;
  }
  private actors(): TrafficActor[] {
    let actors: TrafficActor[] = [];
    if (this.scenario === "following" || this.scenario === "sudden-braking") {
      const motion =
        this.scenario === "sudden-braking"
          ? suddenLead(this.time)
          : cruiseToStop(this.time, 3.5, leadStopX - leadStartX);
      actors = [
        actor("lead-vehicle", "vehicle", {
          x: leadStartX + motion.distance,
          y: southRoad.from[1] - laneOffset,
          heading: 0,
          velocity: motion.speed,
        }),
      ];
    } else if (this.scenario === "normal") {
      const startX = northRoad.from[0] + 10;
      const motion = travelFromRest(this.time, 2.5, leadStopX - startX),
        walk = travelFromRest(
          Math.max(0, this.time - 3),
          1.2,
          northCrossing[1].y - northCrossing[0].y,
          1.2,
        );
      actors = [
        actor("north-vehicle", "vehicle", {
          x: startX + motion.distance,
          y: northRoad.from[1] - laneOffset,
          heading: 0,
          velocity: motion.speed,
        }),
        actor("north-pedestrian", "pedestrian", {
          x: northCrossing[0].x,
          y: northCrossing[0].y + walk.distance,
          heading: Math.PI / 2,
          velocity: walk.speed,
        }),
      ];
    } else if (this.scenario === "pedestrian") {
      const crossing = eastApproach.crossing;
      const walk = travelFromRest(
        Math.max(0, this.time - 3),
        1.2,
        crossing[1].y - crossing[0].y,
        1.2,
      );
      actors = [
        actor("crossing-pedestrian", "pedestrian", {
          x: crossing[0].x,
          y: crossing[0].y + walk.distance,
          heading: Math.PI / 2,
          velocity: walk.speed,
        }),
      ];
    }
    return actors.map((value) => this.halted.get(value.id) ?? value);
  }
  /** Ground-truth crash fail-stop; frozen actors remain halted until scenario reset. */
  haltActor(id: string, fraction: number): void {
    if (!Number.isFinite(fraction) || fraction < 0 || fraction > 1)
      throw new RangeError("Contact fraction must lie in[0,1]");
    const next = this.actors().find((a) => a.id === id),
      previous = this.previousActors.find((a) => a.id === id);
    if (!next) return;
    const a = previous?.state ?? next.state,
      b = next.state;
    const heading =
      a.heading +
      Math.atan2(
        Math.sin(b.heading - a.heading),
        Math.cos(b.heading - a.heading),
      ) *
        fraction;
    this.halted.set(
      id,
      actor(id, next.kind, {
        x: a.x + (b.x - a.x) * fraction,
        y: a.y + (b.y - a.y) * fraction,
        heading,
        velocity: 0,
      }),
    );
  }
  get snapshot(): TrafficSnapshot {
    const signals = SIGNAL_MAP.map((signal) => ({
      ...signal,
      position: { ...signal.position },
      stopLine: signal.stopLine.map((p) => ({ ...p })) as [Point, Point],
      crossing: signal.crossing.map((p) => ({ ...p })) as [Point, Point],
      color: signalColor(
        this.time,
        signal.heading,
        this.scenario === "red-light",
      ),
    }));
    const staticObstacles: WorldObject[] =
      this.scenario === "blocked"
        ? [
            {
              id: "scenario-road-block",
              objectClass: "static-obstacle",
              polygon: rectangle(0, southRoad.from[1] - laneOffset, 2, 2),
              height: 1.2,
              velocity: { x: 0, y: 0 },
              collidable: true,
            },
          ]
        : [];
    // Snapshot copies keep rendering or sensor consumers from mutating the scripted world.
    const actors = this.actors().map((a) => ({
      ...a,
      state: { ...a.state },
      object: {
        ...a.object,
        velocity: { ...a.object.velocity },
        polygon: a.object.polygon.map((p) => ({ ...p })),
      },
    }));
    return {
      scenario: this.scenario,
      time: this.time,
      actors,
      signals,
      staticObstacles,
    };
  }
  /** Dynamic and scenario solids only: physical signal poles remain in worldObjects(). */
  get objects(): WorldObject[] {
    const snapshot = this.snapshot;
    return [
      ...snapshot.actors.map((a) => a.object),
      ...snapshot.staticObstacles,
    ];
  }
}
