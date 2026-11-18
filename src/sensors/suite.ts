import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "../world/types";
import type { SensorConfig, SensorSnapshot } from "./types";
import { generateCamera } from "./camera";
import { generateLidar } from "./lidar";
import { generateRadar } from "./radar";

export const DEFAULT_SENSOR_CONFIGS: SensorConfig[] = [
  {
    id: "roof-lidar",
    source: "lidar",
    mount: { x: 1.35, y: 0 },
    yaw: 0,
    fov: Math.PI * 2,
    maxRange: 40,
    updateHz: 10,
    noise: 0,
    seed: 1001,
    rays: 180,
  },
  {
    id: "front-camera",
    source: "camera",
    mount: { x: 2.25, y: 0.35 },
    yaw: 0,
    fov: Math.PI / 2,
    maxRange: 50,
    updateHz: 10,
    noise: 0,
    seed: 1002,
  },
  {
    id: "front-radar",
    source: "radar",
    mount: { x: 3.6, y: 0 },
    yaw: 0,
    fov: (Math.PI * 2) / 3,
    maxRange: 80,
    updateHz: 10,
    noise: 0,
    seed: 1003,
  },
];

/** Outputs are immutable snapshots shared by rendering and the safety layer. */
function freeze<T>(value: T): T {
  if (value && typeof value === "object" && !Object.isFrozen(value)) {
    Object.freeze(value);
    for (const child of Object.values(value)) freeze(child);
  }
  return value;
}

/** Simulation-time scheduler; counts are generated frames, not obstacle return counts. */
export class SensorSuite {
  private configs: SensorConfig[];
  private latest: SensorSnapshot;
  private lastPeriods = new Map<string, number>();
  private lastTime = -1;

  constructor(
    private readonly objects: WorldObject[],
    configs: SensorConfig[] = DEFAULT_SENSOR_CONFIGS,
  ) {
    const ids = new Set<string>();
    const sources = new Set<string>();
    this.configs = configs.map((config) => {
      if (!config.id || ids.has(config.id))
        throw new Error("Sensor identifiers must be nonempty and unique");
      ids.add(config.id);
      if (sources.has(config.source))
        throw new Error("SensorSnapshot supports one sensor per modality");
      sources.add(config.source);
      if (
        ![
          config.mount.x,
          config.mount.y,
          config.yaw,
          config.fov,
          config.maxRange,
          config.updateHz,
          config.noise,
          config.seed,
        ].every(Number.isFinite) ||
        config.fov <= 0 ||
        config.fov > Math.PI * 2 ||
        config.maxRange <= 0 ||
        config.updateHz <= 0 ||
        config.noise < 0 ||
        (config.rays !== undefined &&
          (!Number.isInteger(config.rays) ||
            config.rays < 1 ||
            config.rays > 720))
      )
        throw new RangeError("Invalid sensor configuration");
      return { ...config, mount: { ...config.mount } };
    });
    this.latest = this.empty();
  }

  private empty(): SensorSnapshot {
    return freeze({
      configs: this.configs,
      lidar: null,
      camera: null,
      radar: null,
      counts: { lidar: 0, camera: 0, radar: 0 },
    });
  }

  get snapshot(): SensorSnapshot {
    return this.latest;
  }

  reset(): void {
    this.lastPeriods.clear();
    this.lastTime = -1;
    this.latest = this.empty();
  }

  update(time: number, vehicle: VehicleState, yawRate = 0): void {
    if (!Number.isFinite(time) || time < 0 || !Number.isFinite(yawRate))
      throw new RangeError(
        "Sensor time and yaw rate must be finite; time must be nonnegative",
      );
    if (time < this.lastTime - 1e-9)
      throw new RangeError("Sensor time cannot run backward; reset first");
    this.lastTime = time;
    let latest = this.latest;
    for (const config of this.configs) {
      const period = Math.floor(time * config.updateHz + 1e-9);
      if (period <= (this.lastPeriods.get(config.id) ?? -1)) continue;
      // Sample once using the actual current pose. Missed periods are dropped;
      // replaying old timestamps with today's pose would invent sensor history.
      this.lastPeriods.set(config.id, period);
      latest = { ...latest, counts: { ...latest.counts } };
      if (config.source === "lidar")
        latest.lidar = generateLidar(vehicle, this.objects, config, time);
      if (config.source === "camera")
        latest.camera = generateCamera(vehicle, this.objects, config, time);
      if (config.source === "radar")
        latest.radar = generateRadar(
          vehicle,
          this.objects,
          config,
          time,
          yawRate,
        );
      latest.counts[config.source] += 1;
    }
    this.latest = freeze(latest);
  }
}
