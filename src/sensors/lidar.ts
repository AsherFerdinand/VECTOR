import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "../world/types";
import type { LidarScan, SensorConfig } from "./types";
import { deterministicNoise, raycast, sensorPose } from "./geometry";

export function generateLidar(
  vehicle: VehicleState,
  objects: WorldObject[],
  config: SensorConfig,
  time: number,
): LidarScan {
  const pose = sensorPose(vehicle, config);
  const count = Math.max(1, Math.floor(config.rays ?? 180));
  const fullCircle = config.fov >= Math.PI * 2 - 1e-9;
  const rays = Array.from({ length: count }, (_, i) => {
    const angle =
      count === 1
        ? 0
        : -config.fov / 2 + (config.fov * i) / (fullCircle ? count : count - 1);
    const hit = raycast(pose, pose.heading + angle, objects, config.maxRange);
    const range = hit.object
      ? Math.max(
          0,
          Math.min(
            config.maxRange,
            hit.distance + deterministicNoise(config, time, i),
          ),
        )
      : config.maxRange;
    return {
      angle,
      range,
      hit: hit.object !== null,
      point: {
        x: pose.x + Math.cos(pose.heading + angle) * range,
        y: pose.y + Math.sin(pose.heading + angle) * range,
      },
    };
  });
  return {
    sensorId: config.id,
    timestamp: time,
    pose,
    rays,
    maxRange: config.maxRange,
  };
}
