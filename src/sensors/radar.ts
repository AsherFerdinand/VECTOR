import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "../world/types";
import type { DetectionFrame, SensorConfig } from "./types";
import {
  deterministicNoise,
  sensorPose,
  visibleObjects,
  wrapAngle,
} from "./geometry";

/** One geometric surface measurement per visible object; Doppler physics is not modelled. */
export function generateRadar(
  vehicle: VehicleState,
  objects: WorldObject[],
  config: SensorConfig,
  time: number,
  yawRate = 0,
): DetectionFrame {
  const pose = sensorPose(vehicle, config);
  const mountX = pose.x - vehicle.x,
    mountY = pose.y - vehicle.y;
  const sensorVelocity = {
    x: vehicle.velocity * Math.cos(vehicle.heading) - yawRate * mountY,
    y: vehicle.velocity * Math.sin(vehicle.heading) + yawRate * mountX,
  };
  const detections = visibleObjects(pose, objects, config).map(
    (visible, index) => {
      const worldBearing = pose.heading + visible.bearing;
      const radialVelocity =
        (visible.object.velocity.x - sensorVelocity.x) *
          Math.cos(worldBearing) +
        (visible.object.velocity.y - sensorVelocity.y) *
          Math.sin(worldBearing) +
        deterministicNoise(config, time, index, 1);
      const range = Math.max(
        0,
        Math.min(
          config.maxRange,
          visible.range + deterministicNoise(config, time, index),
        ),
      );
      const bearing = wrapAngle(
        visible.bearing +
          deterministicNoise(config, time, index, 2) / config.maxRange,
      );
      return {
        observationId: `${config.id}:${index}`,
        source: "radar" as const,
        timestamp: time,
        debugObjectId: visible.object.id,
        relativePosition: {
          x: range * Math.cos(bearing),
          y: range * Math.sin(bearing),
        },
        range,
        bearing,
        radialVelocity,
      };
    },
  );
  return { sensorId: config.id, timestamp: time, pose, detections };
}
