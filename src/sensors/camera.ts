import type { VehicleState } from "../simulation/types";
import type { WorldObject } from "../world/types";
import type { DetectionFrame, SensorConfig } from "./types";
import { deterministicNoise, sensorPose, visibleObjects } from "./geometry";

/** Synthetic geometric class observations, not neural detections or confidence estimates. */
export function generateCamera(
  vehicle: VehicleState,
  objects: WorldObject[],
  config: SensorConfig,
  time: number,
): DetectionFrame {
  const pose = sensorPose(vehicle, config);
  const detections = visibleObjects(pose, objects, config).map(
    (visible, index) => {
      const range = Math.max(
        0,
        Math.min(
          config.maxRange,
          visible.range + deterministicNoise(config, time, index),
        ),
      );
      return {
        observationId: `${config.id}:${index}`,
        source: "camera" as const,
        timestamp: time,
        objectClass: visible.object.objectClass,
        debugObjectId: visible.object.id,
        relativePosition: {
          x: Math.cos(visible.bearing) * range,
          y: Math.sin(visible.bearing) * range,
        },
        range,
        bearing: visible.bearing,
      };
    },
  );
  return { sensorId: config.id, timestamp: time, pose, detections };
}
