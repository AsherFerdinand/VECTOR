import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { Vector3 } from "three";
import type { Simulation } from "../simulation/clock";
import type { CameraMode } from "../simulation/types";

export function Cameras({
  simulation,
  mode,
}: {
  simulation: Simulation;
  mode: CameraMode;
}) {
  const previous = useRef<CameraMode | null>(null);
  const desired = useRef(new Vector3()),
    target = useRef(new Vector3()),
    smoothedTarget = useRef(new Vector3());
  useFrame(({ camera }, dt) => {
    const v = simulation.snapshot.vehicle,
      c = Math.cos(v.heading),
      s = Math.sin(v.heading);
    if (mode === "overview") {
      desired.current.set(v.x, 66, -v.y + 18);
      target.current.set(v.x, 0, -v.y);
    } else if (mode === "follow") {
      desired.current.set(v.x - 10 * c, 6, -v.y + 10 * s);
      target.current.set(v.x + 4 * c, 1, -v.y - 4 * s);
    } else {
      // Driver eye near the front of the cabin, clear of the opaque windscreen mesh.
      desired.current.set(
        v.x + 2.25 * c - 0.35 * s,
        1.48,
        -v.y - 2.25 * s - 0.35 * c,
      );
      target.current.set(v.x + 25 * c, 1.35, -v.y - 25 * s);
    }
    if (previous.current !== mode || mode === "cockpit") {
      camera.position.copy(desired.current);
      smoothedTarget.current.copy(target.current);
      previous.current = mode;
    } else {
      const fraction = 1 - Math.exp(-8 * Math.min(dt, 0.1));
      camera.position.lerp(desired.current, fraction);
      smoothedTarget.current.lerp(target.current, fraction);
    }
    camera.lookAt(smoothedTarget.current);
  });
  return null;
}
