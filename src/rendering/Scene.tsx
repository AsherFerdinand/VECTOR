import { Canvas } from "@react-three/fiber";
import type { Simulation } from "../simulation/clock";
import type { CameraMode } from "../simulation/types";
import { Cameras } from "./Cameras";
import { Town } from "./Town";
import { Vehicle } from "./Vehicle";

/** Rendering reads ground truth only; physics is advanced by the application clock. */
export function Scene({
  simulation,
  cameraMode,
}: {
  simulation: Simulation;
  cameraMode: CameraMode;
}) {
  return (
    <Canvas
      dpr={[1, 1.5]}
      camera={{ position: [-20, 66, 45], fov: 48, near: 0.1, far: 650 }}
      gl={{ antialias: true, powerPreference: "low-power" }}
    >
      <color attach="background" args={["#9db5b6"]} />
      <fog attach="fog" args={["#9db5b6", 120, 330]} />
      <ambientLight intensity={1.5} />
      <directionalLight position={[35, 70, 20]} intensity={2.1} />
      <Town />
      <Vehicle simulation={simulation} />
      <Cameras simulation={simulation} mode={cameraMode} />
    </Canvas>
  );
}
