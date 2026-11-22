import { Canvas } from "@react-three/fiber";
import type { Simulation } from "../simulation/clock";
import type { CameraMode } from "../simulation/types";
import { Cameras } from "./Cameras";
import { Town } from "./Town";
import { Traffic } from "./Traffic";
import { Vehicle } from "./Vehicle";
import { RouteOverlay } from "./RouteOverlay";
import { SensorOverlays } from "./SensorOverlays";
import { SensorCamera } from "./SensorCamera";
import type { SensorSource } from "../sensors/types";

/** Rendering reads ground truth only; physics is advanced by the application clock. */
export function Scene({
  simulation,
  cameraMode,
  engineering = true,
  sensorTab = "lidar",
  sensorOverlay = false,
  collisionOverlay = false,
}: {
  simulation: Simulation;
  cameraMode: CameraMode;
  engineering?: boolean;
  sensorTab?: SensorSource | "compute";
  sensorOverlay?: boolean;
  collisionOverlay?: boolean;
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
      <Town simulation={simulation} />
      <Traffic simulation={simulation} />
      <group userData={{ sensorDebug: true }}>
        <RouteOverlay simulation={simulation} engineering={engineering} />
      </group>
      <group userData={{ sensorCameraHidden: true }}>
        <Vehicle simulation={simulation} />
      </group>
      <Cameras simulation={simulation} mode={cameraMode} />
      <SensorOverlays
        simulation={simulation}
        source={sensorTab}
        enabled={sensorOverlay}
        collisionEnabled={collisionOverlay}
      />
      <SensorCamera simulation={simulation} enabled={sensorTab === "camera"} />
    </Canvas>
  );
}
