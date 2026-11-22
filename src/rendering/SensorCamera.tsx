import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import {
  PerspectiveCamera,
  SRGBColorSpace,
  Vector4,
  WebGLRenderTarget,
} from "three";
import type { Simulation } from "../simulation/clock";
import type { DetectionFrame } from "../sensors/types";

const WIDTH = 320;
const HEIGHT = 180;
const CAMERA_HEIGHT = 1.48;

/** Shares the primary scene; image sampling follows synthetic-camera timestamps. */
export function SensorCamera({
  simulation,
  enabled,
}: {
  simulation: Simulation;
  enabled: boolean;
}) {
  const sampledFrame = useRef<DetectionFrame | null>(null);
  const resources = useMemo(() => {
    const target = new WebGLRenderTarget(WIDTH, HEIGHT);
    target.texture.colorSpace = SRGBColorSpace;
    return {
      target,
      camera: new PerspectiveCamera(60, WIDTH / HEIGHT, 0.1, 160),
      pixels: new Uint8Array(WIDTH * HEIGHT * 4),
      viewport: new Vector4(),
      scissor: new Vector4(),
    };
  }, []);
  useEffect(() => () => resources.target.dispose(), [resources]);
  useFrame(({ gl, scene }) => {
    if (!enabled) {
      sampledFrame.current = null;
      return;
    }
    const sensors = simulation.sensors;
    const frame = sensors.camera;
    const config = sensors.configs.find((sensor) => sensor.source === "camera");
    if (!frame || !config || frame === sampledFrame.current) return;
    const canvas = document.getElementById("camera-image");
    if (!(canvas instanceof HTMLCanvasElement)) return;
    const context = canvas.getContext("2d");
    if (!context) return;
    const camera = resources.camera;
    // The measurement pose already contains the configured mount and yaw.
    camera.position.set(frame.pose.x, CAMERA_HEIGHT, -frame.pose.y);
    camera.lookAt(
      frame.pose.x + Math.cos(frame.pose.heading) * 20,
      CAMERA_HEIGHT,
      -frame.pose.y - Math.sin(frame.pose.heading) * 20,
    );
    camera.fov =
      (2 * Math.atan(Math.tan(config.fov / 2) / camera.aspect) * 180) / Math.PI;
    camera.far = Math.max(100, config.maxRange * 2);
    camera.updateProjectionMatrix();

    const previousTarget = gl.getRenderTarget();
    gl.getViewport(resources.viewport);
    gl.getScissor(resources.scissor);
    const previousScissorTest = gl.getScissorTest();
    const previousAutoClear = gl.autoClear;
    const previousXr = gl.xr.enabled;
    // Debug overlays are not part of the optical scene. These marked groups are
    // direct scene children, so no full scene traversal is necessary.
    const debugGroups = scene.children.filter(
      (child) =>
        child.userData.sensorDebug || child.userData.sensorCameraHidden,
    );
    const visibility = debugGroups.map((group) => group.visible);
    // These render-only ground-truth tags never enter sensor observations or
    // planning. Rewind dynamic visuals to the measurement acquisition time.
    const acquisitionTraffic = simulation.cameraTraffic;
    const actorGroups =
      scene.children
        .find((child) => child.userData.trafficScene)
        ?.children.filter((child) => child.userData.trafficActorId) ?? [];
    const signalGroups =
      scene.children
        .find((child) => child.userData.trafficSignals)
        ?.children.filter((child) => child.userData.signalId) ?? [];
    const actorTransforms = actorGroups.map((object) => ({
      object,
      position: object.position.clone(),
      quaternion: object.quaternion.clone(),
      visible: object.visible,
    }));
    const currentSignals = simulation.traffic.signals;
    try {
      if (acquisitionTraffic) {
        for (const object of actorGroups) {
          const actor = acquisitionTraffic.actors.find(
            (item) => item.id === object.userData.trafficActorId,
          );
          object.visible = actor !== undefined;
          if (actor) {
            object.position.set(actor.state.x, 0, -actor.state.y);
            object.rotation.y = actor.state.heading;
          }
        }
        for (const object of signalGroups) {
          const signal = acquisitionTraffic.signals.find(
            (item) => item.id === object.userData.signalId,
          );
          object.userData.applySignalColor?.(signal?.color);
        }
      }
      for (const group of debugGroups) group.visible = false;
      gl.xr.enabled = false;
      gl.autoClear = true;
      gl.setRenderTarget(resources.target);
      gl.setViewport(0, 0, WIDTH, HEIGHT);
      gl.setScissorTest(false);
      gl.clear();
      gl.render(scene, camera);
      gl.readRenderTargetPixels(
        resources.target,
        0,
        0,
        WIDTH,
        HEIGHT,
        resources.pixels,
      );
      const image = context.createImageData(WIDTH, HEIGHT);
      const rowBytes = WIDTH * 4;
      for (let y = 0; y < HEIGHT; y++)
        image.data.set(
          resources.pixels.subarray(
            (HEIGHT - 1 - y) * rowBytes,
            (HEIGHT - y) * rowBytes,
          ),
          y * rowBytes,
        );
      context.putImageData(image, 0, 0);
      sampledFrame.current = frame;
    } finally {
      for (const saved of actorTransforms) {
        saved.object.position.copy(saved.position);
        saved.object.quaternion.copy(saved.quaternion);
        saved.object.visible = saved.visible;
      }
      if (acquisitionTraffic)
        for (const object of signalGroups) {
          const signal = currentSignals.find(
            (item) => item.id === object.userData.signalId,
          );
          object.userData.applySignalColor?.(signal?.color);
        }
      debugGroups.forEach((group, index) => {
        group.visible = visibility[index];
      });
      gl.setRenderTarget(previousTarget);
      gl.setViewport(resources.viewport);
      gl.setScissor(resources.scissor);
      gl.setScissorTest(previousScissorTest);
      gl.autoClear = previousAutoClear;
      gl.xr.enabled = previousXr;
    }
  });
  return null;
}
