import {
  useEffect,
  useRef,
  useState,
  Component,
  type ReactNode,
  type ErrorInfo,
} from "react";
import { Simulation } from "./simulation/clock";
import type { CameraMode } from "./simulation/types";
import { keyboardControls } from "./controls/keyboard";
import { Scene } from "./rendering/Scene";
import { Dashboard, cameraLabels } from "./components/Dashboard";
import { Minimap } from "./components/Minimap";
import { SensorExplorer } from "./components/SensorExplorer";
import type { SensorSource } from "./sensors/types";

class SceneBoundary extends Component<
  { children: ReactNode },
  { failed: boolean }
> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  componentDidCatch(error: Error, info: ErrorInfo) {
    console.error("3D scene failed", error, info);
  }
  render() {
    return this.state.failed ? (
      <div className="scene-error">
        3D rendering is unavailable. Enable browser hardware acceleration and
        reload.
      </div>
    ) : (
      this.props.children
    );
  }
}

export default function App() {
  const [simulation] = useState(() => new Simulation());
  const [camera, setCamera] = useState<CameraMode>("follow");
  const [snapshot, setSnapshot] = useState(() => simulation.snapshot);
  const [navigation, setNavigation] = useState(() => simulation.navigation);
  const [sensors, setSensors] = useState(() => simulation.sensors);
  const [physical, setPhysical] = useState(() => simulation.physical);
  const [safety, setSafety] = useState(() => simulation.safety);
  const [sensorTab, setSensorTab] = useState<SensorSource | "compute">("lidar");
  const [sensorOverlay, setSensorOverlay] = useState(false);
  const [collisionOverlay, setCollisionOverlay] = useState(false);
  const [engineering, setEngineering] = useState(true);
  const keyboard = useRef<ReturnType<typeof keyboardControls> | null>(null);
  const refresh = () => {
    setSnapshot(simulation.snapshot);
    setNavigation(simulation.navigation);
    setSensors(simulation.sensors);
    setPhysical(simulation.physical);
    setSafety(simulation.safety);
  };
  const reset = () => {
    keyboard.current?.clear();
    simulation.reset();
    refresh();
  };
  useEffect(() => {
    const controls = keyboardControls(
      () => {
        simulation.reset();
        setSnapshot(simulation.snapshot);
        setNavigation(simulation.navigation);
        setSensors(simulation.sensors);
        setPhysical(simulation.physical);
        setSafety(simulation.safety);
      },
      setCamera,
      () => {
        if (simulation.navigation.mode === "AUTONOMOUS") {
          simulation.cancelAutonomy();
          setNavigation(simulation.navigation);
          setSensors(simulation.sensors);
          setPhysical(simulation.physical);
          setSafety(simulation.safety);
        }
      },
    );
    keyboard.current = controls;
    let previous: number | undefined;
    let lastDisplay = 0;
    let frame = 0;
    const tick = (now: number) => {
      simulation.advance(
        previous === undefined ? 0 : (now - previous) / 1000,
        controls.read(),
      );
      previous = now;
      if (now - lastDisplay >= 100) {
        setSnapshot(simulation.snapshot);
        setNavigation(simulation.navigation);
        setSensors(simulation.sensors);
        setPhysical(simulation.physical);
        setSafety(simulation.safety);
        lastDisplay = now;
      }
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    const visibility = () => {
      previous = undefined;
      if (document.hidden) {
        simulation.pause();
        setSnapshot(simulation.snapshot);
        setNavigation(simulation.navigation);
        setSensors(simulation.sensors);
        setPhysical(simulation.physical);
        setSafety(simulation.safety);
      }
    };
    document.addEventListener("visibilitychange", visibility);
    return () => {
      cancelAnimationFrame(frame);
      controls.dispose();
      keyboard.current = null;
      document.removeEventListener("visibilitychange", visibility);
    };
  }, [simulation]);
  return (
    <div className="workbench">
      <header className="topbar">
        <a className="brand" href="." aria-label="VECTOR home">
          <span className="brand-mark">V</span>
          <span>
            VECTOR<small>ENGINEERING STUDIO</small>
          </span>
        </a>
        <div className="project-name">
          Autonomous Driving & Engineering Studio
          <span>COLLISION & SYNTHETIC PERCEPTION / v0.3</span>
        </div>
        <span className="mode-badge" data-mode={navigation.mode}>
          {navigation.mode}
        </span>
        <div className={`status ${snapshot.running ? "running" : ""}`}>
          <i />
          {snapshot.running ? "RUNNING" : "PAUSED"}
        </div>
      </header>
      <main>
        <section className="workspace">
          <div className="workspace-title">
            <div>
              <span className="eyebrow">EXPERIMENT 003</span>
              <h1>Driving & perception laboratory</h1>
            </div>
            <span className="environment-label">
              VECTOR TOWN <span> / </span> DAYLIGHT
            </span>
          </div>
          <div className="viewport">
            <SceneBoundary>
              <Scene
                simulation={simulation}
                cameraMode={camera}
                engineering={engineering}
                sensorTab={sensorTab}
                sensorOverlay={sensorOverlay}
                collisionOverlay={collisionOverlay}
              />
            </SceneBoundary>
            <div className="viewport-label">
              <span className="live-dot" /> {cameraLabels[camera]}
              <small>GROUND TRUTH</small>
            </div>
            <div className="axis-label">PERFECT LOCALISATION · STATIC MAP</div>
            <div className="viewport-caption">
              SIMULATION ENVIRONMENT<span>01 / COMPACT TOWN</span>
            </div>
          </div>
          <div
            className="toolbar"
            onClick={(event) => {
              // Mouse-operated controls return keyboard driving to the viewport.
              if (event.detail > 0 && event.target instanceof HTMLElement) {
                event.target.closest("button")?.blur();
              }
            }}
          >
            <div className="transport">
              <button
                className="primary"
                onClick={() => {
                  simulation.start();
                  refresh();
                }}
                disabled={snapshot.running}
              >
                ▶ {snapshot.time > 0 ? "Resume" : "Start"}
              </button>
              <button
                onClick={() => {
                  keyboard.current?.clear();
                  simulation.pause();
                  refresh();
                }}
                disabled={!snapshot.running}
              >
                Ⅱ Pause
              </button>
              <button onClick={reset}>↺ Reset</button>
            </div>
            <div className="camera-controls" aria-label="Camera selection">
              {(["overview", "follow", "cockpit"] as const).map(
                (mode, index) => (
                  <button
                    key={mode}
                    aria-pressed={camera === mode}
                    onClick={() => setCamera(mode)}
                  >
                    <kbd>{index + 1}</kbd>
                    {cameraLabels[mode]}
                  </button>
                ),
              )}
            </div>
          </div>
          <div
            className="navigation-strip"
            aria-label="Live navigation summary"
          >
            <div>
              <span>Navigation state</span>
              <strong>{navigation.state.replaceAll("_", " ")}</strong>
            </div>
            <div>
              <span>Target speed</span>
              <strong>
                {navigation.targetSpeed === null
                  ? "—"
                  : (navigation.targetSpeed * 3.6).toFixed(1) + " km/h"}
              </strong>
            </div>
            <div>
              <span>Cross-track error</span>
              <strong>
                {navigation.crossTrackError === null
                  ? "—"
                  : navigation.crossTrackError.toFixed(2) + " m"}
              </strong>
            </div>
            <div>
              <span>Route progress</span>
              <strong>
                {navigation.mode === "AUTONOMOUS"
                  ? (navigation.progress * 100).toFixed(1) + " %"
                  : "—"}
              </strong>
            </div>
          </div>
          <div className="view-options">
            <label>
              <input
                type="checkbox"
                checked={engineering}
                onChange={(e) => setEngineering(e.target.checked)}
              />{" "}
              Engineering overlay
            </label>
            <label>
              <input
                type="checkbox"
                checked={collisionOverlay}
                onChange={(e) => setCollisionOverlay(e.target.checked)}
              />{" "}
              Collision & road geometry
            </label>
            <span>Turquoise: planned · Amber: actual · Blue: measured</span>
          </div>
          <div className="keyboard-reference">
            <span className="eyebrow">MANUAL CONTROLS</span>
            <span>
              <kbd>W</kbd>
              <kbd>↑</kbd> Accelerate
            </span>
            <span>
              <kbd>S</kbd>
              <kbd>↓</kbd> Brake / reverse
            </span>
            <span>
              <kbd>A</kbd>
              <kbd>D</kbd>
              <kbd>←</kbd>
              <kbd>→</kbd> Steer
            </span>
            <span>
              <kbd>SPACE</kbd> Brake
            </span>
            <span>
              <kbd>R</kbd> Reset
            </span>
          </div>
          <div
            className="physical-strip"
            aria-label="Physical and safety status"
          >
            <div>
              <span>Road footprint</span>
              <strong
                className={physical.roadStatus === "ON_ROAD" ? "" : "warning"}
              >
                {physical.roadStatus.replaceAll("_", " ")}
              </strong>
            </div>
            <div>
              <span>Physical contacts / departures</span>
              <strong>
                {physical.collisionCount} / {physical.roadDepartureCount}
              </strong>
            </div>
            <div>
              <span>Sensor protection</span>
              <strong className={safety.braking ? "warning" : ""}>
                {safety.state.replaceAll("_", " ")}
              </strong>
            </div>
          </div>
          <SensorExplorer
            simulation={simulation}
            sensors={sensors}
            safety={safety}
            physical={physical}
            navigation={navigation}
            vehicle={snapshot.vehicle}
            tab={sensorTab}
            onTab={(tab) => {
              setSensorTab(tab);
              setSensorOverlay(false);
            }}
            overlay={sensorOverlay}
            onOverlay={setSensorOverlay}
          />
        </section>
        <div
          className="sidebar"
          onClick={(event) => {
            if (event.detail > 0 && event.target instanceof HTMLElement)
              event.target.closest("button")?.blur();
          }}
        >
          <Minimap
            graph={simulation.graph}
            navigation={navigation}
            vehicle={snapshot.vehicle}
            canSelectStart={
              !snapshot.running && Math.abs(snapshot.vehicle.velocity) <= 0.01
            }
            onStart={(position) => {
              keyboard.current?.clear();
              const accepted = simulation.setStart(position);
              refresh();
              return accepted;
            }}
            onDestination={(position) => {
              const accepted = simulation.planTo(position);
              refresh();
              return accepted;
            }}
            onActivate={() => {
              keyboard.current?.clear();
              simulation.activateAutonomy();
              refresh();
            }}
            onCancel={() => {
              keyboard.current?.clear();
              simulation.cancelAutonomy();
              refresh();
            }}
            onClear={() => {
              simulation.clearRoute();
              refresh();
            }}
          />
          <Dashboard state={snapshot} camera={camera} navigation={navigation} />
        </div>
      </main>
      <footer>
        <span>
          VECTOR <b> / </b> Build the world. Sense the road. Navigate
          autonomously.
        </span>
        <span>EDUCATIONAL SIMULATION · v0.3</span>
      </footer>
    </div>
  );
}
