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
  const keyboard = useRef<ReturnType<typeof keyboardControls> | null>(null);
  const refresh = () => setSnapshot(simulation.snapshot);
  const reset = () => {
    keyboard.current?.clear();
    simulation.reset();
    refresh();
  };
  useEffect(() => {
    const controls = keyboardControls(() => {
      simulation.reset();
      setSnapshot(simulation.snapshot);
    }, setCamera);
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
          <span>SIMULATION FOUNDATION / v0.1</span>
        </div>
        <div className={`status ${snapshot.running ? "running" : ""}`}>
          <i />
          {snapshot.running ? "RUNNING" : "PAUSED"}
        </div>
      </header>
      <main>
        <section className="workspace">
          <div className="workspace-title">
            <div>
              <span className="eyebrow">EXPERIMENT 001</span>
              <h1>Manual vehicle dynamics</h1>
            </div>
            <span className="environment-label">
              VECTOR TOWN <span> / </span> DAYLIGHT
            </span>
          </div>
          <div className="viewport">
            <SceneBoundary>
              <Scene simulation={simulation} cameraMode={camera} />
            </SceneBoundary>
            <div className="viewport-label">
              <span className="live-dot" /> {cameraLabels[camera]}
              <small>GROUND TRUTH</small>
            </div>
            <div className="axis-label">X EAST · Y NORTH · SI UNITS</div>
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
        </section>
        <Dashboard state={snapshot} camera={camera} />
      </main>
      <footer>
        <span>
          VECTOR <b> / </b> Build the world. Sense the road. Navigate
          autonomously.
        </span>
        <span>EDUCATIONAL SIMULATION · v0.1</span>
      </footer>
    </div>
  );
}
