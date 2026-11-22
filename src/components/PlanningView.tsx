import { useEffect, useMemo, useState } from "react";
import {
  CorridorSimulation,
  type CorridorFrame,
  type CorridorMetrics,
} from "../planning/corridorSimulation";
import { ManoeuvrePlanner } from "../planning/manoeuvrePlanner";
import { CORRIDOR } from "../planning/corridor";
import type { Track } from "../perception/types";
import type { BehaviourSnapshot } from "../autonomy/behaviourTypes";

/** An explicitly labelled map/planner diagnostic, never a fabricated live sensor feed. */
export function PlanningView({ behaviour }: { behaviour: BehaviourSnapshot }) {
  const [runScenario, setRunScenario] = useState<
    "lane-change" | "overtaking" | "blocked"
  >("overtaking");
  const [recording, setRecording] = useState<readonly CorridorFrame[]>([]);
  const [metrics, setMetrics] = useState<CorridorMetrics | null>(null);
  const [replayIndex, setReplayIndex] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [running, setRunning] = useState(false);
  useEffect(() => {
    if (!playing) return;
    const timer = setInterval(
      () =>
        setReplayIndex((index) => {
          if (index >= recording.length - 1) {
            setPlaying(false);
            return index;
          }
          return index + 1;
        }),
      100,
    );
    return () => clearInterval(timer);
  }, [playing, recording]);
  const recorded = recording[replayIndex];
  const execute = async () => {
    setRunning(true);
    setPlaying(false);
    try {
      await new Promise((resolve) => setTimeout(resolve, 0));
      const experiment = new CorridorSimulation({ scenario: runScenario });
      setMetrics(experiment.run(60));
      setRecording(experiment.recording);
      setReplayIndex(0);
    } finally {
      setRunning(false);
    }
  };
  const [blocked, setBlocked] = useState(false),
    [uncertainty, setUncertainty] = useState(0.1);
  const preview = useMemo(() => {
    const vehicle = { x: -20, y: -90.25, heading: 0, velocity: 6 };
    const track = (id: string, x: number, y: number, vx: number): Track => ({
      id,
      x,
      y,
      vx,
      vy: 0,
      covariance: [
        [uncertainty, 0, 0, 0],
        [0, uncertainty, 0, 0],
        [0, 0, 0.01, 0],
        [0, 0, 0, 0.01],
      ],
      status: "confirmed",
      lastMeasurementTime: 0,
      age: 1,
      sources: ["radar", "camera"],
      objectClass: "vehicle",
    });
    const tracks = [
      track("test-lead", 10, -90.25, 1),
      ...(blocked ? [track("test-adjacent", 0, -86.75, 0)] : []),
    ];
    const planner = new ManoeuvrePlanner();
    planner.update(vehicle, tracks, 0);
    const snapshot = planner.update(vehicle, tracks, 0.51);
    return { vehicle, tracks, snapshot };
  }, [blocked, uncertainty]);
  return (
    <section
      className="engineering-panel"
      aria-label="Trajectory planning diagnostic"
    >
      <div className="sensor-heading">
        <div>
          <span className="eyebrow">EXPLICIT MULTILANE TEST MAP</span>
          <h2>Trajectory feasibility</h2>
        </div>
        <span className="sensor-badge">PLANNER DIAGNOSTIC</span>
      </div>
      <div className="sensor-panel">
        <p className="sensor-safety-reason">
          Live control perception: {behaviour.perceptionMode ?? "raw"} ·{" "}
          {behaviour.reason}
        </p>
        <h3>Closed-loop corridor experiment</h3>
        <p className="sensor-model-note">
          Actual SensorSuite → fusion → manoeuvre planner → ACC/AEB → bicycle
          dynamics, recorded on the explicitly mapped corridor. Ground-truth
          actor geometry is restricted to sensor generation, collision
          enforcement and playback.
        </p>
        <div className="validation-form">
          <label>
            Experiment
            <select
              value={runScenario}
              onChange={(e) =>
                setRunScenario(e.target.value as typeof runScenario)
              }
            >
              <option value="overtaking">Overtake and return</option>
              <option value="lane-change">Lane-change fixture</option>
              <option value="blocked">Stationary lead</option>
            </select>
          </label>
          <button
            className="primary"
            disabled={running}
            onClick={() => void execute()}
          >
            {running ? "Running corridor…" : "Run corridor experiment"}
          </button>
        </div>
        {metrics && (
          <>
            <div className="validation-metrics">
              {Object.entries(metrics).map(([key, value]) => (
                <div key={key}>
                  <span>{key.replace(/([A-Z])/g, " $1")}</span>
                  <strong>
                    {typeof value === "number"
                      ? value.toFixed(3)
                      : String(value)}
                  </strong>
                </div>
              ))}
            </div>
            <div className="validation-actions">
              <button
                aria-pressed={playing}
                onClick={() => setPlaying((value) => !value)}
              >
                {playing ? "Pause corridor replay" : "Play corridor replay"}
              </button>
              <span className="sensor-model-note">
                {recorded?.time.toFixed(2)} s · {recorded?.manoeuvre.state} ·{" "}
                {recorded?.behaviour.state}
              </span>
            </div>
            <input
              className="corridor-seek"
              type="range"
              aria-label="Corridor replay position"
              min="0"
              max={Math.max(0, recording.length - 1)}
              value={replayIndex}
              onChange={(e) => {
                setPlaying(false);
                setReplayIndex(Number(e.target.value));
              }}
            />
            <svg
              className="planning-plot"
              viewBox="-55 77 220 17"
              role="img"
              aria-label="Recorded closed-loop overtaking trajectory"
            >
              <rect x="-55" y="78" width="220" height="14" fill="#1e2c37" />
              {CORRIDOR.lanes.map((l) => (
                <path
                  key={l.id}
                  d={`M -55 ${-l.y} H 165`}
                  stroke="#597181"
                  strokeDasharray="1 1"
                  strokeWidth=".08"
                />
              ))}
              <path d="M -55 85 H 165" stroke="#e0b670" strokeWidth=".15" />
              <polyline
                points={recording
                  .map((f) => `${f.vehicle.x},${-f.vehicle.y}`)
                  .join(" ")}
                fill="none"
                stroke="#a7e8c8"
                strokeWidth=".2"
              />
              {recorded && (
                <>
                  <g
                    transform={`translate(${recorded.vehicle.x} ${-recorded.vehicle.y}) rotate(${(-recorded.vehicle.heading * 180) / Math.PI})`}
                  >
                    <rect
                      x="-.9"
                      y="-.9"
                      width="4.5"
                      height="1.8"
                      fill="#a7e8c8"
                    />
                  </g>
                  <rect
                    x={recorded.actor.x - 0.9}
                    y={-recorded.actor.y - 0.9}
                    width="4.5"
                    height="1.8"
                    fill="#e9bd79"
                  />
                  {recorded.tracking.tracks.map((t) => (
                    <circle
                      key={t.id}
                      cx={t.x}
                      cy={-t.y}
                      r=".3"
                      fill="#74c7f6"
                    />
                  ))}
                </>
              )}
            </svg>
            <p className="sensor-model-note">
              Green: recorded ego trajectory and body · Amber: recorded truth
              actor (debug only) · Blue: actual fused estimates.{" "}
              {recorded?.manoeuvre.reason}
            </p>
            {recorded?.manoeuvre.candidates.map((candidate, i) => (
              <p className="sensor-model-note" key={i}>
                {candidate.targetLane} / {candidate.duration} s ·{" "}
                {candidate.valid ? "Feasible" : candidate.reasons.join("; ")}
              </p>
            ))}
          </>
        )}
        <h3>Candidate constraints diagnostic</h3>
        <p className="sensor-model-note">
          Core town streets have one lane per direction. This isolated planner
          diagnostic uses a separately defined four-lane corridor and synthetic,
          labelled test estimates. It does not change the live vehicle or
          demonstrate a completed town overtake.
        </p>
        <div className="validation-form">
          <label>
            Adjacent lane
            <select
              value={blocked ? "blocked" : "clear"}
              onChange={(e) => setBlocked(e.target.value === "blocked")}
            >
              <option value="clear">Clear alternative</option>
              <option value="blocked">Blocked alternative</option>
            </select>
          </label>
          <label>
            Position variance · m²
            <input
              type="number"
              min=".01"
              max="4"
              step=".1"
              value={uncertainty}
              onChange={(e) =>
                setUncertainty(Math.max(0.01, Number(e.target.value)))
              }
            />
          </label>
        </div>
        <p className="sensor-safety-reason">
          {preview.snapshot.state.replaceAll("_", " ")} ·{" "}
          {preview.snapshot.reason}
        </p>
        <svg
          className="planning-plot"
          viewBox="-30 76 100 19"
          role="img"
          aria-label="Explicit corridor lanes and candidate trajectories"
        >
          <rect
            x="-30"
            y={-CORRIDOR.maxY}
            width="100"
            height={CORRIDOR.maxY - CORRIDOR.minY}
            fill="#1e2c37"
          />
          {CORRIDOR.lanes.map((l) => (
            <g key={l.id}>
              <path
                d={`M -30 ${-l.y} H 70`}
                stroke="#597181"
                strokeDasharray="1 1"
                strokeWidth=".08"
              />
              <text x="60" y={-l.y - 0.3} fontSize=".6" fill="#92aab8">
                {l.id}
              </text>
            </g>
          ))}
          <path
            d={`M -30 ${-CORRIDOR.dividerY} H 70`}
            stroke="#e0b670"
            strokeWidth=".15"
          />
          {preview.snapshot.candidates.map((candidate, i) => (
            <polyline
              key={candidate.id + i}
              points={candidate.points.map((p) => `${p.x},${-p.y}`).join(" ")}
              fill="none"
              stroke={candidate.valid ? "#a7e8c8" : "#ed987f"}
              opacity={0.8}
              strokeWidth=".12"
            />
          ))}
          <rect
            x={preview.vehicle.x - 0.9}
            y={-preview.vehicle.y - 0.9}
            width="4.5"
            height="1.8"
            fill="#a7e8c8"
          />
          {preview.tracks.map((t) => (
            <rect
              key={t.id}
              x={t.x - 2.25}
              y={-t.y - 0.9}
              width="4.5"
              height="1.8"
              fill="#74c7f6"
            />
          ))}
        </svg>
        <div className="sensor-detections">
          <table>
            <thead>
              <tr>
                <th>Candidate</th>
                <th>Feasible</th>
                <th>Peak lateral acceleration</th>
                <th>Constraints / rejection</th>
              </tr>
            </thead>
            <tbody>
              {preview.snapshot.candidates.map((c, i) => (
                <tr key={i}>
                  <td>
                    {c.targetLane} / {c.duration.toFixed(1)} s
                  </td>
                  <td>{c.valid ? "Yes" : "No"}</td>
                  <td>{c.maxLateralAcceleration.toFixed(3)} m/s²</td>
                  <td>
                    {c.reasons.join("; ") ||
                      "Road footprint, speed, curvature and predicted uncertainty envelopes satisfied"}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </section>
  );
}
