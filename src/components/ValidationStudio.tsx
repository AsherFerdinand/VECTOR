import { useEffect, useState } from "react";
import { VALIDATION_SCENARIOS } from "../validation/scenarios";
import { runScenario } from "../validation/runner";
import {
  compareResults,
  engineeringReport,
  exportCSV,
  exportJSON,
} from "../validation/exports";
import type { ValidationResult } from "../validation/types";

const display = (value: unknown) =>
  typeof value === "number"
    ? Number.isInteger(value)
      ? String(value)
      : value.toFixed(3)
    : value === null || value === undefined
      ? "—"
      : String(value);
const labels: Record<string, string> = {
  collisionCount: "Collisions",
  impactSpeed: "Impact speed · m/s",
  minimumClearance: "Minimum clearance · m",
  minimumTTC: "Minimum TTC · s",
  followingDistanceRMSE: "Following error RMSE · m",
  aebResponseTime: "AEB response · s",
  maximumDeceleration: "Peak deceleration · m/s²",
  maximumLateralAcceleration: "Peak lateral acceleration · m/s²",
  maximumCrossTrackError: "Peak cross-track error · m",
  roadDepartures: "Road departures",
  laneDepartures: "Lane departures",
  routeCompleted: "Route completed",
  trackingPositionRMSE: "Tracking position RMSE · m",
  trackingVelocityRMSE: "Tracking velocity RMSE · m/s",
  positionCovarianceConsistency: "Position covariance consistency",
  trackingMatches: "Tracking matches",
  interventions: "Controller interventions",
  manoeuvreOutcome: "Manoeuvre outcome",
};
function download(content: string, extension: string, id: string) {
  const blob = new Blob([content], {
    type:
      extension === "json"
        ? "application/json"
        : extension === "csv"
          ? "text/csv"
          : "text/plain",
  });
  const url = URL.createObjectURL(blob),
    link = document.createElement("a");
  link.href = url;
  link.download = `vector-${id.replace(/[^a-z0-9_-]/gi, "-")}.${extension}`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
export function ValidationStudio() {
  const [preset, setPreset] = useState(VALIDATION_SCENARIOS[0].id);
  const [seed, setSeed] = useState(42),
    [timeGap, setTimeGap] = useState(1.5),
    [noise, setNoise] = useState(0);
  const [results, setResults] = useState<ValidationResult[]>([]),
    [selected, setSelected] = useState(0),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const [index, setIndex] = useState(0),
    [playing, setPlaying] = useState(false),
    [truth, setTruth] = useState(false),
    [baseline, setBaseline] = useState<ValidationResult | null>(null);
  const result = results[selected],
    frame = result?.frames[index];
  useEffect(() => {
    setIndex(0);
    setPlaying(false);
  }, [selected, results]);
  useEffect(() => {
    if (!playing || !result) return;
    const timer = setInterval(
      () =>
        setIndex((i) => {
          if (i >= result.frames.length - 1) {
            setPlaying(false);
            return i;
          }
          return i + 1;
        }),
      100,
    );
    return () => clearInterval(timer);
  }, [playing, result]);
  const run = async (batch: boolean) => {
    setBusy(true);
    setError("");
    setPlaying(false);
    try {
      const configs = (
        batch
          ? VALIDATION_SCENARIOS.slice(0, 12)
          : VALIDATION_SCENARIOS.filter((p) => p.id === preset)
      ).map((p) => ({
        ...p.config,
        seed,
        timeGap,
        noise: noise || p.config.noise,
      }));
      const next: ValidationResult[] = [];
      for (const config of configs) {
        await new Promise((resolve) => setTimeout(resolve, 0));
        next.push(runScenario(config));
        setResults([...next]);
      }
      setSelected(0);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };
  let viewBox = "-60 -60 120 120";
  if (result?.frames.length) {
    const xs = result.frames.map((f) => f.vehicle.x),
      ys = result.frames.map((f) => -f.vehicle.y);
    const minX = Math.min(...xs) - 10,
      minY = Math.min(...ys) - 10;
    viewBox = `${minX} ${minY} ${Math.max(30, Math.max(...xs) - minX + 10)} ${Math.max(30, Math.max(...ys) - minY + 10)}`;
  }
  return (
    <section className="engineering-panel" aria-label="Validation studio">
      <div className="sensor-heading">
        <div>
          <span className="eyebrow">SEEDED FIXED-STEP EXPERIMENTS</span>
          <h2>Validation studio</h2>
        </div>
        <span className="sensor-badge">RECORDED-STATE PLAYBACK</span>
      </div>
      <div className="sensor-panel">
        <div className="validation-form">
          <label>
            Scenario
            <select
              aria-label="Validation scenario"
              value={preset}
              onChange={(e) => setPreset(e.target.value)}
            >
              {VALIDATION_SCENARIOS.map((p) => (
                <option key={p.id} value={p.id}>
                  {p.label}
                </option>
              ))}
            </select>
          </label>
          <label>
            Seed
            <input
              aria-label="Validation seed"
              type="number"
              step="1"
              value={seed}
              onChange={(e) => setSeed(Number(e.target.value))}
            />
          </label>
          <label>
            Time gap · s
            <input
              type="number"
              min="1"
              max="3"
              step=".1"
              value={timeGap}
              onChange={(e) => setTimeGap(Number(e.target.value))}
            />
          </label>
          <label>
            Sensor noise
            <input
              type="number"
              min="0"
              max="2"
              step=".05"
              value={noise}
              onChange={(e) => setNoise(Number(e.target.value))}
            />
          </label>
        </div>
        <p className="sensor-model-note">
          {VALIDATION_SCENARIOS.find((p) => p.id === preset)?.description} Runs
          use isolated simulations, without rendering; the live Drive experiment
          stays paused. Noise 0 preserves each preset's default. Results are
          bounded in memory.
        </p>
        <div className="validation-actions">
          <button
            className="primary"
            disabled={busy}
            onClick={() => void run(false)}
          >
            {busy ? "Running…" : "Run scenario"}
          </button>
          <button disabled={busy} onClick={() => void run(true)}>
            Run batch (up to 12)
          </button>
          {result && (
            <>
              <button onClick={() => setBaseline(result)}>
                Pin as A baseline
              </button>
              <button
                onClick={() =>
                  download(exportJSON(result), "json", result.config.id)
                }
              >
                Export JSON
              </button>
              <button
                onClick={() =>
                  download(exportCSV(result), "csv", result.config.id)
                }
              >
                Export CSV
              </button>
              <button
                onClick={() =>
                  download(engineeringReport(result), "txt", result.config.id)
                }
              >
                Engineering report
              </button>
            </>
          )}
        </div>
        {error && (
          <p role="alert" className="validation-fail">
            {error}
          </p>
        )}
        {!!results.length && (
          <div className="validation-actions">
            {results.map((r, i) => (
              <button
                key={`${r.config.id}-${i}`}
                aria-pressed={selected === i}
                onClick={() => setSelected(i)}
              >
                {r.config.id} · {r.passed ? "PASS" : "FAIL"}
              </button>
            ))}
          </div>
        )}
        {result && (
          <>
            <p
              className={result.passed ? "validation-pass" : "validation-fail"}
            >
              {result.passed ? "PASS" : "FAIL"} · {result.config.id} · seed{" "}
              {result.config.seed}
              {result.failures.length ? ` · ${result.failures.join("; ")}` : ""}
            </p>
            <div className="validation-metrics">
              {Object.entries(result.metrics).map(([key, value]) => (
                <div key={key}>
                  <span>{labels[key] ?? key}</span>
                  <strong>{display(value)}</strong>
                </div>
              ))}
            </div>
            <div className="validation-replay">
              <h3>Recorded replay</h3>
              <div className="validation-actions">
                <button
                  disabled={!result.frames.length}
                  aria-pressed={playing}
                  onClick={() => setPlaying((p) => !p)}
                >
                  {playing ? "Pause replay" : "Play replay"}
                </button>
                <span className="sensor-model-note">
                  {frame?.time.toFixed(2) ?? "0.00"} s ·{" "}
                  {frame?.behaviour.state ?? "No recorded frame"}
                </span>
                <label className="sensor-overlay-toggle">
                  <input
                    type="checkbox"
                    checked={truth}
                    onChange={(e) => setTruth(e.target.checked)}
                  />
                  Ground-truth comparison (validation only)
                </label>
              </div>
              <input
                aria-label="Replay position"
                type="range"
                min="0"
                max={Math.max(0, result.frames.length - 1)}
                value={index}
                onChange={(e) => {
                  setPlaying(false);
                  setIndex(Number(e.target.value));
                }}
              />
              <svg
                viewBox={viewBox}
                role="img"
                aria-label="Recorded ego trajectory and estimated objects"
              >
                <polyline
                  points={result.frames
                    .map((f) => `${f.vehicle.x},${-f.vehicle.y}`)
                    .join(" ")}
                  fill="none"
                  stroke="#a7e8c8"
                  strokeWidth=".2"
                />
                {frame && (
                  <>
                    <circle
                      cx={frame.vehicle.x}
                      cy={-frame.vehicle.y}
                      r=".8"
                      fill="#a7e8c8"
                    />
                    {frame.tracks?.map((t) => (
                      <g key={t.id}>
                        <circle cx={t.x} cy={-t.y} r=".5" fill="#74c7f6" />
                        <ellipse
                          cx={t.x}
                          cy={-t.y}
                          rx={
                            2 *
                            Math.sqrt(Math.max(0, t.covariance[0]?.[0] ?? 0))
                          }
                          ry={
                            2 *
                            Math.sqrt(Math.max(0, t.covariance[1]?.[1] ?? 0))
                          }
                          fill="none"
                          stroke="#74c7f6"
                          strokeWidth=".1"
                        />
                      </g>
                    ))}
                    {truth &&
                      frame.truth?.map((t) => (
                        <path
                          key={t.id}
                          d={`M ${t.x - 0.5} ${-t.y} h 1 M ${t.x} ${-t.y - 0.5} v 1`}
                          stroke="#e9bd79"
                          strokeWidth=".15"
                        />
                      ))}
                  </>
                )}
              </svg>
              <p className="sensor-model-note">
                Green: ego path · Blue: estimated surfaces and marginal 2σ
                uncertainty · Amber: truth object centres. Playback reads
                recorded states; it does not re-run physics. Surface/centre
                mismatch contributes to reported tracking errors.
              </p>
            </div>
            <h3>Decision & safety timeline</h3>
            <ol className="validation-events">
              {result.events.map((event, i) => (
                <li key={i}>
                  {event.time.toFixed(2)} s · {event.kind} · {event.message}
                </li>
              ))}
            </ol>
            {baseline && (
              <>
                <h3>A/B comparison</h3>
                <p className="sensor-model-note">
                  A: {baseline.config.id} / gap {baseline.config.timeGap ?? 1.5}{" "}
                  · B: {result.config.id} / gap {result.config.timeGap ?? 1.5}.
                  Positive delta means B − A; compare matching scenario and
                  seed.
                </p>
                <div className="sensor-detections">
                  <table>
                    <thead>
                      <tr>
                        <th>Metric</th>
                        <th>A</th>
                        <th>B</th>
                        <th>Δ</th>
                      </tr>
                    </thead>
                    <tbody>
                      {compareResults(baseline, result).map((row) => (
                        <tr key={row.metric}>
                          <td>{labels[row.metric] ?? row.metric}</td>
                          <td>{display(row.a)}</td>
                          <td>{display(row.b)}</td>
                          <td>{display(row.delta)}</td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </>
            )}
          </>
        )}
      </div>
    </section>
  );
}
