import type { TrackingSnapshot } from "../perception/types";
import type { VehicleState } from "../simulation/types";

export function PerceptionTracks({
  tracking,
  vehicle,
}: {
  tracking: TrackingSnapshot;
  vehicle: VehicleState;
}) {
  const c = Math.cos(vehicle.heading),
    s = Math.sin(vehicle.heading);
  const tracks = tracking.tracks.slice(0, 60);
  return (
    <section className="engineering-panel" aria-label="Fused object tracks">
      <div className="sensor-heading">
        <div>
          <span className="eyebrow">CONSTANT-VELOCITY ESTIMATION</span>
          <h2>Fused object tracks</h2>
        </div>
        <span className="sensor-badge">
          {tracks.length} tracks · {tracking.timestamp.toFixed(2)} s
        </span>
      </div>
      <div className="sensor-panel">
        <svg
          className="tracking-plot"
          viewBox="-45 -35 90 70"
          role="img"
          aria-label="Ego-relative tracks and positional covariance"
        >
          <path
            d="M -40 0 H 40 M 0 -30 V 30"
            stroke="#294355"
            strokeWidth=".15"
          />
          {[10, 20, 30].map((r) => (
            <circle
              key={r}
              r={r}
              fill="none"
              stroke="#243846"
              strokeWidth=".1"
            />
          ))}
          <rect x="-.9" y="-3.6" width="1.8" height="4.5" fill="#a7e8c8" />
          {tracks.map((track) => {
            const dx = track.x - vehicle.x,
              dy = track.y - vehicle.y;
            const forward = c * dx + s * dy,
              left = -s * dx + c * dy;
            if (Math.abs(forward) > 34 || Math.abs(left) > 44) return null;
            const p = track.covariance;
            const xx = p[0]?.[0] ?? 0,
              yy = p[1]?.[1] ?? 0,
              xy = p[0]?.[1] ?? 0;
            // Rotate world covariance into plot axes (-left,-forward).
            const a = s * s * xx + c * c * yy - 2 * s * c * xy;
            const d = c * c * xx + s * s * yy + 2 * s * c * xy;
            const b = -s * c * xx + s * c * yy + (c * c - s * s) * xy;
            const delta = Math.sqrt(Math.max(0, (a - d) ** 2 + 4 * b * b));
            const major = 2 * Math.sqrt(Math.max(0, (a + d + delta) / 2));
            const minor = 2 * Math.sqrt(Math.max(0, (a + d - delta) / 2));
            const angle = (Math.atan2(2 * b, a - d) * 90) / Math.PI;
            return (
              <g key={track.id} transform={`translate(${-left} ${-forward})`}>
                <ellipse
                  rx={major}
                  ry={minor}
                  transform={`rotate(${angle})`}
                  fill="#74c7f61a"
                  stroke="#74c7f6"
                  strokeWidth=".15"
                />
                <circle
                  r=".35"
                  fill={track.status === "confirmed" ? "#a7e8c8" : "#e5bb76"}
                />
                <path
                  d={`M 0 0 l ${-(-s * track.vx + c * track.vy)} ${-(c * track.vx + s * track.vy)}`}
                  stroke="#b4d8ef"
                  strokeWidth=".15"
                />
                <text x=".6" y="-.6" fill="#c4dbe7" fontSize="1.1">
                  {track.id}
                </text>
              </g>
            );
          })}
        </svg>
        <p className="sensor-model-note">
          Estimated observed surfaces, not hidden object centres. Ellipses show
          2σ positional covariance; arrows show estimated world velocity. Up is
          forward; left is vehicle left. Perfect ego localisation.
          Tentative/coasting estimates are shown explicitly.
        </p>
        <div className="sensor-detections">
          <table>
            <thead>
              <tr>
                <th>Track / class</th>
                <th>Status</th>
                <th>Speed</th>
                <th>Measurement age</th>
                <th>Contributors</th>
                <th>Position σ</th>
              </tr>
            </thead>
            <tbody>
              {tracks.map((t) => (
                <tr key={t.id}>
                  <td>
                    {t.id} / {t.objectClass ?? "unknown"}
                  </td>
                  <td>{t.status}</td>
                  <td>{Math.hypot(t.vx, t.vy).toFixed(2)} m/s</td>
                  <td>
                    {Math.max(
                      0,
                      tracking.timestamp - t.lastMeasurementTime,
                    ).toFixed(2)}{" "}
                    s
                  </td>
                  <td>
                    {t.sources
                      .map(
                        (source) =>
                          `${source}${t.sourceTimes?.[source] === undefined ? "" : ` ${(tracking.timestamp - t.sourceTimes[source]!).toFixed(2)}s`}`,
                      )
                      .join(" + ")}
                  </td>
                  <td>
                    {Math.sqrt(
                      Math.max(
                        0,
                        (t.covariance[0]?.[0] ?? 0) +
                          (t.covariance[1]?.[1] ?? 0),
                      ),
                    ).toFixed(2)}{" "}
                    m
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
          {!tracks.length && (
            <p className="sensor-empty">
              Start the simulation to acquire and confirm tracks.
            </p>
          )}
        </div>
      </div>
    </section>
  );
}
