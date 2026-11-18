import type { Simulation } from "../simulation/clock";
import type {
  SensorSnapshot,
  SensorSource,
  Detection,
  DetectionFrame,
} from "../sensors/types";
import type { SafetySnapshot } from "../autonomy/safetyTypes";
import type { PhysicalSnapshot } from "../simulation/collisionTypes";
import type { NavigationSnapshot } from "../autonomy/types";
import type { VehicleState } from "../simulation/types";

type ExplorerTab = SensorSource | "compute";
interface SensorExplorerProps {
  simulation: Simulation;
  sensors: SensorSnapshot;
  safety: SafetySnapshot;
  physical: PhysicalSnapshot;
  navigation: NavigationSnapshot;
  vehicle: VehicleState;
  tab: ExplorerTab;
  onTab: (tab: ExplorerTab) => void;
  overlay: boolean;
  onOverlay: (enabled: boolean) => void;
}
const degrees = (angle: number) => (angle * 180) / Math.PI;
const time = (value: number | undefined) =>
  value === undefined ? "Not sampled" : `${value.toFixed(2)} s`;
const number = (value: number | null | undefined, digits = 2) =>
  value === null || value === undefined ? "—" : value.toFixed(digits);

function DetectionList({
  frame,
  radar = false,
  detections,
}: {
  frame: DetectionFrame | null;
  radar?: boolean;
  detections?: Detection[];
}) {
  if (!frame && !detections)
    return (
      <p className="sensor-empty">
        No measurement frame yet. Start the simulation to sample this sensor.
      </p>
    );
  const rows = detections ?? frame?.detections ?? [];
  return (
    <div
      className="sensor-detections"
      role="region"
      aria-label="Sensor observations"
      tabIndex={0}
    >
      <table>
        <thead>
          <tr>
            <th scope="col">Observation</th>
            <th scope="col">Source / time</th>
            <th scope="col">Class</th>
            <th scope="col">Range</th>
            <th scope="col">Bearing</th>
            {radar && <th scope="col">Radial speed</th>}
          </tr>
        </thead>
        <tbody>
          {rows.map((d: Detection) => (
            <tr key={d.observationId}>
              <td>{d.observationId}</td>
              <td>
                {d.source} / {time(d.timestamp)}
              </td>
              <td>{d.objectClass ?? "Unclassified"}</td>
              <td>{d.range.toFixed(1)} m</td>
              <td>{degrees(d.bearing).toFixed(1)}°</td>
              {radar && (
                <td>
                  {d.radialVelocity === undefined
                    ? "—"
                    : `${d.radialVelocity.toFixed(2)} m/s`}
                </td>
              )}
            </tr>
          ))}
        </tbody>
      </table>
      {rows.length === 0 && (
        <p className="sensor-empty">
          No objects observed in this field of view.
        </p>
      )}
    </div>
  );
}

/** Plot uses measured sensor-relative coordinates; positive bearing appears left. */
function RadarPlot({
  frame,
  range,
  fov,
}: {
  frame: DetectionFrame | null;
  range: number;
  fov: number;
}) {
  const radius = 90;
  const at = (r: number, angle: number) => ({
    x: -Math.sin(angle) * r,
    y: -Math.cos(angle) * r,
  });
  const left = at(radius, fov / 2),
    right = at(radius, -fov / 2);
  return (
    <svg
      className="sensor-plot radar-plot"
      viewBox="-104 -104 208 116"
      role="img"
      aria-label="Forward radar plot. Vehicle at bottom, distances increase upward."
    >
      <path
        d={`M0 0L${left.x} ${left.y}A${radius} ${radius} 0 ${fov > Math.PI ? 1 : 0} 1 ${right.x} ${right.y}Z`}
        fill="#142c3a"
        stroke="#3b6074"
        strokeWidth=".6"
      />
      {[0.25, 0.5, 0.75, 1].map((scale) => {
        const a = at(radius * scale, fov / 2),
          b = at(radius * scale, -fov / 2);
        return (
          <g key={scale}>
            <path
              d={`M${a.x} ${a.y}A${radius * scale} ${radius * scale} 0 ${fov > Math.PI ? 1 : 0} 1 ${b.x} ${b.y}`}
              fill="none"
              stroke="#3b6074"
              strokeWidth=".5"
            />
            <text x="2" y={-radius * scale + 5} fontSize="4" fill="#91aabd">
              {(range * scale).toFixed(0)} m
            </text>
          </g>
        );
      })}
      <path d="M0-4L-2 2H2Z" fill="#d6eff6" />
      {frame?.detections.map((d) => {
        const point = at(
          Math.min(radius, (d.range / range) * radius),
          d.bearing,
        );
        return (
          <circle
            key={d.observationId}
            cx={point.x}
            cy={point.y}
            r="1.9"
            fill="#74c7f6"
          >
            <title>
              {d.observationId}: {d.range.toFixed(1)} m,{" "}
              {degrees(d.bearing).toFixed(1)}°, {number(d.radialVelocity)} m/s
            </title>
          </circle>
        );
      })}
    </svg>
  );
}

export function SensorExplorer({
  simulation,
  sensors,
  safety,
  physical,
  navigation,
  vehicle,
  tab,
  onTab,
  overlay,
  onOverlay,
}: SensorExplorerProps) {
  const config = sensors.configs.find((c) => c.source === tab);
  const scan = sensors.lidar;
  const lidarRange = scan?.maxRange ?? config?.maxRange ?? 40;
  const frame = tab === "camera" ? sensors.camera : sensors.radar;
  const timestamp = tab === "lidar" ? scan?.timestamp : frame?.timestamp;
  const returns = scan?.rays.filter((ray) => ray.hit).length ?? 0;
  const labels: Record<ExplorerTab, string> = {
    lidar: "LiDAR",
    camera: "Camera",
    radar: "Radar",
    compute: "Compute",
  };
  const observations = [
    ...(sensors.camera?.detections ?? []),
    ...(sensors.radar?.detections ?? []),
  ];
  const sampledTime = simulation.snapshot.time;
  return (
    <section className="sensor-explorer" aria-label="Sensor Explorer">
      <div className="sensor-heading">
        <div>
          <span className="eyebrow">SYNTHETIC PERCEPTION / v0.3</span>
          <h2>Sensor Explorer</h2>
        </div>
        <span className="sensor-badge">Geometric models</span>
      </div>
      <div className="sensor-tabbar" role="tablist" aria-label="Sensor source">
        {(["lidar", "camera", "radar", "compute"] as const).map((source) => (
          <button
            key={source}
            id={`sensor-tab-${source}`}
            role="tab"
            aria-selected={tab === source}
            aria-controls={`sensor-panel-${source}`}
            tabIndex={tab === source ? 0 : -1}
            onClick={() => onTab(source)}
            onKeyDown={(event) => {
              const tabs: ExplorerTab[] = [
                "lidar",
                "camera",
                "radar",
                "compute",
              ];
              const index = tabs.indexOf(tab);
              if (event.key === "ArrowRight" || event.key === "ArrowLeft") {
                event.preventDefault();
                event.stopPropagation();
                const next =
                  tabs[(index + (event.key === "ArrowRight" ? 1 : 3)) % 4];
                onTab(next);
                requestAnimationFrame(() =>
                  document.getElementById(`sensor-tab-${next}`)?.focus(),
                );
              }
            }}
          >
            {labels[source]}
          </button>
        ))}
      </div>
      <div
        className="sensor-panel"
        role="tabpanel"
        id={`sensor-panel-${tab}`}
        aria-labelledby={`sensor-tab-${tab}`}
      >
        <div className="sensor-panel-header">
          <p>
            {tab === "compute"
              ? "Calculated observations and physical status"
              : `${labels[tab]} · measurement ${time(timestamp)}`}
          </p>
          {tab !== "compute" && (
            <label className="sensor-overlay-toggle">
              <input
                type="checkbox"
                checked={overlay}
                onChange={(event) => onOverlay(event.target.checked)}
              />{" "}
              Show {labels[tab]} in 3D
            </label>
          )}
        </div>
        {tab === "lidar" && (
          <>
            <div className="sensor-view-grid">
              <svg
                className="sensor-plot lidar-plot"
                viewBox={`${-lidarRange * 1.12} ${-lidarRange * 1.12} ${lidarRange * 2.24} ${lidarRange * 2.24}`}
                role="img"
                aria-label="LiDAR scan in sensor coordinates. Forward is up; blue points are returns."
              >
                {[0.25, 0.5, 0.75, 1]
                  .map((scale) => lidarRange * scale)
                  .map((radius) => (
                    <g key={radius}>
                      <circle
                        r={radius}
                        fill="none"
                        stroke="#36505e"
                        strokeWidth=".15"
                      />
                      <text
                        x=".8"
                        y={-radius + 2.5}
                        fontSize="1.8"
                        fill="#91aabd"
                      >
                        {radius.toFixed(0)} m
                      </text>
                    </g>
                  ))}
                <path
                  d={`M${-lidarRange} 0H${lidarRange}M0${-lidarRange}V${lidarRange}`}
                  stroke="#36505e"
                  strokeWidth=".15"
                />
                {scan?.rays
                  .filter((_, index) => index % 12 === 0)
                  .map((ray, index) => {
                    const angle = ray.angle;
                    return (
                      <line
                        key={index}
                        x1="0"
                        y1="0"
                        x2={-Math.sin(angle) * ray.range}
                        y2={-Math.cos(angle) * ray.range}
                        stroke="#2c6b8a"
                        strokeWidth=".12"
                      />
                    );
                  })}
                {scan?.rays
                  .filter((ray) => ray.hit)
                  .map((ray, index) => {
                    const angle = ray.angle;
                    return (
                      <circle
                        key={index}
                        cx={-Math.sin(angle) * ray.range}
                        cy={-Math.cos(angle) * ray.range}
                        r=".28"
                        fill="#74c7f6"
                      />
                    );
                  })}
                <path d="M0-1.7L-1.2 1H1.2Z" fill="#e7f8fc" />
              </svg>
              <dl className="sensor-metrics">
                <div>
                  <dt>Scan timestamp</dt>
                  <dd>{time(scan?.timestamp)}</dd>
                </div>
                <div>
                  <dt>Returns / rays</dt>
                  <dd>
                    {returns} / {scan?.rays.length ?? 0}
                  </dd>
                </div>
                <div>
                  <dt>Maximum range</dt>
                  <dd>{number(config?.maxRange, 0)} m</dd>
                </div>
                <div>
                  <dt>Update rate</dt>
                  <dd>{number(config?.updateHz, 0)} Hz</dd>
                </div>
                <div>
                  <dt>Scan count</dt>
                  <dd>{sensors.counts.lidar}</dd>
                </div>
                <div>
                  <dt>Range noise</dt>
                  <dd>{number(config?.noise, 3)} m</dd>
                </div>
              </dl>
            </div>
            <p className="sensor-model-note">
              2D nearest-intersection raycasting. Misses end at maximum range;
              blue points are actual returns. No optical LiDAR physics.
            </p>
          </>
        )}
        {tab === "camera" && (
          <>
            <div className="sensor-view-grid">
              <div className="sensor-camera-image">
                <canvas
                  id="camera-image"
                  width="320"
                  height="180"
                  aria-label="Forward-facing synthetic camera image"
                />
                <span className="camera-image-caption">
                  FORWARD CAMERA · {time(sensors.camera?.timestamp)}
                </span>
              </div>
              <dl className="sensor-metrics">
                <div>
                  <dt>Horizontal FOV</dt>
                  <dd>{number(config ? degrees(config.fov) : null, 0)}°</dd>
                </div>
                <div>
                  <dt>Maximum range</dt>
                  <dd>{number(config?.maxRange, 0)} m</dd>
                </div>
                <div>
                  <dt>Update rate</dt>
                  <dd>{number(config?.updateHz, 0)} Hz</dd>
                </div>
                <div>
                  <dt>Observations</dt>
                  <dd>{sensors.camera?.detections.length ?? 0}</dd>
                </div>
                <div>
                  <dt>Frame count</dt>
                  <dd>{sensors.counts.camera}</dd>
                </div>
              </dl>
            </div>
            <p className="sensor-model-note">
              Rendered forward view plus synthetic geometric observations with
              line-of-sight checks. No neural vision, confidence scores, or
              fabricated image boxes.
            </p>
            <DetectionList frame={sensors.camera} />
          </>
        )}
        {tab === "radar" && (
          <>
            <div className="sensor-view-grid">
              <RadarPlot
                frame={sensors.radar}
                range={config?.maxRange ?? 80}
                fov={config?.fov ?? (Math.PI * 2) / 3}
              />
              <dl className="sensor-metrics">
                <div>
                  <dt>Frame timestamp</dt>
                  <dd>{time(sensors.radar?.timestamp)}</dd>
                </div>
                <div>
                  <dt>Maximum range</dt>
                  <dd>{number(config?.maxRange, 0)} m</dd>
                </div>
                <div>
                  <dt>Horizontal FOV</dt>
                  <dd>{number(config ? degrees(config.fov) : null, 0)}°</dd>
                </div>
                <div>
                  <dt>Update rate</dt>
                  <dd>{number(config?.updateHz, 0)} Hz</dd>
                </div>
                <div>
                  <dt>Frame count</dt>
                  <dd>{sensors.counts.radar}</dd>
                </div>
              </dl>
            </div>
            <DetectionList frame={sensors.radar} radar />
            <p className="sensor-model-note">
              Negative radial speed means closing. Simplified
              range/bearing/motion model; no multipath, Doppler processing, or
              radar cross-section simulation.
            </p>
          </>
        )}
        {tab === "compute" && (
          <>
            <div className="sensor-compute-grid">
              <dl className="sensor-metrics">
                <div>
                  <dt>Driving mode</dt>
                  <dd>{navigation.mode}</dd>
                </div>
                <div>
                  <dt>Navigation state</dt>
                  <dd>{navigation.state}</dd>
                </div>
                <div>
                  <dt>Vehicle speed</dt>
                  <dd>{(vehicle.velocity * 3.6).toFixed(1)} km/h</dd>
                </div>
                <div>
                  <dt>Target speed</dt>
                  <dd>
                    {navigation.targetSpeed === null
                      ? "—"
                      : `${(navigation.targetSpeed * 3.6).toFixed(1)} km/h`}
                  </dd>
                </div>
                <div>
                  <dt>Steering command</dt>
                  <dd>
                    {navigation.steeringCommand === null
                      ? "—"
                      : `${degrees(navigation.steeringCommand).toFixed(1)}°`}
                  </dd>
                </div>
                <div>
                  <dt>Simulation time</dt>
                  <dd>{sampledTime.toFixed(2)} s</dd>
                </div>
              </dl>
              <dl className="sensor-metrics">
                <div>
                  <dt>Safety supervisor</dt>
                  <dd>{safety.state}</dd>
                </div>
                <div>
                  <dt>
                    {safety.braking
                      ? "Trigger stopping envelope"
                      : "Stopping envelope"}
                  </dt>
                  <dd>{safety.stoppingDistance.toFixed(2)} m</dd>
                </div>
                <div>
                  <dt>Assessed obstacle</dt>
                  <dd>{number(safety.obstacleDistance)} m</dd>
                </div>
                <div>
                  <dt>Trigger source / time</dt>
                  <dd>
                    {safety.source ?? "—"} /{" "}
                    {time(safety.measurementTime ?? undefined)}
                  </dd>
                </div>
                <div>
                  <dt>Road footprint status</dt>
                  <dd>{physical.roadStatus}</dd>
                </div>
                <div>
                  <dt>Collisions / departures</dt>
                  <dd>
                    {physical.collisionCount} / {physical.roadDepartureCount}
                  </dd>
                </div>
              </dl>
            </div>
            <p
              className={`sensor-safety-reason ${safety.braking ? "is-braking" : ""}`}
            >
              {safety.reason}
            </p>
            {physical.collision && (
              <p className="sensor-contact">
                Physical contact at {physical.collision.time.toFixed(2)} s ·{" "}
                {physical.collision.objectId} (simulator ground truth)
              </p>
            )}
            {physical.clearance && (
              <p className="sensor-clearance">
                Route clearance:{" "}
                {physical.clearance.valid ? "VALID" : "REJECTED"} ·{" "}
                {physical.clearance.reason}
              </p>
            )}
            <div className="sensor-status-list">
              {sensors.configs.map((sensor) => {
                const sampled = sensors[sensor.source];
                return (
                  <div key={sensor.id}>
                    <strong>{sensor.id}</strong>
                    <span>
                      {sensor.updateHz} Hz · {sensor.maxRange} m · t=
                      {time(sampled?.timestamp)}
                    </span>
                    <span>
                      {sampled
                        ? `${Math.max(0, sampledTime - sampled.timestamp).toFixed(2)} s old`
                        : "Awaiting sample"}
                    </span>
                  </div>
                );
              })}
            </div>
            <p className="sensor-model-note">
              {observations.length} camera/radar observations in the latest
              frames. Safety uses permitted measurements; collision and road
              status are separately labelled simulator ground truth.
            </p>
            <DetectionList frame={null} detections={observations} radar />
          </>
        )}
      </div>
    </section>
  );
}
