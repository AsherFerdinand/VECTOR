import type { CameraMode, SimulationSnapshot } from "../simulation/types";
import type { NavigationSnapshot } from "../autonomy/types";
export const cameraLabels: Record<CameraMode, string> = {
  overview: "Bird’s-eye",
  follow: "Third-person",
  cockpit: "Cockpit",
};
export function Dashboard({
  state,
  camera,
  navigation,
}: {
  state: SimulationSnapshot;
  camera: CameraMode;
  navigation: NavigationSnapshot;
}) {
  const { vehicle, control } = state;
  const degrees = (radians: number) => (radians * 180) / Math.PI;
  return (
    <aside className="telemetry">
      <div className="panel-heading">
        <span className="eyebrow">LIVE TELEMETRY</span>
        <span className="live-dot" />
      </div>
      <h2>Vehicle & navigation state</h2>
      <div className="nav-state" role="status">
        {navigation.state.replaceAll("_", " ")}
      </div>
      <div className="speed">
        <strong>{Math.abs(vehicle.velocity * 3.6).toFixed(1)}</strong>
        <span>km/h</span>
      </div>
      <div className="speed-caption">
        <span>{vehicle.velocity < -0.01 ? "REVERSE" : "FORWARD"}</span>
        <span>LIMIT 50 km/h</span>
      </div>
      <div className="speed-track">
        <div
          style={{
            width: `${Math.min(100, Math.abs(vehicle.velocity) * 3.6 * 2)}%`,
          }}
        />
      </div>
      <dl className="readouts">
        <div>
          <dt>Steering angle</dt>
          <dd>
            {degrees(control.steering).toFixed(1)} <small>°</small>
          </dd>
        </div>
        <div>
          <dt>Acceleration</dt>
          <dd>
            {control.acceleration.toFixed(2)} <small>m/s²</small>
          </dd>
        </div>
        <div>
          <dt>Heading</dt>
          <dd>
            {(((degrees(vehicle.heading) % 360) + 360) % 360).toFixed(1)}{" "}
            <small>°</small>
          </dd>
        </div>
        <div>
          <dt>Position X</dt>
          <dd>
            {vehicle.x.toFixed(2)} <small>m</small>
          </dd>
        </div>
        <div>
          <dt>Position Y</dt>
          <dd>
            {vehicle.y.toFixed(2)} <small>m</small>
          </dd>
        </div>
        <div>
          <dt>Simulation time</dt>
          <dd>
            {state.time.toFixed(2)} <small>s</small>
          </dd>
        </div>
      </dl>
      <dl className="readouts nav-readouts">
        <div>
          <dt>Driving mode</dt>
          <dd>{navigation.mode}</dd>
        </div>
        <div>
          <dt>Target speed</dt>
          <dd>
            {navigation.targetSpeed === null
              ? "—"
              : (navigation.targetSpeed * 3.6).toFixed(1)}{" "}
            <small>km/h</small>
          </dd>
        </div>
        <div>
          <dt>Steering command</dt>
          <dd>
            {navigation.steeringCommand === null
              ? "—"
              : degrees(navigation.steeringCommand).toFixed(1)}{" "}
            <small>°</small>
          </dd>
        </div>
        <div>
          <dt>Remaining distance</dt>
          <dd>
            {navigation.remainingDistance?.toFixed(1) ?? "—"} <small>m</small>
          </dd>
        </div>
        <div>
          <dt>Cross-track error</dt>
          <dd>
            {navigation.crossTrackError?.toFixed(2) ?? "—"} <small>m</small>
          </dd>
        </div>
        <div>
          <dt>Look-ahead distance</dt>
          <dd>
            {navigation.lookAheadDistance?.toFixed(2) ?? "—"} <small>m</small>
          </dd>
        </div>
        <div>
          <dt>Route progress</dt>
          <dd>
            {navigation.mode === "AUTONOMOUS"
              ? (navigation.progress * 100).toFixed(1)
              : "—"}{" "}
            <small>%</small>
          </dd>
        </div>
      </dl>
      <div className="model-note">
        <span className="eyebrow">VEHICLE MODEL</span>
        <strong>Kinematic bicycle</strong>
        <p>
          Rear-axle reference · Euler integration
          <br />
          Fixed step 0.01 s · Wheelbase 2.7 m
        </p>
      </div>
      <dl className="secondary-readouts">
        <div>
          <dt>Camera</dt>
          <dd>{cameraLabels[camera]}</dd>
        </div>
        <div>
          <dt>Backlog discarded</dt>
          <dd>{state.droppedTime.toFixed(2)} s</dd>
        </div>
      </dl>
      <p className="scope-note">
        Ground-truth simulation.
        <br />
        Perfect localisation · no collision response.
      </p>
    </aside>
  );
}
