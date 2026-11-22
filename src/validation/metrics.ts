import { VEHICLE } from "../simulation/vehicle";
import {
  MAX_RECORDED_FRAMES,
  type ValidationConfig,
  type ValidationEvent,
  type ValidationFrame,
  type ValidationMetrics,
  type ValidationResult,
} from "./types";

function boundaryPoint(
  x: number,
  y: number,
  polygon: { x: number; y: number }[],
): { x: number; y: number } {
  let best = { x, y },
    distance = Infinity;
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i],
      b = polygon[(i + 1) % polygon.length];
    const dx = b.x - a.x,
      dy = b.y - a.y,
      length2 = dx * dx + dy * dy;
    const t =
      length2 > 0
        ? Math.max(0, Math.min(1, ((x - a.x) * dx + (y - a.y) * dy) / length2))
        : 0;
    const point = { x: a.x + t * dx, y: a.y + t * dy };
    const d = Math.hypot(point.x - x, point.y - y);
    if (d < distance) {
      distance = d;
      best = point;
    }
  }
  return best;
}
/** Validation-only one-to-one nearest-boundary matching, independent of simulator IDs. */
export function calculateMetrics(
  frames: readonly ValidationFrame[],
): ValidationMetrics {
  let trackingPosition = 0,
    trackingVelocity = 0,
    matches = 0,
    consistency = 0,
    consistencyCount = 0;
  let followingError = 0,
    followingCount = 0;
  let minimumClearance = Infinity,
    minimumTTC = Infinity,
    impactSpeed = -Infinity;
  let maximumDeceleration = 0,
    maximumLateralAcceleration = 0,
    maximumCrossTrackError = 0;
  let laneDepartures = 0,
    interventions = 0,
    hazardTime: number | null = null,
    aebResponseTime: number | null = null;
  let previous: ValidationFrame | undefined;
  for (const frame of frames) {
    if (Number.isFinite(frame.clearance))
      minimumClearance = Math.min(minimumClearance, frame.clearance!);
    if (frame.behaviour.ttc !== null && Number.isFinite(frame.behaviour.ttc))
      minimumTTC = Math.min(minimumTTC, frame.behaviour.ttc);
    if (frame.impactSpeed !== undefined && Number.isFinite(frame.impactSpeed))
      impactSpeed = Math.max(impactSpeed, frame.impactSpeed);
    if (previous && frame.time > previous.time)
      maximumDeceleration = Math.max(
        maximumDeceleration,
        (Math.abs(previous.vehicle.velocity) -
          Math.abs(frame.vehicle.velocity)) /
          (frame.time - previous.time),
      );
    maximumLateralAcceleration = Math.max(
      maximumLateralAcceleration,
      Math.abs(
        (frame.vehicle.velocity ** 2 * Math.tan(frame.control.steering)) /
          VEHICLE.wheelbase,
      ),
    );
    maximumCrossTrackError = Math.max(
      maximumCrossTrackError,
      Math.abs(frame.navigation.crossTrackError ?? 0),
    );
    if (frame.laneDeparture && !previous?.laneDeparture) laneDepartures++;
    const intervention = ["AEB", "STALE_STOP", "BLOCKED"].includes(
      frame.behaviour.state,
    );
    if (intervention && frame.behaviour.state !== previous?.behaviour.state)
      interventions++;
    if (frame.hazardActive && !previous?.hazardActive) hazardTime = frame.time;
    if (
      frame.behaviour.aeb &&
      !previous?.behaviour.aeb &&
      hazardTime !== null
    ) {
      const response = frame.time - hazardTime;
      aebResponseTime = Math.max(aebResponseTime ?? 0, response);
    }
    if (
      frame.behaviour.state === "FOLLOW_LEAD" &&
      frame.behaviour.leadDistance !== null
    ) {
      followingError +=
        (frame.behaviour.leadDistance - frame.behaviour.desiredGap) ** 2;
      followingCount++;
    }
    const tracks = frame.tracks ?? [],
      truth = frame.truth ?? [];
    const pairs = tracks
      .flatMap((track, ti) =>
        truth.map((target, gi) => {
          const point = target.polygon?.length
            ? boundaryPoint(track.x, track.y, target.polygon)
            : target;
          return {
            ti,
            gi,
            point,
            distance: Math.hypot(track.x - point.x, track.y - point.y),
          };
        }),
      )
      .filter((pair) => pair.distance <= 5)
      .sort((a, b) => a.distance - b.distance || a.ti - b.ti || a.gi - b.gi);
    const usedTracks = new Set<number>(),
      usedTruth = new Set<number>();
    for (const pair of pairs) {
      if (usedTracks.has(pair.ti) || usedTruth.has(pair.gi)) continue;
      usedTracks.add(pair.ti);
      usedTruth.add(pair.gi);
      const track = tracks[pair.ti],
        target = truth[pair.gi];
      const dx = track.x - pair.point.x,
        dy = track.y - pair.point.y;
      trackingPosition += dx * dx + dy * dy;
      trackingVelocity +=
        (track.vx - target.vx) ** 2 + (track.vy - target.vy) ** 2;
      matches++;
      const p = track.covariance;
      if (
        !target.polygon?.length &&
        p.length >= 2 &&
        p[0]?.length >= 2 &&
        p[1]?.length >= 2
      ) {
        const determinant = p[0][0] * p[1][1] - p[0][1] * p[1][0];
        if (determinant > 1e-12) {
          consistency +=
            (p[1][1] * dx * dx -
              (p[0][1] + p[1][0]) * dx * dy +
              p[0][0] * dy * dy) /
            determinant /
            2;
          consistencyCount++;
        }
      }
    }
    previous = frame;
  }
  const last = frames.at(-1);
  return {
    collisionCount: Math.max(
      0,
      ...frames.map((f) => f.physical.collisionCount),
    ),
    impactSpeed: impactSpeed === -Infinity ? null : impactSpeed,
    minimumClearance: minimumClearance === Infinity ? null : minimumClearance,
    minimumTTC: minimumTTC === Infinity ? null : minimumTTC,
    followingDistanceRMSE: followingCount
      ? Math.sqrt(followingError / followingCount)
      : null,
    aebResponseTime,
    maximumDeceleration,
    maximumLateralAcceleration,
    maximumCrossTrackError,
    roadDepartures: Math.max(
      0,
      ...frames.map((f) => f.physical.roadDepartureCount),
    ),
    laneDepartures,
    routeCompleted: frames.some(
      (f) => f.navigation.state === "DESTINATION_REACHED",
    ),
    trackingPositionRMSE: matches
      ? Math.sqrt(trackingPosition / matches)
      : null,
    trackingVelocityRMSE: matches
      ? Math.sqrt(trackingVelocity / matches)
      : null,
    positionCovarianceConsistency: consistencyCount
      ? consistency / consistencyCount
      : null,
    trackingMatches: matches,
    interventions,
    manoeuvreOutcome: last?.manoeuvre ?? null,
  };
}

export function buildTimeline(
  frames: readonly ValidationFrame[],
): ValidationEvent[] {
  const events: ValidationEvent[] = [];
  let previous: ValidationFrame | undefined;
  let previousTrackIds = "";
  for (const frame of frames) {
    if (frame.behaviour.state !== previous?.behaviour.state)
      events.push({
        time: frame.time,
        kind: "decision",
        message: `${frame.behaviour.state}: ${frame.behaviour.reason}`,
      });
    if (
      frame.physical.collisionCount > (previous?.physical.collisionCount ?? 0)
    )
      events.push({
        time: frame.time,
        kind: "collision",
        message: `Collision ${frame.physical.collision?.objectId ?? "contact"}`,
      });
    if (
      frame.physical.roadDepartureCount >
      (previous?.physical.roadDepartureCount ?? 0)
    )
      events.push({
        time: frame.time,
        kind: "road",
        message: frame.physical.sweptRoadStatus,
      });
    if (frame.manoeuvre && frame.manoeuvre !== previous?.manoeuvre)
      events.push({
        time: frame.time,
        kind: "manoeuvre",
        message: frame.manoeuvre,
      });
    if (frame.tracks !== undefined) {
      const trackIds = frame.tracks.map((t) => `${t.id}:${t.status}`).join(",");
      if (trackIds !== previousTrackIds)
        events.push({
          time: frame.time,
          kind: "tracking",
          message: `${frame.tracks.length} tracks: ${trackIds}`,
        });
      previousTrackIds = trackIds;
    }
    previous = frame;
  }
  return events.slice(0, MAX_RECORDED_FRAMES);
}

export function createValidationResult(
  config: ValidationConfig,
  source: readonly ValidationFrame[],
): ValidationResult {
  const metrics = calculateMetrics(source);
  const failures: string[] = [];
  if (!source.length) failures.push("No simulation frames recorded");
  if (metrics.collisionCount)
    failures.push(`${metrics.collisionCount} collisions`);
  if (metrics.roadDepartures)
    failures.push(`${metrics.roadDepartures} road departures`);
  if (metrics.laneDepartures)
    failures.push(`${metrics.laneDepartures} lane departures`);
  if (metrics.maximumLateralAcceleration > 1.55)
    failures.push("Lateral acceleration exceeds 1.55 m/s² tolerance");
  if (source.some((f) => f.control.acceleration < -VEHICLE.maxBraking - 1e-6))
    failures.push("Braking command limit exceeded");
  if (source.some((f) => f.physical.containmentStop))
    failures.push("Swept-road containment interlock activated");
  if (
    config.corridorScenario === "blocked" &&
    !(
      source.some((f) => f.manoeuvre === "STOP_FOR_OBSTACLE") &&
      Math.abs(source.at(-1)?.vehicle.velocity ?? Infinity) <= 0.1
    )
  )
    failures.push(
      "Corridor blockage did not produce a stationary planned safe stop",
    );
  const expectedStop =
    config.scenario === "blocked" ||
    config.trafficVariant === "dynamic-block" ||
    config.corridorScenario === "blocked" ||
    config.dropout !== undefined;
  if (!metrics.routeCompleted && !expectedStop)
    failures.push("Route not completed within configured duration");
  if (
    config.scenario === "blocked" &&
    !source.some(
      (f) =>
        f.navigation.state === "ROUTE_UNAVAILABLE" ||
        f.behaviour.state === "BLOCKED",
    )
  )
    failures.push("Blocked route did not produce a safe stop/rejection");
  if (
    config.trafficVariant === "dynamic-block" &&
    !(
      source.some((f) =>
        ["BLOCKED", "AEB", "FOLLOW_LEAD"].includes(f.behaviour.state),
      ) && Math.abs(source.at(-1)?.vehicle.velocity ?? Infinity) <= 0.1
    )
  )
    failures.push(
      "Dynamic blockage did not produce a measured stationary safe hold",
    );
  if (
    config.scenario === "red-light" &&
    !source.some((f) => f.behaviour.state === "STOP_LIGHT")
  )
    failures.push("No observed red-light stop response");
  if (
    config.scenario === "pedestrian" &&
    !source.some((f) => f.behaviour.state === "YIELD_PEDESTRIAN")
  )
    failures.push("No observed pedestrian-yield response");
  if (
    (config.scenario === "following" || config.scenario === "sudden-braking") &&
    !source.some((f) => f.behaviour.state === "FOLLOW_LEAD")
  )
    failures.push("No lead-following response");
  if (config.dropout && !source.some((f) => f.behaviour.state === "STALE_STOP"))
    failures.push("Dropout did not produce degraded-mode stop");
  // Keep complete acquisition snapshots for replay; sparse omitted tracking fields
  // are metric-sampling markers, not evidence that all objects disappeared.
  const sampled = source.filter((frame) => frame.tracks !== undefined);
  const replaySource = sampled.length ? [...sampled] : [...source];
  const final = source.at(-1);
  if (final && replaySource.at(-1) !== final) replaySource.push(final);
  const count = Math.min(MAX_RECORDED_FRAMES, replaySource.length);
  const frames =
    count === replaySource.length
      ? replaySource
      : Array.from(
          { length: count },
          (_, i) =>
            replaySource[
              Math.round((i * (replaySource.length - 1)) / (count - 1))
            ],
        );
  return {
    config: structuredClone(config),
    frames: structuredClone(frames),
    events: buildTimeline(source),
    metrics,
    passed: failures.length === 0,
    failures,
  };
}
