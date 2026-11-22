import type {
  DetectionFrame,
  SensorSnapshot,
  SensorSource,
} from "../sensors/types";
import type { VehicleState } from "../simulation/types";
import type { ObjectClass } from "../world/types";
import { predictCV, positionGate, updateScalar, type Estimate } from "./kalman";
import type { Track, TrackerConfig, TrackingSnapshot } from "./types";
interface InternalTrack {
  id: string;
  estimate: Estimate;
  birth: number;
  time: number;
  last: number;
  hits: number;
  hitTime: number;
  sources: Set<SensorSource>;
  sourceTimes: Partial<Record<SensorSource, number>>;
  objectClass?: ObjectClass;
}
interface Measurement {
  x: number;
  y: number;
  variance: number;
  objectClass?: ObjectClass;
  radial?: { value: number; dx: number; dy: number; variance: number };
}
const DEFAULT: TrackerConfig = {
  accelerationNoise: 2,
  gate: 9.21,
  confirmationHits: 3,
  coastingAfter: 0.35,
  deleteAfter: 1.5,
  maxTracks: 128,
};

/** Sensor-only surface tracking. No debug IDs or hidden geometry enter association. */
export class MultiObjectTracker {
  private config: TrackerConfig;
  private tracks: InternalTrack[] = [];
  private serial = 0;
  private processed = new Map<string, number>();
  private poses = new Map<string, { x: number; y: number; time: number }>();
  private time = 0;
  constructor(config: Partial<TrackerConfig> = {}) {
    this.config = { ...DEFAULT, ...config };
    if (
      !Object.values(this.config).every(Number.isFinite) ||
      this.config.accelerationNoise < 0 ||
      this.config.gate <= 0 ||
      !Number.isInteger(this.config.confirmationHits) ||
      this.config.confirmationHits < 1 ||
      this.config.coastingAfter < 0 ||
      this.config.deleteAfter < this.config.coastingAfter ||
      !Number.isInteger(this.config.maxTracks) ||
      this.config.maxTracks < 1
    )
      throw new RangeError("Invalid tracker configuration");
  }
  reset() {
    this.tracks = [];
    this.serial = 0;
    this.processed.clear();
    this.poses.clear();
    this.time = 0;
  }
  update(
    sensors: SensorSnapshot,
    vehicle: VehicleState,
    now: number,
  ): TrackingSnapshot {
    if (!Number.isFinite(now) || now < 0)
      throw new RangeError("Tracking time must be finite and non-negative");
    if (now < this.time - 1e-9) this.reset();
    const frames = [sensors.lidar, sensors.camera, sensors.radar]
      .filter((f) => f !== null)
      .sort((a, b) => a.timestamp - b.timestamp);
    for (const frame of frames) {
      if (
        !Number.isFinite(frame.timestamp) ||
        ![frame.pose.x, frame.pose.y, frame.pose.heading].every(
          Number.isFinite,
        ) ||
        frame.timestamp > now + 1e-8 ||
        frame.timestamp < this.time - 1e-8 ||
        frame.timestamp <=
          (this.processed.get(frame.sensorId) ?? -Infinity) + 1e-8
      )
        continue;
      this.processed.set(frame.sensorId, frame.timestamp);
      this.predict(frame.timestamp);
      this.tracks = this.tracks.filter(
        (t) => frame.timestamp - t.last <= this.config.deleteAfter,
      );
      const config = sensors.configs.find((c) => c.id === frame.sensorId);
      const noiseVariance =
        (Number.isFinite(config?.noise) ? config!.noise : 0) ** 2 / 3;
      let source: SensorSource;
      let measurements: Measurement[];
      if ("rays" in frame) {
        source = "lidar";
        const clusters: (typeof frame.rays)[] = [];
        let current: typeof frame.rays = [];
        for (const ray of frame.rays) {
          const previous = current.at(-1);
          if (
            !ray.hit ||
            (previous &&
              Math.hypot(
                previous.point.x - ray.point.x,
                previous.point.y - ray.point.y,
              ) > 1.5)
          ) {
            if (current.length) clusters.push(current);
            current = [];
          }
          if (ray.hit) current.push(ray);
        }
        if (current.length) clusters.push(current);
        measurements = clusters
          .filter(
            (c) =>
              c.length >= 2 &&
              Math.hypot(
                c[0].point.x - c.at(-1)!.point.x,
                c[0].point.y - c.at(-1)!.point.y,
              ) < 6,
          )
          .map((c) => {
            const nearest = c.reduce((a, b) => (a.range < b.range ? a : b));
            return {
              x: nearest.point.x,
              y: nearest.point.y,
              variance: 0.35 ** 2 + noiseVariance,
            };
          });
      } else {
        source = frame === sensors.camera ? "camera" : "radar";
        measurements = this.detections(frame, source, vehicle, noiseVariance);
      }
      this.associate(
        measurements.filter(
          (m) =>
            [m.x, m.y, m.variance].every(Number.isFinite) &&
            m.variance > 0 &&
            (!m.radial ||
              [
                m.radial.value,
                m.radial.dx,
                m.radial.dy,
                m.radial.variance,
              ].every(Number.isFinite)),
        ),
        source,
        frame.timestamp,
      );
      this.poses.set(frame.sensorId, {
        x: frame.pose.x,
        y: frame.pose.y,
        time: frame.timestamp,
      });
    }
    this.predict(now);
    this.tracks = this.tracks.filter(
      (t) => now - t.last <= this.config.deleteAfter,
    );
    this.time = now;
    return {
      timestamp: now,
      tracks: this.tracks.map((t) => this.snapshot(t, now)),
    };
  }
  private detections(
    frame: DetectionFrame,
    source: SensorSource,
    vehicle: VehicleState,
    noiseVariance: number,
  ): Measurement[] {
    const c = Math.cos(frame.pose.heading),
      s = Math.sin(frame.pose.heading);
    const previous = this.poses.get(frame.sensorId),
      dt = previous ? frame.timestamp - previous.time : 0;
    const sensorVx =
      previous && dt > 1e-6
        ? (frame.pose.x - previous.x) / dt
        : vehicle.velocity * Math.cos(vehicle.heading);
    const sensorVy =
      previous && dt > 1e-6
        ? (frame.pose.y - previous.y) / dt
        : vehicle.velocity * Math.sin(vehicle.heading);
    return frame.detections.map((d) => {
      const angle = frame.pose.heading + d.bearing,
        dx = Math.cos(angle),
        dy = Math.sin(angle);
      // Range/bearing geometries give a conservative isotropic Cartesian uncertainty.
      const variance =
        (source === "camera" ? 0.65 ** 2 : 0.3 ** 2) +
        noiseVariance * (1 + (d.range / Math.max(1, 80)) ** 2);
      return {
        x: frame.pose.x + c * d.relativePosition.x - s * d.relativePosition.y,
        y: frame.pose.y + s * d.relativePosition.x + c * d.relativePosition.y,
        variance,
        objectClass: d.objectClass,
        radial:
          source === "radar" && d.radialVelocity !== undefined
            ? {
                value: d.radialVelocity + sensorVx * dx + sensorVy * dy,
                dx,
                dy,
                variance: 0.25 + noiseVariance,
              }
            : undefined,
      };
    });
  }
  private predict(time: number) {
    for (const track of this.tracks) {
      track.estimate = predictCV(
        track.estimate,
        time - track.time,
        this.config.accelerationNoise,
      );
      track.time = time;
    }
  }
  private associate(
    measurements: Measurement[],
    source: SensorSource,
    time: number,
  ) {
    const pairs: { track: InternalTrack; index: number; distance: number }[] =
      [];
    for (const track of this.tracks)
      for (let index = 0; index < measurements.length; index++) {
        const m = measurements[index];
        if (
          track.objectClass &&
          m.objectClass &&
          track.objectClass !== m.objectClass
        )
          continue;
        // A finite geometric cap prevents newborn velocity uncertainty bridging separate actors.
        if (
          Math.hypot(
            m.x - track.estimate.state[0],
            m.y - track.estimate.state[1],
          ) > 4
        )
          continue;
        const distance = positionGate(track.estimate, m.x, m.y, m.variance);
        if (distance <= this.config.gate)
          pairs.push({ track, index, distance });
      }
    pairs.sort(
      (a, b) =>
        a.distance - b.distance ||
        a.track.id.localeCompare(b.track.id) ||
        a.index - b.index,
    );
    const assignedTracks = new Set<InternalTrack>(),
      assignedMeasurements = new Set<number>();
    for (const pair of pairs) {
      if (
        assignedTracks.has(pair.track) ||
        assignedMeasurements.has(pair.index)
      )
        continue;
      assignedTracks.add(pair.track);
      assignedMeasurements.add(pair.index);
      this.correct(pair.track, measurements[pair.index], source, time);
    }
    measurements.forEach((m, index) => {
      if (
        assignedMeasurements.has(index) ||
        this.tracks.length >= this.config.maxTracks
      )
        return;
      const estimate: Estimate = {
        state: [m.x, m.y, 0, 0],
        covariance: [
          [m.variance, 0, 0, 0],
          [0, m.variance, 0, 0],
          [0, 0, 25, 0],
          [0, 0, 0, 25],
        ],
      };
      const track: InternalTrack = {
        id: `track-${++this.serial}`,
        estimate,
        birth: time,
        time,
        last: time,
        hits: 1,
        hitTime: time,
        sources: new Set([source]),
        sourceTimes: { [source]: time },
        objectClass: m.objectClass,
      };
      if (m.radial)
        track.estimate = updateScalar(
          track.estimate,
          [0, 0, m.radial.dx, m.radial.dy],
          m.radial.value,
          m.radial.variance,
        );
      this.tracks.push(track);
    });
  }
  private correct(
    track: InternalTrack,
    m: Measurement,
    source: SensorSource,
    time: number,
  ) {
    track.estimate = updateScalar(
      track.estimate,
      [1, 0, 0, 0],
      m.x,
      m.variance,
    );
    track.estimate = updateScalar(
      track.estimate,
      [0, 1, 0, 0],
      m.y,
      m.variance,
    );
    if (m.radial)
      track.estimate = updateScalar(
        track.estimate,
        [0, 0, m.radial.dx, m.radial.dy],
        m.radial.value,
        m.radial.variance,
      );
    if (time > track.hitTime + 1e-8) {
      track.hits++;
      track.hitTime = time;
    }
    track.last = time;
    track.sources.add(source);
    track.sourceTimes[source] = time;
    if (m.objectClass) track.objectClass = m.objectClass;
  }
  private snapshot(track: InternalTrack, now: number): Track {
    const [x, y, vx, vy] = track.estimate.state;
    return {
      id: track.id,
      x,
      y,
      vx,
      vy,
      covariance: track.estimate.covariance.map((r) => [...r]),
      status:
        now - track.last > this.config.coastingAfter
          ? "coasting"
          : track.hits >= this.config.confirmationHits
            ? "confirmed"
            : "tentative",
      objectClass: track.objectClass,
      lastMeasurementTime: track.last,
      age: now - track.birth,
      sources: [...track.sources].sort(),
      sourceTimes: { ...track.sourceTimes },
    };
  }
}
