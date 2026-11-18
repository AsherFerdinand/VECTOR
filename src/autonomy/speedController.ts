import type { Route } from "../navigation/types";
import { clamp, VEHICLE } from "../simulation/vehicle";

export const NAVIGATION_LIMITS = Object.freeze({
  maxSpeed: 6,
  lateralAcceleration: 1.5,
  comfortableBraking: 2,
  speedGain: 2.5,
  stopBuffer: 0.6,
  brakingLead: 3,
  trackingReserve: 0.95,
});

/** Backward braking envelopes anticipate every upcoming curve, not just current curvature. */
export function targetSpeed(
  route: Route,
  progress: number,
): { speed: number; approachingTurn: boolean } {
  const remaining = Math.max(0, route.length - progress);
  let curveLimit: number = NAVIGATION_LIMITS.maxSpeed;
  for (const point of route.points) {
    if (point.s < progress - 0.5) continue;
    const curvature = Math.abs(point.curvature);
    if (curvature < 1e-6) continue;
    const cornerSpeed = Math.sqrt(
      NAVIGATION_LIMITS.lateralAcceleration / curvature,
    );
    const distance = Math.max(
      0,
      point.s - progress - NAVIGATION_LIMITS.brakingLead,
    );
    const envelope = Math.sqrt(
      cornerSpeed * cornerSpeed +
        2 * NAVIGATION_LIMITS.comfortableBraking * distance,
    );
    curveLimit = Math.min(curveLimit, envelope);
  }
  const stoppingLimit = Math.sqrt(
    2 *
      NAVIGATION_LIMITS.comfortableBraking *
      Math.max(0, remaining - NAVIGATION_LIMITS.stopBuffer),
  );
  return {
    speed: Math.min(curveLimit, stoppingLimit),
    approachingTurn:
      curveLimit < NAVIGATION_LIMITS.maxSpeed - 0.05 &&
      curveLimit < stoppingLimit,
  };
}

/** Forward-only navigation; braking is clipped to stop at zero, never enter reverse. */
export function speedControl(
  velocity: number,
  target: number,
  dt: number,
): number {
  const acceleration = clamp(
    NAVIGATION_LIMITS.speedGain * (target - velocity),
    -VEHICLE.maxBraking,
    VEHICLE.maxAcceleration,
  );
  return velocity >= 0
    ? Math.max(acceleration, -velocity / dt)
    : Math.min(VEHICLE.maxBraking, -velocity / dt);
}

/** The reference curve is not the realised bicycle curvature. Reserve 5% of the
 * lateral budget for pursuit/longitudinal tracking error, and cap speed using
 * both current and requested steering. This changes the controller target only;
 * velocity and vehicle dynamics remain untouched. */
export function trackingTargetSpeed(
  reference: { speed: number; approachingTurn: boolean },
  actualSteering: number,
  requestedSteering: number,
): number {
  const curvature =
    Math.max(
      Math.abs(Math.tan(actualSteering)),
      Math.abs(Math.tan(requestedSteering)),
    ) / VEHICLE.wheelbase;
  const trackingLimit =
    curvature > 1e-6
      ? Math.sqrt(
          (NAVIGATION_LIMITS.lateralAcceleration *
            NAVIGATION_LIMITS.trackingReserve) /
            curvature,
        )
      : NAVIGATION_LIMITS.maxSpeed;
  const referenceLimit = reference.approachingTurn
    ? reference.speed * Math.sqrt(NAVIGATION_LIMITS.trackingReserve)
    : reference.speed;
  return Math.min(referenceLimit, trackingLimit);
}
