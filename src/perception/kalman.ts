/** Small constant-velocity filter. Joseph updates preserve covariance symmetry/PSD. */
export type Matrix = number[][];
export interface Estimate {
  state: number[];
  covariance: Matrix;
}
const identity = () =>
  Array.from({ length: 4 }, (_, i) =>
    Array.from({ length: 4 }, (_, j) => +(i === j)),
  );
const transpose = (a: Matrix): Matrix =>
  a[0].map((_, i) => a.map((row) => row[i]));
const multiply = (a: Matrix, b: Matrix): Matrix =>
  a.map((row) =>
    b[0].map((_, j) => row.reduce((sum, v, k) => sum + v * b[k][j], 0)),
  );
export function predictCV(
  estimate: Estimate,
  dt: number,
  accelerationNoise = 2,
): Estimate {
  const t = Math.max(0, dt);
  if (t === 0) return estimate;
  const state = [...estimate.state];
  state[0] += t * state[2];
  state[1] += t * state[3];
  const p = estimate.covariance;
  // Exploit F's sparse structure instead of allocating general matrix products at100Hz.
  const covariance = p.map((row, i) =>
    row.map(
      (value, j) =>
        value +
        (i < 2 ? t * p[i + 2][j] : 0) +
        (j < 2 ? t * p[i][j + 2] : 0) +
        (i < 2 && j < 2 ? t * t * p[i + 2][j + 2] : 0),
    ),
  );
  const q = accelerationNoise ** 2;
  for (const [p, v] of [
    [0, 2],
    [1, 3],
  ]) {
    covariance[p][p] += (q * t ** 3) / 3;
    covariance[p][v] += (q * t ** 2) / 2;
    covariance[v][p] += (q * t ** 2) / 2;
    covariance[v][v] += q * t;
  }
  return { state, covariance };
}
export function innovation(
  estimate: Estimate,
  h: number[],
  measurement: number,
  variance: number,
) {
  const predicted = h.reduce((s, v, i) => s + v * estimate.state[i], 0);
  const ph = estimate.covariance.map((row) =>
    row.reduce((s, v, i) => s + v * h[i], 0),
  );
  return {
    residual: measurement - predicted,
    variance: variance + h.reduce((s, v, i) => s + v * ph[i], 0),
    ph,
  };
}
export function updateScalar(
  estimate: Estimate,
  h: number[],
  measurement: number,
  variance: number,
): Estimate {
  const inn = innovation(estimate, h, measurement, variance);
  if (!(inn.variance > 0)) return estimate;
  const k = inn.ph.map((v) => v / inn.variance);
  const state = estimate.state.map((v, i) => v + k[i] * inn.residual);
  const a = identity().map((row, i) => row.map((v, j) => v - k[i] * h[j]));
  const covariance = multiply(
    multiply(a, estimate.covariance),
    transpose(a),
  ).map((row, i) => row.map((v, j) => v + k[i] * variance * k[j]));
  return { state, covariance };
}
/** Squared 2D Mahalanobis innovation, including correlated positional covariance. */
export function positionGate(
  e: Estimate,
  x: number,
  y: number,
  variance: number,
): number {
  const a = e.covariance[0][0] + variance,
    b = e.covariance[0][1],
    d = e.covariance[1][1] + variance;
  const dx = x - e.state[0],
    dy = y - e.state[1],
    det = a * d - b * b;
  return det > 1e-12
    ? (d * dx * dx - 2 * b * dx * dy + a * dy * dy) / det
    : Infinity;
}
