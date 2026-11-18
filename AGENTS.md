# VECTOR development instructions

VECTOR is a lightweight autonomous-driving engineering laboratory. The current milestone combines v0.2B collision engineering and v0.3 virtual perception.

## Architecture

- Keep simulation calculations independent of React and Three.js; rendering consumes state.
- Use SI units internally and explicit TypeScript interfaces.
- Keep physics deterministic for identical simulation-step inputs.
- Derive navigation and rendering from the same structured town data.
- Both driving modes must use the same vehicle model; run autonomous control on fixed simulation steps.
- Keep exact world geometry in collision checks and sensor generators; online autonomous safety must use sensor observations, not hidden obstacle geometry.
- Keep ground-truth objects, raw measurements, and processed detections distinct. Debug object IDs are not inferred perception.
- Label perfect localisation and simplified geometric sensors explicitly; do not imply learned perception or production AEB.
- Schedule sensors from simulation time with deterministic seeded noise; rendering FPS must not drive measurements.
- Keep obstacle placement shared between rendering, collision, and sensor geometry. Manual control must not be silently overridden by autonomous safety.
- Prefer simple, readable TypeScript and necessary dependencies only.
- Preserve the distinction between simulator ground truth and future vehicle observations.

## Development

- Work incrementally and keep changes scoped to the requested feature.
- Do not implement future milestones prematurely or add empty speculative modules.
- Add meaningful tests for engineering calculations and document important assumptions.
- Avoid unrelated refactoring and preserve existing user work.

## Git

- Prefer feature branches; never force-push or rewrite history.
- Do not commit node_modules, dist, coverage, logs, or other generated output.

## Documentation

- Update README when user-visible capabilities change; distinguish plans from implemented behavior.
- Update the relevant engineering document when vehicle, navigation, collision, sensor, or protective-braking assumptions change. Keep historical milestone results identified as historical.
- Do not claim AUTOSAR compliance or real-world autonomous-driving safety.

## Verification

Before reporting completion, run `npm run typecheck`, `npm test`, and `npm run build`.
Report failures, skipped checks, unverified requirements, and measurement limitations honestly.
