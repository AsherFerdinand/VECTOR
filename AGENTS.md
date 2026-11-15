# VECTOR development instructions

VECTOR is a lightweight autonomous-driving engineering laboratory. The current milestone is v0.1, Simulation Foundation.

## Architecture

- Keep simulation calculations independent of React and Three.js; rendering consumes state.
- Use SI units internally and explicit TypeScript interfaces.
- Keep physics deterministic for identical simulation-step inputs.
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
- Update docs/engineering-model.md when model assumptions change.
- Do not claim AUTOSAR compliance or real-world autonomous-driving safety.

## Verification

Before reporting completion, run `npm run typecheck`, `npm test`, and `npm run build`.
Report failures, skipped checks, unverified requirements, and measurement limitations honestly.
