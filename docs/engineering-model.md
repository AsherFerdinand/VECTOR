# Engineering model — v0.1

## State and coordinate system

The vehicle state is `(x, y, ψ, v)`. Position `(x, y)` locates the centre of the **rear axle**, in metres on a flat simulation plane. Heading `ψ` is in radians, measured counterclockwise from the positive x axis. Signed longitudinal velocity `v` is in m/s: positive is forward and negative is reverse. Steering `δ` and longitudinal acceleration `a` are the control inputs.

The 3D renderer maps simulation `(x, y)` to Three.js `(x, height, -y)`. Local vehicle forward is +x and the rendered yaw is `ψ` around Three.js +y. Thus positive steering turns a forward-moving vehicle left in both model and view. A reversing vehicle has the opposite yaw-rate sign for the same steering angle.

Reset places the rear axle at `(-20, -29.25)` m, heading `0` rad, at rest in the eastbound lane of the southern road. Reset also clears time, wheel travel, controls, and timing backlog, and leaves the simulation paused.

## Kinematic bicycle equations

The front and rear wheel pairs are represented by one wheel each. With wheelbase `L = 2.7 m`:

```text
dx/dt = v cos(ψ)
dy/dt = v sin(ψ)
dψ/dt = (v/L) tan(δ)
dv/dt = a
```

This is an ideal rolling model with no lateral tyre slip. At constant nonzero steering, its signed turning radius is:

```text
R = L / tan(δ)
```

For example, at `δ = 30°`, `R ≈ 4.68 m`. At `v = 5 m/s`, the yaw rate is approximately `1.07 rad/s`. A zero steering angle gives zero yaw rate. The permitted maximum steering and speed are independent software bounds; using both simultaneously does not represent a credible real-vehicle handling condition.

## Integration and limits

Explicit Euler evaluates all derivatives from the old state:

```text
x_next = x + v cos(ψ) dt
y_next = y + v sin(ψ) dt
ψ_next = ψ + (v/L) tan(δ) dt
v_next = clamp(v + a dt, -10/3.6, 50/3.6)
```

The fixed integration interval is **`dt = 0.01 s`**. For example, starting from rest with `a = 3 m/s²`, the first step gives `v = 0.03 m/s` and no position change, since position uses the previous velocity. After one second the speed is `3 m/s` and displacement is `1.485 m`; the continuous solution is `1.5 m`. This first-order integration error decreases with the step size.

| Parameter                 | Value                |
| ------------------------- | -------------------- |
| Vehicle length            | 4.5 m                |
| Vehicle width             | 1.8 m                |
| Wheelbase                 | 2.7 m                |
| Road-wheel steering       | ±π/6 rad (±30°)      |
| Forward propulsion        | 3 m/s²               |
| Reverse propulsion        | 3 m/s² magnitude     |
| Maximum braking magnitude | 6 m/s²               |
| Maximum forward speed     | 50 km/h (13.889 m/s) |
| Maximum reverse speed     | 10 km/h (2.778 m/s)  |

The low-level model clamps requested acceleration to `[-6, 3] m/s²` while moving forward or at rest and `[-3, 6] m/s²` while reversing. This permits 6 m/s² braking in either travel direction. It also clamps steering to the road-wheel bounds and final velocity to the speed bounds. The keyboard controller provides the direction-sensitive behavior below. Dashboard acceleration is the **realized velocity change divided by dt**, so a clipped speed limit or a final braking step reports the applied acceleration rather than an unrealized request.

## Manual controller

A and D request left and right steering. The road-wheel angle approaches its target at **60°/s**. Releasing steering returns it to zero at **90°/s**. Pressing both steering directions is treated as a zero target. A full centre-to-lock movement therefore takes 0.5 s. These are road-wheel angles, not steering-wheel rotation.

W requests forward propulsion and S requests reverse propulsion. If the requested direction opposes the current velocity, the controller first brakes at up to `6 m/s²`. It limits that step's braking to `|v|/dt` so velocity reaches zero without crossing it. Continued directional input can start motion in the opposite direction on the following step. Reverse propulsion uses `-3 m/s²`; braking while reversing uses positive acceleration.

Space takes priority and brakes toward zero. Simultaneous W and S also brake toward zero. At rest either rule holds the vehicle stationary. With no longitudinal input the vehicle coasts at constant speed: v0.1 includes no drag, rolling resistance, or automatic creep.

## Simulation time and rendering

The application passes elapsed rendering-frame time to a fixed-step accumulator. Each running frame accepts at most **0.1 s**, then performs at most **10 physics steps** of 0.01 s each. Fractional time below one step remains in the accumulator. Wall time above the 0.1 s frame limit is discarded and counted in `droppedTime`.

This bounds work per frame and prevents an ever-growing catch-up backlog after a stalled or hidden browser tab. A one-second frame can advance only 0.1 simulation seconds; the simulation deliberately slows relative to wall time rather than changing the integration interval. Pausing advances neither the state nor simulation time; a fractional accumulator remainder is retained until resume. Hiding the browser tab pauses the application and clears keyboard input, so resuming requires an explicit Start/Resume action. Simulation time is the integer step count multiplied by dt.

Equivalent frame partitions produce the same result when they advance the same number of physics steps with the same step-by-step inputs and do not discard time. Keyboard events are sampled for each frame's physics steps, so different human input timing is not a deterministic replay format. Determinism is a property of initial state and simulation-step input sequence, not a guarantee that every live keyboard session will match.

Rendering reads snapshots; it never integrates vehicle motion. Wheel rotation uses signed model travel and front-wheel steering uses the current control angle. Camera smoothing is a visual operation and does not affect dynamics. UI unit conversions do not feed back into the model.

## Scope and limitations

The model has no mass, tyre-force calculation, lateral slip, suspension, load transfer, surface friction, collision response, grade, powertrain, or energy consumption. It does not constrain the vehicle to roads. Town buildings, markings, and traffic-light objects are visual geometry, not sensed objects or active traffic rules. There is no autonomous-driving stack in this milestone.

The model is useful for verifying coordinate conventions, basic trajectories, control interfaces, and simulation timing. It is not a validation of real-vehicle stability, braking distance, autonomous-driving safety, or regulatory compliance. Ground-truth state must remain separate from any noisy or delayed sensor observations introduced in future milestones.

## Verification

Vitest exercises straight-line motion, acceleration and braking, yaw direction, turning-radius relationships, limits, steering behavior, directional transitions, determinism, pause/reset, and equivalent rendering-frame timing. Run `npm test`, `npm run typecheck`, and `npm run build` after changing the model. Browser interaction and device performance require their own checks; passing unit tests does not establish an FPS target.
