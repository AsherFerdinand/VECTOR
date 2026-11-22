# Navigation model — v0.2A

> Historical baseline. Current road-containment, dynamic-traffic and behaviour changes are documented in [Traffic and autonomy](traffic-autonomy.md).

VECTOR v0.2A implements **map-based autonomous path following with perfect localisation**. The controller knows the static map, selected route, destination, and exact vehicle pose. It has no simulated sensors, dynamic-object observations, future-event knowledge, traffic-light response, or collision avoidance.

The unchanged rear-axle bicycle dynamics, units, and numerical integration are documented in [Engineering model — v0.1](engineering-model.md). This document describes the additional navigation layer and the reset behavior introduced in v0.2A.

## Coordinates and directed road graph

Simulation coordinates are `(x, y)` in metres, with heading counterclockwise from +x. Three.js uses `(x, height, -y)` and the SVG minimap uses `(x, -y)`. Positive heading therefore turns a forward-moving vehicle left in all views.

`src/world/town.json` is the common source for roads, intersections, and buildings. Each 9 m two-way road produces two explicit directed lanes with centre-lines 2.25 m to the right of their travel direction. Straight lane edges end 7.5 m before each intersection centre and restart 7.5 m beyond it. The same positions become entry and exit ports for intersection connectors.

Each incoming port connects only to compatible outgoing ports: straight, left, or right; never a U-turn into an opposing lane. Straight connectors remain lane aligned. Tangent quarter-circle turns have radii:

```text
R_right = 7.5 - 2.25 = 5.25 m
R_left  = 7.5 + 2.25 = 9.75 m
```

Both exceed the model's minimum rear-axle radius, `2.7 / tan(30°) ≈ 4.68 m`. Straight lanes use samples no farther than 0.5 m apart; turn sampling is at most 0.4 m. Nominal vehicle-body footprint samples on the reference geometry are checked against the pavement, but this is not a guarantee for the actual controller-tracked trajectory. Road endpoints are dead ends; no artificial turnaround is inserted. The selectable rear-axle range at each road end is trimmed by the geometric overhangs: 0.9 m after the upstream edge and 3.6 m before the downstream edge. This prevents the nominal vehicle body extending beyond the rendered dead end.

Map clicks are converted from SVG to world coordinates, checked against the rendered road surface, and projected onto a straight directed lane. Straight intersection approaches are selectable; the central 9 × 9 m intersection square is rejected because crossing directions are ambiguous there. Grass, buildings, and curved-turn-only selections are unavailable. The chosen start includes a lane-aligned heading and zero speed. Start relocation requires both a paused simulation and speed at most 0.01 m/s. Destination selection never moves the vehicle.

## A* and reference path

A* uses sampled edge length as cost and Euclidean distance to the destination node as its admissible heuristic. Deterministic graph insertion and tie handling give repeatable route choices. Partial first and last lanes connect the selected offsets to the node route. A destination ahead on the same directed lane uses that lane directly; a destination behind requires a valid directed loop. Disconnected or malformed selections return no route. Routes shorter than 1 m are rejected.

The resulting reference path contains ordered points, cumulative distance `s`, and signed curvature estimated from three consecutive points. Duplicate joins are removed. No reverse edges or connections across buildings are invented to make an unreachable route succeed.

Planning occurs on explicit route requests, autonomous activation, or reset. It does not run in rendering callbacks. Activation replans from the current pose without teleporting. The vehicle must be within 1 m of a selectable directed lane, have a heading error below 0.35 rad (about 20°), and not be reversing. A paused vehicle inside an intersection may need a reset or manual repositioning before replanning.

## Pure Pursuit steering

The controller projects the rear axle onto the local reference-path neighborhood. Progress is monotonic. Limiting each update's search to a short interval around the previous progress prevents a repeated visit to a junction from selecting an unrelated later segment.

The nominal look-ahead distance is speed dependent:

```text
Ld_requested = 2.5 + 0.35 max(v, 0)   [m]
```

The target is interpolated at that distance ahead along the path, clipped to its endpoint. Pure Pursuit uses the actual rear-axle-to-target distance `Ld`:

```text
delta_command = atan(2 L sin(alpha) / Ld)
```

Here `L = 2.7 m`, and `alpha` is the target bearing relative to vehicle heading. Steering is bounded to ±30° and the applied angle moves toward the command at no more than 60°/s. Telemetry distinguishes the unslewed bounded steering command from the applied angle. Cross-track error is the signed perpendicular distance to the locally selected path segment; left of the travel direction is positive. Safe-stop checks separately use Euclidean distance to the bounded path projection, so lost progress cannot be hidden by a small perpendicular error.

## Longitudinal control

Autonomous driving is forward only and capped at **6 m/s (21.6 km/h)**, below the unchanged manual-driving limit. A proportional controller with gain **2.5 s⁻¹** requests:

```text
a = clamp(2.5 (v_target - v), -6, 3)   [m/s²]
```

A final braking step is clipped so it cannot propel the vehicle into reverse. Steering and acceleration are passed to the same bicycle integrator used by keyboard driving.

For every upcoming nonzero path curvature `kappa`, the controller calculates a conservative corner speed:

```text
v_curve = sqrt(1.5 / abs(kappa))
```

The lateral acceleration budget is 1.5 m/s². A braking envelope anticipates a lower curve speed at distance `d`:

```text
v_allowed_now = sqrt(v_curve² + 2 * 2 * d)
```

Here `d = max(0, curve_distance - 3 m)`: the extra 3 m braking lead compensates for proportional speed-control lag. The comfortable planning deceleration is 2 m/s²; physical braking remains bounded by 6 m/s². Straight sections use the navigation speed cap rather than dividing by near-zero curvature. The minimum envelope over upcoming points becomes the curve speed constraint.

The destination constraint uses remaining route distance `r` and a 0.6 m stop buffer:

```text
v_stop = sqrt(2 * 2 * max(0, r - 0.6))
v_target = min(6, curve_envelope, v_stop)
```

This reduces the target progressively before curves and before arrival. The endpoint is not a position constraint imposed on the vehicle: residual position error is measured after integrating the vehicle motion.

## Navigation states and command authority

| State                 | Implemented behavior                                                                               |
| --------------------- | -------------------------------------------------------------------------------------------------- |
| `IDLE`                | No selected route; manual driving remains available.                                               |
| `ROUTE_READY`         | A valid route is previewed, awaiting activation.                                                   |
| `ROUTE_FOLLOWING`     | Pure Pursuit and speed control issue commands.                                                     |
| `APPROACHING_TURN`    | An upcoming curve limits target speed before the destination constraint does.                      |
| `DESTINATION_REACHED` | Remaining distance ≤0.8 m, endpoint error ≤0.8 m, and speed ≤0.1 m/s; braking holds rest.          |
| `ROUTE_UNAVAILABLE`   | The requested start/destination cannot produce a valid route.                                      |
| `SAFE_STOP`           | Tracking tolerance is exceeded or direction/endpoint checks fail; braking replaces path following. |

Safe-stop detection includes Euclidean distance to the bounded path projection above 3 m, reverse speed below −0.1 m/s, a target behind the vehicle away from the destination, and excessive endpoint overshoot. This is a controller fault response, not obstacle detection or an emergency-driving safety claim.

Only one controller owns commands on a simulation step. A manual driving keypress immediately disengages autonomous command generation, including a tap shorter than one simulation step. The manual controller then supplies the next step's commands. Taking over preserves velocity and pose; use Space or the opposite direction input to brake. Pausing stops both dynamics and controller time. Camera changes have no effect on state.

Reset restores the selected start pose, clears time, controls, trajectory, and backlog, pauses in manual mode, and recalculates the retained destination. Selecting a new start also makes that pose the deterministic reset location. Clearing a route removes navigation data without teleporting or instantly stopping the vehicle.

## Timing and visualization

Autonomous control runs at the existing **0.01 s** physics step. The bounded real-time accumulator still accepts at most 0.1 s per frame and performs at most ten steps. Excess wall time is reported as discarded backlog; the controller does not change its step size to compensate. Headless engineering scenarios use explicit fixed steps, independent of render timing.

The planned route is turquoise. The optional engineering overlay adds the actual trajectory in amber, the look-ahead point, and steering direction. Actual trajectory storage is bounded to 600 samples separated by about 0.3 m. Route geometry is rebuilt when its route changes, not every rendering frame. Telemetry reports calculated values and uses an unavailable state for inactive controller values.

## Engineering validation

Acceptance bounds were defined before evaluating the scenarios. A lane centre is 2.25 m from its edge; the chosen path-tracking bounds are deliberately tighter, but do not constitute a swept-body or collision-clearance guarantee.

| Measurement                             | Acceptance bound        |
| --------------------------------------- | ----------------------- |
| Maximum absolute cross-track error      | ≤1.0 m                  |
| RMS cross-track error                   | ≤0.5 m                  |
| Final rear-axle distance to destination | ≤0.8 m                  |
| Final speed                             | ≤0.1 m/s                |
| Progress advance per 0.01 s step        | ≤0.3 m                  |
| Route completion                        | <180 simulation seconds |

Corner-entry speed is additionally checked against `sqrt(1.5 / abs(kappa)) + 0.25 m/s`. The 0.25 m/s tolerance was declared before evaluating the final controller tuning and accounts for proportional-controller tracking lag. The 1.5 m/s² value is a target-speed budget, not a hard bound on realised lateral acceleration.

The deterministic suite starts at `(-20, -29.25)` m, heading east, at rest. It uses explicit 0.01 s steps through the production bicycle model. RMS cross-track error is computed over controller samples through the stopped terminal state. Results from the completed scenarios are:

| Scenario                            | Destination (m) | Max CTE (m) | RMS CTE (m) | Final error (m) | Final speed (m/s) | Time (s) |
| ----------------------------------- | --------------- | ----------: | ----------: | --------------: | ----------------: | -------: |
| A — Straight / D — Destination stop | `(10, -29.25)`  |       0.000 |       0.000 |           0.102 |                 0 |     7.67 |
| B — 90° right turn                  | `(24.75, -45)`  |       0.263 |       0.105 |           0.087 |                 0 |    14.79 |
| C — Multiple intersections          | `(-40, -24.75)` |       0.260 |       0.085 |           0.034 |                 0 |    46.54 |

All three reached `DESTINATION_REACHED` within the predefined thresholds. Maximum progress advance was below 0.06 m per tick, with no unexplained segment jumps. The right-turn scenario used a maximum applied steering angle of about 28.06° and entered its first curve at 2.930 m/s. The multiple-intersection scenario entered its first curve at 3.962 m/s. Their maximum model lateral accelerations were 1.557 and 1.581 m/s² respectively, illustrating the residual tracking lag rather than a hard 1.5 m/s² guarantee. Maximum speed stayed below 6 m/s. An identical fixed-step replay produced exactly identical measurements.

Scenario E uses integration tests to verify that manual takeover transfers command authority without teleporting or resetting simulation time; pause freezes the vehicle and controller, and reset restores the selected start and recalculates the route. The complete milestone run passed 56 automated tests, including all 20 unchanged v0.1 tests. TypeScript checks and production build passed. Browser checks, their scope, and performance limitations are summarized in the README.

## Limitations

The map is a compact orthogonal town, with no lane changes or endpoint turnarounds. A* optimizes distance, not travel time or traffic behavior. Curved connections are tangent circular samples with curvature changes at their joins; they are not clothoids. Pure Pursuit can cut inside a curve and retains discretisation and tracking error. Its lateral-acceleration constraint is a planning approximation, not a dynamic tyre-force model.

Perfect localisation and a static map are explicit baseline assumptions. Traffic lights, buildings, and crossings are not perceived or obeyed. There are no moving road users, collisions, dynamic replanning around obstacles, sensor noise, or localisation uncertainty. Manual driving is still unrestricted by road boundaries. Neither passing these tests nor a visually completed route demonstrates real-world autonomous-driving safety. Windows-device FPS remains a separate unverified performance target.

## v0.3 controller update and regression results

The sections and measurements above document the **historical v0.2A, collision-free and sensor-free baseline**. Current collision constraints, geometric sensors, and sensor-based protective braking are described in [Collision engineering and virtual perception](collision-sensors.md). The vehicle equations and fixed 0.01 s integration step are unchanged.

### Lateral-acceleration investigation

The original reference-curvature speed envelope did not fully account for Pure Pursuit cutting inside a curve, transient applied steering, and proportional speed-control lag. It produced measured `v² * abs(tan(delta_applied)) / L` peaks of 1.55698 m/s² in the right turn and 1.58146 m/s² through multiple intersections, above the intended 1.5 m/s² budget.

The controller now computes an additional tracking target:

```text
kappa_tracking = max(abs(tan(delta_applied)), abs(tan(delta_requested))) / L
v_tracking = sqrt(1.5 * 0.95 / kappa_tracking)
v_reference_reserved = v_reference * sqrt(0.95)  [when curve-limited]
v_controller_target = min(v_reference_reserved, v_tracking)
```

Near zero curvature, the tracking limit is the existing 6 m/s navigation cap; straight-path target speed is unchanged. The 5% lateral reserve accommodates speed-controller lag and realised Pure Pursuit curvature. It adjusts only the autonomous speed target: it does not change the bicycle model, impose an artificial velocity clamp, weaken the 1.5 m/s² acceptance budget, or alter manual-driving limits.

Five regression scenarios check lateral acceleration using actual velocity and applied steering both before and after each integration step, with an acceptance tolerance of **1.500001 m/s²**. The following current headless measurements retain the earlier cross-track and stopping thresholds:

| Scenario               | Max CTE (m) | RMS CTE (m) | Final error (m) | Final speed (m/s) | Time (s) | Peak model lateral acceleration (m/s²) |
| ---------------------- | ----------: | ----------: | --------------: | ----------------: | -------: | -------------------------------------: |
| Straight               |       0.000 |       0.000 |        0.101804 |                 0 |     7.67 |                                      0 |
| Right turn             |    0.258224 |    0.103697 |        0.086823 |                 0 |    14.96 |                               1.461621 |
| Left turn              |    0.163971 |    0.058155 |        0.101604 |                 0 |    17.10 |                               1.463489 |
| Multiple intersections |    0.256085 |    0.083397 |        0.031543 |                 0 |    47.25 |                               1.463517 |

These results demonstrate the correction in the measured scenarios; they are not a formal global handling constraint for arbitrary initial speed, steering, pose error, or future trajectories. Maximum straight-line speed remains about 5.9991 m/s.

### Physical route containment

The current production `Simulation` also runs collision resolution, full-body road classification, static route preflight, real sensor scheduling, and safety arbitration. The right-turn and multiple-intersection regressions complete with zero collisions and zero final speed, but each records **one partial road-departure event** from tracking error. Their nominal reference-path footprints pass static clearance with the default margin. Static preflight therefore does not guarantee that the actual tracked body remains entirely on the pavement. Road departure is an exposed engineering warning and does not act as an invisible collision wall.

The right-turn run has 150 LiDAR frames over 14.96 s; the multiple-intersection run has 473 frames over 47.25 s. Sensor generation remains tied to simulation time rather than rendering rate. The [collision and sensor validation](collision-sensors.md#production-integration-and-navigation-regression) distinguishes real-scheduler integration from isolated supervisor tests.
