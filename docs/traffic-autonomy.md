# Traffic and autonomy — v0.3.1 to v0.5

## Ground truth and observations

Traffic scripts, signal phases, collision geometry and rendering share the world model. The online behaviour planner receives sensor frames, the static route and perfect ego localisation. It never reads actor objects, their debug identifiers, or future script values. Camera-visible signal colours are synthetic geometric observations; radar reports range, bearing and radial velocity without a classifier. Nearby camera semantics are associated with radar points within 2.5 m. This is a small geometric association, not robust multi-object tracking.

The main image and camera detections use acquisition-time ego, actor and signal poses. The offscreen image temporarily rewinds render-only actor/light data and restores the live scene afterwards; that data is never sent to the planner.

## Containment and limits

The earlier long-look-ahead controller cut the inside of tight turns. The navigation controller now requests `Ld = 1.2 + 0.15v` metres; the standalone Pure Pursuit utility retains its earlier default for compatibility. No position is projected, clipped or corrected to hide a departure. Sixteen rotated straight/right/left/multiple-turn cases and three stop/restart approaches test complete body footprints at each 0.01 s step and 0.002 s interpolated samples.

Runtime validation conservatively covers translation and rotation between proposed poses. Body-boundary travel is divided into intervals of at most 0.01 m, with half-interval inflation bounding intermediate footprints. An autonomous command that fails this road-union check is stopped and explicitly reported as a simulator containment interlock. Manual driving may still leave the road. Successful route validation requires zero interlock activations, as well as zero departures.

The bicycle equations, ±30° steering, 60°/s steering rate, +3/−6 m/s² acceleration bounds and 1.5 m/s² navigation lateral budget are retained. Current/requested steering curvature constrains speed. Navigation braking has priority over comfort jerk smoothing so turn-entry and destination constraints are not delayed. Only velocity round-off below 1e−12 m/s is normalized to zero; meaningful motion and position are unchanged.

## Deterministic traffic

Six scenarios reset the vehicle, traffic, sensor schedules, event histories and simulation time:

| Scenario            | Script                                                                                     |
| ------------------- | ------------------------------------------------------------------------------------------ |
| Normal traffic      | A bounded northern-road vehicle and a crossing pedestrian.                                 |
| Lead following      | A lead vehicle initially moving at 3.5 m/s, then stopping before its mapped lane endpoint. |
| Sudden lead braking | Lead braking at 5 s, a 4 s hold, and a bounded restart.                                    |
| Red traffic light   | East/west red for 10 s, green to 24 s, yellow to 27 s, then red in a 40 s cycle.           |
| Pedestrian crossing | A pedestrian enters the western crossing of the eastern junction at 3 s.                   |
| Blocked route       | A mapped 2 m square solid blocks the initial lane.                                         |

Normal signal cycles use east/west green 0–12 s, yellow 12–15 s, red 15–30 s; north/south phases are complementary. All 16 approaches derive poles, stop lines and crossing geometry from the town. Vehicle acceleration/deceleration scripts are bounded; pedestrians ramp to 1.2 m/s. Actors stop at their endpoints and do not teleport into loops. These small scenarios are not a free-running microscopic traffic model.

The application explicitly selects normal traffic. Programmatic `new Simulation()` without a scenario retains the historical fixture mode for compatibility; production scenario tests explicitly enable the same scenarios used by the application.

## Collision and route validation

Relative conservative advancement checks ego and moving convex actor footprints, including rotation and actors crossing a stationary ego. A contacting actor is frozen at the contact fraction until reset; this is conservative crash handling without impulses or avoidance. Static physical constraints remain separate from sensed obstacle responses.

Static route preflight excludes dynamic actors. If a known mapped solid blocks a route, blocked graph edges are removed and A* may produce an alternative. The entire alternative is checked again for road and obstacle clearance. No feasible directed alternative means rejection; no illegal U-turn or hidden-actor rerouting is invented.

## Behaviour priorities

The state machine uses measured evidence:

1. Missing/stale primary LiDAR (>0.30 s) requests stopping.
2. AEB considers corridor overlap, closing speed, TTC, braking capability and sensor age. Its closing-distance threshold is `v_closing² / 12 + v_closing × (sample_age + 0.2) + 0.7` metres, with an additional 1 s TTC trigger and a 0.5 m immediate-clearance trigger. The separate displayed stopping envelope uses ego speed and a 0.3 s allowance. It overrides comfort limits and normal longitudinal control.
3. Pedestrian yielding, observed traffic controls and blocked-corridor stopping constrain speed.
4. ACC selects a relevant observed lead and uses `d_desired = 3 + T_gap × v_ego`, with configurable 1–3 s time gap.
5. Otherwise navigation supplies the cruise/turn/destination command.

Normal longitudinal constraints are combined: stopping for a light cannot remove a nearer lead-vehicle constraint. Comfort jerk is limited to 3 m/s³ where compatible with urgent and navigation braking. Applied acceleration remains bounded by the vehicle limits. Pedestrian and signal stopping anticipate braking/jerk delay and hold rest before the hazard. A last observed red remains a stop condition if occluded; a fresh green clears it. Yellow is stopped for when a comfortable stop is feasible, otherwise the vehicle proceeds. Clear observations must remain stable for 0.50 s before resumption. Manual takeover immediately returns command authority.

Compute displays actual state/reason, observation time/source, lead range and relative speed, desired gap, TTC, stopping envelope, AEB, target speed and commands. Unavailable quantities remain unavailable. Relative speed is a radial/longitudinal approximation, not hidden actor velocity.

## Measured validation

Acceptance is fixed before evaluation: no collisions, road departures or containment interlocks in production scenarios; route completion within 180 simulation seconds; peak model lateral acceleration ≤1.500001 m/s²; body separation ≥1 m in the urgent-takeover case; red-line crossing tolerance 0.05 m. Every original test remains unchanged.

| Production case                     | Result                                                                                                                                                                     |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Normal route                        | Completed in 8.26 s.                                                                                                                                                       |
| Lead following                      | Completed in 10.02 s; minimum body gap 5.908 m; maximum measured-gap error 0.035 m.                                                                                        |
| Sudden lead braking                 | Completed in 10.36 s; minimum gap 5.626 m; maximum measured-gap error 0.049 m.                                                                                             |
| Red signal                          | Stopped before green and resumed; maximum ego front before green 17.634 m against stop line x=19.5 m; completed in 18.19 s.                                                |
| Pedestrian                          | Yielded before the crossing, resumed after clearance and completed in 18.18 s.                                                                                             |
| Blocked route                       | Rejected before motion; no infeasible autonomous activation.                                                                                                               |
| Right turn with traffic enabled     | Completed in 15.02 s; maximum lateral acceleration 1.473 m/s².                                                                                                             |
| Multiple intersections with signals | Completed in 54.65 s; maximum lateral acceleration 1.484 m/s².                                                                                                             |
| Urgent takeover at 10.5 m/s         | AEB on the first autonomous tick; TTC 0.775 s, −6 m/s² braking; minimum gap 1.309 m and zero collisions. Speed falls below 0.1 m/s after 1.74 s and 9.241 m of ego travel. |
| Mapped alternative                  | Obstructed 227.049 m route replaced by a validated 241.187 m directed route.                                                                                               |

All listed production routes have zero collisions, road departures and containment stops. The 16 rotated containment cases have worst tracking error about 0.053 m and peak lateral acceleration 1.491386 m/s². The urgent-takeover case starts with deliberate manual overspeed relative to the 6 m/s navigation cap; it is distinct from the default gentle-braking preset. Further tests cover moving-body contact/freeze/reverse escape, stationary-ego pedestrian contact prevention, exact scenario replay, sensor timing, pause and manual takeover.

## Limits

Actors use bounded scripts, perfect localisation remains assumed, and sensors are geometric models. Camera/radar association is intentionally simple and unseen signals cannot be inferred. There is no overtaking, general dynamic rerouting, pedestrian intent prediction or production AEB guarantee. Numerical conservative sweeps may reject extremely close clearances. Windows hardware FPS is unverified; browser checks establish functionality rather than real-world safety or performance.

## Verification and scope

The final suite passes 186 tests: all 110 baseline tests plus 19 containment, 23 traffic/collision, 16 behaviour, 11 production-scenario, three alternative-route and four traffic-safety integration tests. TypeScript checks and production build pass. Scenario reset/replay and start-pose checks include rejection of a pose that would overlap an actor after scenario time resets.

Production Chromium/software-WebGL checks passed for all six scenario presets, time-gap adjustment, red/green and pedestrian stop/resumption, moving camera images, sensor tabs/overlays, manual takeover, pause/reset, camera switching and desktop/mobile layout. No page or console errors were observed. Urgent AEB and moving-contact thresholds are evaluated deterministically in the production fixed-step integration tests. The committed screenshot is genuine. Vite reports its existing bundle-size warning (about 351 kB gzip JavaScript); no dependencies were added.
