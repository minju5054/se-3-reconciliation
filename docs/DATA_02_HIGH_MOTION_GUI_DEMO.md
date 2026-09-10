# DATA-02 Saved High-Motion GUI Demo

## Purpose and boundary

This is a professor-facing visualization of immutable DATA-02 evidence, not an algorithm change,
new experiment, or scientific sample-selection result. It supersedes the earlier short collection
demo because that default moved only `0.15539962298348772 m` from FRESH observation to model
readiness and looked nearly stationary in the Hospital viewport.

The replacement reads saved telemetry and directly displays the official Jackal mesh at recorded
poses. It does not start LightNav, recollect data, issue trajectory-follower or wheel commands,
advance physics to recreate motion, reconcile OLD/FRESH, scale `x/y/yaw`, or modify DATA-02. The
one-command default is:

```bash
./scripts/isaac/run_data02_high_motion_demo.sh
```

Optional overrides are `--run PATH`, `--episode ID`, `--transition N`, `--duration 12..18`,
`--hold`, and `--show-rgb`. RGB is hidden by default so it cannot obscure the Jackal or paths.

## Deterministic selection

Before Isaac opens, the launcher scans both immutable runs and writes generated, ignored analysis
to `data/data02_collection_demo_selection/high_motion_candidates.csv` and
`selected_transition.json`. It checks the collection-manifest hash of each episode telemetry file,
the manifest hash of every selected transition JSON, and the artifact hashes checked by the DATA-02
validator. Only `ELIGIBLE_MOVING` transitions with valid P/B ordering, pre-observation samples,
post-switch samples, and nonzero saved motion are candidates.

The chosen transition must first be in the top 10% by observation-to-model-ready translation
(rank-based, including boundary ties). Within that set, the primary sort is decreasing replay path
length, followed by decreasing inference translation, decreasing replay net displacement,
non-straight FRESH geometry, decreasing absolute desired-omega jump, then lexicographic identity.
No GUI appearance is used to choose a candidate.

The resulting default is v2 `episode_000062`, `transition_03`:

| Saved measurement | New default | Previous `episode_000061/transition_04` |
|---|---:|---:|
| inference translation | 0.344356561 m | 0.155399623 m |
| inference path length | 0.344356787 m | 0.155587229 m |
| full replay path length | 1.410891385 m | 0.804321695 m |
| full replay net displacement | 1.410890601 m | 0.690719358 m |
| absolute full-window yaw change | 0.000485740 rad | 1.083185727 rad |
| absolute desired-v jump | 0.035927807 m/s | 0.365681231 m/s |
| absolute desired-omega jump | 0.002997163 rad/s | 1.737875451 rad/s |
| FRESH geometry | STRAIGHT_LIKE | POSITIVE_TURNING |

The new inference translation is 2.216 times the previous value; its full replay path is 1.754
times as long. This selection is for motion legibility, not representativeness or scientific
difficulty.

## Replay and presentation semantics

The default uses 236 saved episode-telemetry samples from simulation time
`19.800001033 s` through `23.716667904 s`: approximately 1 second before
`t_obs=20.783334417 s`, through `t_ready=t_switch=21.733334467 s`, to approximately 2 seconds
after the selected switch. No pose outside that saved interval is invented. Because DATA-02 is a
successive stream, later collection events can occur inside the two-second tail; the visualization
only highlights the selected transition and makes no claim that its one FRESH chunk controls the
entire tail.

The default 15-second presentation clock is distinct from immutable scientific time:

| Presentation | Phase | Saved interval shown |
|---:|---|---|
| 0–4 s | OLD EXECUTING | pre-observation approach |
| 4–9 s | FRESH INFERENCE — OLD STILL EXECUTING | observation to model readiness |
| 9–11 s | FRESH READY / SWITCH AT B | readiness-to-switch hold; equal timestamps here |
| 11–15 s | FRESH ACTIVE | saved motion after the selected B |

For smooth rendering, `x/y` are linearly interpolated and yaw follows the wrapped shortest angle
only between the two adjacent saved samples bracketing the display time. This
`DISPLAY INTERPOLATION BETWEEN SAVED TELEMETRY SAMPLES` is presentation-only. Scientific samples,
endpoints, event times, and path geometry remain unchanged.

## View and legend

The stable elevated oblique camera is computed once from saved actual, OLD, and FRESH bounds. For
the default it uses a 55-degree horizontal field of view, targets
`[19.000864, 27.278085, 0.18]`, and estimates the full actual replay extent at `63.4%` of the
viewport and all displayed geometry at `82.0%`. It does not rotate or follow the robot. Hospital
floor, walls, and furniture remain stationary visual references.

- Blue, thick line: saved OLD world trajectory.
- Magenta, thick line: saved raw observation-anchored FRESH world trajectory.
- Green, growing thick line: saved actual robot history.
- Yellow footprint, point, heading, and observation-to-current line: fixed FRESH observation.
- Orange marker and heading: P, last saved pose immediately before the selected switch.
- Red marker and heading: B, saved selected switch boundary.

The official Jackal mesh moves while the yellow observation ghost remains fixed. During inference,
the overlay prominently reports `MOVED DURING FRESH INFERENCE` and current observation-to-Jackal
displacement. The bottom text says `Saved replay — no LightNav inference / no physics
re-execution`.

## Captures and integrity

Every completed run creates a new ignored directory under
`data/data02_high_motion_demo/<UTC>/` containing:

- `01_before_observation.png`
- `02_during_inference.png`
- `03_after_switch.png`
- `capture_manifest.json`

The manifest records each capture's saved-time bracket, exact lower/upper saved poses,
interpolation fraction, displayed pose, camera bounds, `d_before_during`, `d_during_after`, source
hashes, and numeric confirmation that the displayed pose changed substantially. A non-black image
alone is not accepted. Source arrays returned by the loader are copies marked read-only, and the
source DATA-02 trees remain untouched.

These displays establish that the Jackal changed position in the recorded wheel-driven DATA-02
execution. They do not establish a causal controller, wheel, tire-contact, or LightNav explanation,
and presentation interpolation must not be treated as a new measurement.
