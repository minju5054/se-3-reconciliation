# DATA-02 Saved Current-OLD GUI Demo

## Purpose and boundary

This is a professor-facing visualization of immutable DATA-02 evidence, not an algorithm
change, new experiment, or scientific sample-selection result. It reads saved telemetry and
directly displays the official Jackal mesh at recorded poses. It does not start LightNav,
recollect data, issue controller or wheel commands, advance physics to recreate motion,
reconcile OLD/FRESH, or spatially scale `x/y/yaw`.

The one-command default is:

```bash
./scripts/isaac/run_data02_high_motion_demo.sh
```

It animates for 15 presentation seconds and then holds the final B view until Isaac is closed.
Use `--no-hold` for automated capture-and-exit. Optional overrides are `--run PATH`,
`--episode ID`, `--transition N`, `--duration 12..18`, and `--show-rgb`. `--hold` is accepted
explicitly but is already the default. RGB is hidden by default so it cannot obscure the robot.

## Correct current-OLD semantics

The earlier version used an arbitrary episode window from one second before FRESH observation
until two seconds after the selected switch. That mixed motion from adjacent active chunks into
the green line and could make the displayed Jackal look unrelated to the displayed OLD.

The corrected loader treats each selected transition's `actual.npy` and `telemetry.csv` as the
primary source. The collector created this segment after the current OLD was activated and
before its FRESH was promoted. It verifies that the transition CSV is an exact contiguous slice
of the hash-checked episode telemetry and requires every telemetry row to have
`active_chunk_id == old_chunk_id`.

For transition `i > 0`, the exact activation event is the preceding transition's saved switch
time and B pose. That B is prepended to the display path as an explicit boundary event; it is
not claimed to be a current-OLD telemetry row, because the first current-OLD telemetry sample
arrives one physics step later. For transition zero, the earliest saved bootstrap-OLD telemetry
state is used and no earlier pose is extrapolated. The interval ends exactly at the selected
transition's B. Observation must lie inside it, P must precede B, and the last actual sample must
equal B. No previous-chunk actual path and no post-switch actual path are displayed.

## Deterministic selection

Before Isaac opens, the launcher scans both immutable runs and writes generated, ignored
analysis to `data/data02_collection_demo_selection/high_motion_candidates.csv` and
`selected_transition.json`. It verifies collection-manifest, transition, array, and telemetry
hashes. All 959 saved `ELIGIBLE_MOVING` transitions satisfy the corrected active-OLD
reconstruction.

The visualization-only selection first restricts candidates to the top 10% by
observation-to-model-ready translation (rank-based, boundary ties included). It then ranks by
decreasing `active_old_path_length_m`, decreasing inference translation, decreasing active-OLD
net displacement, non-straight geometry, absolute desired-omega jump, and lexical identity.
No GUI appearance is used to choose the case.

The corrected default is v1 `episode_000007`, `transition_03`, current OLD `chunk_03`:

| Saved measurement | Corrected default | Pre-correction default v2 `episode_000062/transition_03` |
|---|---:|---:|
| OLD activation time | 20.133334383 s | 20.233334389 s |
| FRESH observation time | 20.783334417 s | 20.783334417 s |
| selected switch B time | 21.733334467 s | 21.733334467 s |
| active-OLD path length | 0.580704094 m | 0.542412192 m |
| active-OLD net displacement | 0.580703864 m | 0.542411814 m |
| inference translation | 0.330802986 m | 0.344356561 m |
| absolute desired-v jump | 0.042670638 m/s | 0.035927807 m/s |
| absolute desired-omega jump | 0.001111356 rad/s | 0.002997163 rad/s |

The old extended-window path length of `1.410891385 m` is no longer a selection or display
metric: approximately `0.160660 m` belonged before current-OLD activation and `0.707819 m`
belonged after the selected switch. The corrected default's unscaled active-OLD motion is about
`0.581 m` and remains plainly visible when replayed at presentation speed.

## Replay phases and legend

The presentation clock is deliberately distinct from immutable scientific simulation time:

| Presentation | Phase | Saved state |
|---:|---|---|
| 0-4 s | CURRENT OLD ACTIVE | activation boundary/earliest bootstrap state to observation |
| 4-11 s | FRESH REQUEST / INFERENCE, OLD STILL ACTIVE | observation to selected B |
| 11-15 s | STOPPED AT SELECTED B | robot stays at B; no FRESH execution |

For smooth display only, XY is interpolated linearly and yaw by the wrapped shortest angle
between the two adjacent saved samples. This does not create a scientific measurement.

- Blue: current OLD world trajectory.
- Green: growing saved actual path while that OLD alone was active.
- Magenta: raw observation-anchored FRESH trajectory, revealed after observation.
- Yellow: FRESH observation.
- Orange: P, the last saved controller-boundary pose before the selected switch.
- Red: selected switch boundary B.

There are no per-waypoint headings, footprint polygons, observation-to-current connector,
previous-chunk trail, or post-switch trail. At B the Jackal stops. Raw FRESH is shown as a
reference and is not executed.

## Camera, captures, and integrity

The stable elevated oblique camera is computed once from current OLD, raw FRESH, and the exact
active-OLD path. Hospital geometry remains visible as a spatial reference. Each completed run
creates a new ignored directory under `data/data02_high_motion_demo/<UTC>/` containing:

- `01_old_active.png`
- `02_at_observation.png`
- `03_at_B.png`
- `capture_manifest.json`

The manifest records source hashes, activation semantics, exact active interval, saved-time
brackets and interpolation fractions, camera bounds, observed display displacement, and explicit
`false` flags for previous-chunk and post-switch actual display. It also confirms that the final
display pose equals B. Scientific source arrays are copied read-only and source DATA-02 trees
remain untouched.

This visualization establishes how the saved Jackal moved while the selected OLD was active. It
does not establish a causal controller, wheel, tire-contact, or LightNav explanation.
