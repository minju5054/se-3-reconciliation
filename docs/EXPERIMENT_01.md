# EXP-01: Latency-Induced Discontinuity in Successive LightNav Trajectory Chunks

## Research question and hypothesis

**Question:** Does asynchronous LightNav inference produce measurable trajectory
discontinuity when an executing OLD chunk is naively replaced by a newly generated NEW
chunk?

**Hypothesis:** Because the robot continues executing OLD while NEW inference is pending,
and because NEW is anchored to the robot pose at its earlier observation time, a direct
switch to NEW's first temporally usable waypoint will exhibit non-zero pose mismatch and/or
motion mismatch at the boundary in real LightNav runs.

This experiment establishes whether the transition problem exists. It does not propose or
evaluate reconciliation.

## Inputs

For each transition, record an OLD and NEW `TrajectoryChunk`. Each chunk contains:

- `poses_local`: finite `N x 3` floating-point rows `[x, y, yaw]`; `N` is arbitrary.
- `observation_time`: timestamp when the model input was observed, seconds.
- `ready_time`: timestamp when that prediction became available, seconds.
- `inference_latency`: `ready_time - observation_time`, seconds.
- `waypoint_dt`: configured execution interval, seconds; this is external to LightNav.
- `robot_pose_at_observation`: world pose `[x, y, yaw]` at observation time.
- `frame`: explicit coordinate-frame label.
- `source`: model/checkpoint/run provenance string.

Raw pose arrays and metadata remain immutable under `data/exp01/raw/`. Validated, combined
chunk JSON is derived data under `data/exp01/derived/`. Both directories are ignored because
recordings may be large; preservation and backup policy is external to Git.

## Outputs

For every OLD/NEW pair, the analysis emits:

- switch time, OLD boundary index, NEW usable-suffix index, and stale-prefix length;
- whether OLD's discrete horizon was already exhausted;
- OLD boundary and first NEW usable world poses;
- all boundary metrics listed below;
- an optional world-frame diagnostic plot.

Generated per-run outputs go under `results/exp01/` and are ignored by default. A later,
explicit analysis task may create a reviewed compact aggregate.

## Timeline and raw-switch baseline

```text
NEW observation                 NEW ready / switch
      |<--- inference latency ------->|
      |                               |
OLD:  continues discrete execution --X
NEW:  predicted from observation pose | discard stale prefix; use first future row
```

At NEW ready time:

1. Treat OLD as having continued along its discrete waypoint schedule.
2. Select the last OLD waypoint scheduled at or before the switch time. If none has been
   reached, use OLD's observation-time robot pose.
3. Discard NEW waypoints scheduled strictly before NEW ready time.
4. Switch directly from the OLD boundary pose to the first usable NEW waypoint.

There is no interpolation, smoothing, GTSAM, graph optimization, or correspondence factor.
If fewer than two NEW waypoints remain, pose mismatch could be observed but the required
first-NEW-segment motion metrics cannot be computed, so the analysis rejects that pair.

## Coordinate-frame convention

Poses use `[x, y, yaw]`, metres and radians, with positive yaw counter-clockwise. Each local
trajectory is transformed independently:

```text
T_world_waypoint = T_world_robot_at_observation * T_robot_waypoint
```

In particular, NEW is transformed using `NEW.robot_pose_at_observation`. The robot pose at
NEW ready time must not be substituted: that would silently re-anchor the prediction and
erase part of the phenomenon under study. Angles are wrapped to `[-pi, pi)`.

LightNav's documented local convention is forward metres, lateral metres (positive left),
and yaw radians (positive counter-clockwise). Collection adapters must record any conversion
into the `[x, y, yaw]` convention explicitly; the analysis never silently swaps axes.

## Temporal convention

LightNav waypoints do not carry an intrinsic time base. For configured `waypoint_dt = dt`,
row `i` is treated as an endpoint reached after `i + 1` intervals:

```text
t_i = observation_time + (i + 1) * dt
```

The first usable NEW index is the smallest `i` for which `t_i >= ready_time`. Equality is
usable. The stale-prefix length is therefore the number of rows with `t_i < ready_time`.
This gives index 0 for zero latency and for latency smaller than one interval; it advances
across multiple intervals without assuming a fixed horizon.

The OLD boundary is the last row whose scheduled time is `<= NEW.ready_time`. No temporal
interpolation is performed in EXP-01. If NEW ready time exceeds OLD's horizon, OLD is treated
as holding its last waypoint and the output flags `old_horizon_exhausted`.

## Metrics

Pose mismatch and motion mismatch are reported separately.

Let `O-` be the pose immediately preceding OLD's boundary sample, `O` the OLD boundary,
`N0` the first usable NEW waypoint, and `N1` the next NEW waypoint.

- **translation gap [m]:** `||xy(N0) - xy(O)||`.
- **yaw gap [rad]:** absolute wrapped yaw difference between `N0` and `O`.
- **previous OLD segment translational motion [m]:** `||xy(O) - xy(O-)||`.
- **first NEW segment translational motion [m]:** `||xy(N1) - xy(N0)||`.
- **translational motion jump [m]:** absolute difference of those segment lengths.
- **previous OLD yaw increment [rad]:** wrapped `yaw(O) - yaw(O-)`.
- **first NEW yaw increment [rad]:** wrapped `yaw(N1) - yaw(N0)`.
- **yaw-motion jump [rad]:** absolute wrapped difference of the two yaw increments.

For an OLD boundary at its first row, `O-` is the OLD observation-time robot pose. If no
OLD row has yet been reached, both `O-` and `O` are that pose and previous OLD motion is zero.

## Success and failure criteria

The committed configuration provides operational reporting thresholds of 0.05 m and
0.05 rad for each corresponding gap/jump. Before real collection, confirm that these exceed
the platform's pose/timestamp noise floor and record any justified change in a new config.

- **Supports the hypothesis:** at least one real LightNav-generated transition exceeds a
  predeclared threshold in a pose-gap or motion-jump metric, and repeated trials make the
  distribution and latency relationship reportable.
- **Does not support the hypothesis under tested conditions:** all valid real transitions
  remain at or below the confirmed measurement noise/thresholds.
- **Inconclusive:** only synthetic inputs are analyzed, timing/frame metadata is missing,
  there are too few usable NEW waypoints, or collection/clock quality is insufficient.

Synthetic fixtures can validate code paths but can never satisfy the success criterion.

## Known limitations

- The discrete endpoint model does not represent within-interval controller motion.
- The baseline assumes recorded clocks share a consistent time base.
- Pose-estimation error and model discontinuity are not disentangled in EXP-01.
- Segment-length and yaw-increment jumps are kinematic proxies, not acceleration or jerk.
- No correspondences are available yet, so geometric overlap is not assessed.
- No reconciliation, intent preservation, or graph optimization is implemented.
- Actual research evidence still requires manual LightNav/Isaac Sim collection with a
  checkpoint; no model weights or simulator recordings are committed.
