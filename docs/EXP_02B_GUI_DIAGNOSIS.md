# EXP-02B GUI Execution Diagnosis

## Purpose and scope

This diagnostic answers one question: why does the planned OLD LightNav trajectory appear
separated from the actual Jackal trajectory during the EXP-02B OLD replay? It exposes the
reference, controller, wheel-target, measured-motion, and reset layers without changing any
reconciliation method. It does **not** add or tune an optimization residual, graph factor,
gate, selector, `k_old`/`k_fresh` interface, navigation benchmark, or LightNav model.

The pipeline is separate from `scripts/isaac/exp02b_branch_execution.py`. It reads the frozen
EXP-02B candidates and immutable redesigned EXP-01B source, then writes a new ignored,
immutable diagnostic directory. It never writes below the frozen `data/exp02b/` run.

The four observed layers must not be conflated:

1. LightNav emits an arbitrary `N x 3` waypoint reference `[x, y, yaw]`.
2. `TrajectoryFollower` maps the active reference and current pose to body commands
   `[v, omega]`.
3. Isaac's `DifferentialController` maps `[v, omega]` to left/right wheel velocity targets;
   the runtime helper expands these to the four Jackal wheel DOFs.
4. Isaac reports measured pose, finite-difference body motion, and measured wheel-joint
   velocity.

An “action error” is therefore not a single metric. Reference-to-actual geometry,
body-command execution, and wheel-target tracking are reported separately.

## Frozen replay and exact-reset protocol

The diagnostic preserves the quantitative runner's replay setup:

1. restore the source initial pose and zero wheel state, then settle;
2. replay the stored source OLD controller commands;
3. compare the resulting `B_reproduced` with frozen `B_saved` using the unchanged EXP-02B
   comparability gate;
4. hold the simulation during `PRE_RESET_INSPECTION` for visual inspection;
5. reset the articulation pose exactly to `B_saved` and restore the final OLD wheel target;
6. execute the selected frozen candidate with a new `TrajectoryFollower`.

`B_reproduced` and `B_saved` remain separate logical markers and metadata fields even when
their numerical values coincide. The cyan OLD actual history ends before reset. The green
post-switch actual history begins at the exact-reset pose. They are never drawn as one
continuous physical history. Both diagnostic holds pause the simulation timeline; they add
wall-clock viewing time only and cannot advance vehicle dynamics.

The terminal and metadata expose the ordered phases:

```text
SETTLING
OLD_REPLAY
PRE_RESET_INSPECTION
EXACT_BOUNDARY_RESET
POST_SWITCH_EXECUTION
FINISHED
```

## Run

The default command uses the smallest requested falsifiable diagnosis: the frozen maximum
angular-discontinuity case, `k=3`, and the exact raw suffix.

```bash
./scripts/isaac/run_exp02b_gui_diagnosis.sh \
  --case case_high_delta_omega \
  --k 3 \
  --method raw_k
```

The default diagnostic playback is 0.5x real time, with a 2 s wall-clock pre-reset hold and a
1 s exact-reset hold. Use `--pre-reset-hold-s <seconds>` or `--real-time-factor <factor>` for
viewing convenience. Use `--no-hold` to close after saving or `--headless` for automated
telemetry checks. `rigid` and `graph` may be selected only from the already frozen EXP-02B
candidates; the diagnostic does not create or modify method outputs.

The launcher's environment cleanup is local to its Isaac child process. It does not modify
the system Python, ROS 2, CUDA, Isaac Sim, or LightNav installations.

## Viewport legend and markers

All trajectory and marker geometry uses the existing Isaac DebugDraw approach shared with
Stage 0. The diagnostic hides only the Jackal LiDAR sensor's viewport clutter; this does not
change articulation, contact, controller, or command settings. Exact RGBA values and marker
poses are saved in `metadata.json`.

| Geometry | Color | Meaning |
|---|---|---|
| planned OLD | blue | frozen OLD world-frame waypoint polyline |
| OLD replay actual | cyan | measured Jackal pose history before reset |
| raw full FRESH | gray | complete frozen FRESH world trajectory |
| selected `FRESH[k:]` | orange | suffix chosen by k |
| current candidate | magenta | frozen `raw_k`, `rigid`, or `graph` candidate |
| post-switch actual | green | measured history after exact reset |

Markers are: white `FRESH_observation`, red `B_saved`, cyan `B_reproduced`, yellow
`B_exact_reset`, orange `F_k`, and magenta `X_k`. They use different draw heights and sizes so
coincident logical states are still represented independently. Marker names and exact poses
are also printed in the terminal because DebugDraw text labels are not relied upon. For
`raw_k`, the orange suffix and magenta candidate, and `F_k` and `X_k`, coincide by definition.

During OLD replay, `EXP02B_DIAG_TELEMETRY` records phase, simulation time, actual SE(2),
commanded `v/omega`, measured body `v/omega`, four wheel targets, and four measured wheel
velocities. During post-switch execution it records commanded `v/omega`, `nearest_index`,
`target_index`, and `goal_reached`. `EXP02B_DIAG_PHASE` makes every phase transition explicit.

## Metrics and timing semantics

LightNav waypoint rows have no intrinsic timestamp. The diagnostic therefore does **not**
align waypoint row i with execution sample i and does not call its spatial metric a
time-aligned tracking error. Each actual OLD pose is projected onto the nearest OLD polyline
segment. The summary reports mean, RMS, maximum, and final nearest spatial distance. Optional
yaw values are wrapped differences from yaw interpolated on that same nearest segment.

Measured body velocity is the finite-difference body-forward displacement and wrapped yaw
change over each physics interval. It is compared with the command applied during the interval
ending at the row. Wheel RMSE compares each canonical target with direct articulation joint
velocity in this order: `front_left`, `front_right`, `rear_left`, `rear_right`. CSV row 0 is
the pre-interval state. In the post-switch CSV, row 0 is the exact-reset boundary with the
restored final OLD wheel target; rows 1 onward contain the selected FRESH command applied over
the ending interval.

Visibility labels use the documented thresholds in `configs/exp02b_gui_diagnosis.yaml`.
Multiple labels may apply. They are descriptive flags, not acceptance gates and not causal
classifiers.

## Immutable output

Each run creates, without overwrite, `data/exp02b_gui_diagnosis/<run_id>/`:

- `metadata.json`: source/config/candidate SHA-256 provenance; source observation, model-ready,
  and FRESH-usable execution timestamps; diagnostic host/phase timestamps; coordinate frame,
  axis/yaw conventions, physics/control timing, wheel order, runtime DOF names, paths, marker
  poses, and legend;
- `diagnosis_summary.json`: case, k, method, saved/reproduced/reset poses, pre-reset error,
  OLD spatial metrics, body and wheel execution metrics, post-switch metrics, and non-causal
  interpretation labels;
- `old_replay_actual.npy` and `post_switch_actual.npy`: world-frame `[x_m,y_m,yaw_rad]` samples;
- `old_replay_telemetry.csv`: sample/time/source-command index, actual pose, body command,
  finite-difference measured body motion, four wheel targets, and four measured velocities;
- `post_switch_telemetry.csv`: the same fields plus control-command index, follower
  `nearest_index`, `target_index`, and `goal_reached`.

Raw inputs and the frozen EXP-02B quantitative output remain untouched. The full diagnostic
root is ignored by Git through the repository's existing `data/` rule.

## Representative GUI smoke observation

The 2026-09-07 GUI smoke used frozen
`case_high_delta_omega / k=3 / raw_k`, sourced from
`primary/G1_turn/L1_added_050/attempt_004`. The Jackal visibly moved through OLD replay and
post-switch execution; all six trajectory layers and all logical markers were visible; live
terminal telemetry updated; the simulation time stayed at `2.500000130 s` through both holds;
and strict output validation found 89 OLD and 121 post-switch pose samples.

Repository-confirmed facts:

- the existing EXP-02B smoke selection already contains exactly this case/k for `raw_k`,
  `rigid`, and `graph`;
- the current EXP-02B graph still uses the unchanged EXP-02A geometric objective;
- the frozen runner replays OLD commands, checks comparability, then resets to saved B before
  executing each candidate;
- the source contains controller command progress but no OLD wheel target/measured telemetry,
  so wheel diagnosis requires this instrumented replay.

Observed in this diagnostic run:

| Level | Result |
|---|---:|
| OLD nearest-polyline distance mean / RMS / max / final | 0.05290 / 0.06460 / 0.15243 / 0.04661 m |
| OLD nearest-segment yaw absolute RMS / max / final | 0.10404 / 0.13928 / 0.01451 rad |
| body `v` command-versus-measured RMSE | 0.12133 m/s |
| body `omega` command-versus-measured RMSE | 0.63989 rad/s |
| mean commanded / measured `v` | 0.22191 / 0.22092 m/s |
| mean commanded / measured `omega` | 0.75534 / 0.13920 rad/s |
| aggregate wheel target-versus-measured RMSE | 0.84874 rad/s |
| wheel RMSE FL / FR / RL / RR | 1.04616 / 0.83351 / 0.75309 / 0.72462 rad/s |
| `B_saved` versus `B_reproduced` | 0.000000 m / 0.000000 rad |
| reset pose numerical difference from reproduced B | 0.000000 m / 1.51e-8 rad |
| raw-k immediate post-switch `delta_v` / `delta_omega` | 0.54023 m/s / 1.42799 rad/s |

The configured visibility flags were
`CONTROLLER_REFERENCE_DEVIATION_VISIBLE`, `BODY_EXECUTION_MISMATCH_VISIBLE`, and
`WHEEL_TRACKING_MISMATCH_VISIBLE`. `REPLAY_RESET_ARTIFACT_RELEVANT` was not emitted because
the replay reproduced B exactly and the reset jump was numerical. Thus the displayed OLD gap
is real in this replay, is especially associated with commanded-versus-measured angular-motion
discrepancy, and also coexists with measurable wheel tracking error. For this representative
run, it is not explained by mixing pre-reset and post-reset histories or by a failed replay
boundary reproduction.

## Interpretation limits

These observations separate where discrepancy is visible, but they do not identify a unique
physical cause. A large angular command/body mismatch is compatible with skid-steer/contact
effects, controller limitations, wheel tracking error, or their interaction. Because wheel
tracking RMSE is itself non-negligible here, this run cannot isolate tire/contact behavior from
actuation/joint tracking. Nearest-polyline deviation also does not establish whether the
LightNav reference is semantically good, safe, or dynamically feasible. No causal claim such
as “skid steer caused the OLD gap” is supported without controlled follow-up interventions.

The result is one outcome-selected development/stress case in simulation. It is neither a
new optimization result nor evidence of benchmark, real-robot, obstacle-avoidance, selector,
or generalization performance.
