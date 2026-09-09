# Stage 0-G3 — Jackal moving-egocentric-history qualification

## Purpose and scientific boundary

Stage 0-G and Stage 0-G2 used 64 copies of a stationary final view. Both failed to elicit
reproducible non-straight LightNav geometry. Stage 0-G3 therefore asks one narrower question:

> Under the same Jackal, Hospital scene, instruction, final observation pose, camera, model,
> and decoder settings, does replacing stationary past views with a deterministic moving
> egocentric history change the decoded trajectory enough to qualify?

This is an upstream, inference-only qualification. It does not test physical path following,
navigation success, obstacle avoidance, a controller, skid-steer dynamics, OLD/FRESH collection,
or reconciliation. No LightNav, controller, physics, graph, residual, factor, selector, gate,
smoothing, or optimization code was changed.

Primary run:

```text
data/stage0/lightnav_moving_history_qualification/20260909T_stage0g3_primary_r6/
```

Final decision:

```text
STAGE0G3_MOVING_HISTORY_QUALIFICATION_FAILED
```

The result means that deterministic moving visual history changed some outputs but was not
sufficient to meet the frozen six-scenario qualification. It does not authorize successive
OLD/FRESH collection.

## Paired design and frozen variables

The read-only stationary control is Stage 0-G2 primary r3. Its 30 cases, instructions, raw
hashes, and final observation poses were strictly revalidated. The frozen control config hash is:

```text
49d339d16694d96dc5113271a2236e79ce413ae2411915e340885f8310fc9f01
```

The following remained fixed:

- official Clearpath Jackal `/Isaac/Robots/Clearpath/Jackal/jackal.usd`;
- Isaac Hospital 6.0.1 `/Isaac/Environments/Hospital/hospital.usd`;
- LightNav checkout `a645828d81a8439651172197ca80a75dc1377977`;
- checkpoint revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`;
- `vln` → `vlnce` / `vlnce_traj`, `unified_traj` task and prompt;
- 480 x 270 HWC uint8 RGB, 112-degree HFOV, level robot-forward camera at
  `[0.30, 0.0, 0.48]` m;
- stretch to 448 x 256 bilinear preprocessing;
- 64 frames at 4 Hz, the six G2 scenarios, five G2 variants, and all G2 geometry thresholds;
- float32 `10 x 3 [forward_m, lateral_m_left_positive, yaw_rad_ccw_positive]` output.

Only the prior visual history changed. For G2, all 64 robot poses equal the saved final pose. For
G3, the official Jackal is rendered at 64 monotonically advancing scripted poses and pose 63
equals the corresponding G2 final pose. The final scene, final robot/camera pose, and instruction
are consequently paired.

Scripted pose replay isolates temporal viewpoint history from tracking and skid-steer effects.
The Jackal is present in every input frame, but the pose is set directly; no wheel command,
`TrajectoryFollower`, `DifferentialController`, or physical tracking is involved.

## Approach and timing contract

For final pose `R_f = [x_f, y_f, yaw_f]`, the history is

```text
R(s) = [x_f - (1-s) D cos(yaw_f),
        y_f - (1-s) D sin(yaw_f),
        yaw_f]
```

with exactly 64 linearly spaced samples including `s=0` and `s=1`. Yaw is constant. The largest
common distance was selected by oriented Jackal-footprint overlap queries against authored
Hospital collision shapes before capture or inference. Distances 1.5, 1.4, and 1.3 m failed for
all five Q2 variants; 1.2 m was collision-free for all 30 histories and was frozen. The collision
query is a feasibility diagnostic, not planning or controller execution.

The frame history has 64 timestamps at 4 Hz and therefore 63 intervals spanning 15.75 s. LightNav
waypoint rows have no intrinsic timestamps and remain untimed spatial poses. A predicted local
trajectory is transformed to world coordinates once, using only the actual pose at the final
input frame; history poses are not accumulated over output rows.

Across the 30 captured histories:

- actual history translation: min 1.1999981, mean 1.2000002, max 1.2000015 m;
- mean inter-frame translation: min 0.01904759, mean 0.01904762, max 0.01904764 m;
- maximum final-pose translation mismatch: `4.76837158203125e-7` m;
- maximum wrapped-yaw mismatch: `3.4547587013378234e-7` rad.

Both final-pose maxima satisfy the frozen `1e-6` gates. Isaac base-pose reads are float32, which
accounts for the sub-micrometre-scale reported residuals.

## Runtime order and commands

The executed order was control validation, pose-only proposal, collision feasibility, config
freeze, RGB capture, six-V0 visual inspection, Q0/Q1/Q2 V0 smoke inference, the other 27 cases
once, paired summary, strict validation, and GUI replay. No result-dependent retry or scientific
retuning occurred.

```bash
# Create the run and proposed 1.5 m pose histories; no inference
.venv/bin/python scripts/prepare_stage0g3_moving_history_qualification.py \
  --run-id RUN_ID

# Search the largest common collision-free distance
./scripts/isaac/run_stage0g3_moving_history_qualification.sh RUN_DIR \
  --mode feasibility --headless

# Freeze only after inspecting the approach plot
.venv/bin/python scripts/prepare_stage0g3_moving_history_qualification.py \
  --freeze RUN_DIR --visual-inspection-passed

# Capture all 30 moving RGB histories
./scripts/isaac/run_stage0g3_moving_history_qualification.sh RUN_DIR \
  --mode capture --all --headless

# Build and then explicitly record the pre-inference six-V0 inspection
.venv/bin/python scripts/preview_stage0g3_moving_histories.py RUN_DIR
.venv/bin/python scripts/preview_stage0g3_moving_histories.py RUN_DIR \
  --record-inspection-passed

# Exactly one inference for each frozen input
./scripts/lightnav/run_stage0g3_moving_history_inference.sh RUN_DIR --smoke
./scripts/lightnav/run_stage0g3_moving_history_inference.sh RUN_DIR --remaining

.venv/bin/python scripts/summarize_stage0g3_moving_history_qualification.py \
  RUN_DIR --visual-inspection-passed
.venv/bin/python scripts/validate_stage0g3_moving_history_qualification.py RUN_DIR
```

Technical pre-runs r1 through r5 produced no LightNav inference. They exposed, in order, simulator
timestamp precision, float32 start-distance precision, and a stale first-frame Jackal render at
case boundaries. The capture path was corrected only for those technical validity issues. In r6,
the robot is first rendered outside the Hospital and that discarded render is followed by an
explicit post-teleport render before each captured frame. All six r6 V0 contact sheets were
inspected before inference and showed moving scene progression without the stale-render artifact.

## Stored artifacts and provenance

Generated data are Git-ignored. The G2 directory is read-only input and was not regenerated or
overwritten. G3 case creation and raw prediction writes are exclusive. The run records the frozen
G3 config hash `0515f6b8c8bd86d55e39db5d6128acbd5acac9f0fe1bccb6c3a620511ef08cf8`,
the G2 control config and per-case hashes, approach-feasibility hash, source Git SHA, LightNav and
checkpoint revisions, resolved camera/robot/environment contracts, and case-level provenance.

Each `qualification/<Q>/<V>/` case contains:

- `input/history/frame_000000.png` through `frame_000063.png`, plus `latest_rgb.png`;
- `input/history_pose_se2.npy` and `input/requested_history_pose_se2.npy`, both 64 x 3;
- `input/history_pose.csv`, `frame_samples.csv`, and `capture_metadata.json`;
- immutable `raw/lightnav.npy` and `raw/lightnav_raw_text.txt`;
- final-observation-anchored `derived/trajectory_world.npy`;
- model, control, and input provenance; frozen-criteria descriptors and paired metrics;
- a five-frame contact sheet and local stationary-versus-moving plots.

At run level, `approach_feasibility.json`, `summary.json`, `qualification_result.json`,
`stage0g2_vs_stage0g3_summary.json`, paired JSON/CSV, overview plots, and strict `validation.json`
make the result reconstructable.

## Observed 30-case result

| Scenario | G2 stationary match | G3 moving match | G3 unique raw | endpoint lateral range [m] | signed net-yaw range [rad] | Result |
|---|---:|---:|---:|---:|---:|---|
| Q0 straight | 5/5 | 5/5 | 2 | `[0.00000395, 0.00002658]` | `[0.00000842, 0.00006368]` | PASS |
| Q1 left | 0/5 | 3/5 | 4 | `[0.04136, 0.81417]` | `[0.18481, 0.98970]` | FAIL |
| Q2 right | 0/5 | 0/5 | 1 | `[0.00012820, 0.00012820]` | `[0.00066985, 0.00066985]` | FAIL |
| Q3 doorway | 5/5 | 5/5 task-local | 3 | `[-0.006344, 0.000128]` | `[-0.002972, 0.000670]` | global distinctness FAIL |
| Q4 detour left | 0/5 | 0/5 | 2 | `[0.00002658, 0.00012820]` | `[0.00006368, 0.00066985]` | FAIL |
| Q5 detour right | 0/5 | 0/5 | 2 | `[0.00002658, 0.00012820]` | `[0.00006368, 0.00066985]` | FAIL |

Q3 satisfied the same task-local forward criterion, but only 2/5 intended-match raw arrays were
distinct from Q0/Q1/Q2 raw groups; the required distinct-doorway count was 4/5. No prediction was
invalid or classified STOP/nonmoving.

The paired continuous comparison answers the diagnostic question directly:

- exact raw array changed in 16/30 pairs and stayed identical in 14/30;
- rowwise translation RMS-difference distribution was min 0, median 0.000555, mean 0.083643,
  max 0.467266 m;
- wrapped rowwise yaw RMS-difference distribution was min 0, median 0.000238, mean 0.055288,
  max 0.663120 rad;
- frozen geometry class stayed the same in 23/30 and changed in 7/30;
- three stationary failures became moving-history passes; no stationary pass became a failure.

Thus moving history genuinely changed some decoded trajectories, most visibly in three Q1
variants, rather than leaving every output byte-identical. It did not yield the required 4/5 left
reproducibility, any right response, either detour family, or four distinct doorway responses.

Output diversity improved descriptively from G2 to G3: unique exact arrays 5 → 9, dominant exact
group fraction 0.667 → 0.367, and coarse geometry signatures 3 → 4. These pass the diversity
sub-gates but cannot compensate for the scenario failures.

## Image diagnostics and visualization

Image diagnostics demonstrate that the rendered input changed; they are not semantic-quality
metrics. Across 30 cases, mean consecutive-frame RGB MAE had min 0.637, median 1.779, mean 1.602,
and max 2.048 uint8 levels. First-to-final RGB MAE had min 4.978, median 17.402, mean 17.537, and
max 30.279.

Required plots are under `overview/`:

- `history_examples.png`: five time samples for each scenario V0;
- `all_scenarios_stationary_vs_moving.png`: G2/G3 local paths for all V0 cases;
- `all_variants_by_scenario.png`: every paired case;
- `intended_match_comparison.png`: per-scenario G2/G3 counts.

Persistent GUI replay:

```bash
./scripts/isaac/run_stage0g3_moving_history_qualification.sh \
  data/stage0/lightnav_moving_history_qualification/20260909T_stage0g3_primary_r6 \
  --mode gui --scenario G2_Q1_LEFT_TURN --variant V1
```

The official Jackal visibly advances through all 64 poses at 4 Hz and then repeats until the user
closes Isaac Sim. This is labelled `SCRIPTED HISTORY REPLAY`; it is not physical execution. Green
is the moving-history path, magenta the G3 moving-history prediction, cyan the saved G2 stationary
prediction, and yellow the shared final observation. A one-pass screenshot smoke can be run with
`--no-hold`; r6 stores `overview/gui_scripted_replay_G2_Q1_LEFT_TURN_V1.png`.

## Decision and claim limitation

The strict status is `STAGE0G3_MOVING_HISTORY_QUALIFICATION_FAILED`: Q1, Q2, and the required
doorway-or-detour condition failed. Simply replacing a stationary history with this deterministic
moving egocentric history was not sufficient for the frozen qualification.

The data support an association between moving history and changed LightNav responses under the
paired setup. They do not establish that motion caused correct navigation, that the resulting
paths are physically followable, that LightNav navigation succeeds, or that a controller,
obstacle-avoidance method, or reconciliation algorithm works. Because the experiment failed, no
next collection stage is authorized here and the Hospital/setup is not redesigned inside G3.
