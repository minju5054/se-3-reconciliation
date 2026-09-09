# DATA-02 Diverse Real Successive LightNav OLD/FRESH Transitions

## Status and research role

DATA-02 is data collection only. It replaces DATA-01 as the primary coverage-oriented input
for future reconciliation-formulation research. DATA-01 is
`RETIRED_FROM_PRIMARY_FORMULATION_USE` because its EXP-01B cohort characterized latency and
G0/G1/G2 conditions and produced insufficient trajectory diversity. Historical EXP-01B,
EXP-02B, EXP-02C, and DATA-01 Git history remain unchanged.

No optimizer, graph, factor, selector, correspondence, smoothing, blending, obstacle cost,
controller tuning, physics tuning, or LightNav modification is present in DATA-02. No
reconciliation outcome enters qualification, inclusion, stopping, classification,
representative selection, or splitting.

## Successive-pair and frame semantics

One DATA-02 context comes from one online episode and navigation instruction:

1. Reset Jackal pose, wheels, follower, and LightNav episode; settle.
2. Capture/send the checkpoint-required 64-frame stationary history at 4 Hz.
3. Atomically capture the OLD observation pose and request raw OLD.
4. Derive `old_world = T_world_robot(old_observation) * old_lightnav`.
5. Execute OLD through the frozen Stage 0-D `pi_strong` calibrated execution controller and
   unchanged Stage 0-B `TrajectoryFollower`.
6. At the fixed 0.50 s simulation trigger, send the current RGB observation and submit FRESH
   asynchronously while OLD continues.
7. B is the actual pose when the FRESH response becomes usable; P is the last actual
   control-update pose strictly before B. Natural semantics set `t_usable=t_model_ready` and
   add no delay.
8. Derive `fresh_world = T_world_robot(fresh_observation) * fresh_lightnav`. It is never
   re-anchored at B.
9. Probe the last OLD desired `[v,omega]` and first raw FRESH desired command at B, then stop.
   FRESH and reconciled candidates are not executed.

Raw arrays are immutable `N x 3 [x_forward_m, y_left_m, yaw_ccw_rad]` poses in their own
observation-time robot frames. Derived world arrays are separate. Waypoint rows have no
intrinsic timestamp, so geometry/yaw metrics are spatial and never described as time-aligned
tracking error.

## Frozen scenarios and variations

The versioned config defines ten explicit static scenes and instructions:

| Family | Intended coverage | Instruction |
| --- | --- | --- |
| D0_STRAIGHT | straight corridor | Go straight down the corridor. |
| D1_GENTLE_LEFT | gentle left | Follow the corridor as it bends gently left. |
| D2_GENTLE_RIGHT | gentle right mirror | Follow the corridor as it bends gently right. |
| D3_STRONG_LEFT | blocked left turn | Turn left at the blue wall and continue along the left corridor. |
| D4_STRONG_RIGHT | blocked right mirror | Turn right at the red wall and continue along the right corridor. |
| D5_OBSTACLE_DETOUR_LEFT | left obstacle detour | Go around the yellow obstacle on the left and continue forward. |
| D6_OBSTACLE_DETOUR_RIGHT | right obstacle mirror | Go around the yellow obstacle on the right and continue forward. |
| D7_BRANCH_OR_DOORWAY_LEFT | left doorway | Enter the doorway on your left. |
| D8_BRANCH_OR_DOORWAY_RIGHT | right doorway mirror | Enter the doorway on your right. |
| D9_S_CURVE_OR_CHICANE | alternating chicane | Navigate through the chicane and continue forward. |

Each scene records exact box centers/sizes/colors, initial pose, the common attached camera,
expected class, and qualification rule. Primary variations are fixed in order: baseline,
`+0.075/-0.075 rad` yaw, and `+0.10/-0.10 m` body-left lateral displacement. Attempts cycle
this order deterministically; there is no random retry for an interesting output.

## Qualification and primary protocol

Qualification retains every attempt and requires three `VALID_MOVING` FRESH outputs per
family. A non-straight family qualifies only when its predeclared geometry rule holds for at
least two of the first three valid outputs; straight requires all three. Rules use only pose
yaw progression, endpoint lateral departure, XY-tangent turn, and curvature sign changes.
Thresholds are in the config and must be frozen before primary.

Primary targets five valid moving contexts per family with a hard cap of ten attempts. Every
attempt is retained as `VALID_MOVING`, `MODEL_STOP_OUTPUT`, `OLD_EXHAUSTED`, `TIMING_INVALID`,
`INFERENCE_ERROR`, `EXECUTION_INVALID`, or `OTHER_FAILURE`. STOP is not treated as a benign
zero-curvature transition. OLD nearest-polyline deviation, calibrated command versus measured
body motion, target versus measured four-wheel motion, saturation, follower progress, and the
full actual history are recorded. These diagnostics can invalidate a source episode but do
not tune the platform.

Dataset readiness requires at least 45 valid contexts, all ten families, left/right
non-straight representation, and at least 30 unique ordered raw pairs. A failed requirement
returns `DATA02_DIVERSITY_TARGET_NOT_MET`; scenes are not changed after primary outputs are
seen.

## Attempt, bank, telemetry, and plots

Every completed pair attempt stores:

```text
raw/{old_lightnav.npy,fresh_lightnav.npy,old_raw_text.txt,fresh_raw_text.txt}
raw/rgb/frame_*.png
raw/history_manifest.json
derived/{old_world.npy,fresh_world.npy}
actual/old_execution.npy
telemetry/old_execution.csv
{attempt,context,descriptors,provenance}.json
```

Telemetry distinguishes follower desired body motion, calibrated executed body command,
measured body motion, four canonical wheel targets, four measured wheel velocities, actual
pose, follower indices, inference-in-flight state, PI state, and saturation. `context.json`
records OLD/FRESH observations, P/B, all simulation/host timing, frames, axes, and units.
Provenance includes repository/LightNav commits, checkpoint/server environment, Stage 0-D
model hash and parameters, config hashes, wheel discovery, GPU, and all trajectory hashes.

For every valid bank context, `world_xy.png` shows blue OLD, orange FRESH, black actual motion
from the FRESH observation to B, red-star B, black-circle P, and separate OLD/FRESH observation
markers. `boundary_canonical_xy.png` applies the same visualization-only `T_B^-1` transform to
all objects. `yaw_vs_path_length.png` uses cumulative spatial length, explicitly not time.
Overview sheets show all contexts, a 10-family-by-context grid, one deterministic minimum-ID
representative per family, descriptive geometry groups, and raw command distributions.

The deterministic 70/30 split assigns complete `(old_raw_sha256,fresh_raw_sha256)` groups and
uses scenario/geometry metadata for coverage only. Identical raw pairs cannot cross. Held-out
must remain untouched by future formulation/weight work until those choices are frozen.

## Commands

```bash
./scripts/isaac/run_data02_transition_collection.sh qualification \
  --run-id <qualification_run_id>
./scripts/isaac/run_data02_transition_collection.sh smoke \
  --run-id <gui_smoke_run_id> --scenario all --gui --no-hold
./scripts/isaac/run_data02_transition_collection.sh primary \
  --run-id <primary_run_id> \
  --qualification-run data/data02_lightnav_diverse_transitions/<qualification_run_id>
.venv/bin/python scripts/validate_data02_transitions.py \
  data/data02_lightnav_diverse_transitions/<primary_run_id>
```

Omit `--no-hold` for an interactive GUI that remains open. Qualification and primary output
directories are exclusive and never overwritten.

## Observed result and claim limits

The observed qualification/primary statistics will be appended after actual execution.
Until then the implementation status is `DATA02_COLLECTION_NOT_YET_EXECUTED`.

DATA-02 can support claims about genuine successive collection, predefined scenario coverage,
natural latency, preserved observation/B context, measured output geometry, and a deterministic
split. It cannot estimate natural transition prevalence or establish reconciliation
improvement, graph superiority, optimal k, obstacle safety, end-to-end selection, real-world
performance, or a causal controller/contact explanation.
