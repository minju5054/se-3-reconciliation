# Stage 0-C — LightNav Single-Chunk Integration

## Purpose and scope

Stage 0-C asks whether one real LightNav-0 prediction can move through the complete simulator
interface without losing its meaning:

```text
Isaac Jackal RGB history
  -> external LightNav checkpoint
  -> immutable decoded action chunk
  -> canonical local SE(2)
  -> observation-pose world SE(2)
  -> Stage 0-B follower and Jackal
```

This is simulator integration validation, not research evidence and not EXP-01. It does not
evaluate navigation quality, generate successive OLD/NEW chunks, run asynchronous inference,
choose a latency-stale prefix, or perform reconciliation, correspondence, GTSAM, or graph
optimization.

## Environment separation

The three Python environments remain separate:

- Isaac Sim 6.0.1 scripts run with `~/isaacsim/python.sh`.
- Pure derivation, validation, and tests run with this repository's Python 3.12 `.venv`.
- Inference runs with `~/Workspace/external/LightNav-0/.venv/bin/python` (Python 3.11).

The launchers remove inherited ROS/CUDA/Python library variables only in the Isaac child
process. They do not modify the shell, system Python, ROS, CUDA, the NVIDIA driver, or Isaac
Sim. LightNav is not installed in the research or Isaac environment, and Isaac APIs are not
installed in the LightNav environment.

## Verified LightNav and checkpoint

The validation run used:

- local checkout: `a645828d81a8439651172197ca80a75dc1377977`, package `lightnav 0.1.0`;
- upstream `origin/main` at inspection: `0e9971784a04da2210bfccc446a68d45256e2894`;
- source difference: only `docs/assets/wechat_group.png`; no inference-code difference;
- checkpoint: `LightOriginsHQ/LightNav-0`, revision
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`;
- backend: official `vllm_local`, vLLM 0.19.1, torch 2.10.0+cu128;
- GPU: NVIDIA GeForce RTX 5060 Ti, 16 GB, compute capability 12.0.

The checkpoint's `eval_config.json` selects 64 input frames at 4 Hz, output horizon 10,
input size 256 x 448, and its RVQ bundle. The bundle manifest declares `se2_diff` as the
*internal* representation.

## Source-confirmed action semantics

LightNav public output is `(H, 3)` float32 in this order:

```text
[forward_m, lateral_m, yaw_rad]
```

Positive lateral is robot-left and positive yaw is counter-clockwise about +Z. Every public
row is a cumulative future pose relative to the robot at the current/observation frame; row 0
is the first future pose and the identity pose is not included.

This conclusion is cross-checked in the local checkout rather than inferred from a variable
name:

1. `docs/PROTOCOL.md` defines cumulative current-frame robot-local rows, +left, and +CCW.
2. `docs/VISUALIZATION.md` calls row `k` the local ground-plane pose `k` steps ahead and
   prepends the robot origin for rendering.
3. `src/lightnav/traj_vocab.py::compose_to_abs` defines internal `se2_diff` values as
   previous-step-frame transforms and SE(2)-composes them into chunk-start-frame absolute
   poses; `RVQTrajectoryTokenizer.decode_waypoints` performs this conversion.
4. `src/lightnav/tracking.py::decode_waypoints` returns that decoder result through the public
   tracking API.
5. The deployment MPC transforms returned waypoints using the observation/capture robot pose.

Consequently, the released checkpoint's raw-to-local conversion is an axis-preserving copy.
Composing it again would be wrong. A separately tested incremental-SE(2) helper exists for an
explicit future decoder contract, and it composes transforms rather than element-wise
`cumsum`, but Stage 0-C does not invoke it.

## Time convention

The LightNav rows have no intrinsic timestamps or waypoint period. The checkpoint's
`video_fps = 4` describes the RGB input history only. A `waypoint_dt`, simulated `ready_time`,
or `observation_time + latency` is therefore not fabricated in raw or derived metadata.

Stage 0-C records each RGB simulation timestamp, the final input frame's observation time,
and host monotonic inference start/end plus latency. Playback is geometric and uses the
existing feedback follower, so it does not require a model waypoint time base. EXP-01 must
later define observation time, inference-ready time, control period, OLD continuation, and
NEW usability explicitly in an online loop.

## RGB observation

The config creates flat ground and a simple two-wall corridor with a distant end marker. The
official Jackal asset exposes stereo camera prims, which are recorded for inspection. Stage
0-C uses an experiment camera attached to the runtime articulation root so its pose and input
contract are explicit:

- prim: `/World/JackalReference/Stage0CEgocentricCamera`;
- parent: `/World/JackalReference`;
- relative translation: `[0.30, 0.0, 0.48]` m;
- relative XYZ rotation: `[90, 0, -90]` degrees;
- USD optical axes: `-Z` forward, `+Y` up, mapped to robot `+X` and world `+Z`;
- resolution: 448 x 256, horizontal FOV 90 degrees;
- delivered array: `256 x 448 x 3`, `uint8`, RGB.

The Jackal remains stationary while 64 frames are sampled at 4 Hz. The anchor
`robot_pose_at_observation` is measured at the final inference input frame, not at inference
completion.

## Raw and derived artifacts

Each immutable, ignored run is stored below
`data/stage0/lightnav_single_chunk/<run_id>/`:

```text
raw/
  rgb/frame_*.png
  frame_samples.csv
  observation_metadata.json
  lightnav_actions.npy
  lightnav_raw_text.txt
  lightnav_inference.json
derived/
  lightnav_local_path.npy
  lightnav_world_path.npy
  execution_reference.npy
  jackal_actual_trajectory.npy
  execution_samples.csv
  derivation.json
results/
  validation.json
  execution_metrics.json
  execution_metadata.json
metadata.json
```

`raw/lightnav_actions.npy` is saved directly from the public API without changing its dtype,
values, order, or frame. Raw files are never overwritten. Derived arrays are separate and
exclusive-create writes fail if a destination already exists.

For the confirmed public semantics:

```text
lightnav_local_path = float64 copy(raw decoded absolute local poses)
T_world_waypoint = T_world_robot_at_observation * T_robot_waypoint
```

There is no axis swap, interpolation, clipping, smoothing, or waypoint correction. The
execution reference explicitly prepends the observation anchor because the model output
contains future poses only; this extra array is not presented as model output.

## Execution and visualization

The validator checks shape, dtype, finite values, configured horizon, artifact separation,
exact raw-to-derived reproduction, observation-pose world transformation, frame counts,
absence of fabricated timing, and configured magnitude/spacing safety limits. Only a safe,
unambiguous path is executable.

Playback reuses Stage 0-B's `TrajectoryFollower` and Isaac Sim 6.0.1 experimental
`DifferentialController`. It does not introduce a new controller. DebugDraw renders the
derived reference, all headings, start/end markers, and the live actual path above the ground.
`--visualize-only` also reloads an existing saved actual path so reference and actual can be
inspected together. DebugDraw has no physics effect.

## Reproduction

Run each GPU-heavy stage sequentially. Capture prints the immutable run directory to use in
the following commands:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_lightnav_single_chunk_capture.sh

./scripts/lightnav/run_lightnav_single_chunk_inference.sh \
  data/stage0/lightnav_single_chunk/<run_id>

.venv/bin/python scripts/validate_lightnav_single_chunk.py \
  data/stage0/lightnav_single_chunk/<run_id>

./scripts/isaac/run_lightnav_single_chunk_playback.sh \
  data/stage0/lightnav_single_chunk/<run_id>

.venv/bin/python scripts/validate_lightnav_single_chunk.py \
  data/stage0/lightnav_single_chunk/<run_id> --require-execution
```

Capture exits after saving by default to free GPU memory. Playback keeps the GUI open by
default; close the window to exit. Use `--no-hold` only for automated validation. Use
`--visualize-only` to inspect a safe path without commanding the robot.

## Observed validation run

Run `20260903T010425Z` used the instruction “Go straight down the corridor.” It produced:

- 64 actual Isaac RGB frames at 4 Hz;
- observation time `17.0333342217 s` and pose
  `[-0.0010703253, -0.0000460156, -0.0006066629]`;
- inference latency `49561.998 ms` (LightNav reported `49267.944 ms`), excluding model load;
- raw `float32 (10, 3)` with first row
  `[0.15485105, 0.00006702, -0.00419930]` and last row
  `[0.71874577, -0.00634444, -0.00297201]`;
- world path first/last
  `[0.15378074, -0.00007294, -0.00480597]` and
  `[0.71767146, -0.00682649, -0.00357868]`;
- actual path `(24, 3)`, goal reached in `2.3000 s`;
- position RMSE `0.045857 m`, final position error `0.078326 m`;
- yaw RMSE `0.007308 rad`, final yaw error `0.000107 rad`.

The output validator passed. GUI-mode capture and playback both ran, and the final GUI was
visually inspected with the Jackal, reference path/headings, and saved actual path visible
together. These metrics validate controller consumption of the chunk only; they are not a
claim about LightNav navigation performance.

## Limitations and next step

- The scene is deliberately simple, stationary-history input is repetitive, and only one
  instruction/chunk was tested.
- The experimental 90-degree camera differs from some evaluation camera configurations;
  navigation quality and camera-domain matching were not evaluated.
- The 49.6-second eager vLLM inference is an offline observation on this software/GPU stack,
  not an online latency benchmark or accepted execution policy.
- No intrinsic waypoint time base exists, so Stage 0-C cannot decide which row would be stale
  at inference completion.
- Isaac reports non-fatal legacy Jackal wheel-collision warnings.

The interface, frames, raw preservation, transform, visualization, and single-chunk execution
are validated. The next stage may build the EXP-01 online OLD/NEW loop, but must separately
measure simulation-time observation/ready events and define the control-period usability
policy before interpreting latency-induced switching.
