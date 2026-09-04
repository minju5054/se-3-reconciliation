# SE(3) Reconciliation Research

This repository studies how to reconcile successive navigation action chunks while
preserving the intent of a newly predicted trajectory. The final target is a local SE(2)
back end that uses OLD/NEW segment correspondences and relative-transform measurements to
distribute corrections over the editable future.

The platform roles are deliberately separate:

- **Isaac Sim 6.0.1** is the simulator.
- **ROS 2 Jazzy** is the robotics middleware and uses the machine's system Python 3.12.
- **LightNav-0** is the upstream navigation-VLA baseline that generates trajectory chunks.
- The reconciliation method developed here is **not LightNav**.

The current research-experiment scope remains **EXP-01 only**: characterize the discontinuity
produced by a naive raw OLD→NEW switch under asynchronous LightNav inference. There is no
interpolation, smoothing, correspondence factor, GTSAM dependency, or graph optimization in
this stage.

Before connecting LightNav, **Stage 0** validates the standalone Isaac Sim trajectory
pipeline with the official Clearpath Jackal asset. It generates a deterministic SE(2)
reference from configured unicycle commands, executes those commands through all four
runtime-discovered wheel joints, records the actual world SE(2) trajectory, and displays
both paths with non-physical DebugDraw geometry. Stage 0 is simulation pipeline validation,
not research evidence.

## Local layout and environments

Keep the repositories separate:

```text
~/Workspace/
├── se-3-reconciliation/       # this repository, Python 3.12 .venv
└── external/
    └── LightNav-0/            # upstream checkout, Python 3.11 .venv
```

The environments do not replace `/usr/bin/python3` and do not modify ROS 2, the NVIDIA
driver, CUDA, or Isaac Sim. Reproduce the research environment with:

```bash
cd ~/Workspace/se-3-reconciliation
uv venv --python /usr/bin/python3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

LightNav must be installed in its own checkout according to its upstream instructions; do
not install it here. The setup used for EXP-01 is recorded in `docs/WORK_LOG.md`.

## Stage 0 Jackal GUI smoke test

Run the default GUI workflow with one command:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_jackal_trajectory_demo.sh
```

The launcher uses `~/isaacsim/python.sh`, not the research `.venv`. It scrubs inherited
ROS/CUDA library environment variables only for that child process, then lets the Isaac
launcher establish its own runtime paths. The GUI stays open after recording so the ground,
Jackal, reference path, and accumulated actual path can be inspected together; close the
window to exit. Outputs are written to an immutable ignored directory below
`data/stage0/jackal_trajectory/<run_id>/`.

Validate any saved run from the research environment:

```bash
.venv/bin/python scripts/validate_stage0_output.py \
  data/stage0/jackal_trajectory/<run_id>
```

For direct graph inspection in VS Code, install this repository's custom editor once:

```bash
.venv/bin/python scripts/install_vscode_trajectory_graph_viewer.py
```

Associate `*.npy` with `reconciliation.trajectoryNpyGraph` in VS Code user settings. Clicking
a finite `N x 3 [x, y, yaw]` file then shows the same XY/yaw Matplotlib image as the CLI—never
a heatmap. When a Stage 0 run contains sibling `reference_trajectory.npy` and
`actual_trajectory.npy` files, opening either one compares both automatically. If an old NPY
editor remains open after installation, reload the VS Code window once.

The following optional CLI can compare the saved reference and actual `N x 3 [x, y, yaw]`
arrays or export them to PNG/CSV:

```bash
.venv/bin/python scripts/view_trajectory_npy.py \
  data/stage0/jackal_trajectory/<run_id>/reference_trajectory.npy \
  data/stage0/jackal_trajectory/<run_id>/actual_trajectory.npy
```

The viewer prints the shape, endpoint summary, and stored rows, then opens an interactive
XY/yaw plot. It never transforms coordinates, unwraps yaw, or interpolates samples. Use
`--no-show --save trajectory.png` for a non-interactive image, or `--csv-dir <directory>`
to export readable `x,y,yaw` CSV files. Existing exported files are protected unless
`--force` is explicitly supplied.

Configuration is in `configs/stage0_jackal_trajectory.yaml`; the full contract and observed
limitations are documented in [Stage 0](docs/STAGE_00_JACKAL_TRAJECTORY.md).

## Stage 0-B Jackal controller validation

Stage 0-B separates wheel conversion, articulation tracking, and four-wheel skid-steer
effects with straight, left/right rotation, and arc primitives. It then uses measured Jackal
world-pose feedback with a monotonic nearest/lookahead path follower. Isaac Sim 6.0.1's
`isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController` converts the
resulting `[v, omega]` command to left/right wheel targets; runtime USD geometry maps those
targets to the actual four wheel DOFs.

Run the GUI primitive diagnostics and closed-loop composite validation:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_jackal_controller_validation.sh
./scripts/isaac/run_jackal_controller_validation.sh \
  --controller closed_loop --scenarios composite
```

Both commands keep the GUI open by default with the Jackal, reference line, live actual line,
and reference heading markers. Add `--no-hold` only for automated runs. Immutable local output
is written below `data/stage0/controller_validation/<session_id>/` and remains ignored by Git.
Validate and compare sessions from the research environment with:

```bash
.venv/bin/python scripts/summarize_controller_validation.py \
  data/stage0/controller_validation/<primitive_session_id> \
  data/stage0/controller_validation/<closed_loop_session_id>
```

The diagnostic evidence, telemetry schema, closed-loop timing/progress convention, measured
before/after metrics, and limitations are documented in
[Stage 0-B](docs/STAGE_00_CONTROLLER_VALIDATION.md). This is execution-layer validation, not
LightNav integration or research evidence.

## Stage 0-C LightNav single-chunk integration

Stage 0-C keeps Isaac Sim, research Python, and LightNav Python isolated while passing one
real checkpoint output through the complete interface. Run the stages sequentially so Isaac
Sim GUI and the LightNav model do not compete for GPU memory:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/run_lightnav_single_chunk_demo.sh
```

This wrapper runs all four stages sequentially. The first GUI is capture-only: the Jackal
intentionally stays still while RGB history is recorded. After the inference and validation
steps finish in the terminal, a second GUI opens, waits three seconds, and plays the Jackal
motion at quarter speed so the short chunk is plainly visible.

The same stages can be run individually when inspecting intermediate artifacts:

```bash
./scripts/isaac/run_lightnav_single_chunk_capture.sh
./scripts/lightnav/run_lightnav_single_chunk_inference.sh \
  data/stage0/lightnav_single_chunk/<run_id>
.venv/bin/python scripts/validate_lightnav_single_chunk.py \
  data/stage0/lightnav_single_chunk/<run_id>
./scripts/isaac/run_lightnav_single_chunk_playback.sh \
  data/stage0/lightnav_single_chunk/<run_id>
```

Capture prints `<run_id>` and exits after saving 64 RGB frames. Inference uses only the
external LightNav Python 3.11 environment. Playback uses Isaac's runtime and stays open so the
Jackal, derived reference/headings, and actual path can be inspected together. Raw model
actions, RGB, derived paths, and execution artifacts occupy separate immutable directories
under ignored `data/stage0/lightnav_single_chunk/`.

To watch an already executed run move again without overwriting its immutable outputs:

```bash
./scripts/isaac/run_lightnav_single_chunk_playback.sh \
  data/stage0/lightnav_single_chunk/<run_id> --replay
```

The released LightNav API returns cumulative observation-frame local poses in
`[forward, lateral-left, yaw-CCW]`; its RVQ decoder has already composed the internal
`se2_diff` representation. The rows have no intrinsic waypoint time base. See
[Stage 0-C](docs/STAGE_00_LIGHTNAV_SINGLE_CHUNK.md) for the source evidence, exact transform,
validated run, and why this remains pre-EXP-01 integration validation.

## EXP-01A LightNav latency benchmark

Measure first-versus-warm `predict_waypoints(...)` latency with one model build and nine
requests in the same isolated LightNav process:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/lightnav/run_exp01a_lightnav_latency.sh
```

The default controlled workload reuses the validated Stage 0-C 64-frame history for one
first and eight warm trials. Output is immutable and ignored below
`data/exp01a/lightnav_latency/<benchmark_id>/`. Validate a saved result with
`.venv/bin/python scripts/summarize_exp01a_latency.py <benchmark_directory>`. See
[EXP-01A](docs/EXP_01A_LIGHTNAV_LATENCY.md) for the timing boundary, cache interpretation,
measurements, and limits.

## EXP-01B online OLD/NEW raw switch

Run the persistent warmed LightNav server and the concurrent real-time-paced Isaac client with
one command:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_exp01b_online_raw_switch.sh
```

The default measurement is headless but keeps RGB camera rendering active. It records at least
three timing-valid OLD→NEW transitions below the ignored immutable
`data/exp01b/<experiment_id>/` directory. It uses NEW row 0 directly at measured ready time;
there is no model waypoint-time assumption, stale-row deletion, smoothing, or reconciliation.
Validate a saved experiment with
`.venv/bin/python scripts/summarize_exp01b.py data/exp01b/<experiment_id>`. Add `--gui --hold`
only for a separate qualitative run. See [EXP-01B](docs/EXP_01B_ONLINE_RAW_SWITCH.md) for the
IPC architecture, timing gates, measured result, and claim boundary.

## EXP-02 oracle SE(2) graph

Run the offline synthetic known-answer gate followed by the one-pair real LightNav oracle graph
experiment:

```bash
cd ~/Workspace/se-3-reconciliation
.venv/bin/python scripts/run_exp02_oracle_graph.py
```

The runner uses no Isaac Sim, GPU, or new LightNav inference. It reads the immutable EXP-01B
development pair, preserves its raw hashes, and writes ignored immutable output below
`data/exp02/<run_id>/`. Validate with
`.venv/bin/python scripts/summarize_exp02.py data/exp02/<run_id>`. See
[EXP-02](docs/EXP_02_ORACLE_GRAPH.md) for the factor equations, oracle rationale, ablations,
weight sensitivity, measured trade-off, and strict claim boundary.

## EXP-01 data workflow

Never overwrite a raw VLA recording. Store uncommitted inputs below `data/exp01/raw/`, and
write derived chunk JSON below `data/exp01/derived/`. A raw waypoint file is an arbitrary
`N x 3` `.npy` array or JSON array in `[x, y, yaw]` order. It needs a separate metadata JSON:

```json
{
  "observation_time": 123.0,
  "ready_time": 123.43,
  "inference_latency": 0.43,
  "waypoint_dt": 0.25,
  "robot_pose_at_observation": [1.0, 2.0, 0.1],
  "frame": "robot_local_at_observation",
  "source": "LightNav-0:<checkpoint-or-run-id>"
}
```

The `waypoint_dt` value comes from the collection/controller configuration; LightNav rows
have no intrinsic time base. Build validated derived OLD and NEW chunks separately:

```bash
.venv/bin/python scripts/exp01_build_chunks.py \
  --poses data/exp01/raw/old_poses.npy \
  --metadata data/exp01/raw/old_metadata.json \
  --output data/exp01/derived/old_chunk.json
```

Then run the raw-switch analysis:

```bash
.venv/bin/python scripts/exp01_analyze_raw_switch.py \
  --old data/exp01/derived/old_chunk.json \
  --new data/exp01/derived/new_chunk.json \
  --output results/exp01/raw_switch_metrics.json \
  --plot results/exp01/raw_switch.png
```

The exact same command can be smoke-tested with
`tests/fixtures/exp01_synthetic/{old_chunk,new_chunk}.json`. That fixture is synthetic test
data and is **not research evidence**.

## Conventions

A LightNav-local waypoint is transformed only with the robot pose captured at that chunk's
observation time:

```text
T_world_waypoint = T_world_robot_at_observation * T_robot_waypoint
```

NEW is never transformed with the robot pose at NEW ready time. The offline/discrete EXP-01
scaffold assigns row `i` to `observation_time + (i + 1) * waypoint_dt`; that is a configured
controller convention, not a LightNav model time base. Live EXP-01B instead uses measured
observation/ready events and raw NEW row 0 without time-based row deletion. See
[EXP-01](docs/EXPERIMENT_01.md) and [EXP-01B](docs/EXP_01B_ONLINE_RAW_SWITCH.md) for their
separate protocols and limitations.

## Tests

```bash
.venv/bin/python -m pytest
```

The suite covers SE(2) identity and inverse/compose consistency, angle wrapping, local-to-world
transforms, arbitrary horizons, timing boundary cases, I/O validation, and transition metrics.
