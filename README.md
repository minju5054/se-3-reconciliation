# SE(3) Reconciliation Research

This repository studies how to reconcile successive navigation action chunks while
preserving the intent of a newly predicted trajectory. The revised backend interface takes
fixed OLD execution, a FRESH world-frame trajectory, and a structured pre-treatment context
that selects a spatial entry index `k`; it then operates only on `FRESH[k:]`.

The platform roles are deliberately separate:

- **Isaac Sim 6.0.1** is the simulator.
- **ROS 2 Jazzy** is the robotics middleware and uses the machine's system Python 3.12.
- **LightNav-0** is the upstream navigation-VLA baseline that generates trajectory chunks.
- The reconciliation method developed here is **not LightNav**.

EXP-01/01A/01B characterize raw switching and timing. The previous EXP-02 is retained as a
graph-machinery pilot based on an obsolete collaborator-interface assumption (`Z_ij` as an
SE(2) correspondence measurement). EXP-02A is the current formulation experiment: it treats
collaborator-level Z as `SpatialEntryContext(k, evidence)`, never as a pose transform, and
does not use evidence values as graph residuals or weights.

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

## Stage 0-D Jackal execution-layer calibration

Stage 0-D isolates desired body commands, wheel targets, measured wheel motion, and measured
body motion before further reconciliation research. Its headless primary run freezes a
calibration grid and held-out gates, characterizes the nominal Jackal, tests a minimal
effective-width/feedforward plus yaw-rate PI correction, and validates it on disjoint primitive
conditions, the unchanged Stage 0-B composite path, and frozen EXP-02B OLD replays:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_jackal_execution_calibration.sh --run-id <new_run_id>
.venv/bin/python scripts/summarize_execution_calibration.py \
  data/stage0/execution_calibration/<new_run_id>
```

Run the three qualitative suites against a completed primary run:

```bash
./scripts/isaac/run_jackal_execution_calibration_gui.sh \
  data/stage0/execution_calibration/<run_id> \
  --suite primitive --scenario all --real-time-factor 0.5 --no-hold
./scripts/isaac/run_jackal_execution_calibration_gui.sh \
  data/stage0/execution_calibration/<run_id> \
  --suite composite --real-time-factor 0.5 --no-hold
./scripts/isaac/run_jackal_execution_calibration_gui.sh \
  data/stage0/execution_calibration/<run_id> \
  --suite exp02b --case case_high_delta_omega --k 3 --method raw_k \
  --real-time-factor 0.5 --no-hold
```

Blue is the reference/planned OLD, orange-red is nominal actual, green is calibrated actual,
and grey is raw FRESH. Live terminal output separates desired/executed/measured body motion and
target/measured wheels. Outputs are immutable and ignored below
`data/stage0/execution_calibration/<run_id>/`; no frozen result or physical asset is modified.
The primary 2026-09-07 run ended `EXECUTION_LAYER_NOT_YET_VALIDATED`: the correction improved
some angular/composite metrics but failed the predefined held-out and EXP-02B replay gates. See
[Stage 0-D](docs/STAGE_00_EXECUTION_LAYER_CALIBRATION.md) for protocol, results, plots, GUI
legend, and claim limits.

## Stage 0-E closed-loop trajectory execution validation

Stage 0-E compares the nominal and frozen Stage 0-D `pi_strong` execution layers on the
**same trajectory references**. Unlike Stage 0-D historical command replay, every mode starts
from an independently reset pose/wheel state and a new `TrajectoryFollower` recomputes
`[v, omega]` from the measured pose at every control step. The frozen primary protocol covers
seven deterministic held-out paths, the unchanged Stage 0-B composite, and all three frozen
EXP-02B `derived/old_world.npy` references with three repetitions per mode:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_jackal_closed_loop_execution_validation.sh --run-id <new_run_id>
.venv/bin/python scripts/summarize_closed_loop_execution_validation.py \
  data/stage0/closed_loop_execution_validation/<new_run_id>
```

Run the three qualitative GUI suites against a completed primary run:

```bash
./scripts/isaac/run_jackal_closed_loop_execution_validation_gui.sh \
  data/stage0/closed_loop_execution_validation/<run_id> \
  --suite controlled --scenario all --real-time-factor 0.5 --no-hold
./scripts/isaac/run_jackal_closed_loop_execution_validation_gui.sh \
  data/stage0/closed_loop_execution_validation/<run_id> \
  --suite composite --real-time-factor 0.5 --no-hold
./scripts/isaac/run_jackal_closed_loop_execution_validation_gui.sh \
  data/stage0/closed_loop_execution_validation/<run_id> \
  --suite exp02b --case all --real-time-factor 0.5 --no-hold
```

Blue is the reference/planned OLD, orange-red is nominal actual, green is calibrated actual,
and grey is EXP-02B FRESH context excluded from metrics. Terminal telemetry keeps follower
desired, low-level executed, measured body motion, and four target/measured wheel velocities
separate. The 2026-09-08 primary run ended `EXECUTION_PLATFORM_NOT_READY`: calibrated passed
the composite and all three frozen OLD-reference cases but only 5/7 controlled paths, while
nominal failed all aggregate gates. This selects no future reconciliation execution platform
and does not trigger controller retuning. See
[Stage 0-E](docs/STAGE_00_CLOSED_LOOP_EXECUTION_VALIDATION.md).

## Stage 0-F LightNav trajectory execution envelope

Stage 0-F asks the application-specific question left open by Stage 0-E: whether the frozen
Stage 0-D calibrated candidate covers the geometry actually emitted by the frozen redesigned
EXP-01B LightNav cohort. It does not tune the controller or test reconciliation. OLD and FRESH
references are deduplicated by hash, model STOP outputs are reported separately, and FRESH is
executed from its recorded observation pose. All waypoint descriptors are spatial because
LightNav rows have no intrinsic timestamp.

Prepare an immutable source/geometry inventory, execute every unique moving OLD/FRESH
reference three times in Isaac, and validate the result with:

```bash
cd ~/Workspace/se-3-reconciliation
.venv/bin/python scripts/prepare_lightnav_execution_envelope.py \
  --run-id <new_run_id>
./scripts/isaac/run_jackal_lightnav_execution_envelope.sh \
  data/stage0/lightnav_execution_envelope/<new_run_id>
.venv/bin/python scripts/summarize_lightnav_execution_envelope.py \
  data/stage0/lightnav_execution_envelope/<new_run_id>
```

Inspect deterministic low, median, high, or nearest-strong-turn representatives in the actual
Isaac viewport:

```bash
./scripts/isaac/run_jackal_lightnav_execution_envelope_gui.sh \
  data/stage0/lightnav_execution_envelope/<run_id> \
  --selection high --real-time-factor 1.0 --no-hold
```

`BLUE` is the LightNav reference, `GREEN` calibrated actual, `ORANGE/RED` nominal actual when
the geometry-frozen representative belongs to the nominal subset, and `MAGENTA` is the FRESH
observation/start pose. The frozen 2026-09-08 run covered all 9 unique OLD and 16 unique FRESH
world references under the unchanged Stage 0-E absolute gates. This result is limited to the
observed deterministic LightNav workload; it does not erase the two Stage 0-E strong-turn
fixture failures or declare the execution platform generally validated. See
[Stage 0-F](docs/STAGE_00_LIGHTNAV_EXECUTION_ENVELOPE.md).

## DATA-01 retirement

DATA-01 is `RETIRED_FROM_PRIMARY_FORMULATION_USE`. Its EXP-01B source cohort was collected
for latency and G0/G1/G2 characterization and did not provide the trajectory diversity needed
for primary formulation research. The generated local bank and its EXP-01B-specific builder
have therefore been removed in a forward change. The historical EXP-01B source artifacts and
Git history remain intact for reproducibility. See the retained
[DATA-01 retirement record](docs/DATA_01_FROZEN_LIGHTNAV_TRANSITION_BANK.md).

## EXP-02B-R frozen calibrated re-evaluation

EXP-02B-R re-executes only the 27 exact historical EXP-02B `raw_k`, `rigid`, and
`graph` candidate files (three cases x `k={0,3,6}`) through the frozen Stage 0-D
calibrated post-switch controller. OLD replay remains historical nominal, the saved
boundary and final OLD wheel target are restored, and the calibrated PI state starts
at zero. Desired follower commands, calibrated executed commands, measured body
motion, and target/measured wheels are stored separately.

Prepare, execute, summarize, and strictly validate a new immutable run with:

```bash
cd ~/Workspace/se-3-reconciliation
.venv/bin/python scripts/prepare_exp02b_calibrated_reeval.py \
  --run-id <new_run_id>
./scripts/isaac/run_exp02b_calibrated_reeval.sh \
  data/exp02b_calibrated_reeval/<new_run_id>
.venv/bin/python scripts/summarize_exp02b_calibrated_reeval.py \
  data/exp02b_calibrated_reeval/<new_run_id>
.venv/bin/python scripts/summarize_exp02b_calibrated_reeval.py \
  data/exp02b_calibrated_reeval/<new_run_id> --validate-only
```

Inspect one branch in the actual viewport:

```bash
./scripts/isaac/run_exp02b_calibrated_reeval_gui.sh \
  data/exp02b_calibrated_reeval/<run_id> \
  --case case_high_delta_omega --k 3 --method graph
```

Blue is the frozen candidate, green calibrated actual, orange historical nominal
actual, and grey raw FRESH context; yellow/cyan/magenta mark `B_saved`, candidate
entry, and raw `F_k`. The reset inspection hold pauses physics. The completed run
passed the first-desired-command invariant 27/27 with zero difference and concluded
`EXP02B_FORMULATION_CONCLUSION_UNCHANGED`: angular desired tracking improved, but M4
did not become consistently better than raw_k or rigid. See
[EXP-02B-R](docs/EXP_02B_CALIBRATED_REEVALUATION.md).

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

### EXP-01B expanded characterization cohort

Run the frozen four-condition, headless extension with one persistent warmed LightNav process:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_exp01b_extension.sh
```

The primary protocol targets six timing-valid transitions per condition and stops each condition
after at most ten attempts without weakening the RTF gate. Outputs are immutable, ignored below
`data/exp01b_extension/<run_id>/`, retain STOP separately, and include aggregate CSV/JSON plus
deterministically selected review plots. Validate an existing cohort with
`.venv/bin/python scripts/summarize_exp01b_extension.py <run_dir> --validate-only`. This remains
raw-switch characterization and does not invoke EXP-02A or any reconciliation code. See
[EXP-01B Extension](docs/EXP_01B_EXTENSION.md).

### Redesigned EXP-01B controlled latency × geometry

Run the current 3-geometry × 2-latency raw-switch protocol with one persistent warmed LightNav
server and headless Isaac RGB/control:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_exp01b_controlled_latency.sh primary
```

This records canonical Stage 0-B controller commands `[v, omega]` around the switch and treats
the old untimed waypoint-spacing comparison as a secondary spatial descriptor. The frozen
protocol targets five timing-valid moving FRESH samples per cell (30 total), preserves STOP and
failure attempts, and never changes or timestamps LightNav rows. Generated outputs are ignored
under `data/exp01b_redesign/<run_id>/`. Validate a saved run with
`.venv/bin/python scripts/summarize_exp01b_controlled_latency.py <run_dir> --validate-only`.
See [Redesigned EXP-01B](docs/EXP_01B_REDESIGNED_CONTROLLED_LATENCY.md) for qualification,
timing semantics, actual results, and the mixed execution-level conclusion.

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

This command reproduces the historical graph-machinery pilot. Its `Z_ij` measurement is not
the current collaborator-facing Z interface and its saved results are not reinterpreted.

## EXP-02A spatial-entry transition reconciliation

Run the revised offline experiment with synthetic early/middle/late entries followed by the
immutable EXP-01B LightNav backend pilot:

```bash
cd ~/Workspace/se-3-reconciliation
.venv/bin/python scripts/run_exp02a_spatial_entry.py
```

The runner compares raw `FRESH[k:]`, the diagnostic pose anchor, entry preservation, and the
incoming-motion-aware transition formulation for predeclared `k` values. Outputs are immutable
and ignored under `data/exp02a/<run_id>/`. Validate a saved run with:

```bash
.venv/bin/python scripts/summarize_exp02a.py data/exp02a/<run_id>
```

See [EXP-02A](docs/EXP_02A_SPATIAL_ENTRY_RECONCILIATION.md) for the exact Z schema, factor
units, synthetic and LightNav pilot results, inter-k retention metrics, and claim limits.

## EXP-02B controller-aware reconciliation evaluation

EXP-02B evaluates the unchanged EXP-02A incoming-motion-aware graph through the validated
Stage 0-B Jackal controller. It uses three deterministically selected, immutable redesigned
EXP-01B stress/development cases and compares naive `FRESH[0:]`, same-k `FRESH[k:]`, the
diagnostic pose anchor, an analytic rigid SE(2) baseline, and the graph for `k = 0, 3, 6`.
The selector remains manual/oracle; this is not an end-to-end LightNav comparison.

Run the offline candidate generation, headless Isaac branch evaluation, and strict summary in
sequence:

```bash
.venv/bin/python scripts/run_exp02b_controller_aware.py --run-id <run_id>
./scripts/isaac/run_exp02b_branch_execution.sh data/exp02b/<run_id>
.venv/bin/python scripts/summarize_exp02b.py data/exp02b/<run_id>
```

Generated runs are immutable and ignored under `data/exp02b/`. See
[EXP-02B](docs/EXP_02B_CONTROLLER_AWARE_RECONCILIATION.md) for the frozen case-selection
rules, controller metrics, full results, and claim boundaries.

To diagnose why the OLD reference and the executed Jackal path differ, run the separate GUI
pipeline on the representative frozen branch:

```bash
./scripts/isaac/run_exp02b_gui_diagnosis.sh \
  --case case_high_delta_omega \
  --k 3 \
  --method raw_k
```

The viewport separates planned OLD, replayed OLD actual, full FRESH, selected FRESH suffix,
the current candidate, and post-reset actual history. The terminal prints phase and live
controller/wheel telemetry. Diagnostic artifacts are immutable and ignored under
`data/exp02b_gui_diagnosis/<run_id>/`; they do not modify the frozen EXP-02B result. See
[EXP-02B GUI diagnosis](docs/EXP_02B_GUI_DIAGNOSIS.md) for colors, reset semantics, output
schema, observed representative-case values, and interpretation limits.

## EXP-02C current-M4 factor isolation

EXP-02C is an offline failure-attribution diagnostic for the exact historical M4 factors. It
does not change the graph objective, tune weights, add a gate, or claim improved navigation.
Run the five synthetic mechanism fixtures and all 9 frozen real case/k conditions, then render
the 13 saved plots:

```bash
.venv/bin/python scripts/run_exp02c_factor_isolation.py --run-id <run_id>
.venv/bin/python scripts/summarize_exp02c_factor_isolation.py \
  data/exp02c_factor_isolation/<run_id>
```

Inspect the primary benign-case attribution and the S4 downstream-conflict counterfactual in
Isaac. The viewport includes the official Jackal USD as a **static visual reference at real
saved boundary B (or the declared synthetic B)** together with the DebugDraw geometry; it does
not execute physics:

```bash
RUN=data/exp02c_factor_isolation/<run_id>
./scripts/isaac/run_exp02c_factor_isolation_gui.sh "$RUN" \
  --view real_benign_k0
./scripts/isaac/run_exp02c_factor_isolation_gui.sh "$RUN" \
  --view synthetic_s4
```

The interactive command stays open until the Isaac window is closed or `Ctrl-C` is pressed.
Do not add `--no-hold` for manual inspection: that option is reserved for automated capture
checks and intentionally closes the window after saving the screenshot.

All arrays, residuals, costs, numerical Jacobian gradients, desired-command probes, geometry,
rigidity diagnostics, summaries, plots, and GUI captures are written exclusively below the
ignored run directory. LightNav rows remain untimed spatial waypoints. See
[EXP-02C](docs/EXP_02C_FACTOR_ISOLATION.md) for exact variants, observed attribution, and the
single recommended next formulation experiment.

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
