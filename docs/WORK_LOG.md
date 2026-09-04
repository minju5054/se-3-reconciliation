# Work Log

This file is append-only. Add each completed task at the bottom.

## 2026-09-02 15:51:26 KST (+0900) — Local environments and EXP-01 initialization

- **Purpose:** Establish isolated research and LightNav development environments, then
  implement EXP-01's naive latency-aware OLD→NEW raw-switch characterization without graph
  optimization.
- **Environment setup:** Cloned this empty research repository at
  `~/Workspace/se-3-reconciliation` and upstream LightNav at
  `~/Workspace/external/LightNav-0`. Installed user-local uv 0.12.9. Created this repository's
  `.venv` with system Python 3.12.3 and LightNav's separate `.venv` with uv-managed Python
  3.11.16. Installed LightNav editable with official `vllm`, `video`, and `habitat` extras plus
  its test extra. Confirmed LightNav 0.1.0, vLLM 0.19.1, CUTLASS DSL 4.5.2, Transformers 5.8.0,
  Torch 2.10.0, CUDA availability, and an NVIDIA GeForce RTX 5060 Ti. `/usr/bin/python3`
  remained Python 3.12.3; ROS 2, NVIDIA driver, CUDA, and Isaac Sim installations were not
  modified.
- **Implemented:** Added the arbitrary-horizon `TrajectoryChunk` schema and validation;
  minimal SE(2) compose/inverse/relative/local-to-world utilities; configurable discrete
  execution timing and stale-prefix logic; raw-switch boundary selection; separate pose-gap
  and motion-jump metrics; `.npy`/JSON ingestion; chunk-building and analysis/plotting CLIs;
  versioned EXP-01 config; a clearly labeled synthetic test/demo fixture; repository safety
  rules; and complete experiment documentation. No smoothing, interpolation, GTSAM, graph
  optimization, correspondence logic, model weights, or real experiment claims were added.
- **Major files:** `README.md`, `AGENTS.md`, `.gitignore`, `pyproject.toml`,
  `requirements.txt`, `configs/exp01.yaml`, `src/reconciliation/`, `scripts/`, `tests/`,
  `docs/EXPERIMENT_01.md`, `docs/WORK_LOG.md`, and `results/exp01/README.md`.
- **Commands:** `git status --short --branch`; `git branch --show-current`; `git remote -v`;
  `git clone`; uv user-local installation; `uv python install 3.11`; `uv venv`; `uv pip
  install`; `uv pip check`; LightNav `make test`; research `python -m pytest`; `python -m
  compileall`; both EXP-01 CLI `--help` checks; synthetic `exp01_analyze_raw_switch.py` run;
  `git diff --check`; Git diff/status/staging/commit/push checks described below.
- **Verification:** Research suite: 30 passed. LightNav CPU suite: 599 passed, 1 skipped.
  Dependency checks: 17 research and 182 LightNav packages compatible. Synthetic analysis
  generated valid ignored JSON/PNG artifacts. Both repositories were clean of unintended
  tracked changes outside this task; the LightNav source checkout remained unmodified.
- **Issues and limitations:** The first LightNav `make test` was launched without its `.venv`
  first on `PATH`, so the Makefile selected system Python 3.12 and failed dependency imports;
  rerunning with LightNav's `.venv/bin` on `PATH` passed all CPU tests. No checkpoint was
  downloaded or committed. Actual EXP-01 evidence still requires manual Isaac Sim/LightNav
  collection with model access and explicit timestamp/frame metadata.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-02 17:13:29 KST (+0900) — Stage 0 Jackal trajectory smoke test

- **Purpose:** Validate the pre-LightNav Isaac Sim trajectory pipeline using a deterministic
  SE(2) reference, the official Clearpath Jackal execution platform, actual world-pose
  recording, and simultaneous viewport visualization. This is simulation pipeline validation,
  not research evidence.
- **Implemented:** Added pure configurable unicycle trajectory generation and finite arbitrary
  `N x 3` validation; immutable reference/actual NPY, aligned CSV, metadata output, sanity
  metrics, overwrite protection, and an Isaac-independent validator. Added a GUI-default
  Isaac runner and child-only environment-scrubbing launcher. The runtime resolves the asset
  root and official `Clearpath/Jackal/jackal.usd`, discovers the articulation and actual DOFs,
  derives side mapping from revolute-joint lateral positions, reads wheel radius from collision
  cylinders, computes wheel separation from the asset, applies four wheel velocity targets,
  extracts `[world_x, world_y, world_yaw]`, and updates reference/actual DebugDraw geometry at
  every sample. Added config, tests, README instructions, and full Stage 0 documentation.
- **Major files:** `README.md`, `configs/stage0_jackal_trajectory.yaml`,
  `docs/STAGE_00_JACKAL_TRAJECTORY.md`, `docs/WORK_LOG.md`,
  `src/reconciliation/trajectory.py`, `src/reconciliation/stage0_jackal.py`,
  `scripts/isaac/jackal_trajectory_demo.py`,
  `scripts/isaac/run_jackal_trajectory_demo.sh`, `scripts/validate_stage0_output.py`,
  `tests/test_trajectory.py`, and `tests/test_stage0_jackal.py`.
- **Commands:** Repository status/branch/remote/base inspection; local Isaac 6.0.1 API/example
  inspection; isolated headless runtime asset probes; `.venv/bin/python -m pytest`;
  `python -m compileall`; `bash -n`; two GUI-mode `run_jackal_trajectory_demo.sh --no-hold`
  smoke runs; independent `validate_stage0_output.py`; `uv pip check`; Git diff/status,
  explicit staging, cached-diff, commit, and normal push checks.
- **Verification:** Full research suite: 47 passed, including all prior 30 EXP-01 tests.
  Research environment dependency check: 17 packages compatible. Isaac Sim GUI mode opened
  without headless/no-window flags, loaded the ground and Jackal, executed all four wheel DOFs,
  and completed real-time DebugDraw updates. Final validated run ID:
  `codex-stage0-smoke-validated-20260902`. Reference shape `(161, 3)`; actual shape `(161, 3)`.
  Actual start `[-0.0002078209, -0.0000000562, 0.0000003236]`; actual end
  `[3.7261931896, 0.0875781551, 0.0595319028]`; displacement `3.7274300040 m`; total yaw
  change `0.0595315792 rad`. Straight-segment maximum displacement `1.1957686317 m` and
  turn-segment absolute yaw change `0.0500163637 rad` passed configured smoke thresholds.
  Independent saved-output validation returned `valid: true`.
- **Runtime discovery:** Articulation prim `/World/Jackal`; DOF count 4; names
  `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, and
  `rear_right_wheel_joint`. Asset-derived wheel radius `0.0979999974 m`; separation
  `0.3755899966 m`.
- **Issues and limitations:** The official asset emitted non-fatal obsolete
  `customGeometry` PhysX warnings. Open-loop skid-steer motion did not closely track the ideal
  unicycle turn (final position error `1.2761473886 m`, final yaw error `0.6904680972 rad`),
  which is acceptable because Stage 0 is not a controller-quality study. DebugDraw is transient
  viewport-only geometry. Generated run directories remain ignored and were not staged.
  LightNav inference, checkpoints, OLD/NEW generation, ROS control, Nav2, MPC, correspondence,
  GTSAM, and graph optimization were not introduced. System Python remained 3.12.3; ROS,
  CUDA, NVIDIA driver, and Isaac Sim installations were not modified.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-02 19:54:06 KST (+0900) — Canonical trajectory NPY viewer

- **Purpose:** Make recorded trajectory `.npy` files directly inspectable as canonical
  `N x 3 [x, y, yaw]` data in both the terminal and an interactive plot.
- **Implemented:** Added a safe non-pickled NPY loader backed by the existing finite SE(2)
  validator; endpoint, path-length, and wrapped endpoint-yaw summaries; configurable
  leading/trailing row output; multi-file XY and stored-yaw comparison plots with explicit
  start/end markers; optional PNG output; and lossless row-wise `x,y,yaw` CSV export with
  overwrite protection. The viewer performs no coordinate transform, interpolation,
  resampling, or yaw unwrapping. Added README usage and unit/CLI coverage.
- **Major files:** `README.md`, `docs/WORK_LOG.md`,
  `src/reconciliation/trajectory_view.py`, `scripts/view_trajectory_npy.py`, and
  `tests/test_trajectory_view.py`.
- **Commands:** Repository status/branch/remote/base inspection; `.venv/bin/python -m pytest`;
  `.venv/bin/python -m compileall`; viewer `--help`; non-interactive viewer run against the
  validated Stage 0 reference and actual arrays with PNG and CSV export; rendered-image
  inspection; `file`; `wc -l`; `git diff --check`; Git diff/status, explicit staging,
  cached-diff, commit, and normal push checks.
- **Verification:** Full research suite: 53 passed, including all existing 47 tests. The
  validated Stage 0 `(161, 3)` reference and `(161, 3)` actual arrays loaded successfully.
  A 1920 x 880 PNG showed both XY paths, start/end markers, and stored yaw histories; each
  CSV contained its header plus all 161 unmodified pose rows.
- **Issues and limitations:** The automated check used Matplotlib's non-interactive backend
  and visually inspected the saved PNG; opening the default interactive window remains a
  user desktop action. The viewer accepts only finite `N x 3` `.npy` trajectory arrays and
  intentionally does not infer frames or timestamps that are absent from the file.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-02 20:00:07 KST (+0900) — VS Code direct NPY file viewing

- **Purpose:** Open `.npy` files directly by clicking them in VS Code instead of requiring a
  terminal command or showing the unsupported-binary-file message.
- **Implemented:** Installed the user-scoped `subh-tools.npy-viewer` VS Code extension version
  1.0.2. Verified its installed manifest declares the read-only `npyViewer.arrayEditor`
  custom editor for `*.npy` with default priority and requires no Python for its built-in
  parser. Reopened an actual Stage 0 trajectory through the current VS Code window and updated
  README instructions so direct Explorer viewing is the primary workflow; the repository CLI
  remains an optional comparison/PNG/CSV export tool.
- **Major files:** `README.md` and `docs/WORK_LOG.md`. The extension itself is installed in the
  user's VS Code extension directory and is not vendored into this repository.
- **Commands:** Repository status/branch/remote/base inspection; VS Code version and installed
  extension inspection; `code --install-extension subh-tools.npy-viewer --force`; installed
  extension manifest inspection; VS Code user association check; and `code --reuse-window`
  against the validated Stage 0 actual trajectory.
- **Verification:** VS Code reported successful installation of
  `subh-tools.npy-viewer@1.0.2`; the manifest targets `*.npy` as a default custom editor, and
  no conflicting user or workspace `.npy` editor association was present. The actual
  trajectory open command returned successfully.
- **Issues and limitations:** A text-editor tab that was already open before extension
  installation may retain the old binary warning until it is closed and reopened. `.npz` is
  outside this extension's scope. No system Python, research virtual environment, ROS, CUDA,
  driver, or Isaac Sim installation was modified.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-02 20:08:38 KST (+0900) — Image-only VS Code NPY preview

- **Purpose:** Replace the generic NPY viewer's unwanted heatmap rendering with an image-only
  `.npy` preview when files are clicked in VS Code.
- **Implemented:** Installed the user-scoped `Kiameow.npy-image-preview` extension version
  1.1.0, verified that its `npy-image-preview.preview` custom editor renders NPY data as RGBA
  or grayscale images, and removed the conflicting `subh-tools.npy-viewer` heatmap/statistics
  extension. Added an explicit user-level `workbench.editorAssociations` mapping from `*.npy`
  to the image preview editor and reopened a Stage 0 reference trajectory. Updated README to
  document the corrected image-only workflow.
- **Major files:** `README.md` and `docs/WORK_LOG.md`. User-local VS Code settings were updated
  at `~/.config/Code/User/settings.json`; no extension code or generated image was vendored.
- **Commands:** Repository status/branch/remote/base inspection; VS Code/extension/tooling
  inspection; image-preview extension installation and manifest/README inspection; generic
  viewer uninstallation; user editor-association verification; and `code --reuse-window`
  against actual and reference Stage 0 trajectory arrays.
- **Verification:** `kiameow.npy-image-preview@1.1.0` is the only installed NPY-related
  extension. Its manifest registers `npy-image-preview.preview` as the default editor for
  `*.npy`, and the VS Code user association explicitly selects that same editor. Both open
  commands completed successfully.
- **Issues and limitations:** A tab retained from the previous extension may need to be closed
  and reopened once. The extension interprets numeric NPY contents as pixels; it does not
  infer SE(2) trajectory semantics. No system Python, virtual environment, ROS, CUDA, driver,
  or Isaac Sim installation was modified.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-02 20:26:10 KST (+0900) — VS Code SE(2) trajectory graph custom editor

- **Purpose:** Make a clicked trajectory `.npy` file display the exact XY and stored-yaw graph
  produced by the existing CLI, with no heatmap or pixel-image interpretation.
- **Implemented:** Added a dependency-free repository-specific VS Code read-only custom editor
  and a Python VSIX builder/installer. The editor invokes the tested
  `scripts/view_trajectory_npy.py` through this repository's `.venv/bin/python` without a
  shell, embeds only the resulting PNG in its webview, watches the opened file for changes,
  and reports validation/render errors without modifying data. When sibling
  `reference_trajectory.npy` and `actual_trajectory.npy` files exist, clicking either plots
  both exactly as the CLI comparison does. Removed the conflicting generic image extension,
  installed `se3-reconciliation.trajectory-npy-graph-viewer@0.1.0`, and explicitly associated
  `*.npy` with `reconciliation.trajectoryNpyGraph` in VS Code user settings.
- **Major files:** `.gitignore`, `README.md`, `docs/WORK_LOG.md`,
  `scripts/install_vscode_trajectory_graph_viewer.py`,
  `tools/vscode-trajectory-npy-viewer/{package.json,extension.js,README.md}`, and
  `tests/test_vscode_trajectory_viewer.py`.
- **Commands:** Repository status/branch/remote/base inspection; VSIX-format inspection;
  targeted and full pytest; Electron/Node JavaScript syntax check; build-only VSIX and ZIP
  integrity checks; installer execution; installed extension/source comparison; user editor
  association update; VS Code open/new-window commands; Extension Host activation-log check;
  actual Stage 0 two-file plot generation; rendered-image inspection; compileall; and Git
  diff/status/staging/commit/push checks.
- **Verification:** Full research suite: 55 passed. The built VSIX contained only the custom
  editor manifest, JavaScript, and documentation; the installed package and JavaScript matched
  repository source (apart from VS Code's injected metadata). Extension Host activated
  `se3-reconciliation.trajectory-npy-graph-viewer` for the configured custom editor without a
  render error. The real `(161, 3)` reference and actual trajectories produced the expected
  1920 x 880 XY/yaw comparison image, which was visually inspected.
- **Issues and limitations:** The workspace must be trusted and the repository `.venv` plus
  plotting script must exist because the extension deliberately reuses the canonical tested
  renderer. Other `.npy` shapes fail explicit `N x 3` validation. A tab retained from an older
  NPY extension may need one VS Code window reload. No system Python, ROS, CUDA, driver, or
  Isaac Sim installation was modified; generated VSIX and PNG artifacts are not committed.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-03 00:19:24 KST (+0900) — Stage 0-B Jackal controller validation

- **Purpose:** Separate the Stage 0-A tracking error into wheel-conversion, wheel-drive, and
  four-wheel skid-steer effects, then establish a pose-feedback execution layer suitable for
  later LightNav trajectory integration. This is simulation pipeline validation, not research
  evidence.
- **Implemented:** Added finite differential-wheel conversion and geometry-based four-DOF side
  mapping; a monotonic nearest/path-distance-lookahead SE(2) follower with terminal yaw
  alignment; actual body-rate estimation; desired body, target wheel, directly measured wheel,
  and actual pose telemetry; reference-index-aware metrics for unequal reference/actual sample
  counts without interpolation; strict immutable NPY/CSV/JSON output validation; cross-session
  summary tooling; GUI DebugDraw heading markers; child-only runtime environment scrubbing;
  config, unit tests, README instructions, and full Stage 0-B documentation. Used Isaac Sim
  6.0.1's non-deprecated experimental `DifferentialController`; did not modify the official
  USD asset or override drive/contact properties.
- **Controller comparison:** With runtime-derived wheel radius `0.0979999974 m` and separation
  `0.3755899966 m`, custom and official left/right outputs had maximum absolute difference
  `0 rad/s` for stop, `v=0.5` straight, `omega=+0.25` rotation, `v=0.4/omega=0.25` arc, and
  `omega=-0.25` rotation. Conclusion: `wheel conversion formula bug is not supported`.
- **Primitive validation:** Final GUI session `stage0b-official-primitives-20260903` produced
  valid `(51, 3)` reference/actual arrays for all runs. Straight desired/measured mean speed
  was `0.30000/0.29605 m/s`, wheel RMSE `0.04842 rad/s`, displacement `1.19522 m`, and yaw
  drift `0.00021 rad`. Rotate-left desired/measured mean yaw rate was
  `0.25000/0.07607 rad/s`, wheel RMSE `0.29289 rad/s`, and total yaw `0.30430 rad`.
  Rotate-right was `-0.25000/-0.03696 rad/s`, wheel RMSE `0.10103 rad/s`, and total yaw
  `-0.14785 rad`. Arc was desired/measured `v=0.30000/0.29348 m/s` and
  `omega=0.20000/0.00800 rad/s`, wheel RMSE `0.06885 rad/s`; expected radius `1.5 m`,
  measured effective radius `36.67637 m`.
- **Root cause:** Straight/arc wheel and linear-speed tracking were adequate while yaw response
  was far below ideal; pure rotation also exposed wheel tracking loss and left/right
  asymmetry. The most likely large Stage 0-A error is the ideal-unicycle/physical-track-width
  mismatch under four-wheel skid-steer tire/contact physics, not the conversion formula.
  No arbitrary effective-width calibration, friction tuning, damping change, or effort change
  was used.
- **Closed-loop result:** Final GUI session `stage0b-closed-loop-accepted-20260903` saved
  reference `(101, 3)` and actual `(143, 3)`, reached the goal before `18 s`, and passed strict
  output validation. Position RMSE `0.0503478 m`, final position error `0.0740490 m`, yaw RMSE
  `0.0568929 rad`, and final yaw error `0.0772074 rad` passed all four engineering thresholds.
  Stage 0-A final errors were `1.2761474 m` and `0.6904681 rad`. The GUI displayed the ground,
  Jackal, reference path/headings, and live accumulated actual path; the saved unequal-length
  arrays also rendered correctly in the existing CLI/VS Code trajectory graph viewer.
- **Major files:** `README.md`, `configs/stage0_jackal_controller_validation.yaml`,
  `docs/STAGE_00_CONTROLLER_VALIDATION.md`, `docs/WORK_LOG.md`,
  `src/reconciliation/controller_validation.py`,
  `src/reconciliation/controllers/{__init__,differential,trajectory_follower}.py`,
  `scripts/isaac/jackal_controller_validation.py`,
  `scripts/isaac/run_jackal_controller_validation.sh`,
  `scripts/summarize_controller_validation.py`, and the three Stage 0-B test modules.
- **Commands:** Git status/branch/remote/base inspection; installed Isaac 6.0.1 source/API and
  official asset inspection; official/custom numerical comparison; full pytest and compileall;
  shell syntax and diff checks; two GUI-mode closed-loop trials and two GUI-mode four-primitive
  sessions; strict JSON and output validation; cross-session table summary; saved trajectory
  graph rendering and visual inspection; Git diff/status, explicit staging, cached-diff,
  commit, and normal push checks.
- **Verification:** Full research suite: `76 passed`, including all existing EXP-01, Stage 0-A,
  CLI viewer, and VS Code viewer tests plus 21 new Stage 0-B tests. Isaac GUI runs used the
  runtime articulation `/World/JackalReference` with DOFs `front_left_wheel_joint`,
  `front_right_wheel_joint`, `rear_left_wheel_joint`, and `rear_right_wheel_joint`. Both final
  sessions passed the independent run validator, and metadata passed strict JSON parsing.
- **Issues and limitations:** Desired and measured body yaw rates remain different because the
  feedback controller compensates over additional time instead of making skid-steer physics
  ideal. Validation covers flat ground and one deterministic composite; it is not a safety or
  general controller-performance claim. The official asset still emits non-fatal obsolete
  `customGeometry` warnings. Generated local sessions remain ignored and uncommitted. LightNav,
  OLD/NEW chunks, correspondence, GTSAM, graph optimization, ROS control, Nav2, MPC, and
  obstacle avoidance remain unimplemented. No system Python, ROS, CUDA, driver, or Isaac Sim
  installation was modified.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-03 10:13:49 KST (+0900) — Stage 0-C LightNav single-chunk integration

- **Purpose:** Validate that actual stationary Jackal egocentric RGB from Isaac Sim 6.0.1 can
  be passed to the real LightNav-0 checkpoint in its isolated Python 3.11 environment, then
  preserve and source-validate the decoded chunk semantics, transform it from observation
  robot-local to world SE(2), visualize it, and consume it through the existing Stage 0-B
  closed-loop execution layer. This is simulation integration validation, not EXP-01 or
  research evidence.
- **Implemented:** Added an immutable sequential capture/inference/derive/playback workflow,
  child-process environment isolation, a configurable corridor and attached egocentric RGB
  camera, raw/derived artifact separation, public-decoder semantics validation, exact
  observation-pose world transformation, safety and full-output validation, saved-actual GUI
  replay, config, tests, README commands, and detailed Stage 0-C documentation. Raw model
  output is never rewritten; no interpolation, clipping, smoothing, synthetic success data,
  or fabricated waypoint period/ready timestamp is introduced.
- **LightNav/source facts:** Local clean checkout
  `a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0, was one upstream commit behind
  `0e9971784a04da2210bfccc446a68d45256e2894`; the only diff was
  `docs/assets/wechat_group.png`, so no inference-code difference was found. Local decoder,
  tokenizer, protocol, visualization, and deployment code jointly establish public `(H, 3)`
  output as cumulative observation-frame `[forward, lateral-left, yaw-CCW]` poses. Although
  the checkpoint RVQ manifest uses internal `se2_diff`, its official decoder SE(2)-composes
  to absolute chunk-start poses before returning. Rows carry no intrinsic time base;
  checkpoint `video_fps=4` concerns input history.
- **Checkpoint/inference:** Downloaded the public `LightOriginsHQ/LightNav-0` checkpoint
  revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6` into the ignored external checkout and
  verified all 20 remote files with the Hugging Face CLI. Actual `vllm_local` inference used
  vLLM 0.19.1, torch 2.10.0+cu128, and an RTX 5060 Ti (compute capability 12.0). Excluding
  model load, host latency was `49561.998 ms` and LightNav reported `49267.944 ms`. Output was
  float32 `(10, 3)`, first row `[0.15485105, 0.00006702, -0.00419930]`, last row
  `[0.71874577, -0.00634444, -0.00297201]`.
- **Observation/transform:** Actual GUI capture saved 64 `256 x 448 x 3 uint8 RGB` frames at
  4 Hz. Observation simulation time was `17.0333342217 s`; the final-frame Jackal pose was
  `[-0.0010703253, -0.0000460156, -0.0006066629]`. The attached camera prim was
  `/World/JackalReference/Stage0CEgocentricCamera` at `[0.30, 0, 0.48]` m and
  `[90, 0, -90]` degrees relative to `/World/JackalReference`, resolution 448 x 256 and 90
  degree HFOV. Raw-to-local was an identity-axis float64 derived copy; local/world shapes
  were `(10, 3)`. World start/end were `[0.15378074, -0.00007294, -0.00480597]` and
  `[0.71767146, -0.00682649, -0.00357868]`.
- **Execution/GUI result:** The path passed all magnitude, yaw, spacing, timing, artifact, and
  metadata checks, then executed with the unchanged Stage 0-B `TrajectoryFollower` plus Isaac
  experimental `DifferentialController`. Actual trajectory was `(24, 3)`, goal reached in
  `2.3000 s`, position RMSE `0.0458569 m`, final position error `0.0783258 m`, yaw RMSE
  `0.00730783 rad`, and final yaw error `0.000107417 rad`. GUI-mode capture and execution both
  ran. A subsequent saved-run GUI replay was screen-inspected with Jackal, reference path,
  heading/start/end markers, and actual path visible together after camera/light correction.
  These are controller integration metrics, not LightNav navigation-quality metrics.
- **Major files:** `README.md`, `configs/stage0_lightnav_single_chunk.yaml`,
  `docs/STAGE_00_LIGHTNAV_SINGLE_CHUNK.md`, `docs/WORK_LOG.md`,
  `src/reconciliation/lightnav_adapter.py`, `tests/test_lightnav_adapter.py`,
  `scripts/validate_lightnav_single_chunk.py`,
  `scripts/lightnav/{infer_single_chunk.py,run_lightnav_single_chunk_inference.sh}`, and
  `scripts/isaac/{lightnav_stage0c_runtime.py,lightnav_capture_observation.py,
  lightnav_playback_single_chunk.py,run_lightnav_single_chunk_capture.sh,
  run_lightnav_single_chunk_playback.sh}`.
- **Commands:** Initial research/LightNav Git status, branch, remote, HEAD, Python/package/GPU
  inspection; `hf download` and `hf cache verify`; checkpoint JSON/manifest and local source
  cross-checks; upstream fetch/diff without pull/rebase; full and targeted pytest; compileall;
  shell syntax and diff checks; GUI Isaac capture; real LightNav inference; derivation and
  strict pre/post-execution validation; GUI closed-loop playback; GUI saved-path replay and
  desktop screenshot inspection; explicit Git diff/status/staging/commit/push checks.
- **Verification:** Full research suite: `87 passed`, including every existing EXP-01,
  Stage 0-A/B, CLI/VS Code viewer test and 11 new Stage 0-C tests. `compileall`, launcher
  `bash -n`, `git diff --check`, checkpoint verification, actual RGB/action/path checks, and
  strict execution-output validation passed. Generated run
  `data/stage0/lightnav_single_chunk/20260903T010425Z` remains ignored and uncommitted.
- **Issues and limitations:** The single stationary-history corridor run does not evaluate
  navigation quality or domain generalization. The experiment camera's 90 degree HFOV is an
  explicit interface choice, not proof of checkpoint camera-domain equivalence. Eager vLLM
  inference took about 49.6 seconds, so online concurrency/latency policy remains unresolved.
  No model-intrinsic waypoint time base exists; EXP-01 must separately define observation,
  ready, execution-period, OLD-continuation, and NEW-usability events. Isaac emitted the same
  non-fatal legacy Jackal wheel-collision warnings seen previously. No system Python, ROS,
  CUDA, driver, Isaac installation, official USD, or LightNav source was modified.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-03 14:17:59 KST (+0900) — Observable and repeatable Stage 0-C GUI playback

- **Purpose:** Diagnose why rerunning the documented Stage 0-C GUI workflow appeared to leave
  the Jackal stationary, and make the short LightNav chunk visibly and safely replayable.
- **Diagnosis:** Five recent local run directories contained RGB capture metadata and frames
  but no inference, derived path, or playback artifacts. The first GUI command is intentionally
  a stationary observation-history capture, which was not prominent enough in the workflow.
  In addition, the 0.72 m chunk's 2.3 s simulation execution previously ran near full speed
  and completed before a user could reliably move attention to the new GUI window. An already
  executed run could not be run again because immutable output protection correctly rejected
  the existing files.
- **Implemented:** Added a top-level sequential `run_lightnav_single_chunk_demo.sh` that
  captures, extracts the generated run path, runs isolated LightNav inference, validates, and
  launches playback with one command. Capture now logs that the first GUI is stationary by
  design. Playback shows the initial scene for three seconds and paces every physics step at
  configurable 0.25x real time for smooth visible motion. Added `--replay` to execute an
  already recorded run in a fresh Isaac stage without writing or replacing any artifact; a
  normal second execution now gives an explicit instruction to use that option. README and
  Stage 0-C documentation distinguish capture, first execution, visualization-only, and
  immutable replay commands.
- **Major files:** `README.md`, `configs/stage0_lightnav_single_chunk.yaml`,
  `docs/STAGE_00_LIGHTNAV_SINGLE_CHUNK.md`, `docs/WORK_LOG.md`,
  `scripts/isaac/lightnav_capture_observation.py`,
  `scripts/isaac/lightnav_playback_single_chunk.py`, and
  `scripts/run_lightnav_single_chunk_demo.sh`.
- **Commands:** Initial Git/status/remote/HEAD inspection; local Stage 0-C run artifact
  inventory; playback/config inspection; compileall; full pytest; launcher `bash -n`;
  `git diff --check`; strict saved-run validation; two real GUI replay runs; mid-motion and
  final desktop screenshot inspection; SHA-256 manifests of all derived/result files before
  and after replay; explicit diff/status/staging/commit/push checks.
- **Verification:** Full research suite: `87 passed`. The real validated run replayed with a
  visible countdown and smooth quarter-speed motion, still reached the same 0.6417 m actual
  displacement and `goal_reached=true`, and remained open at the final pose. Mid-motion and
  final screenshots showed different Jackal positions. The pre/post artifact SHA-256 manifest
  diff was empty, and the independent execution validator remained `valid: true`.
- **Issues and limitations:** The first GUI remains stationary because moving during history
  capture would change the intended observation protocol. A new full one-command run still
  includes roughly 50 seconds of model inference between the two GUI phases. The existing
  user modification to `configs/stage0_jackal_controller_validation.yaml` was preserved and
  excluded from this task's staging. No generated data, checkpoint, environment, or external
  LightNav source is committed.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-03 15:28:48 KST (+0900) — EXP-01A LightNav steady-state inference latency

- **Purpose:** Measure first and warm LightNav `predict_waypoints(...)` latency with one model
  build and one persistent process, using a controlled repeated actual Stage 0-C RGB history.
  Compare warm host latency contextually with the validated Stage 0-C chunk execution time.
  This task did not run Isaac Sim, execute Jackal, define waypoint timing, or implement an
  online OLD/NEW loop.
- **Implemented:** Added a configurable isolated-LightNav benchmark runner, one-command shell
  launcher, pure statistics/validation module, immutable per-trial action and text artifacts,
  strict CSV/JSON metadata and summary output, independent research-environment validator,
  12 unit tests, README instructions, and the full measured protocol/result document. The
  runner decodes images before timing, calls `build_tracking_agent(...)` exactly once, then
  performs `reset` + 64 `observe` calls + one timed prediction for trial 0 and eight warm
  trials. Host `monotonic_ns` latency, LightNav-reported latency, reset/ingest time, internal
  data-prep/ViT/LLM timing, action contract, and cache entry count are all preserved.
- **Benchmark:** Final immutable benchmark ID `exp01a-20260903T062524Z` used Stage 0-C input
  `20260903T010425Z`, instruction `Go straight down the corridor.`, and 64 actual
  `256 x 448 x 3 uint8 RGB` frames at 4 Hz. LightNav checkout was
  `a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0; checkpoint revision was
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`; backend was vLLM 0.19.1 `vllm_local` with
  torch 2.10.0+cu128 on NVIDIA GeForce RTX 5060 Ti compute capability 12.0.
- **Cache inspection:** Runtime configuration had vLLM prefix caching enabled, LightNav ViT
  caching enabled, eager mode enabled, 2 GiB explicit KV cache, multimodal embeddings enabled,
  and chunked prefill disabled. Installed source shows `agent.reset()` clears both direct and
  session ViT caches; each trial repopulated 23 direct-cache entries. The embedding patch uses
  a fresh monotonic-derived video hash per request, so there is no cross-trial LightNav ViT or
  vLLM image-embedding reuse. Text-only prefix reuse remains possible, but hit metrics were not
  enabled. No cache setting was changed and no clear identical-image full-request cache hit
  evidence justified a separate varied-image benchmark.
- **Measurements:** Model/engine build was `10206.579 ms`; timed build-plus-nine-trial wall
  time was `15447.836 ms`. First host/reported latency was `652.138/380.212 ms`. Warm host raw
  values were `[556.915281, 556.759021, 557.341634, 556.714522, 556.838378, 557.131075,
  556.581784, 556.469596] ms`; mean `556.844 ms`, median `556.799 ms`, population standard
  deviation `0.266 ms`, min/max `556.470/557.342 ms`, and linear p90/p95
  `557.194/557.268 ms`. First/warm-median ratio was `1.1712`. Warm LightNav-reported median
  was `305.295 ms`; the distinction remains explicit because reported `llm_ms` excludes
  preprocessing and ViT.
- **Output validation:** All nine calls produced finite float32 `(10, 3)` arrays and separate
  `actions/trial_NNN.npy` plus `raw_text/trial_NNN.txt` files. Output was deterministic across
  trials, with first row `[0.15485105, 0.00006702, -0.00419930]` and last row
  `[0.71874577, -0.00634444, -0.00297201]`. `trials.csv`, `trials.json`, `metadata.json`, and
  `summary.json` passed strict JSON/artifact validation; generated data remains ignored and
  uncommitted.
- **Interpretation:** Warm median / validated Stage 0-C `2.300000 s` execution was `0.2421`
  (execution was about `4.13x` longer), placing this observed stack in Case C and supporting an
  EXP-01B persistent-preloaded asynchronous prototype. The prior Stage 0-C `49561.998 ms`
  one-shot latency was about 89 times the warm median and is not steady-state latency. Its
  exact cold/JIT/cache composition was not isolated here, so future online work must explicitly
  warm the persistent process and keep cold start separate. The 2.3 s value belongs to one
  0.72 m controller run; it is not a generic LightNav horizon because the action rows have no
  intrinsic timestamps.
- **Major files:** `README.md`, `configs/exp01a_lightnav_latency.yaml`,
  `docs/EXP_01A_LIGHTNAV_LATENCY.md`, `docs/WORK_LOG.md`,
  `src/reconciliation/latency_benchmark.py`,
  `scripts/lightnav/{benchmark_repeated_inference.py,run_exp01a_lightnav_latency.sh}`,
  `scripts/summarize_exp01a_latency.py`, and `tests/test_latency_benchmark.py`.
- **Commands:** Initial research/LightNav Git status, branch, remote, and HEAD checks; local
  LightNav/vLLM cache and blocking-call source inspection; GPU/process check; focused and full
  pytest; compileall; shell syntax and diff checks; actual one-load RTX benchmark; independent
  saved-output validation; per-trial NPY SHA/action/cache inspection; explicit Git
  diff/status/staging/commit/push checks.
- **Verification:** Focused suite `12 passed`; full research suite `99 passed`, including all
  existing EXP-01 and Stage 0-A/B/C/controller/viewer tests. Compileall, `bash -n`,
  `git diff --check`, the exact-one-build source check, strict JSON parsing, and final output
  validation all passed.
- **Issues and limitations:** Eight warm trials provide weak tail-percentile evidence and only
  one scene/instruction/input history was tested. Prefix cache hits were not directly exposed.
  The benchmark did not reproduce a clean-host cold start, vary requests, run Isaac, assess
  navigation quality, or settle EXP-01B observation/ready/control/OLD-exhaustion timing. The
  user's pre-existing controller-camera and Stage 0-C 0.45x playback config changes were
  preserved and excluded from staging. No system Python, ROS, CUDA, driver, Isaac installation,
  external LightNav source, checkpoint, or generated data was modified or committed.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-04 00:58:23 KST (+0900) — EXP-01B online LightNav raw-switch discontinuity

- **Purpose:** Measure whether warmed, stateful LightNav produces a raw OLD→NEW boundary
  mismatch when Jackal continues executing OLD during asynchronous NEW inference. This task
  measured problem existence only; it added no reconciliation or new controller.
- **Implemented:** Added versioned standard-library Unix-socket IPC with strict RGB/action
  contracts; a LightNav Python 3.11 server that builds once, warms once, resets once per live
  episode, and preserves history between OLD and NEW; and an isolated Isaac client that primes
  64 stationary frames, runs the existing Stage 0-B follower and official differential
  controller, appends live 4 Hz RGB, requests NEW asynchronously, and continues OLD physics and
  control. NEW is anchored at the observation pose. Waypoint-time-free metrics are computed
  before a separate controller-only ready-pose prepend. Absolute simulation-time deadline
  pacing recovers render stalls. Invalid attempts remain saved; a bounded protocol gathers
  three timing-valid transitions. Added validators, aggregation, 18 focused tests, README
  commands, and full experiment documentation.
- **Actual run:** Validated run `exp01b-20260903T155402Z` used LightNav SHA
  `a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0, checkpoint revision
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, vLLM 0.19.1 `vllm_local`, and RTX 5060 Ti.
  Model load was 10,160.357 ms; warm-up was 622.080 ms host / 361.828 ms reported. The
  coexistence gate used 13,270 MiB with 2,559 MiB free and no OOM. EXP-01A's 0.90 utilization /
  2 GiB KV configuration was explicitly changed to 0.65 / 1 GiB for coexistence; installed
  vLLM reports explicit KV bytes supersede utilization.
- **Timing and motion:** Four attempts yielded three valid transitions; attempt 0 was retained
  but excluded for RTF 0.654. Valid client latencies were
  `[421.654, 423.228, 423.663] ms`, server prediction latencies
  `[421.419, 422.951, 423.401] ms`, simulation latencies all 0.433333 s, and RTF values
  `[1.027700, 1.023876, 1.022826]`. Jackal moved
  `[0.158705, 0.161989, 0.162930] m`; OLD progress was `1 -> 2`, and all 26 in-flight timeline
  rows per valid trial carried nonzero OLD commands.
- **Raw-switch result:** Valid translation gaps were
  `[0.003854, 0.161989, 0.008079] m`, yaw gaps
  `[0.004164, 0.000043, 0.004113] rad`, translation-motion jumps
  `[0.100789, 0.039447, 0.098742] m`, and yaw-motion jumps
  `[0.006785, 0.000001, 0.006770] rad`. One of three exceeded the descriptive 0.05 m pose-gap
  threshold and two exceeded the translation-motion threshold; no yaw threshold was exceeded.
  Trial 2's real NEW response was an all-zero stop chunk; it remains unchanged and produced a
  gap equal to inference-window movement. This is positive but narrow translational problem-
  existence evidence, not evidence that a reconciliation method works.
- **Major files:** `README.md`, `configs/exp01b_online_raw_switch.yaml`,
  `docs/EXP_01B_ONLINE_RAW_SWITCH.md`, `docs/WORK_LOG.md`,
  `src/reconciliation/{online_ipc.py,online_switch.py}`,
  `scripts/lightnav/serve_online_lightnav.py`,
  `scripts/isaac/{exp01b_online_raw_switch.py,run_exp01b_online_raw_switch.sh}`,
  `scripts/summarize_exp01b.py`, and `tests/{test_online_ipc.py,test_online_switch.py}`.
- **Commands:** Initial Git and external-LightNav state/environment inspection; installed
  history/decoder/backend source checks; focused/full pytest; compileall; shell and whitespace
  checks; three real concurrent engineering runs; GPU snapshots; strict final artifact, raw
  hash, action/text, event, timeline, pose/progress, and aggregate validation; explicit Git
  diff/status/staging/commit/push checks.
- **Verification:** Focused tests: `18 passed`; full suite: `117 passed`. Compileall, `bash -n`,
  `git diff --check`, and strict reconstruction/validation of all four final trial directories
  passed. Raw arrays were finite `(10, 3)` and stayed separate from derived and controller
  artifacts. Generated data is ignored and uncommitted.
- **Issues and limitations:** Two development runs had no valid trials and are not evidence.
  The final run's first attempt also failed its RTF gate. Nominally repeated episodes produced
  different chunks, including stop, without cause attribution. Evidence is limited to three
  valid transitions in one straight corridor and says nothing about navigation quality, real
  robots, or reconciliation effectiveness. Existing user edits in
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` were preserved and excluded from staging.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-04 14:40:19 KST (+0900) — EXP-02 oracle SE(2) graph reconciliation

- **Purpose:** Validate the reconciliation mechanism itself using a synthetic known-answer gate
  and then one valid non-stop real EXP-01B LightNav pair with manually verified oracle
  correspondences. No learned detector, GTSAM, online Isaac execution, or new inference was
  added.
- **Implemented:** Added numerically stable SE(2) Exp/Log and right-local retraction; a
  fixed-OLD/fixed-boundary, arbitrary-horizon editable-NEW graph; identity boundary,
  `Z_ij = O_i^-1 X_j` oracle correspondence, and full-chain NEW relative-motion factors;
  configurable residual scales/weights; and a NumPy damped Gauss–Newton/LM solver with central
  numerical Jacobians, cost-decrease acceptance, finite/singular failure checks, and complete
  histories. Added oracle validation/source hashes, graph/correction metrics, immutable output
  validation, plots, correction CSVs, full/no-correspondence/no-NEW-motion ablations, the fixed
  correspondence/NEW-motion ratio grid `[0.25, 1.0, 4.0]`, 22 focused tests, README commands,
  and the experiment document.
- **Synthetic gate:** Used a nine-pose curved GT with local step `[0.22, 0, 0.045]`, seven-pose
  NEW suffix, global left perturbation `[0.20, 0.15, 0.10]`, boundary equal to GT NEW 0, and
  identity pairs `(2,0),(3,1),(4,2)`. Cost fell `62.8351106 -> 1.8382e-26` in three iterations.
  Pose recovery max was `2.9072e-14 m / 1.7319e-14 rad`; correspondence translation/yaw RMS
  fell `0.290306/0.100000` to `2.3650e-15/2.7733e-15`; NEW-motion distortion was numerical
  zero. Non-corresponded downstream poses recovered and the boundary input did not change.
- **Real pair and oracle:** Selected valid, non-stop EXP-01B
  `exp01b-20260903T155402Z/trial_001` because its `0.100788914 m` translation-motion jump was
  the largest eligible value. OLD/NEW were `(10,3)`. Raw hashes remained
  `8edd60bf794d067dba1820f557e39388a8c3af352db7443774108f550219cf06` and
  `072c29e991bea91fad89d00ad458c00697960dc684bbdfc68cf105e3ace362fe`.
  The ordered identity oracle pairs `(OLD 1,NEW 0)`, `(2,1)`, `(3,2)` represent the same three
  consecutive corridor poses; their raw translation differences were
  `0.019257, 0.008998, 0.012177 m`.
- **Full result:** Balanced weights `B/C/N=4/1/1` converged in three iterations with cost
  `0.100380521 -> 0.050148214`. Boundary translation/yaw changed
  `0.00385449/0.00416388 -> 0.00560869/0.00086345`; translation/yaw motion jump changed
  `0.10078891/0.00678489 -> 0.09585413/0.00312881`. Correspondence translation/yaw RMS
  improved `0.01414317/0.00957319 -> 0.01031208/0.00150561`. NEW-motion distortion was
  `0.00186831 m / 0.00130968 rad` RMS, max `0.00493834 m / 0.00365608 rad`.
- **Propagation/ablation:** Full-graph per-pose translation correction increased smoothly from
  `0.001756 m` at index 0 to about `0.01046 m` at indices 7–9; yaw stabilized near
  `0.010122 rad`, so correction reached every non-corresponded pose without an end cutoff.
  No-correspondence rigidly satisfied the boundary and preserved motion but worsened
  correspondence RMS and left motion jumps raw. No-NEW-motion moved only oracle-connected
  indices 0–2; correction dropped to zero at index 3 and created a
  `0.012697 m / 0.011561 rad` edge kink.
- **Sensitivity:** Ratios `0.25/1/4` all converged in three iterations. Translation boundary
  gaps were `0.002230/0.005609/0.011722 m`; correspondence translation RMS values were
  `0.013504/0.010312/0.006587 m`; translation-motion jumps were
  `0.097087/0.095854/0.099448 m`. Higher correspondence influence improved alignment but
  worsened translation boundary continuity. No post-result weights were added or selected.
- **Interpretation:** Synthetic mathematics and full-chain propagation are valid. On the real
  pair, correspondence and yaw terms improved with millimeter/milliradian distortion, but the
  already-small translation pose gap worsened and the dominant translation-motion jump improved
  only about 4.9%. The minimal pose-only boundary factor therefore gives mixed mechanism
  evidence and exposes a formulation limitation; a boundary-transition motion treatment should
  be scoped before freezing weights for unseen-pair evaluation.
- **Major files:** `README.md`, `configs/exp02_oracle_graph.yaml`,
  `configs/oracles/exp02_lightnav_development_pair.yaml`, `docs/EXP_02_ORACLE_GRAPH.md`,
  `docs/WORK_LOG.md`, `src/reconciliation/{se2.py,se2_graph.py,graph_optimizer.py,
  graph_metrics.py,oracle_correspondence.py,exp02.py}`, `scripts/{run_exp02_oracle_graph.py,
  summarize_exp02.py}`, and `tests/{test_se2_lie.py,test_se2_graph.py,
  test_oracle_correspondence.py}`.
- **Commands and verification:** Initial Git/base/data inspection; EXP-01B pair/hash/pose
  inspection; focused pytest; actual offline synthetic and real run; strict summary/source-hash
  validation; PNG visual inspection; full pytest; compileall; ignore/diff/status/staging/commit/
  push checks. Focused suite: `22 passed`; full suite: `139 passed`. Validated immutable output
  `data/exp02/exp02-20260904T053813Z` contains synthetic `(7,3)`, real `(10,3)`, three
  ablations, three predefined sensitivities, and matching source hashes. Generated data remains
  ignored and uncommitted.
- **Issues and limitations:** The first focused-test command inherited ROS 2 `PYTHONPATH`, so
  pytest auto-loaded `launch_testing` and failed because `lark` was absent. No package or system
  environment was changed; rerunning with subprocess-local `PYTHONPATH` removal and pytest
  plugin autoload disabled passed. Real conclusions are limited to one manually annotated
  straight-corridor pair. Existing user edits in the two Stage 0 config files remain preserved
  and excluded from staging.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.
