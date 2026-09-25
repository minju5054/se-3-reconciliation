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

## 2026-09-05 01:21:43 KST (+0900) — EXP-02A k-conditioned SE(2) transition reconciliation

- **Purpose:** Replace the obsolete collaborator-facing `Z_ij` correspondence assumption with
  the official OLD + FRESH + `SpatialEntryContext(k, evidence)` backend interface, and test
  whether a given spatial entry can be reconciled with incoming OLD motion without erasing k
  or deforming the selected FRESH suffix. The previous EXP-02 implementation and outputs remain
  intact as a graph-machinery pilot.
- **Implemented:** Added immutable arbitrary-length transition inputs, exact `FRESH[k:]`
  extraction, nested JSON-safe context/evidence round trips, explicit `k=N-1` rejection, and
  separate planned/measured OLD-tail semantics. Generalized only the numerical optimizer core
  behind its backward-compatible EXP-02 wrapper. Added pose-anchor, entry-preservation, and
  incoming-motion-aware graphs with the complete suffix relative-motion chain. The incoming
  factor aligns a raw-entry-radius-normalized B→entry vector with measured incoming direction
  and compares yaw increments; it never equates entry distance with an OLD sample. Evidence is
  structurally absent from cost and weights. Added transition/deformation/downstream/inter-k
  metrics, immutable artifacts, strict validation, plots, runner/summarizer, config, 17 tests,
  README instructions, and full experiment documentation.
- **Synthetic result:** For fixed k=`0/3/6`, incoming-aware direction jumps changed
  `0.588003→0.024987`, `0.282898→0.094924`, and `0.298377→0.179070 rad`; yaw-motion jumps were
  approximately halved. Entry-separation retention was `0.964–0.998`, while pose anchoring
  collapsed it to `1.3e-13–4.2e-13`. FRESH edge distortion remained numerical and correction
  reached every suffix pose. Entry displacement (`0.120–0.146 m`) and weak reduction of the
  larger translation-magnitude jumps remain explicit trade-offs.
- **Real LightNav pilot:** Final ignored immutable run `exp02a-20260904T162149Z` referenced
  timing-valid, non-stop EXP-01B `exp01b-20260903T155402Z/trial_001` with predeclared k=`0/3/6`.
  Raw hashes remained `8edd60bf...cf06` (OLD) and `072c29e...2fe` (FRESH). Incoming-aware
  direction jumps changed `3.121188→3.03e-7`, `0.009687→0.001680`, and
  `0.011247→0.002669 rad`; yaw-motion jumps approximately halved. Entry displacement was
  `0.007708/0.003668/0.004785 m`, edge distortion was numerical, and inter-k entry retention
  was `0.9833–0.99995`. The large k=3/6 translation-magnitude jumps were essentially unchanged,
  so the formulation is not accepted as a complete transition solution.
- **Validation:** Focused graph/SE(2)/interface suite: `36 passed`. Full research suite:
  `156 passed`. `compileall`, strict JSON/output reconstruction, source hash validation,
  `git diff --check`, generated-data ignore verification, and visual inspection of synthetic
  and real incoming-aware plus pose-anchor inter-k PNGs passed. The first diagnostic run exposed
  a sine-only direction residual's antiparallel ambiguity at real k=0; the final formulation
  replaced it with the explicit two-component normalized transition vector and reran all tests
  and experiments. Failed/diagnostic ignored directories were not overwritten.
- **Major files:** `README.md`, `configs/exp02a_spatial_entry.yaml`,
  `docs/{EXP_02_ORACLE_GRAPH.md,EXP_02A_SPATIAL_ENTRY_RECONCILIATION.md,WORK_LOG.md}`,
  `src/reconciliation/{spatial_entry.py,transition_graph.py,transition_metrics.py,exp02a.py,
  graph_optimizer.py}`, `scripts/{run_exp02a_spatial_entry.py,summarize_exp02a.py}`, and
  `tests/{test_spatial_entry.py,test_transition_graph.py}`.
- **Commands:** Required Git/branch/remote/base/history and repository inspection; immutable
  EXP-01B numeric/hash inspection; focused/full pytest; compileall; actual offline synthetic and
  real-pilot runs; strict summarizer; plot inspection; ignore/diff/status checks; explicit
  staging/cached-diff/commit/push workflow.
- **Issues and limitations:** Evidence features and WHICH-k correctness are unvalidated and do
  not influence optimization. The real result is one manual-k straight-corridor backend pilot
  from only three valid EXP-01B transitions, not a representative dataset. Pose anchoring
  destroys k semantics; entry preservation alone changes nothing; incoming motion improves
  direction/yaw but not the spatial transition-length mismatch. No Isaac/LightNav execution,
  online optimization, navigation-quality, obstacle, NavDP, selector, generalization, or
  real-robot claim is made. Existing user changes to the two Stage 0 config files were preserved
  and excluded from staging. Generated EXP-02A data and previous EXP-02/EXP-01B data remain
  ignored and uncommitted.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-05 09:32:41 KST (+0900) — EXP-01B expanded online characterization

- **Purpose:** Extend the immutable N=3 EXP-01B problem-existence pilot with a predeclared,
  controlled four-condition online cohort while preserving the persistent warmed LightNav,
  asynchronous OLD execution, observation-time FRESH anchoring, measured-ready, and raw row-0
  semantics. This remains characterization; no reconciliation or EXP-02A code is used.
- **Implemented:** Added the frozen A nominal `[0,0,0]/0.50 s`, B left
  `[0,+0.15,+0.10]/0.50 s`, C right `[0,-0.15,-0.10]/0.50 s`, and D later
  `[0,0,0]/0.75 s` conditions with target six valid and maximum ten attempts each. Extended the
  existing Isaac episode runner without changing its original-config branch. Added explicit
  classifications, FRESH STOP preservation, actual-vs-planned geometry descriptors, in-flight
  OLD-command activity, all/STOP/non-STOP statistics and threshold indicators, strict immutable
  validation, CSV summaries, deterministic representative selection, and eight review plots.
  Added a one-command persistent LightNav + headless Isaac + research summarizer launcher.
- **Actual collection:** Smoke `exp01b-extension-smoke-20260905T000000Z` passed with one valid
  real STOP transition (RTF 1.023, 0.1645 m robot motion). Primary immutable run
  `exp01b-extension-20260905T002500Z` executed the finite maximum 40 attempts with one model
  build, one warm-up, 40 episode resets and 80 live predictions. It yielded A/B/C/D valid counts
  `5/4/3/4` (16 total), including five FRESH STOP outputs; 21 attempts exhausted zero-action
  OLD and three failed the unchanged RTF gate. No timeout, other protocol failure, or OOM
  occurred. The predeclared N=24 target was not met and was not rescued by changing conditions,
  gates, or attempt limits.
- **Observed result:** Across all valid transitions, translation gap mean/median was
  `0.08065/0.02395 m` and translation-motion jump `0.08349/0.10145 m`; threshold exceedance was
  `7/16` and `11/16`. Excluding STOP, translation gap was `0.02908/0.01499 m` with `2/11`
  exceedances, while translation-motion jump was `0.10640/0.10699 m` with `11/11`
  exceedances. All five FRESH STOP outputs exceeded translation gap but not motion jump. No yaw
  pose or motion metric exceeded 0.05 rad. Actual incoming motion was `0.0281–0.0428 m`, FRESH
  first motion `0.1392–0.1511 m`, and non-STOP tangent disagreement only
  `0.00017–0.00261 rad`; B/C therefore did not realize meaningful recovery steering despite
  their controlled initial offsets.
- **Runtime/GPU:** LightNav SHA `a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0,
  checkpoint revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, vLLM 0.19.1, prefix
  caching enabled, GPU utilization config 0.65 and 1 GiB explicit KV cache. RTX 5060 Ti memory
  was 11,526 MiB after LightNav warm-up, 13,500 MiB after concurrent Isaac/camera load, and
  13,571 MiB at the final snapshot; no OOM. Host request/response latency among valid attempts
  was 0.4315–0.7028 s, RTF 1.0076–1.0767, and every valid in-flight timeline sample retained a
  nonzero OLD command.
- **Validation:** Focused extension/EXP-01B/IPC suite: `30 passed`. Full research suite:
  `168 passed`. `compileall`, strict 40-attempt reconstruction/hashes, summary/CSV consistency,
  original EXP-01B top-level hash recheck, generated-data ignore check, `git diff --check`, and
  visual inspection of translation/yaw/scatter and corrected full observation-to-ready
  representative overlays passed. The first pytest invocation was blocked only by ROS
  `launch_testing` plugin auto-loading without `lark`; no package was installed, and all tests
  passed with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
- **Major files:** `README.md`, `configs/exp01b_extension.yaml`,
  `docs/{EXP_01B_EXTENSION.md,WORK_LOG.md}`, `src/reconciliation/exp01b_extension.py`,
  `scripts/isaac/{exp01b_online_raw_switch.py,run_exp01b_extension.sh}`,
  `scripts/summarize_exp01b_extension.py`, and `tests/test_exp01b_extension.py`.
- **Commands:** Required Git/branch/remote/base/history and repository/doc/config/source/test
  inspection; original artifact hash/inventory inspection; focused/full pytest; compileall;
  actual LightNav+Isaac smoke and 40-attempt primary runs; strict saved-output validation;
  numeric STOP/geometry/threshold inspection; plot rendering/visual review; diff/status/ignore
  checks; explicit staging/cached-diff/commit/push workflow.
- **Issues and limitations:** Reviewed expected SHA `cf773d2` differed from actual clean HEAD
  `0f22b36` only by the already reviewed/pushed EXP-02A commit, so this work continued on top
  without rewriting history. The target shortfall and lack of realized tangent diversity make
  occurrence-frequency and steering-geometry claims insufficient. Generated data remains
  ignored. The original EXP-01B hashes and outputs were unchanged. Existing user changes to the
  Stage 0-B camera and Stage 0-C playback factor were preserved and excluded from staging.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and the final staged-diff review.

## 2026-09-06T21:45:34+09:00 — Redesigned EXP-01B controlled latency × geometry

- **Purpose:** Replace the untimed waypoint-spacing comparison as the primary discontinuity
  signal with canonical Stage 0-B command changes, then test natural versus +0.5 s effective
  latency across prequalified straight, turn, and route-change scenes. This is raw-switch
  characterization only; no EXP-02A, graph, `k`, smoothing, or correction was added.
- **Implementation:** Added exact signed/absolute `delta_v` and `delta_omega`, last-three OLD /
  first-three FRESH command windows, discrete command-slew labelling, near-zero-safe geometry
  descriptors, exact 2×3 protocol validation, separate model-ready/FRESH-usable events, finite
  attempt caps, STOP/failure classification, immutable raw hashing, strict reconstruction,
  deterministic summaries, and 18 review plots. Reused the existing online runner, IPC,
  Stage 0-B follower, and single-build/single-warmup LightNav server.
- **Qualification:** Retained all diagnostics. G0 qualified at tangent `0.00273 rad`. Initial
  G1/G2 and two further G2 prompts were rejected as straight. Revised G1 qualified by yaw
  progression `0.32728 rad`/lateral `0.07977 m`; final G2 qualified at tangent `0.68042 rad`, yaw
  progression `0.82458 rad`, and lateral `1.06728 m`. Config was frozen before primary result
  inspection. G0-natural and G2-delayed smoke passed; G2 measured `0.500000026 sim s` added delay
  with OLD-command activity fraction `1.0`.
- **Actual run:** Ignored immutable
  `data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/` obtained all 30 planned
  moving transitions in 37 attempts (`7/9/6/5/5/5` per cell): three FRESH STOP, three OLD
  exhausted, one RTF invalid, no timeout/other failure/OOM. One model build, one warm-up, 37
  episode resets, and 74 live OLD/FRESH predictions were used.
- **Results:** Natural→delayed effective latency mean was `0.45889→0.97000 s`, robot travel
  `0.16506→0.33305 m`, with comparable model wall latency `0.44519→0.44247 s` and travel
  correlation `r=0.926`. G0 mean/median `|delta_v|` increased
  `0.01947/0.02638→0.11578/0.12745 m/s`; G1 decreased
  `0.09436/0.01328→0.07393/0.00904`; G2 was outlier-sensitive
  `0.00747/0.00781→0.06895/0.00489`. Median `|delta_omega|` changed G0
  `0.01417→0.03178`, G1 `0.02646→0.03357`, G2 `0.00118→0.00197 rad/s`. One actually divergent
  G1 sample per latency cell dominated means (`0.512`, `1.405 rad/s`). Effective-latency
  correlations with `|delta_v|/|delta_omega|` were `0.241/0.117`; tangent disagreement versus
  `|delta_omega|` was `0.673` (descriptive only).
- **Geometry/boundary:** Translation pose-gap means increased with delay in G0/G1/G2:
  `0.0164→0.1736`, `0.0307→0.1674`, `0.0156→0.2046 m`. Pose gap versus `|delta_v|` was only
  `r=0.087`. Primary G1 retained curved futures but most entries were parallel; G2 did not
  reproduce its qualified route change (primary tangent `0.00036–0.00146 rad`). This and repeated
  discrete outputs are limitations, not removed outliers. Spatial step mismatch is not velocity.
- **Runtime:** Isaac Sim 6.0.1; LightNav SHA `a645828d81a8439651172197ca80a75dc1377977`, package
  0.1.0, checkpoint revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, vLLM 0.19.1,
  prefix caching enabled, GPU config 0.65/1 GiB KV. RTX 5060 Ti snapshot was 13,589 MiB used.
- **Validation:** Targeted suite `37 passed`; full suite `184 passed`; compileall, shell syntax,
  strict 37-attempt/30-moving reconstruction, plot visual review, ignore rules, and original
  immutable hashes passed. Prior summary/metadata hashes remained `4908f078...bf90`,
  `b3a27447...ec8c`, `ed34450...e71f`, and `1c75ad8e...843`.
- **Major files:** `README.md`, `configs/exp01b_controlled_latency.yaml`,
  `docs/{EXP_01B_EXTENSION.md,EXP_01B_REDESIGNED_CONTROLLED_LATENCY.md,WORK_LOG.md}`,
  `src/reconciliation/{controller_switch_metrics.py,exp01b_controlled_latency.py}`,
  the reused Isaac/LightNav runners, new launcher/summarizer, and two test modules.
- **Commands:** Git/repository/source inspection; targeted/full pytest with ROS plugin auto-load
  disabled; compileall/shell syntax; retained qualification cohorts; two smoke and one 37-attempt
  primary run; strict validation, hashes, numeric contrasts, plots, diff/stage/commit/push flow.
- **Issues:** Evidence is mixed: delay robustly changed travel/pose gap, but immediate commands
  were not uniformly larger and the lookahead follower absorbed most switches. G2 qualification
  was not repeatable. Generated data remain ignored. Existing user changes to Stage 0-B camera
  and Stage 0-C playback factor were preserved and excluded from staging.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and final staged-diff review.

## 2026-09-07T00:00:25+09:00 — EXP-02B controller-aware reconciliation evaluation

- **Purpose:** Evaluate whether the unchanged EXP-02A k-conditioned geometric graph reduces
  immediate Stage 0-B controller-command discontinuity on real, immutable LightNav transitions,
  while keeping same-k FRESH intent; compare it with raw FRESH[0:], raw FRESH[k:], the diagnostic
  pose anchor, and a graph-independent analytic rigid SE(2) baseline. This remains a
  manual/oracle-k three-case development/stress evaluation, not end-to-end LightNav evidence.
- **Implementation:** Added deterministic maximum-delta-v, maximum-delta-omega, and benign-L1
  source selection with full-path tie breaks and seven-file SHA-256 verification per case. Added
  an analytic rigid suffix transform, unchanged EXP-02A graph reuse, complete geometric/intent
  metrics, inter-k retention, controller improvement helpers, strict output reconstruction, and
  immutable candidate/result writing. Added a headless Isaac runner that replays saved OLD
  commands, enforces a frozen B/command comparability gate, resets every accepted post-switch
  branch to the exact saved B, reuses the Stage 0-B `TrajectoryFollower` and official
  `DifferentialController`, and stores actual trajectory plus controller telemetry. Added summary
  generation and nine static review figures.
- **Frozen cases/protocol:** Case A was
  `primary/G1_turn/L0_natural/attempt_003` (`delta_v=0.416563`,
  `delta_omega=0.511720`); Case B was `primary/G1_turn/L1_added_050/attempt_004`
  (`0.332694`, `1.405038`); Case C was
  `primary/G2_route_change/L1_added_050/attempt_002` (`0.004422`, `0.001972`). Evaluated only
  k=`0/3/6` and M0–M4. Three qualification replays reproduced B and the pre-switch command
  exactly at stored precision; frozen tolerances are 0.005 m, 0.01 rad, and `1e-12` command.
- **Actual run:** Final ignored immutable run
  `data/exp02b/exp02b-controller-aware-20260906T150400Z/` generated and executed all 45 branches;
  45/45 passed comparability. Isaac Sim reported `6.0.1-rc.7+release.42383.32955d8d.gl`, Jackal
  four wheel DOFs were recorded, and wheel radius/separation were runtime-derived as
  `0.0979999974/0.375589997 m`. Strict validation reported source hashes matched, execution and
  summary present, and nine plots. Turning, command trace, benign, and trade-off plots were
  visually inspected. No LightNav inference or source experiment was rerun.
- **Observed result:** The graph did not consistently improve controller smoothness. Case A k=0
  reduced delta-v only `0.4166→0.4137` while delta-omega rose `0.5117→1.0267`; k=3/6 were worse.
  Case B k=0 reduced delta-omega `1.4050→1.2747` but increased delta-v, while k=3/6 increased
  delta-omega to `2.2092/2.4170`. In benign Case C k=0, delta-v grew
  `0.0044→0.2410`. Turn sign and yaw progression were preserved, and internal graph edge
  deformation was at most numerical (`1.2e-11`), but endpoint/lateral changes and benign-case
  modification were material. Rigid and graph results were geometrically similar; neither
  consistently improved controller metrics, so the current factors provide no evidence that a
  nonlinear graph is necessary. Pose anchoring again collapsed inter-k distinction.
- **Validation:** Targeted rigid/EXP-02B/transition/controller/source suite: `37 passed`. Full
  research suite: `197 passed`. Python compileall, shell syntax, source/output reconstruction,
  strict JSON/NPY/CSV validation, generated-data ignore check, and `git diff --check` passed.
  The first provenance-enhanced diagnostic output `...T150300Z` completed its branch files but
  intentionally remained incomplete because the new version recorder exposed that the launcher
  had not exported `ISAAC_SIM_ROOT`; the launcher was fixed, and the immutable final T150400Z run
  was created rather than overwriting it.
- **Major files:** `README.md`, `configs/exp02b_controller_aware.yaml`,
  `docs/{EXP_02B_CONTROLLER_AWARE_RECONCILIATION.md,WORK_LOG.md}`,
  `src/reconciliation/{exp02b.py,rigid_reconciliation.py}`,
  `scripts/{run_exp02b_controller_aware.py,summarize_exp02b.py}`,
  `scripts/isaac/{exp02b_branch_execution.py,run_exp02b_branch_execution.sh}`, and
  `tests/{test_exp02b.py,test_rigid_reconciliation.py}`.
- **Commands:** Required Git/base/history/remote and repository/doc/config/source inspection;
  frozen cohort selection/hash inspection; targeted/full pytest with external plugin autoload
  disabled; compileall and shell syntax; offline generation; three-replay qualification; turning
  smoke; full 45-branch Isaac runs; strict summarization; numerical table extraction; plot visual
  review; ignore/diff/status checks; explicit stage/cached-diff/commit/push workflow.
- **Issues and limitations:** The chosen cases are outcome-selected stress/development cases,
  k is manual, and no selector, controller residual, LightNav rerun, navigation benchmark,
  obstacle metric, held-out evaluation, or generalization claim exists. M0 and same-k M1 answer
  different questions. Existing user changes to the Stage 0-B camera and Stage 0-C playback
  factor were preserved and excluded from staging. All generated EXP-02B data remain ignored.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after this entry and final staged-diff review.

## 2026-09-07T18:04:46+09:00 — EXP-02B GUI OLD replay execution diagnosis

- **Purpose and scope:** Answer only why planned OLD and actual Jackal motion differ in EXP-02B,
  separating nearest-reference geometry, body command versus measured motion, wheel target
  versus measured joint motion, and replay/reset display artifacts. No optimization objective,
  graph residual/weight, gate, selector, k interface, correspondence, benchmark, or LightNav
  implementation changed. Starting local and fetched `origin/main` were both
  `8a0563e26df3fb64d9cded96c2fd95d239c25137`.
- **Implementation:** Added a diagnostic-only GUI runner/config/launcher that reads the frozen
  EXP-02B candidate, verifies source/candidate hashes, reproduces the existing OLD replay and
  comparability gate, visibly pauses before reset, explicitly resets to saved B with the last
  OLD wheel target, and executes one existing candidate. Reused/refactored the Stage 0
  DebugDraw geometry into a small helper. The viewport separates planned OLD, OLD actual,
  full raw FRESH, selected FRESH suffix, current candidate, and post-reset actual; saved,
  reproduced, reset, observation, selected-entry, and candidate-entry poses remain separate
  logical markers. LiDAR display clutter is disabled only in the diagnostic viewport.
- **Telemetry/metrics:** Added physics-step OLD/post pose, command, canonical four-wheel target,
  directly measured joint velocity, and finite-difference body-motion logging. Added
  nearest-OLD-polyline distance/yaw descriptors explicitly marked non-time-aligned, body and
  per-wheel RMSE, immutable six-file output, strict validation, non-causal multi-label status,
  source observation/model-ready/FRESH-usable timestamps, diagnostic phase timestamps, world
  frame/yaw/units, and overwrite/non-finite guards. Outputs are ignored under
  `data/exp02b_gui_diagnosis/<run_id>/`.
- **Representative observation:** Actual GUI run
  `exp02b-gui-case-high-omega-k3-raw-k-20260907T-final` completed
  `case_high_delta_omega / k=3 / raw_k` with 89 OLD and 121 post samples. OLD nearest-polyline
  mean/RMS/max/final were `0.05290/0.06460/0.15243/0.04661 m`. Body v/omega command-versus-
  measured RMSE were `0.12133 m/s` and `0.63989 rad/s`; mean commanded/measured omega was
  `0.75534/0.13920 rad/s`. Aggregate wheel RMSE was `0.84874 rad/s` (FL/FR/RL/RR
  `1.04616/0.83351/0.75309/0.72462`). `B_saved` and `B_reproduced` matched exactly; reset
  difference was `0 m / 1.51e-8 rad`. Raw-k post-switch delta-v/delta-omega remained the frozen
  `0.54023 m/s / 1.42799 rad/s` result.
- **GUI smoke:** Jackal moved during both OLD replay and post-switch execution. All six path
  layers and marker states were visually inspected in the Isaac Sim 6.0.1 viewport; terminal
  phase/live telemetry updated. PRE_RESET and exact-reset holds kept simulation time fixed at
  `2.500000130 s`, and OLD cyan/post green histories remained split. The final no-hold GUI run
  exited successfully with strict output validation. An untouched original runner GUI
  qualification was also used while isolating initialization parity and reproduced the frozen
  boundary exactly.
- **Interpretation:** Reference deviation, body execution mismatch, and wheel tracking mismatch
  are all visible under documented diagnostic thresholds. This representative replay rules out
  a failed B reproduction and reset-history mixing as explanations for the displayed OLD gap,
  but it does not isolate skid-steer/contact, controller behavior, wheel tracking, or their
  interaction as a unique causal root cause.
- **Validation:** New/related targeted suite `20 passed`; full suite `203 passed`. Python
  compileall, launcher `bash -n`, strict output reconstruction, generated-data ignore check,
  and `git diff --check` passed. Frozen recursive manifests remained
  `d6a8176f...54533` for EXP-02B and `a3ec7b83...efaf` for its EXP-01B source. The existing
  user camera/playback config changes were preserved and excluded from staging.
- **Major files:** `README.md`, `configs/exp02b_gui_diagnosis.yaml`,
  `docs/{EXP_02B_CONTROLLER_AWARE_RECONCILIATION.md,EXP_02B_GUI_DIAGNOSIS.md,WORK_LOG.md}`,
  `src/reconciliation/exp02b_diagnosis.py`,
  `scripts/isaac/{debug_draw_trajectories.py,exp02b_gui_diagnosis.py,lightnav_playback_single_chunk.py,run_exp02b_gui_diagnosis.sh}`,
  and `tests/test_exp02b_diagnosis.py`.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final status/diff/staged-diff review.

## 2026-09-07T19:54:01+09:00 — Stage 0-D Jackal execution-layer calibration and validation

- **Purpose and scope:** Isolate the large EXP-02B OLD yaw-execution mismatch from
  reconciliation, characterize the official Jackal command/wheel/body chain, test only the
  smallest predeclared feedforward/PI correction, and gate it on calibration-independent
  execution experiments. No LightNav output, trajectory, graph factor/weight, candidate,
  physical/contact parameter, or historical result changed. Starting local and fetched
  `origin/main` were both `7f52c1439575b4068844a474f563a27979e15a4e`.
- **Protocol:** Added a frozen 61-trial nominal grid (`v=0/0.15/0.30 m/s`,
  `omega=0/+-0.15/+-0.30/+-0.60 rad/s`; three moving repetitions), a disjoint 72-trial paired
  held-out grid (`v=0.10/0.22/0.35`, `omega=+-0.20/+-0.45`; three repetitions per mode), the
  unchanged Stage 0-B composite in both modes, and nominal/calibrated replay of all three
  frozen EXP-02B OLD command histories. Acceptance gates, config, raw/derived separation,
  physical/effective geometry fields, every raw hash, and the no-held-out-tuning flag are
  explicit. Primary actual run:
  `data/stage0/execution_calibration/stage0d-20260907T102700Z/`.
- **Baseline evidence:** Nominal yaw response was deterministic but strongly condition
  dependent: steady gain ranged `0.01950–0.36440`, group gain range was `0.34490`, and the
  largest paired left/right response difference was `0.04922 rad/s`. Effective-width medians
  where defined ranged approximately `0.898–8.216 m`. Several moving arcs retained low yaw
  gain with small wheel RMSE (for example gain `0.0603`, wheel RMSE `0.0384 rad/s` at
  `v=0.15, omega=-0.30`), while other conditions also had material wheel error. Wheel tracking
  alone therefore does not explain the body mismatch; a body/contact contribution is plausible
  but not causally identified.
- **Correction selection:** The through-origin gain `c=0.136006` implied feedforward scale
  `7.35263` and controller-only `b_eff=2.76157 m`. It failed worst-condition prediction error
  (`0.12748>0.12 rad/s`) and condition-gain range (`0.34490>0.20`). Only then were the frozen
  gentle/moderate/strong PI candidates run; all failed calibration accuracy. Strong
  `(kp,ki)=(3,2)` was fixed solely as the lowest calibration angular-RMSE diagnostic candidate
  (`0.19020 rad/s`), not as a passing calibration. It uses saturation, wheel bounds,
  deterministic reset, finite checks, sign protection, and conditional anti-windup without
  changing the physical asset.
- **Held-out/composite:** Selected calibrated held-out omega RMSE improved
  `0.32423→0.18354 rad/s`, but remained above `0.12`; v RMSE worsened
  `0.02664→0.10130 m/s`, above `0.06`, and per-condition omega accuracy failed. Sign,
  repeatability, no-oscillation, and saturation checks passed. The calibrated composite passed
  all absolute Stage 0-B gates (position RMS/final `0.02201/0.07863 m`, yaw RMS/final
  `0.02308/0.00398 rad`, goal reached), but strict non-degradation failed because final position
  error increased from `0.05334 m`.
- **Frozen replay result:** For representative high-delta-omega OLD, nominal→calibrated spatial
  RMS was `0.06460→0.09296 m`, body omega RMSE `0.63989→0.62880 rad/s` (only 1.73% reduction),
  body v RMSE `0.12133→0.21764 m/s`, and wheel RMSE `0.84874→1.05665 rad/s`. Nominal B remained
  exact; calibrated B differed by `0.13212 m / 1.60263 rad` because its executed commands were
  intentionally changed, not because of hidden reset mixing. High-delta-v also worsened;
  benign motion was nearly unchanged in body metrics but wheel RMSE rose.
- **Decision:** `EXECUTION_LAYER_NOT_YET_VALIDATED`. The held-out primitive and representative
  replay gates failed, so the candidate must not be frozen as the future reconciliation
  platform. Thresholds were not altered and no more complex controller was invented. This is
  execution engineering evidence, not reconciliation-method evidence.
- **GUI:** Actual Isaac GUI runs completed all five primitives (straight, both turns, both arcs),
  the composite, and representative EXP-02B replay. Blue reference/planned OLD, orange nominal,
  green calibrated, grey FRESH, and separate saved/nominal/calibrated boundary markers are drawn
  using the existing DebugDraw helper. Final overlays were raised above the chassis after visual
  review so the short failed calibrated replay remains visible. Jackal motion, reset between
  modes, left/right turns, three histories, and live desired/executed/measured body plus four-
  wheel target/measured telemetry were inspected. Diagnostic slowdown/hold is wall-time-only;
  viewport captures are under the ignored primary run `gui_metadata/`.
- **Validation:** Focused pure suite `16 passed`; complete suite `219 passed`. Python compileall,
  both launcher syntax checks, config/split reconstruction, generated-data ignore check, visual
  review of all eight quantitative plots plus GUI captures, and `git diff --check` passed.
  Actual Isaac Sim was `6.0.1-rc.7+release.42383.32955d8d.gl`; runtime physical radius/separation
  were `0.0979999974/0.375589997 m` and physics overrides were empty. Existing user camera and
  playback-factor edits were preserved and excluded from staging.
- **Major files:** `README.md`, `configs/stage0_jackal_execution_calibration.yaml`,
  `docs/{STAGE_00_EXECUTION_LAYER_CALIBRATION.md,EXP_02B_CONTROLLER_AWARE_RECONCILIATION.md,WORK_LOG.md}`,
  `src/reconciliation/{execution_calibration.py,controllers/jackal_execution_controller.py}`,
  headless/GUI Isaac runners and launchers, summarizer, and two pure test modules.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final status/diff/staged-diff review.

## 2026-09-08T12:10:00+09:00 — Stage 0-E closed-loop trajectory execution validation

- **Purpose and scope:** Compare nominal and the frozen Stage 0-D `pi_strong` candidate on the
  same trajectory references with a fresh `TrajectoryFollower` command at every measured-pose
  control step. This is an execution-platform selection gate, not reconciliation evidence. No
  follower/controller tuning, physics/USD change, LightNav change, or EXP-02A/02B graph change
  was made. Starting local and fetched `origin/main` were both
  `8ddaf2265201b552d878d2dffe37405a0bb17b54`. Existing user camera and playback-factor edits
  were preserved and excluded from staging.
- **Frozen protocol:** Added seven deterministic arbitrary-`N` controlled references, the
  unchanged Stage 0-B composite, and all three frozen EXP-02B OLD arrays as a
  `reference-matched closed-loop evaluation`. Every condition runs nominal/calibrated three
  times from an independent world/pose/zero-wheel/reset controller state. Stage 0-B follower,
  physics/control timing, asset, scene, and gates remain identical across modes. The Stage 0-D
  model/metadata/config hashes and selected-candidate provenance are verified before execution;
  historical command CSV rows are never used.
- **Primary evidence:** Actual Isaac Sim run
  `data/stage0/closed_loop_execution_validation/stage0e-20260908T021433Z/` completed 66/66
  trials. Nominal passed 1/7 controlled conditions and failed composite and all three OLD
  references on yaw/goal gates. Calibrated passed 5/7 controlled paths, including both gentle
  turn directions, S-curve, and straight-turn-straight, but strong-left/right yaw RMS was
  `0.17309/0.17028 rad`, above `0.10`; controlled aggregate therefore failed the frozen 6/7
  gate. Calibrated passed composite (position/yaw RMS `0.00250 m / 0.01900 rad`, 3/3 goals) and
  all three frozen source cases.
- **Representative OLD reference:** In high-delta-omega same-reference closed loop,
  nominal/calibrated position RMS was `0.06697/0.03997 m`, yaw RMS `0.38108/0.07980 rad`,
  desired-omega RMSE `0.74357/0.20583 rad/s`, desired-v RMSE `0.10313/0.11402 m/s`, and wheel
  RMSE `1.11290/0.57746 rad/s`; both reached 3/3 goals. Calibrated saturation was 0.0667 with
  two sign-protection events. Its executed-omega versus measured RMSE was `1.60261 rad/s`, kept
  separate from desired tracking. High-delta-v has the same frozen OLD hash/geometry and thus
  identical Stage 0-E measurements; it remains a distinct source transition, not independent
  OLD geometry.
- **Historical replay distinction:** Stage 0-D held saved commands fixed and changed execution,
  yielding representative calibrated spatial/omega/wheel RMS `0.09296 m / 0.62880 rad/s /
  1.05665 rad/s` plus a changed reproduced boundary. Stage 0-E holds the reference fixed and
  permits each measured-pose feedback loop to regenerate commands until goal/timeout. The
  improved closed-loop result therefore answers a different question and does not revise the
  frozen EXP-02B result.
- **Decision:** `EXECUTION_PLATFORM_NOT_READY`. Nominal failed controlled/composite/EXP-02B;
  calibrated passed composite/EXP-02B but failed controlled 5/7. No execution mode was frozen,
  no threshold was changed, and the termination rule prevented further controller development.
  Repeatability standard deviations were zero or floating-point epsilon across all conditions.
- **GUI and artifacts:** Actual non-headless S-curve, Stage 0-B composite, and representative
  EXP-02B runs completed. Existing DebugDraw helpers render blue reference/planned OLD,
  orange-red nominal actual, green calibrated actual, and grey FRESH context excluded from
  metrics. Nominal ghosts persisted across the explicit reset, Jackal motion and live
  desired/executed/measured body plus target/measured four-wheel telemetry were observed, and
  all three viewport captures were inspected. The immutable ignored run contains raw telemetry,
  hashes, five required summaries, and 15 unit-labelled plots. A first failed plot attempt was
  retained separately as `plots_failed_matplotlib_api/`; the completed official plots are in
  `plots/`.
- **Validation:** Focused pure suite `24 passed`; complete suite `227 passed`. Python compileall,
  both launcher syntax checks, strict 66-trial reconstruction, generated-data ignore check,
  visual review of representative plots plus all three GUI captures, and `git diff --check`
  passed.
- **Major files:** `README.md`, `configs/stage0_closed_loop_execution_validation.yaml`,
  `docs/{STAGE_00_CLOSED_LOOP_EXECUTION_VALIDATION.md,STAGE_00_EXECUTION_LAYER_CALIBRATION.md,WORK_LOG.md}`,
  `src/reconciliation/closed_loop_execution_validation.py`, shared/headless/GUI Isaac runners
  and launchers, summarizer, and `tests/test_closed_loop_execution_validation.py`.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final status/diff/staged-diff review.

## 2026-09-08T13:48:25+09:00 — Stage 0-F LightNav trajectory execution-envelope analysis

- **Purpose and scope:** Resolve only whether the frozen redesigned EXP-01B LightNav OLD/FRESH
  workload is covered by the current frozen calibrated Jackal trajectory-execution layer. This
  is geometry characterization and execution-envelope validation, not reconciliation evidence.
  No controller/follower gain, Stage 0-D candidate, track width, physics/contact/USD, LightNav,
  EXP-02 objective, graph weight, selector, or correspondence changed. Starting local and
  fetched `origin/main` were both `dd9ec20e52b456fb5903ce70b3363cb643f31a29` on `main`.
  Existing user camera-eye and playback-factor config edits were preserved and excluded.
- **Frozen inventory:** Strict reconstruction/hash validation retained 37 attempts: 30 eligible
  timing-valid moving transitions, three `MODEL_STOP_OUTPUT`, three `OLD_EXHAUSTED`, and one
  `TIMING_INVALID`. Six attempts had all-zero FRESH arrays, but only the three model STOPs are
  labelled `STOP_OUTPUT`; the other three remain failed OLD-exhaustion attempts. Moving data
  deduplicated to 8 OLD/8 FRESH raw-action hashes and 9 OLD/16 FRESH world-reference hashes.
  No new LightNav inference ran and no source artifact changed.
- **Geometry:** Added untimed spatial descriptors with wrapped pose-yaw and XY-tangent
  progression kept separate, a 1 mm undefined-edge convention, finite curvature/radius
  handling, lateral departure, tangent turn, sign change, and near-zero statistics. Added all
  144 valid-length `FRESH[k:]` suffix descriptors without selecting k or testing a splice. The
  fixture-only ten-dimensional range normalizer, outside radius, ambiguity margin, and
  lexicographic severity order were frozen before execution. All 25 LightNav references were
  descriptively outside the sparse Stage 0-E fixture neighborhood; this was not treated as an
  execution prediction or a learned classifier.
- **Primary execution:** Actual Isaac Sim run
  `data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z/` completed 75 calibrated
  trials (25 unique references x 3) plus 24 nominal trials (8 geometry-frozen representatives
  x 3). OLD started at the recorded OLD observation pose and FRESH at its FRESH observation
  pose. Every run used the unchanged Stage 0-B follower, frozen Stage 0-D `pi_strong`, official
  Jackal, flat ground, zero-wheel/full-controller reset, and Stage 0-E absolute gates. Strict
  validation passed all 99 trials and provenance hashes.
- **Results:** Calibrated passed OLD 9/9 and FRESH 16/16 unique conditions with 75/75 goals.
  Worst yaw RMSE was `0.08606 rad` for `old_aac92c8f6993b6f9`; hardest-severity
  `fresh_b42af924228e8aaf` passed at position/yaw RMS `0.04022 m / 0.08121 rad`, final
  `0.07296 m / 0.04183 rad`, desired-omega RMSE `0.22423 rad/s`, wheel RMSE
  `0.53145 rad/s`, and saturation `0.06667`. The representative subset was nominal 3/8 versus
  calibrated 8/8; its three most consequential nominal yaw failures improved from
  `1.48171`, `1.54814`, and `0.88711 rad` to `0.05488`, `0.05813`, and `0.08121 rad`.
- **Decision and limitation:** Final label is
  `LIGHTNAV_ENVELOPE_COVERED_BY_CURRENT_CALIBRATED_PLATFORM`, with separate
  `OLD_EXECUTION_ENVELOPE_COVERED` and
  `FRESH_EXECUTION_ENVELOPE_COVERED_AT_OBSERVATION_STATE`. The decision intentionally keeps
  `execution_platform_validated=false`, preserves both Stage 0-E strong-turn failures, and does
  not claim suffix/splice or reconciliation success, future-distribution coverage, obstacles,
  stochastic/real-robot robustness, or a unique controller/contact cause.
- **GUI and plots:** Actual non-headless low, global-median, highest-severity, and nearest-strong
  runs completed. Blue reference, green calibrated, red/orange nominal where available, magenta
  FRESH start, explicit nominal-to-calibrated reset, Jackal motion, and live pose/follower/body/
  wheel/PI telemetry were observed. The high view retained the nominal failure loop next to the
  passing calibrated path. All four viewport captures were visually inspected; no calibrated
  failure existed for a `first_fail` view. Ten requested plots were generated and visually
  reviewed with cross-frame XY paths separated or explicitly display-canonicalized.
- **Validation:** Focused Stage 0-F suite `10 passed`; complete suite `237 passed`. Python
  compileall, both new shell launchers' `bash -n`, strict 99-trial `--validate-only`
  reconstruction, generated-data ignore checks, source/reference hash validation, and
  `git diff --check` passed. Several immutable superseded local runs were retained rather than
  overwritten while correcting preprocessing edge semantics, output schema, and per-reference
  provenance before the final run.
- **Major files:** `README.md`, `configs/stage0_lightnav_execution_envelope.yaml`,
  `docs/{STAGE_00_LIGHTNAV_EXECUTION_ENVELOPE.md,WORK_LOG.md}`,
  `src/reconciliation/lightnav_execution_envelope.py`, preparation/headless/GUI/summarization
  scripts and launchers, and `tests/test_lightnav_execution_envelope.py`.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final status/diff/staged-diff review.

## 2026-09-08T15:13:04+09:00 — EXP-02B-R frozen calibrated candidate re-evaluation

- **Purpose and scope:** Re-execute only the exact historical EXP-02B M1 `raw_k`, M3
  `rigid`, and M4 `graph` candidates through the already frozen Stage 0-D calibrated
  post-switch execution layer and determine whether the formulation conclusion changes.
  This added no optimization residual, graph/weight change, selector, gate, correspondence,
  controller tuning, LightNav run, or physics/USD modification. Starting local and fetched
  `origin/main` were both `d9f496d331cd9af6dd0f642e10fa63b915dfa431` on `main`.
  Existing unrelated user edits to the Stage 0 controller-validation and LightNav
  single-chunk configs were preserved and excluded from this work.
- **Frozen protocol:** Strictly validated historical run
  `data/exp02b/exp02b-controller-aware-20260906T150400Z/`, then hash-verified and loaded
  its 27 candidate arrays in place for three cases x `k={0,3,6}` x
  `{raw_k,rigid,graph}`. Historical nominal OLD commands were replayed, the comparability
  gate was checked, saved B was restored exactly, the last OLD wheel target was restored,
  and the frozen calibrated `pi_strong` PI state was reset at the switch. The final
  pre-reset OLD interval supplies the first feedback measurement; no derivative crosses
  the exact-reset teleport. Stage 0-D model SHA-256 remained
  `40821584...1464` (`kp=3`, `ki=2`, feedforward `7.352630`, effective separation
  `2.761574 m`), and Stage 0-F bounded-scope provenance was verified before execution.
- **Primary execution:** Actual Isaac Sim run
  `data/exp02b_calibrated_reeval/exp02b-r-20260908T054233Z/` completed and strictly
  validated 27/27 branches. All 27 nominal OLD replay gates passed with zero saved-versus-
  reproduced translation/yaw/command error. All 27 historical-versus-reevaluated first
  desired `[v,omega]` invariants passed at `1e-12`; both maximum differences were exactly
  zero. Each branch stores desired/executed/measured body layers, control-interval poses,
  four target/measured wheel rates, PI/saturation state, follower progress, spatial
  candidate metrics, raw hashes, and immutable source provenance.
- **Physical result:** At common 0.1 s control-time sampling, aggregate nominal-to-
  calibrated desired-omega tracking RMSE improved `0.596104 -> 0.217624 rad/s`, while
  desired-v RMSE worsened `0.116737 -> 0.119536 m/s`, nearest-candidate position RMS
  worsened `0.268973 -> 0.281053 m`, and nearest-candidate yaw RMS worsened
  `0.133262 -> 0.178637 rad`. Immediate measured transition means were essentially
  unchanged (`|delta v| 0.00394544 -> 0.00394546 m/s`, `|delta omega| 0.02861954 ->
  0.02861958 rad/s`). Calibrated wheel RMSE averaged `0.689193 rad/s`; historical
  EXP-02B has no equivalent wheel telemetry, so no historical wheel-improvement claim was
  made. Spatial metrics are nearest-polyline/pose descriptors, never time-aligned waypoint
  errors.
- **Method result:** Graph was not consistently better. Against raw_k, the 18 immediate
  measured components counted 12 better / 6 worse, while all nine metrics across nine
  pairs counted 37 better / 44 worse. Against rigid, immediate components counted 10
  better / 6 worse / 2 ties and all metrics counted 43 better / 36 worse / 2 ties. In
  benign Case C k=0, raw/graph desired `|delta v|` remained exactly the historical
  `0.004422/0.240968 m/s`; calibration reduced graph's immediate measured consequence,
  but graph still displaced the selected entry `0.381925 m` with only numerical suffix
  deformation. Final status is `EXP02B_FORMULATION_CONCLUSION_UNCHANGED`; the execution
  result is mixed rather than a broad physical-tracking improvement.
- **GUI and plots:** Actual non-headless GUI runs completed for high-delta-omega k=3
  raw_k/rigid/graph and benign k=0 raw_k/graph. Existing DebugDraw helpers show blue
  candidate, green calibrated actual, orange historical nominal actual, grey raw FRESH,
  and separate saved/candidate/raw-entry markers; terminal rows show all three body-command
  layers, measured transition, follower progress, wheels, PI, saturation, and sign events.
  A first GUI-only inspection exposed that an unpaused viewport hold advanced physics; the
  superseded captures were retained under `gui_metadata_pre_fix_do_not_use/`. The hold was
  changed to pause/play the world, all five cases were rerun, and corrected first intervals
  exactly matched headless outputs. Seven focused plots were generated and visually reviewed.
- **Validation:** New focused suite `9 passed`; complete suite `246 passed`. Python
  compileall, both new launcher `bash -n` checks, strict 27-branch reconstruction, seven-plot
  validation, generated-data ignore checks, viewport/plot visual review, and
  `git diff --check` passed. Frozen recursive manifests remained
  `d6a8176fe2a2370b1bb1d812c72e52a1151f8fce45ed3688c9be71b7b7054533` for EXP-02B and
  `a3ec7b83af54789b1d582257b7916c732b2b093e65010ceb3f2587d0b7e4efaf` for its source.
- **Major files:** `README.md`, `configs/exp02b_calibrated_reeval.yaml`,
  `docs/{EXP_02B_CALIBRATED_REEVALUATION.md,WORK_LOG.md}`,
  `src/reconciliation/exp02b_calibrated_reeval.py`, preparation/headless/GUI/summarization
  scripts and launchers, and `tests/test_exp02b_calibrated_reeval.py`.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final status/diff/staged-diff review.

## 2026-09-08T17:00:09+09:00 — EXP-02C current-M4 factor isolation

- **Purpose and scope:** Attribute the unchanged EXP-02A/EXP-02B M4 failure to its current
  entry, incoming-direction, incoming-yaw, and FRESH relative-motion factors. This is an
  offline formulation diagnostic, not an improved optimizer. No production graph residual,
  weight, selector, gate, controller, physics, LightNav artifact, or frozen EXP-02A/B/B-R
  result changed. Work started on fetched `origin/main` and local `main` at
  `57dbd4317abb4360333954315e58b7bb1e832295`. Existing unrelated user camera-eye and
  LightNav playback-factor edits were preserved and excluded.
- **Implementation:** Added a separate `exp02c_factor_isolation` problem that reuses the
  exact physical M4 residual functions, frozen scales/weights, right-local retract, central
  finite difference (`epsilon=1e-6`), and LM algorithm while exposing V0–V8 factor subsets.
  V8 reconstructs downstream nodes from exact raw FRESH so only X_k can change. Every solve
  records exact accepted/rejected steps, initial/final physical and weighted per-factor
  residuals/gradients, first/downstream node gradient norms, initial/final gradient cosines,
  controller desired-command spatial probes, geometry/correction profiles, and best-fit left
  SE(2) rigidity metrics. No intrinsic waypoint time was invented.
- **Frozen regression and execution:** Final immutable ignored run
  `data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z/` completed 5
  synthetic mechanism conditions and all 3 frozen real cases x `k={0,3,6}` x 9 variants,
  producing 126 variant directories. Frozen EXP-02B top-level and candidate hashes were
  checked against the EXP-02B-R manifest before output creation. V4 residual vectors and all
  nine historical M4 candidate poses matched exactly: maximum residual, translation, and
  wrapped-yaw differences were all `0.0` against the `1e-9` gate.
- **Synthetic checks:** S0 was an exact all-factor/no-motion no-op. S1 had only direction
  nonzero and V2/FULL moved the entry `0.271964 m` while V3 was a no-op. S2 had only yaw
  nonzero and V3 changed entry yaw `0.175000 rad` while V2 was a no-op. S3's magnitude stress
  left every current factor zero, exposing only a missing-mechanism candidate. In S4, FULL
  moved entry/endpoint `0.271964/0.271964 m` with `5.53e-14 m` rigid-fit RMS; V8 kept the
  endpoint raw but incurred `0.121626 m` edge RMS and `0.0880515 m` rigid-fit RMS. Synthetic
  outputs are mechanism demonstrations only.
- **Real attribution:** Every raw entry cost was zero and every raw fresh-motion cost was
  at numerical zero. Benign Case C k=0 direction/yaw costs were `400.000` and
  `1.31683e-5`, with weighted gradient norms `1007.6283` and `0.0362881`. Direction-only
  changed desired `|delta v|` from `0.00442238` to `0.24096762 m/s`; yaw-only left it at
  `0.00442236 m/s`; FULL was `0.24096764 m/s`. FULL entry displacement was `0.381925 m`.
  At the solution, entry-versus-direction gradient cosine was `-0.999999886`, while removing
  entry increased displacement to `0.396972 m`. Across all nine real FULL suffixes, worst
  best-fit-single-left-SE(2) translation RMS was `1.26703e-11 m`.
- **Interpretation:** Incoming direction supplies the undesirable benign-case target; entry
  strongly conflicts with and restrains it; relative fresh-motion preservation propagates
  the entry correction as an almost rigid suffix; no absolute downstream recovery or direct
  OLD-to-entry magnitude residual exists. No factor is established as globally redundant.
  The one recommended next experiment is to redesign incoming-direction residual semantics
  before adding another factor; no such redesign was implemented here.
- **Plots and GUI:** Generated and visually checked all 13 required plots, including raw and
  final cosine panels, correction propagation, real overlay, and S1/S2/S4 mechanisms. Actual
  non-headless Isaac Sim 6.0 DebugDraw runs completed for real benign k=0 and synthetic S4;
  both viewport captures were inspected. The real view separates OLD/raw/V1–V5/B/F_k/X_k;
  S4 separates raw/desired diagnostic target/FULL/no-propagation/B. Terminal JSON prints
  factor costs/gradients, desired deltas, displacement, and rigidity. Both are static
  qualitative views with `physics_executed=false`.
- **Validation:** Focused EXP-02C suite `26 passed`; complete suite `272 passed`. Strict
  artifact reconstruction passed `126` variants, `9` V4 regressions, `13` plots, and `2` GUI
  captures. Whole-repository Python compileall, new launcher `bash -n`, all strict JSON/finite
  checks, generated-data Git-ignore checks, and `git diff --check` passed.
- **Major files:** `README.md`, `configs/exp02c_factor_isolation.yaml`,
  `docs/{EXP_02C_FACTOR_ISOLATION.md,WORK_LOG.md}`,
  `src/reconciliation/exp02c_factor_isolation.py`, offline runner, plot summarizer, Isaac
  DebugDraw runner/launcher, and `tests/test_exp02c_factor_isolation.py`.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final status/diff/staged-diff review.

## 2026-09-09T14:34:29+09:00 — EXP-02C GUI visible-Jackal and hold correction

- **Issue and scope:** The saved EXP-02C GUI showed only DebugDraw trajectories because the
  runner never added a robot USD, and automated smoke commands using `--no-hold` intentionally
  closed the window after capture. Work started from fetched local/origin `main` at
  `3ada4301cd14db8b7e3f33ddc4205be945274261`. Existing unrelated user edits to the Stage 0
  controller-validation and LightNav single-chunk configs were preserved and excluded.
- **Correction:** The GUI now reuses the Stage 0/EXP-01B asset resolver, loads the official
  `Clearpath/Jackal/jackal.usd`, and places it at real saved B or declared synthetic B. It is
  explicitly a visual-only reference: no articulation initialization, timeline playback,
  physics step, or robot execution is performed. GUI metadata records the resolved asset
  provenance, world-frame B pose and units, boundary kind, spawn height, prim path, and
  `visual_only`/`physics_executed` flags. Terminal output likewise identifies the model as a
  static reference rather than experimental evidence.
- **Lifetime behavior:** Normal interactive launch now always reaches
  `EXP02C_GUI_PHASE=READY_AND_HOLDING` and remains open until the window is closed or `Ctrl-C`
  is pressed. `--no-hold` remains available only for automated capture and its help text and
  documentation now state that it deliberately closes the GUI.
- **Actual GUI verification:** A non-headless real-benign-k0 capture completed at
  `gui_metadata/real_benign_k0-20260909T053224Z/`; the viewport image was inspected and showed
  the Jackal at B together with OLD/raw/V1--V5 trajectories and markers. A final-code
  synthetic-S4 capture at `gui_metadata/synthetic_s4-20260909T053822Z/` was also inspected and
  showed the Jackal at declared B with raw/target/FULL/no-propagation paths. A second
  non-headless real launch without `--no-hold` reached and remained at `READY_AND_HOLDING`;
  it was then stopped manually with `Ctrl-C` after the persistence check. Generated captures
  remain below the ignored EXP-02C run directory.
- **Validation:** Complete suite `272 passed`; whole-repository Python compileall, launcher
  `bash -n`, generated-capture Git-ignore check, recorded metadata inspection, and
  `git diff --check` passed. No frozen EXP-02C numeric artifact or earlier EXP-02A/B/B-R result
  was modified.
- **Major files:** `README.md`, `docs/{EXP_02C_FACTOR_ISOLATION.md,WORK_LOG.md}`, and
  `scripts/isaac/exp02c_factor_isolation_gui.py`.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md 로 확인)`
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final status/diff/staged-diff review.

## 2026-09-09T16:56:53+09:00 — DATA-01 frozen LightNav transition bank

- **Purpose and scope:** Freeze real OLD/FRESH inputs and outcome-independent development /
  held-out partitions before EXP-02D formulation work. This is data curation and static
  visualization only. No residual, graph solve, factor weight, selector, controller,
  LightNav inference, or Isaac physics execution was added or run. Local and fetched
  `origin/main` both started at `275d17b39317fee0faf8790856329d5b172305ea` on `main`.
  Existing unrelated edits to the Stage 0 controller-validation and LightNav single-chunk
  configs were preserved and excluded.
- **Source and schema:** Strict EXP-01B/Stage 0-F validation reconstructed 37 attempts: 36
  timing-valid, 30 `VALID_MOVING`, three `MODEL_STOP_OUTPUT`, three `OLD_EXHAUSTED`, and one
  `TIMING_INVALID`. All 30 eligible transition contexts were retained even when arrays repeat.
  Every pair stores byte-identical raw OLD/FRESH and derived world NPY copies, exact P/B event
  and timeline semantics, observation/model-ready/usable times and poses, complete source
  hashes/provenance, Stage 0-F spatial descriptors, frozen historical raw-switch controller
  metrics, and deterministic ranks. LightNav rows remain untimed.
- **Identity and split:** The bank contains 8/8 unique OLD/FRESH raw arrays, 9/16 unique
  OLD/FRESH world arrays, 15 ordered raw-pair groups, and 16 world-pair groups. Stable pair IDs
  hash the source-relative trial, four array hashes, and boundary/timing context. Exhaustive
  deterministic assignment of complete raw-pair groups produced 20 development and 10
  held-out pairs with zero leakage. Every G0/G1/G2 x L0/L1 stratum appears in both partitions;
  counts are `3/2, 3/2, 3/2, 3/2, 4/1, 4/1` development/held-out respectively.
- **Observed raw descriptors:** Across 30 pairs, `|delta v|` min/median/max was
  `0.002782/0.013260/0.416563 m/s`; `|delta omega|` was
  `0.001184/0.016439/1.405038 rad/s`; P→B versus B→F0 direction disagreement was
  `0.754941/3.140092/3.141545 rad`; yaw-increment disagreement was
  `0.000337/0.000812/0.226989 rad`; and effective simulation latency was
  `0.450000/0.716667/1.000000 s`. These are descriptive raw k=0 context statistics, not
  optimization or optimal-k outcomes.
- **Visualization:** Generated 90 per-pair PNGs (world XY, visualization-only B-centered XY,
  and yaw versus cumulative spatial length) and nine overview/contact sheets. Blue OLD,
  orange FRESH, red B, black P/P→B, independent world axes, canonical B at zero, explicit
  non-time yaw axis, titles, legends, equal aspect, and clipping were visually checked for
  low/median/high-delta-v/high-delta-omega/largest-direction representatives, a held-out
  example, and both all-pair overviews.
- **Validation:** Two isolated end-to-end immutable preflight banks completed. Final-code
  strict validation passed 30 pairs, 20/10 split, 15 groups, 481 frozen source artifact
  hash/mtime records, zero leakage, and all 99 PNG signatures/hashes. Focused tests passed
  `17`; complete suite passed `289`. Whole-repository compileall, generated-data ignore check,
  and `git diff --check` passed. The official ignored bank is generated from the focused
  commit before push so its manifest records that generator commit.
- **Representatives:** raw-only roles selected `pair_4a4d7f199b92` (minimum combined),
  `pair_1d5f64ce0384` (median combined), `pair_a7703fd6b06d` (maximum delta-v),
  `pair_6f8a42dd8306` (maximum delta-omega and yaw disagreement), and
  `pair_4bd02d56c173` (maximum direction disagreement). They are visualization examples, not
  an evaluation subset.
- **Major files:** `README.md`, `configs/lightnav_transition_bank_v1.yaml`,
  `docs/{DATA_01_FROZEN_LIGHTNAV_TRANSITION_BANK.md,WORK_LOG.md}`,
  `src/reconciliation/frozen_transition_bank.py`, three build/plot/validate scripts, and
  `tests/test_frozen_transition_bank.py`.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md` after commit).
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after official bank generation and final validation.
## 2026-09-09T18:20:00+09:00 — DATA-01 primary-use retirement

- **Status:** DATA-01 is `RETIRED_FROM_PRIMARY_FORMULATION_USE`. Its EXP-01B cohort was
  designed for latency and G0/G1/G2 characterization and did not provide sufficient
  trajectory diversity for primary formulation research.
- **Forward-only change:** Work started from local and fetched `origin/main` at
  `758a67209771fa2083b5b8ea5f46faa3056fcecd` on `main`. The previous DATA-01 commit and log
  entry were not amended. Active README/EXP-02D instructions were removed, the historical
  DATA-01 document was retained with a retirement banner, and its EXP-01B-specific config,
  builder, plotter, validator, implementation, and focused tests were removed.
- **Local artifacts:** The ignored generated bank at
  `data/frozen_transition_bank/lightnav_exp01b_v1/` was moved to the desktop trash. The
  historical source `data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/` and
  all EXP-01A/B/02A/B/C artifacts remain present and unchanged.
- **Next role:** DATA-02 will be the primary diverse same-episode successive LightNav
  OLD/FRESH collection. DATA-01 may only be described as a legacy regression artifact.
- **Unrelated changes:** Existing user edits to
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` were preserved and excluded.

- **Validation:** Complete suite passed `272`; generated-bank absence, historical-source
  presence, stale import/reference search, and `git diff --check` passed.

## 2026-09-09T19:20:00+09:00 — DATA-02 collector implementation (pre-execution)

- **Purpose:** Add an outcome-independent collector for genuine same-episode successive
  LightNav OLD/FRESH pairs across ten predeclared navigation families. This is collection and
  telemetry only; no graph, optimization, factor, selector, smoothing, LightNav change,
  controller tuning, or physics tuning was added.
- **Frozen protocol:** Added explicit D0-D9 static scenes/instructions, mirrored left/right
  cases, five deterministic initial-condition variants, a fixed 0.50 s FRESH observation
  trigger, natural-latency timing, 3-output qualification rules, 5-valid/10-attempt primary
  limits, geometry descriptors, and dataset acceptance gates.
- **Runtime:** The Isaac collector reuses the existing LightNav IPC, Stage 0-B follower,
  official Jackal asset, and hash-verified frozen Stage 0-D calibrated controller. It records
  all RGB inputs, raw/world paths, actual OLD execution, desired/executed/measured body motion,
  target/measured wheels, P/B, timing, raw-switch probe, and provenance. FRESH remains
  observation-anchored and is not executed.
- **Curation:** Added immutable attempt/bank writing, raw/context identities, STOP/failure
  retention, predeclared geometric qualification, descriptive output labels, raw-pair-grouped
  70/30 split with scenario/geometry coverage, per-pair plots, six overview plots, acceptance
  summary, and strict reconstruction validation.
- **Pre-execution validation:** DATA-02 focused pure suite passed `19`; Python compilation,
  launcher syntax, config validation, and `git diff --check` passed. Actual server/Isaac
  smoke, qualification, GUI, primary collection, plot review, and strict artifact validation
  follow from the committed collector so artifacts record a stable generator SHA.
- **Unrelated changes:** Existing user edits to the two Stage 0 configs remain preserved and
  excluded.

## 2026-09-09T19:27:06+09:00 — repository cleanup after DATA-02 abandonment

- **Purpose and scope:** Performed a cleanup-only, forward change. No dataset collection,
  LightNav inference, Isaac execution, graph/formulation work, controller change, or new
  experiment was performed. Work started on local `main` at
  `351f6a171bea0d2720733f5b98a60272ac673df4`; fetched `origin/main` was
  `758a67209771fa2083b5b8ea5f46faa3056fcecd`. The branch was two commits ahead before this
  cleanup.
- **DATA-02 removal:** Removed the complete tracked DATA-02 config/document/collector,
  launcher, summarizer, validator, library, and test; reverted the DATA-02-only instruction
  allowance in the shared LightNav server; removed all 343,287,660 bytes below
  `data/data02_lightnav_diverse_transitions/`; and removed 17 `/tmp/data02-*` runtime
  directories. Historical WORK_LOG entries remain unchanged because this log is append-only.
- **DATA-01 removal:** Removed the remaining dedicated DATA-01 document. The preceding
  forward retirement commit had already removed its config/build/plot/validate/library/test
  pipeline, and its generated bank was already absent. The independent frozen EXP-01B source
  cohort remains preserved.
- **Generated cleanup:** Removed repository Python/test/build caches and explicit
  smoke/test/failed/pre-fix/pre-annotation/superseded outputs. The incomplete EXP-02B T150300Z
  run documented above was removed; T150400Z remains the frozen primary. Nine non-final
  EXP-02B GUI outputs were removed while the documented `...T-final` diagnosis was retained.
- **Preserved evidence:** Verified the requested EXP-01B primary, EXP-02B primary, EXP-02B-R,
  EXP-02C, and Stage 0-D/E/F directories remain present. No source/raw artifact in those
  directories was changed. Additional uncited legacy runs were classified, not mass-deleted.
  Full path/size/reference decisions are recorded in
  `docs/REPOSITORY_CLEANUP_AUDIT_20260909.md`.
- **Storage:** Final `du -sh` changed from `817M` to `429M` for the repository and from `626M`
  to `241M` for `data/`; `.venv` remained `182M` and untouched. Explicit project deletion targets
  totaled 382,630,936 bytes, plus 195,388 bytes of DATA-02 `/tmp` state.
- **Current state:** README now states that feedback-1 execution-platform diagnosis is
  complete within its claim limits, EXP-02C identified the current incoming-direction-factor
  problem, DATA-01 is retired, replacement formulation data collection must be redesigned,
  and EXP-02D has not started.
- **Validation:** Full suite passed `272`; compileall passed for `src`, `scripts`, and `tests`;
  every retained tracked shell launcher passed `bash -n`; stale executable DATA-01/DATA-02
  references were absent; and `git diff --check` passed. Generated caches from validation were
  removed again afterward.
- **Unrelated changes:** Pre-existing user edits to
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` were preserved and excluded from the cleanup
  commit.
- **Commit reference:** `SELF (git log -1 -- docs/WORK_LOG.md` after commit).
- **Branch:** `main`
- **Push:** Target `origin/main`; planned after final staged-diff review.

## 2026-09-09T20:30:00+09:00 — Stage 0-G LightNav input/scene qualification

- **Purpose and scope:** Audited the exact LightNav input contract, built six coherent indoor
  qualification scenes, captured five deterministic nearby stationary histories per scene,
  ran exactly 30 single-chunk predictions, computed immutable descriptors/diversity evidence,
  and added a non-executing Isaac GUI. No OLD/FRESH collection, controller execution/tuning,
  LightNav change, graph/objective/factor/selector/gate/smoothing change, or DATA-02 pipeline
  was introduced.
- **Baseline:** Work started on clean-history local/origin `main` at
  `524ced9ce193acf1b796f34ddc85010f8ce61338`. The separate, clean LightNav checkout was
  `a645828d81a8439651172197ca80a75dc1377977`; checkpoint revision was
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`. Expected checkout, revision, and six checkpoint
  file hashes all matched.
- **Input contract:** Frozen 480 x 270 HWC uint8 RGB, 112-degree HFOV, level forward 0.48 m
  Jackal-safe camera, stretch-to-256 x 448 bilinear preprocessing, 64 stationary frames at
  4 Hz, `vln`/`vlnce`/`vlnce_traj` `unified_traj`, and cumulative observation-local float32
  `[forward, left, CCW yaw]` output. Frame timestamps are recorded; waypoint rows remain
  explicitly untimed. World paths use only the final input-frame observation pose.
- **Preview/freeze:** A first pre-freeze run failed before audit completion due to a duplicated
  checkpoint manifest path. A second pre-freeze preview exposed that the Replicator camera did
  not follow articulation teleports. Both produced zero inference and are retained as technical
  invalidities. The transform was fixed without changing intended research conditions. All six
  V0 scenes were then visibly inspected before freezing r2 config SHA-256
  `8be2c0c65b7188a3a1e38d2528b99dd38006a6842239ce4d303cf7648a4d5d57`.
- **Primary run:** `data/stage0/lightnav_scene_qualification/20260909T_stage0g_primary_r2/`.
  Smoke used Q0/Q1/Q2 V0, then the other 27 cases ran once. Q0 straight and Q3 doorway were
  5/5; Q1 left, Q2 right, Q4 detour-left, and Q5 detour-right were 0/5. All 30 were finite and
  moving, but there were only 3 unique raw outputs and 2 coarse geometry signatures. No
  condition or threshold was tuned after inspecting predictions.
- **Decision:** `STAGE0G_SCENE_QUALIFICATION_FAILED`. Input-contract, preview legibility,
  validity/non-STOP, Q0, and Q3 forward-progress checks passed; left, right, detour, and global
  diversity requirements failed. This does not authorize DATA-02 or successive OLD/FRESH
  collection and is not a navigation/controller/reconciliation performance conclusion.
- **GUI:** Actual GUI smoke Q1/V0 showed the stationary official Jackal, T-junction scene,
  cyan world prediction, yellow headings, and endpoint marker. An initial viewpoint exposed
  wall occlusion, and a second exposed LiDAR-ray clutter; viewport-only framing and the existing
  sensor-clutter suppression pattern were then applied. Final screenshot visibly contains both
  Jackal and prediction. No trajectory execution occurs.
- **Artifacts:** Stored all histories/timestamps, raw decoder outputs/text, observation-anchored
  world paths, metadata/provenance, descriptors, per-case plots, two inspected overviews,
  duplicate groups, pairwise distances, summary, decision, and strict validation. Generated
  content remains ignored and primary raw outputs are exclusively written.
- **Validation:** Focused Stage 0-G tests passed `12`; full repository suite passed `284`;
  strict generated-artifact validation passed all 30 cases with the scientific status
  remaining FAILED. Compileall, both new shell launchers, and `git diff --check` passed.
- **Unrelated changes:** Existing user edits to
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` were preserved and excluded.

## 2026-09-09T21:30:00+09:00 — Stage 0-G2 Jackal realistic-domain qualification

- **Scope:** Kept Jackal, LightNav/checkpoint, camera, preprocessing, stationary history, output
  semantics, and Stage 0-G geometry thresholds fixed; changed only the visual/navigation domain.
  No controller, physics tuning, planner, DATA-02, OLD/FRESH, graph, residual, factor, selector,
  gate, optimization, or smoothing work was introduced.
- **Asset audit/selection:** Official Isaac 6.0.1 Office (4,653 prims, one detected collision),
  Hospital (1,910 prims, 126 collisions), and Simple_Room (139 prims, 60 collisions) loaded.
  Simple_Warehouse (3,417 prims, 781 collisions) and Grid were also considered. Hospital was
  selected before any G2 inference for its coherent realistic corridors, doors, hospital
  objects, Jackal-scale clearance, and collision
  coverage. Exact audit decisions are in the run manifest.
- **Preview/freeze:** Two pre-freeze runs produced zero inference: r1 exposed a closed-door
  doorway and r2 exposed a marginally displaced right-turn pose. Both remain non-primary ignored
  diagnostics. r3 passed all six explicit V0 visibility/camera/route/settling gates and froze
  config SHA-256 `49d339d16694d96dc5113271a2236e79ce413ae2411915e340885f8310fc9f01`.
- **Primary protocol:** `data/stage0/lightnav_scene_qualification_g2/20260909T_stage0g2_primary_r3/`.
  Captured 30 immutable 64-frame histories. Ran Q0/Q1/Q2 V0 smoke and then the remaining 27 once,
  without retry or tuning. All outputs were finite and moving.
- **Observed result:** Q0 straight 5/5 and Q3 doorway task-local forward 5/5; Q1 left, Q2 right,
  Q4 detour-left, and Q5 detour-right each 0/5. Total exact unique outputs were 5, dominant group
  fraction 0.667, and coarse signatures 3. Doorway distinct intent was not demonstrated.
- **Stage 0-G comparison:** Descriptively, unique arrays changed 3→5, dominant fraction
  ~0.767→0.667, and signatures 2→3, while all four non-straight turn/detour families remained
  0/5. No causal statistical claim is made.
- **Decision:** `STAGE0G2_SCENE_QUALIFICATION_FAILED`; DATA-02 remains unauthorized.
- **GUI:** Actual Isaac GUI smoke on Q3/V0 initially exposed a wall-occluded camera. The GUI-only
  chase view was corrected without touching inference artifacts; the second screenshot visibly
  shows stationary Jackal, Hospital doorway, cyan prediction, heading marker, and endpoint.
- **Validation:** Strict 30-case artifact validation passed; focused G2 tests passed `13`; the
  full repository suite passed `297`; compileall, both new launcher syntax checks, ignored-output
  check, and `git diff --check` passed.
- **Unrelated changes:** Existing user edits to the two Stage 0 configs remain preserved and
  excluded.

## 2026-09-09T23:40:00+09:00 — Stage 0-G3 moving-egocentric-history qualification

- **Purpose and scope:** Tested exactly one variable against frozen Stage 0-G2: 64 stationary
  views versus 64 deterministic moving egocentric views ending at the same observation pose.
  This remained inference-only qualification. No controller/wheel execution, physics tuning,
  LightNav change, OLD/FRESH collection, DATA-02, graph, objective, factor, selector, gate,
  optimization, smoothing, or new environment was introduced.
- **Repository and control:** Work started from fetched local/origin `main` at
  `a2272ead99cca7431e2fefdcc4867729b970cb66`. The read-only Stage 0-G2 r3 control was strictly
  revalidated at config SHA-256
  `49d339d16694d96dc5113271a2236e79ce413ae2411915e340885f8310fc9f01`:
  all 30 cases, instructions, final observation poses, raw hashes, official Jackal, Hospital,
  LightNav SHA, and checkpoint revision matched.
- **Pre-inference approach:** Oriented-footprint collision checks rejected the common 1.5, 1.4,
  and 1.3 m approach distances for all Q2 variants and selected the largest common feasible
  distance, 1.2 m. All 30 selected 64-pose histories were collision-query-feasible. The G3
  config was frozen before capture/inference at SHA-256
  `0515f6b8c8bd86d55e39db5d6128acbd5acac9f0fe1bccb6c3a620511ef08cf8`.
- **Technical pre-runs:** r1-r5 produced zero LightNav predictions. They exposed simulator-clock
  tolerance, float32 start-distance tolerance, and stale first-frame Jackal rendering across
  cases. Only these technical validity issues were corrected. r6 contact sheets confirmed a
  moving scene and no stale robot before any G3 inference.
- **Primary run:**
  `data/stage0/lightnav_moving_history_qualification/20260909T_stage0g3_primary_r6/` captured all
  30 histories and ran Q0/Q1/Q2 V0 smoke followed by the remaining 27 exactly once. Maximum
  G2/G3 final-pose mismatch was `4.76837158203125e-7` m and
  `3.4547587013378234e-7` rad. Mean inter-frame translation averaged 0.01904762 m. Mean
  consecutive RGB MAE averaged 1.6021 and first-to-final RGB MAE averaged 17.5371 uint8 levels.
- **Observed result:** G2→G3 intended matches were Q0 5→5, Q1 0→3, Q2 0→0, Q3 task-local
  5→5, Q4 0→0, and Q5 0→0. Only 2/5 doorway matches were distinct from unrelated raw groups.
  Moving history changed the exact raw array in 16/30 pairs, changed the frozen geometry class in
  7/30, produced 3 fail→pass and 0 pass→fail transitions, 9 unique raw arrays, dominant fraction
  0.367, four signatures, and zero STOP/nonmoving cases. Translation/yaw rowwise RMS-difference
  means were 0.083643 m and 0.055288 rad. This shows an association with changed output, not a
  causal or navigation-success result.
- **Decision:** `STAGE0G3_MOVING_HISTORY_QUALIFICATION_FAILED`. Q1 missed 4/5, Q2 was 0/5, and
  neither distinct doorway nor a detour family reached 4/5. Moving visual history alone was not
  sufficient; successive collection remains unauthorized and the setup was not redesigned.
- **Visualization:** Inspected all six V0 pre-inference contact sheets, all six V0 paired plots,
  and all overview figures. A real Isaac GUI smoke on Q1/V1 visibly showed the official Jackal,
  green 64-pose approach, magenta G3 prediction, cyan G2 prediction, and yellow shared final
  pose. The 4 Hz `SCRIPTED HISTORY REPLAY` completed one loop, entered the next, and persisted
  until manually closed; it issued no controller command. A screenshot is retained under r6
  `overview/`.
- **Validation:** Focused G3 suite passed `18`; full repository suite passed `315`; strict G2
  control validation and strict 30-case G3 artifact validation passed. Python compileall, both
  new launcher `bash -n` checks, ignored-output verification, and `git diff --check` passed.
- **Unrelated changes:** Existing user edits to
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` were preserved and excluded.

## 2026-09-10T01:39:20+09:00 — DATA-02 online-successive collector freeze

- **Research decision and scope:** Implemented the target-data collector for genuine same-episode
  successive LightNav OLD/FRESH contexts while the wheel-driven Isaac Jackal continues OLD during
  natural asynchronous FRESH latency. This is data collection/characterization only. No Stage 0-G4,
  reconciliation, graph, objective, residual, correspondence, selector, gate, smoothing, obstacle
  avoidance, controller tuning, physics tuning, or LightNav modification was introduced. Historical
  Stage 0-G/G2/G3 failures remain failures.
- **Starting repository:** Fetched local/origin `main` both pointed to
  `4c37b5c9ff37ca8b460b1466c9acdee1e1f58a45`. The clean external LightNav checkout remained
  `a645828d81a8439651172197ca80a75dc1377977`; checkpoint revision remained
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, with all declared checkpoint hashes matching.
- **Frozen system:** Actual Isaac `6.0.1-rc.7+release.42383.32955d8d.gl`, official Hospital and
  Jackal assets, the Stage 0-G2 480 x 270/112-degree/4 Hz input contract, persistent warmed
  LightNav at 0.65 GPU-memory utilization and 1 GiB KV cache, and the unchanged
  `TrajectoryFollower` + frozen Stage 0-D `pi_strong` + `DifferentialController` wheel stack.
  Stage 0-F's `execution_platform_validated=false` limitation remains explicit.
- **Frozen design:** Declared 12 Hospital templates across straight/left/right/doorway/detour/
  compound setups and seven common deterministic pose variants, yielding the fixed 84-episode,
  maximum-504-opportunity primary grid. Template previews and all 84 collision/settling feasibility
  checks occur before primary inference. FRESH triggers at exactly 0.50 simulation seconds; its raw
  observation-anchored path is activated unchanged and becomes the next OLD without an episode
  reset. Waypoint rows remain spatial and untimed. Frozen config SHA-256 is
  `b92f4ddc1ba7e80d5544573ffedd8b8175e6b92ae79c8d79637cbb3e17c5a313`.
- **Artifacts and validation:** Added immutable episode RGB/chunk/transition streams, explicit
  observation/host/model-ready/switch clocks, model-ready/P/B poses, body/wheel/controller telemetry,
  raw/world ordered identities, run protocol/collection manifests, strict reconstruction, frozen
  coverage/readiness gates, leakage-isolated splitting, ten required plots, and saved-only GUI replay.
  GUI uses a replay-only chase view/light and phase panel; it does not rerun LightNav.
- **Technical smoke:** `data02-online-successive-smoke-r6` completed one episode and three successive
  transitions with one model build, one episode reset, and four predictions. Strict validation
  passed all raw/world/frame/timing/hash checks. Statuses were 2 `ELIGIBLE_MOVING` and 1 preserved
  `TIMING_INVALID`; eligible host latency was 0.556--0.827 s, effective latency 0.600--0.850 s,
  and motion to model readiness 0.176--0.258 m. All ten smoke plots were produced. GUI smoke on
  transition 1 visibly showed illuminated Jackal motion, OLD/FRESH/actual, observation/P/B, and
  live phase. Smoke is not primary scientific evidence and did not change the frozen protocol.
- **Validation/resources:** Full repository suite passed `357`; compileall, launcher `bash -n`,
  `git diff --check`, and generated-data ignore checks passed. The filesystem had 1.6 TiB free;
  the smoke episode used 5.83 MiB, giving a conservative sub-1-GiB 84-episode data estimate before
  primary collection (plots/replay products excluded from the per-episode estimate).
- **Next provenance step:** Commit and push the frozen collector, then run the entire primary grid
  from a separate clean worktree at that exact collector SHA. Primary results will be documented in
  the second result-only commit.
- **Unrelated changes:** Existing user edits to
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` remain preserved and will not be staged.

## 2026-09-10T02:31:00+09:00 — DATA-02 frozen primary collection and decision

- **Immutable collection:** Ran the complete 12-template x 7-variant primary grid from a separate
  clean detached worktree at collector commit
  `254ab6f07ce69847b8612bc40082a28adf5f45bd`. Run
  `data/data02_online_successive_v1/data02-online-successive-primary-v1/` completed all 84 planned
  episodes and 480 actual transition attempts. Frozen config SHA-256 was
  `b92f4ddc1ba7e80d5544573ffedd8b8175e6b92ae79c8d79637cbb3e17c5a313`; protocol SHA-256 was
  `883099b35d0cec8724b08367057cd6b008b7d3eb3a96190a8ba36e14199e1868`.
- **Persistent model/execution:** One LightNav process built the model once, reset its history 84
  times, served 564 predictions, and exited cleanly with zero process restart or OOM. The actual
  official Jackal remained wheel-driven through the frozen `TrajectoryFollower` + Stage 0-D
  `pi_strong` + `DifferentialController` stack; no controller, physics, LightNav, timing-gate, or
  scenario changes were made after the collector freeze.
- **Exact status result:** 248 `ELIGIBLE_MOVING`, 219 `TIMING_INVALID`, 4
  `EXECUTION_COLLISION`, 6 `OLD_EXHAUSTED`, and 3 `MODEL_STOP`; all four other protocol statuses
  were zero. Timing-invalid and terminal contexts remain stored rather than being retried or
  silently discarded.
- **Eligible characterization:** Host latency min/median/p90/p95/max was
  0.470697/0.569578/0.837373/0.844859/0.851770 s; effective latency was
  0.500000/0.600000/0.850000/0.850000/0.950000 s; inference translation was
  0.076784/0.188265/0.269348/0.285378/0.330803 m. There were 45 unique OLD raw chunks, 59 unique
  FRESH raw chunks, and 96 unique ordered raw pairs, but the largest pair occupied 93/248 = 37.5%.
  Geometry was 169 straight-like, 39 positive-turning, 34 negative-turning, and 6 other;
  difficulty was 80 benign, 123 intermediate, and 45 challenging.
- **Split and decision:** Bipartite episode/raw-pair components yielded 213 development and 35
  held-out eligible transitions with zero episode leakage and zero exact raw-pair leakage. The
  held-out side missed the required size and meaningful full scenario/geometry coverage. Sample
  size (248 < 300), duplicate domination (37.5% > 20%), and split feasibility failed; raw-pair,
  chunk, geometry, difficulty, and artifact gates passed. Exact decision:
  `DATA02_COLLECTED_BUT_DIVERSITY_INSUFFICIENT`. EXP-02D is not authorized.
- **Artifacts/visual review:** Strict validation reconstructed all 84 frame streams and all 480
  transition transforms/times/hashes with zero errors. Generated all ten required figures and a
  deterministic 30-transition representative manifest. Saved-only GUI smoke on
  `episode_000014_transition_04` visibly showed the official Jackal moving through Hospital with
  blue OLD, magenta FRESH, green actual, observation/P/B markers, and live phase changes; the
  validated non-black capture is `gui_replays/episode_000014_transition_04.png`. Replay performs
  neither LightNav inference nor physics re-execution.
- **Regression validation:** Full repository suite passed `357`; compileall, new launcher
  `bash -n`, strict dataset validator, raw-hash verification, generated-data ignore, clean
  collector-worktree, and `git diff --check` all passed. The ignored primary run is about 857 MiB.
- **Claim limitation:** This is a scenario-stratified coverage dataset, not an estimate of natural
  navigation frequencies. It does not validate instruction satisfaction, general navigation
  quality, collision avoidance, causal tire/contact effects, or a reconciliation formulation.
- **Unrelated changes:** Pre-existing user edits to
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` remain preserved and excluded from this result
  commit.

## 2026-09-10T13:25:12+09:00 — DATA-02 v2 independent extension and combined assessment

- **Repository/provenance:** Work started after fetching local/origin `main` at
  `2362ae3dd8a2c9f8906b89690422d750a63daa14`. The code/config/test freeze was committed and
  pushed as `ee87d3897f03a81371af494ed258fe263d750ebe` (`data: freeze independent online extension
  cohort`). The 168-episode primary then ran from a clean detached worktree at exactly that SHA;
  recorded collector status is empty. Existing user edits to the two Stage 0 configs remained
  unstaged and unchanged.
- **v1 preservation and diagnosis:** The strict read-only validator reconfirmed all 84 episodes,
  480 attempts, 248 original eligible contexts, transforms, clocks, P/B, successive chains, and
  hashes with zero errors. No v1 file, status, or `[0.90, 1.10]` RTF threshold changed. Of 219
  timing-invalid v1 attempts, 158 were below and 61 above the gate. Saved associations do not
  support model latency alone: response-detection delay versus RTF was `r=-0.561`, while
  simulation-ready latency was `r=0.506`. The v1 loop synchronously persisted PNGs; offline median
  encode/hash estimates were 14.506/0.049 ms. These are descriptive, not causal findings.
- **Technical-only cleanup:** v2 polls immediately after camera capture and defers identical
  lossless PNG encode/write/hash work until episode end. Capture ordering, exact LightNav input,
  output, controller/physics, trigger, clocks, and RTF gate remain unchanged. Across primary,
  14,813 frames were persisted after their episode loops; episode-median mean persistence cost was
  22.017 ms/frame. This timing comparison is not a controlled scientific effect estimate.
- **Frozen independent bank:** Declared 24 new templates before inference, exactly four each for
  straight/left/right/doorway/detour/compound, with the same seven variants (168 episodes). All
  168 initial states passed collision/settling preview. IDs are new, family balance passes, and no
  candidate met the conjunction of `<0.4 m`, `<0.15 rad`, and semantic equivalence to v1.
  Selection used no LightNav output. Frozen config/protocol SHA-256 values are
  `8e5ed277729adb479e15309f9843780e22138080e0560b62ea227560245dfbe4` and
  `f395409b9fb86e0671f12e6b216fe503e2829274e43c965798c1e74b40b2d064`.
- **Full v2 collection:** Run
  `data/data02_online_successive_v2/data02-online-successive-extension-v2/` completed all 168
  planned episodes and 999 actual transition attempts in one invocation. One persistent model
  build served 1,167 predictions with 168 history resets, zero process restart, and no OOM.
  Statuses were 711 `ELIGIBLE_MOVING`, 282 `TIMING_INVALID`, 4 `OLD_EXHAUSTED`, 1 `MODEL_STOP`,
  and 1 `EXECUTION_COLLISION`; every other status was zero. Strict validation reconstructed every
  frame stream/transition with zero errors.
- **v2 characterization:** Eligible v2 contains 52 unique OLD, 53 unique FRESH, and 113 unique
  ordered raw pairs; 18 pair identities overlap v1. Geometry is 617 straight-like, 55 positive,
  34 negative, and 5 other. Difficulty is 605 benign, 55 intermediate, and 51 challenging. The
  largest raw pair is 500/711 = 70.323%; it was retained unchanged.
- **Combined result:** Built read-only reference corpus
  `data/data02_combined_v1_v2/data02-combined-v1-v2-final/` without copying raw data. It contains
  1,479 attempts, 959 eligible, 82 unique OLD, 94 unique FRESH, and 191 unique pairs. Geometry is
  786/94/68/11 straight/positive/negative/other; difficulty is 685/178/96
  benign/intermediate/challenging. One pair is 593/959 = 61.835%.
- **Split and gates:** Episode-or-exact-pair connectivity yielded 12 components; the largest has
  920 eligible transitions. Whole-component assignment produced 39 development and 920 held-out
  transitions with zero episode and raw-pair leakage, but missed the 25% held-out target and
  development lacks straight-family coverage. Gates A/B/D/E/F/H pass; C duplicate domination and
  G isolated-split feasibility fail. Exact decision:
  `DATA02_COMBINED_DIVERSITY_INSUFFICIENT`. EXP-02D remains unauthorized.
- **Plots and GUI:** Generated and visually inspected all ten required combined plots; pair-rank,
  component-size, and split plots expose the failure. A saved-only real Isaac GUI replay of
  challenging representative `episode_000061_transition_04` visibly showed Hospital, the official
  moving Jackal, blue OLD, magenta raw FRESH, green actual, observation/P/B markers, and live
  phases. It exited cleanly and saved a non-black capture; it ran no inference or physics
  re-execution.
- **Validation:** Full suite passed `365`; compileall, launcher `bash -n`, `git diff --check`, v1
  strict validator (84/480), v2 strict validator (168/999), and combined validator (1,479 source
  hashes plus 11 combined artifacts) all passed with zero errors. v1, v2, and combined generated
  roots are Git-ignored; the primary collector worktree remains clean.
- **Claim boundary:** This is collection, characterization, duplicate control, and split
  diagnosis only. It does not validate instructions, estimate deployment frequencies, establish a
  causal timing effect, evaluate reconciliation, or justify inspecting/tuning against held-out
  transitions.

## 2026-09-10T15:49:49+09:00 — Short saved DATA-02 collection GUI demo

- **Repository and scope:** Started from local/origin `main` at
  `7ad6d1a3406b910e5097191d8732331280f56763`. Added a presentation-only mode to the existing
  DATA-02 saved replay plus the one-command launcher
  `./scripts/isaac/run_data02_collection_demo.sh`. No optimization, reconciliation, EXP-02D,
  LightNav invocation, recollection, controller action, physics replay, or scientific result
  change was made. The existing user edits to the two Stage 0 configs remain unstaged and
  untouched.
- **Default saved evidence:** The launcher selects immutable v2 run
  `data/data02_online_successive_v2/data02-online-successive-extension-v2`, episode
  `episode_000061`, transition `4`. The loader verifies stored hashes for OLD, raw FRESH,
  transition actual poses/telemetry, and observation RGB; checks strict timing, P-before-B,
  OLD-active in-flight samples, and post-switch FRESH activation; and returns read-only copies.
  It writes captures only to the separate ignored `data/data02_collection_demo/` root.
- **Presentation:** The default 12-second mapping is OLD EXECUTING (0–2 s), FRESH INFERENCE while
  OLD continues (2–5 s), FRESH READY (5–7 s), OLD-to-raw-FRESH switch (7–10 s), and FRESH ACTIVE
  (10–12 s). The GUI labels this `PRESENTATION-TIME REPLAY` and
  `NOT REAL-TIME SCIENTIFIC TIMING`, while retaining saved `t_obs=21.533334456384182 s`, reported
  model latency `0.43086534799658693 s`, effective latency `0.5000000260770321 s`, switch time
  `22.033334482461214 s`, and inference motion `0.15539962298348772 m`.
- **Visual content:** Stable Hospital/Jackal viewport with blue OLD, magenta raw
  observation-anchored FRESH, green saved actual history, yellow observation, orange P, red B,
  FRESH/P/B heading arrows, a large phase panel, inference progress, exact saved RGB frame 81,
  and a minimal event timeline. The raw FRESH path remains anchored at observation and is never
  translated to B.
- **GUI validation:** The exact default launcher completed all five phases automatically in
  `12.025 s` and saved the non-black capture
  `data/data02_collection_demo/20260910T065343Z/episode_000061_transition_04.png`. Direct visual
  inspection confirmed Hospital, the official Jackal mesh, and all final paths/markers were
  visible. An initial implementation reproduced the historical lines-without-robot symptom;
  changing display refresh from plain `APP.update()` to render-only `World.render()` synchronized
  articulation kinematics without a physics step and made the Jackal visibly follow saved poses.
- **Validation:** Full repository suite passed `374`; focused demo suite passed `9`; compileall
  for `src scripts tests`, shell syntax for all involved DATA-02 launchers, and
  `git diff --check` passed. The GUI code contains no `world.step`, controller application,
  DifferentialController construction, or LightNav client construction. Existing headless and
  saved replay behavior retains its defaults because the new path is gated by explicit `--demo`.

## 2026-09-10T18:08:39+09:00 — DATA-02 saved high-motion GUI replacement

- **Repository and supersession:** Started from local/origin `main` at
  `0617b75ea121ad40ac7eec57e006b3116c8c56b9`. Removed the active low-motion demo module,
  tests, launcher, and documentation and reverted only its presentation-specific changes in the
  DATA-02 collector. The historical entry immediately above remains intact. Added a standalone
  saved-data viewer; no history rewrite, LightNav, collection, controller, graph, reconciliation,
  physics replay, or scientific DATA-02 artifact change was made. The user's unrelated edits to
  the two Stage 0 configs remain unstaged and untouched.
- **Deterministic saved-data scan:** Hash-checked both immutable v1/v2 collections. Of `959`
  `ELIGIBLE_MOVING` transitions, `813` had valid P/B timing, pre-observation telemetry, post-switch
  telemetry, and actual saved motion; the other `146` lacked a post-switch sample window. The
  top-decile inference-motion threshold was `0.24825657265982382 m` (`82` candidates). Ranking
  those candidates first by full replay path deterministically selected v2 `episode_000062`,
  `transition_03`, `STRAIGHT_LIKE`.
- **Motion evidence and comparison:** The selected saved transition has inference translation
  `0.3443565605774114 m`, inference path length `0.34435678710392725 m`, full-window path
  `1.4108913846051951 m`, net displacement `1.4108906013484364 m`, absolute endpoint yaw change
  `0.00048574010784285804 rad`, `|delta_v_des|=0.03592780662560641 m/s`, and
  `|delta_omega_des|=0.0029971625633567174 rad/s`. Versus the previous default, inference
  translation is `2.216x`, full path `1.754x`, and net displacement `2.043x` larger. Generated
  ranking/selection files are under the ignored `data/data02_collection_demo_selection/` root.
- **Presentation:** The default 15 seconds map 236 immutable episode samples from saved simulation
  time `19.80000103265047` to `23.716667903587222 s` into four phases: OLD approach (0–4 s),
  FRESH inference while OLD continues (4–9 s), ready/switch hold (9–11 s), and post-switch motion
  (11–15 s). The camera is fixed and dynamically fit to put the actual replay at an estimated
  `63.4%` of the viewport. Thick blue OLD, magenta raw FRESH, growing green actual, fixed yellow
  observation footprint/heading, orange P, red B, Hospital geometry, and prominent motion text are
  shown. Smooth display uses only linear XY and shortest-wrapped-yaw interpolation between adjacent
  saved samples; no spatial scaling is used.
- **GUI validation:** The exact default launcher completed all four phases. An initial capture
  check correctly rejected a visually stationary articulation despite moving telemetry; the final
  implementation moves a parent USD display transform so the official Jackal mesh itself visibly
  traverses the scene without a physics step. The final inspected run is
  `data/data02_high_motion_demo/20260910T090924.941474Z/`. Its before/during/after displayed poses
  moved `0.319115 m` and `0.525728 m`; visual inspection confirmed obvious Jackal displacement,
  the observation ghost left behind, growing green trail, fixed Hospital references, distinct
  OLD/FRESH, and visible P/B. Human-visible motion verdict: **YES**.
- **Integrity and validation:** v1/v2 full-tree digests remained
  `b7041d4e42bfec2530eb6b9b0fd57e6e5cbe00ec5ad8818ca36f6897b100923c` and
  `ae22a160f94284d094efb5e1102d023300be77b3f42c767330e0614807c59462`. Full pytest passed
  `373`; focused high-motion tests passed `8`; compileall for `src scripts tests`, shell syntax for
  the new launcher, and `git diff --check` passed.

## 2026-09-11T01:44:29+09:00 — EXP-02D protocol freeze and active-OLD GUI correction

- **Repository and scope:** Started from local and fetched `origin/main` at
  `bad40aa60743da26f1b7263475ececfeb0e767f2`. Added the development-only EXP-02D formulation,
  frozen evaluation and representative rules, offline result validator/plots, and saved-evidence
  GUI. Historical M4 is called through the production implementation and is not modified. No
  LightNav, system Python, ROS 2, CUDA, Isaac Sim, historical generated evidence, or source
  DATA-02 artifact was changed. The user's unrelated edits to the two Stage 0 configs remain
  unstaged and untouched.
- **Isolated formulation:** `M0_RAW` is the raw selected suffix; `M1_HISTORICAL_M4` is the exact
  historical entry + fresh-motion + `B->F_k` direction + yaw objective; `M2_NO_DIRECTION` removes
  only direction; and `M3_LOOKAHEAD` changes only that factor to fixed-radius `B->F_q`, where one
  frozen follower call at exact `B` supplies raw `k/q`. All weights remain `1.0`, scales remain
  `0.10 m`, `0.10 rad`, and `0.10`, and the historical solver settings remain frozen. The
  controller command score is evaluation-only and never enters the objective.
- **Dry reconstruction:** Hash-checked all `959` v1+v2 `ELIGIBLE_MOVING` transitions; all `959`
  reconstructed and `0` had undefined direction geometry or input failure. Raw follower command
  reconstruction had maximum absolute error `0`; the corpus contains `191` exact ordered raw-pair
  groups. The observed distributions were
  `k={0:823, 1:133, 2:1, 5:2}`, `q={2:818, 3:133, 4:1, 9:7}`, and
  `q-k={2:951, 4:3, 7:1, 9:4}`. These counts are observed dry-validation output, not hard-coded
  corpus gates. The primary run has deliberately not yet occurred; it must run once from the
  clean protocol commit recorded after this entry.
- **Correct active-OLD semantics:** Added a shared strict loader that displays only the contiguous
  interval in which the selected transition's `old_chunk_id` was active. For transition `i>0`,
  the preceding saved `B` is an explicit activation-boundary event followed by current-OLD
  telemetry; transition zero starts at its earliest bootstrap-OLD sample. Every displayed saved
  telemetry row belongs to the current OLD and the path ends exactly at selected `B`. No
  previous-chunk or post-switch actual path is shown.
- **Real corrected GUI validation:** The deterministic corrected high-motion default is immutable
  v1 `episode_000007/transition_03`, current OLD `chunk_03`, with activation
  `20.133334383368492 s`, observation `20.783334417268634 s`, P
  `21.63333446159959 s`, and B/switch `21.733334466814995 s`. Its unscaled current-OLD active path
  is `0.5807040935130896 m` (`0.5807038644341246 m` net). The real Isaac GUI run
  `data/data02_high_motion_demo/20260910T161500.312194Z/` visibly showed the official Jackal move
  from activation through B, with uncluttered blue OLD, green current-OLD actual, magenta raw
  FRESH, and observation/P/B markers. It contained `96` current-OLD telemetry rows plus the
  explicit activation boundary and no adjacent-chunk trail.
- **Validation before protocol commit:** Full repository pytest passed `429`; EXP-02D/high-motion
  focused tests passed `57`; compileall for `src scripts tests`, `bash -n` for the new/modified
  launchers, and `git diff --check` passed. Strict v1 (84 episodes/480 transitions), v2 (168/999),
  and combined (1,479 attempts/959 eligible) validators passed. The generated EXP-02D root is
  Git-ignored, and optimized candidates remain offline counterfactuals that are never physically
  executed by the GUI.

## 2026-09-11T02:08:27+09:00 — EXP-02D invalid primary attempt and protocol correction

- **Invalid attempt:** Protocol commit `67d0444b1bed7bc8b431c91d6545716cf78f9f28` was pushed and
  one clean-worktree attempt was generated as
  `exp02d-lookahead-primary-20260910T165213Z`. The solver completed `959/959`, but the strict
  post-run validator initially exposed missing `host_latency_s` declarations in its CSV/input
  schemas and an invalid `math.isclose(..., atol=...)` call. Correcting those read-only checks
  allowed all 12,489 artifacts to be inspected and exposed a substantive protocol mismatch:
  geometry-correlation bootstrap draws used `bootstrap_seed + 1` instead of the single frozen
  config seed `20260910`.
- **Disposition:** No scientific result from that attempt is retained or interpreted. It was
  removed from the official ignored output root and recoverably quarantined at
  `/tmp/exp02d-invalid-primary-20260910T165213Z`. Immutable DATA-02 sources were not touched.
- **Correction:** The geometry cluster bootstrap now consumes exactly the configured seed, and
  the strict validator requires that seed/repetition/unit metadata. CSV emission and validator
  field order now have a direct regression assertion; input-reference latency fields are included
  in strict validation, and numeric closeness has regression coverage. A new protocol commit and
  clean worktree are required before the replacement one-time primary execution.

## 2026-09-11T03:56:15+09:00 — EXP-02D valid primary, regime analysis, and final GUI evidence

- **Valid replacement primary:** Committed and pushed the technical protocol correction as
  `f25fda2877bcfda8a599b739c252ab174d62f5b9`, then generated exactly one replacement primary from
  a clean detached worktree at that SHA. The canonical ignored result is
  `data/exp02d_lookahead_direction/exp02d-lookahead-primary-20260910T171139Z/`. Strict validation
  recomputed `959` `ELIGIBLE_MOVING`, `959` input-reconstruction-valid, `959` solver-valid, zero
  input failures, zero undefined geometries, zero solver failures, `191` ordered-pair groups,
  `12,489` hashed artifacts, and all `10` predeclared plots. The invalid attempt documented above
  remains excluded from every result and claim.
- **Follower oracle and frozen mechanisms:** Exact raw first-command reconstruction error was
  `0.0` (tolerance `1e-12`). Observed distributions were
  `k={0:823, 1:133, 2:1, 5:2}`, `q={2:818, 3:133, 4:1, 9:7}`, and
  `q-k={2:951, 4:3, 7:1, 9:4}`. M0 is raw; M1 is unchanged historical
  entry + `B->F_k` direction + yaw + fresh-motion; M2 removes only direction; M3 changes only the
  direction target to fixed-raw-radius `B->F_q`. All factor weights are `1.0`, scales are
  `0.10 m / 0.10 rad / 0.10`, and the existing right-local central-difference LM solver remains
  frozen. `J_cmd` is evaluation-only and no candidate was physically executed.
- **Aggregate command evidence:** Pair-balanced/transition-weighted mean `J_cmd` was
  RAW `1.393617454/0.536701738`, M1 `0.917927768/0.633044601`, M2
  `1.208490073/0.483677523`, and M3 `0.872420716/0.381861584`. Pair-cluster bootstrap differences
  were M3-RAW `-0.521196739` (95% CI `[-0.761689581,-0.279911498]`), M3-M2
  `-0.336069357` (`[-0.554956379,-0.109996964]`), and M3-M1 `-0.045507053`
  (`[-0.150310910,0.066939864]`). This is development-corpus immediate-command evidence, not a
  held-out generalization or navigation result.
- **Success and negative evidence:** Among `685` raw-BENIGN transitions, M1/M2/M3 broke
  `662/0/1`; M3 preserved `684` and satisfied `M4_FALSE_CORRECTION_RESCUED` in `661`. Among `96`
  raw-CHALLENGING transitions, M3 rescued/improved/mixed/worsened `42/17/31/6`. M3 was lower than
  RAW and M2 in the CHALLENGING pair-balanced bootstrap, but indistinguishable from M1; in the
  INTERMEDIATE partition its frozen differences versus every comparator had CIs above zero. The
  geometry-benefit association was modest (`r_pair=0.222586234`, 95% CI
  `[0.092550510,0.342933471]`) and is not causal. R1 contained `807` transitions and concentrated
  the false-correction rescues; R2 contained only `3` mixed turning cases; R3/R5/R6 contained
  zero; R4 contained `2` high-deformation improvements. M1/M2 were nearly rigid under the
  best-fit diagnostic, while M3 retained nonzero rigid-fit residual; none proves downstream intent
  preservation or feasibility.
- **Frozen representatives:** S1 is `v1:episode_000016_transition_00` (RAW/M1/M2/M3
  `J=0.098274/0.879276/0.077402/0.079256`); S2 is
  `v1:episode_000027_transition_01` (`5.645091/1.701474/5.502338/0.382505`); F1 is
  `v1:episode_000044_transition_04` (`0.912903/9.854300/0.458701/9.886601`); and F2 remained
  exactly `F2_NOT_AVAILABLE`. Criteria were not relaxed. F1 is an
  `INTENT_CONFLICT_CANDIDATE`, not proof of instruction-intent violation.
- **Final real Isaac GUI:** Opened and visually inspected all four captures for every available
  representative. Final directories are `gui/S1_20260910T185116.949864Z/`,
  `gui/S2_20260910T184930.187443Z/`, and `gui/F1_20260910T185024.163005Z/` under the canonical
  primary. The official Jackal visibly changes pose in Phase A; at `B` it remains fixed while
  offline RAW/M1/M3 are revealed. All telemetry rows carry the displayed current OLD, the displays
  end at exact saved `B`, and no previous-chunk or post-switch actual row is present. Active
  intervals/path lengths are S1 `[17.0500008892,18.1333342791] s` / `0.388668168 m`, S2
  `[18.1333342791,19.6333343573] s` / `0.560807006 m`, and F1
  `[22.1333344877,23.5333345607] s` / `0.227760859 m`.
- **GUI framing and provenance:** A dedicated 55-degree USD camera now read-backs exact
  `54.999999319/36.045051678 deg` horizontal/vertical FOV. Deterministic azimuth search, explicit
  Hospital open-view sectors, a visual-only fill light, and containment of every path, marker, and
  swept Jackal proxy avoid the former ceiling/rack views. Candidate fractions are S1 `0.679339`,
  S2 `0.350917`, and F1 `0.090259`; S1 meets the requested range, while S2 prioritizes an
  unobstructed scene and F1 is limited by its roughly `0.09 m` RAW/M1 geometry. No XY scaling was
  applied. Raised line/marker Z and lighting are presentation-only. Every manifest stores PNG,
  source/result, GUI runner/helper/launcher, quantitative-config, and GUI-render-config hashes.
- **Corrected collection viewer:** Reconfirmed the separate v1
  `episode_000007/transition_03`, current OLD `chunk_03`, output
  `data/data02_high_motion_demo/20260910T161500.312194Z/`: activation/observation/P/B are
  `20.1333343834/20.7833344173/21.6333344616/21.7333344668 s`; all `96` telemetry rows carry the
  current OLD ID, the activation-boundary pose is prepended separately, active path/net distance is
  `0.580704094/0.580703864 m`, and neither previous nor post-switch actual motion is displayed.
- **Final validation:** Full pytest passed `447`; compileall over `src scripts tests`, launcher
  `bash -n`, and `git diff --check` passed. Strict EXP-02D, DATA-02 v1 (`84/480`), DATA-02 v2
  (`168/999`), and EXP-02A validators passed. EXP-02C was regenerated successfully from a
  read-only canonical copy in `/tmp` with all `13` plots, leaving its source untouched. DATA-02
  validators rechecked all canonical hashes; previously recorded v1/v2 full-tree digests remain
  `b7041d4e42bfec2530eb6b9b0fd57e6e5cbe00ec5ad8818ca36f6897b100923c` and
  `ae22a160f94284d094efb5e1102d023300be77b3f42c767330e0614807c59462`.
- **Integrity and boundary:** Historical M4 production code/output and EXP-02C evidence remain
  unchanged. DATA-02's exact decision remains `DATA02_COMBINED_DIVERSITY_INSUFFICIENT`; EXP-02D is
  development-only. It cannot establish physical Jackal improvement, feasibility, obstacle or
  instruction success, correspondence quality, or generalization. The user's unrelated changes to
  `configs/stage0_jackal_controller_validation.yaml` and
  `configs/stage0_lightnav_single_chunk.yaml` remain untouched and unstaged.


## 2026-09-13T04:59:56+00:00 — EXP-02B archived OLD failure presentation

- **Request and starting state:** From fetched `main` / `origin/main` at
  `095f985c66874bd502f056a74e49006f10dca59d`, created a professor-facing GUI for the exact
  archived high-angular case (`k=3`, `raw_k`) with OLD spatial RMS 6.46 cm and mean
  commanded/measured omega 0.755/0.139 rad/s. The two unrelated Stage 0 config edits remain
  untouched and excluded from the commit.
- **Evidence integrity:** The presentation config freezes all six original diagnostic hashes.
  Its pure loader also checks the original EXP-01B source/config hash chain, CSV/NPY pose
  identity, strictly increasing saved time, wrapped ending-interval body velocities, and
  recomputed spatial/body/wheel metrics to absolute tolerance `1e-12`. Exact values reproduced:
  spatial RMS `0.06460064998428094 m`, commanded omega mean `0.7553427249409104 rad/s`,
  measured omega mean `0.13919805112548234 rad/s`. Rechecked all input hashes after capture.
- **Presentation:** Added `run_exp02b_failure_demo.sh`, an independent saved-record Isaac viewer,
  a small read-only evidence/timing helper, fixed source/presentation config, and 11 tests.
  Reuses the existing DebugDraw helpers and official Jackal asset. Replays 89 recorded world
  SE(2) poses (88 ending intervals, 1.466666743 saved seconds) over 12 wall seconds, with
  restart and pause/resume buttons. Blue OLD and cyan actual share visual Z=0.72 m and retain
  exact world XY. Recorded SE(2) supplies body position/yaw; wheel rotation, roll/pitch, and
  vertical dynamics are not reconstructed. Display Z=0.15 m is explicitly visual.
- **Scope and clocks:** The presentation stops at B_reproduced before exact reset and displays
  no post-switch actual. No inference, controller, optimization, or physics re-execution is
  performed. Rendering initializes once, then the timeline is paused and its simulation clock
  is asserted constant. Original observation/readiness/execution timestamps, diagnostic clock,
  source frames/units, identity XY transform, and display-time mapping are recorded separately.
  No waypoint timestamps or interpolated poses are invented. The initial spatial gap is about
  15.24 cm and the final gap is 4.66 cm; the displayed 6.46 cm is full-interval RMS, not evidence
  that deviation continually grows. The physical root cause remains unresolved.
- **Actual GUI validation:** Final output:
  `data/exp02b_failure_demo/20260913T045548.871626Z/`. Inspected the moving Jackal, growing cyan
  trail, planned blue OLD, nested saved/reproduced-B markers, full-record statistics, original
  simulation-time plots, current ending-interval telemetry, and pre-reset hold. Explicit docking
  fixes the hidden floating panel; the editor's property/content panels are hidden for space.
  Clicked Restart, Pause, and Resume in the real GUI; pause and resume both reported saved
  index 15, while the body/time display held and then progressed. Saved start/motion/final
  viewport PNGs, standalone plot, hash manifest, and an unmodified XComposite capture of only
  the Isaac window (`presentation_window.png`). `gui_validation.json` records the review.
- **Checks:** Full pytest `458 passed in 21.22s`. An initial sandboxed run had 456 passes and
  two existing local-socket PermissionErrors; rerunning with permitted local Unix sockets
  passed all 458. New helper tests cover unequal-interval playback, no future/interpolated
  samples, invalid clocks, wrapped yaw-rate reconstruction, immutable arrays, telemetry/pose
  mismatch, and hash/summary tampering. Compileall, launcher `bash -n`, and whitespace checks
  passed. No original experiment output or user config was overwritten. Generated outputs
  remain under the ignored `data/` tree. README and diagnosis documentation include the command,
  legend, source contract, and explanation for the professor.

## 2026-09-13 — Check the current frozen controller against nominal execution

- **Request and boundary:** User requested checking the effect of the currently modified
  controller, following questions about DATA-02 cases 14 and 44. Compared correction on/off
  with new Isaac physics runs; did not tune or change the execution controller, follower,
  objective, original VLA outputs, or historical experiment results. Initial branch was `main`,
  HEAD `b9efd18`, origin `https://github.com/minju5054/se-3-reconciliation.git`; refreshed
  origin/main and confirmed no divergence before committing. Preserved/excluded the user's
  camera edit in `configs/stage0_jackal_controller_validation.yaml` and playback-speed edit
  in `configs/stage0_lightnav_single_chunk.yaml`.
- **Implementation:** Added versioned `configs/controller_effect_check.yaml`, a small
  `controller_effect_check.py` recorder/metric helper, an independent Isaac runner/launcher,
  a strict validating plot summarizer, and 10 synthetic timing/metric tests. Existing
  Stage 0-E runtime and strict telemetry validators are reused. No new SE(2) operation,
  optimizer, correspondence factor, LightNav modification, or acceptance gate was added.
  README links `docs/CURRENT_CONTROLLER_EFFECT_CHECK.md`, which records commands, results,
  frame/timing conventions, GUI legend, and claim limits.
- **Frozen controller:** Selected Stage 0-D `pi_strong`; model SHA-256
  `40821584e14a3f444fdd19ff27acc03e752ec3d0c5f2e8635b824a1419a81464`;
  controller source SHA-256
  `0bc4a97b4dac6407fe9bedd9d37451ab2d53a19c8efc396d84f496b8115c04a5`.
  Checked the new runtime's source and input hashes before/after execution. The additional
  run protocol includes the exact uncommitted runner/helper hashes as well as starting HEAD.
  DATA-02 follower values match the frozen source protocol. Nominal means disabling this
  correction in the same runtime, not checking out an entire historical software version.
- **Existing suite, 66 trials:** Ran `run_jackal_closed_loop_execution_validation.sh
  --run-id controller-current-20260913T062219Z`, then the existing summarizer. Primary output
  is `data/stage0/closed_loop_execution_validation/controller-current-20260913T062219Z/`.
  Seven controlled fixtures, one composite and three saved OLD references, two modes,
  three reset repetitions each. Gentle-left position RMS was 11.6694 -> 0.3478 cm and
  S-curve 9.6530 -> 0.9833 cm. Strong-left/right calibrated yaw RMS remained
  0.173091/0.170283 rad above the unchanged 0.10 rad threshold. Controlled scenarios
  passed 5/7 (minimum 6); composite and all three saved EXP-02B references passed. Exact
  overall decision remains `EXECUTION_PLATFORM_NOT_READY`.
- **Additional suite, 36 trials:** Ran `run_current_controller_effect_check.sh --run-id
  controller-current-20260913T062219Z-r2`; primary output is
  `data/controller_effect_check/controller-current-20260913T062219Z-r2/`. Same saved
  world OLD/previous activation B pose for v1 `episode_000014_transition_04` and
  `episode_000044_transition_04`, nominal/calibrated, three resets, goal or 8-second timeout.
  Case 14 position RMS was 6.7107 -> 1.4488 cm; calibrated goal 3/3 at 5.1 s. Case 44
  was 21.8004 -> 17.3218 cm; calibrated goal 3/3 at 7.5 s, but maximum deviation worsened
  27.0405 -> 37.3327 cm. Different goal/timeout durations are explicit. Original online
  Hospital history, velocity, contact state and PI integral were not restored; this is
  a reset-state flat-ground comparison, not full reproduction of the original episodes.
- **Rotation engineering fixtures:** Four fixed v=0 commands (omega -1.5/-0.3/+0.3/+1.5
  rad/s), two modes, three repetitions; 0.5 s initial stop, 2 s active, 0.5 s final stop.
  Last-0.5-second measured omega for nominal -> calibrated was
  -0.619708 -> -2.535843, -0.057984 -> -0.435329, +0.096252 -> +0.456467,
  +0.483088 -> +2.411332 rad/s. Active omega RMS error increased in three of four
  conditions; -0.3 improved slightly. Active-end root XY displacement was respectively
  12.4049 -> 29.9874, 3.5858 -> 11.4288, 2.8033 -> 10.9512, 11.6122 -> 10.2176 cm.
  The last condition had a larger calibrated peak displacement despite smaller end
  displacement (11.6122 -> 17.0959 cm). These are simulator engineering checks with
  synthetic commands, not reconciliation research evidence or real-robot evidence.
  Correction on/off does not isolate feedforward vs P/I/limits or uniquely identify the
  physical cause of original case 44. It does not invalidate all DATA-02 recordings.
- **Figures and GUI:** Validating new summarizer generated and visually inspected
  `plots/saved_old_comparison.png`, `plots/rotation_response_and_drift.png` and
  `plots/comparison.json` with three-run means/min/max/std and raw/plot hashes. Repetition
  metrics agree at reported precision. Ran the existing S-curve GUI at real-time factor
  0.5 with `--no-hold`; output is the 66-trial run's
  `gui_metadata/controlled-20260913T063329Z/`. Visually inspected `final_viewport.png`:
  Jackal, full blue reference, orange-red nominal and green calibrated paths are visible.
  GUI's 1081 nominal and 397 calibrated world-pose rows exactly equal headless repetition
  00 (maximum absolute difference 0). `gui_validation.json` records comparisons, source
  hashes, display semantics and visual review. GUI is a separate diagnostic physics run,
  excluded from the 102 quantitative trials; report includes the command to keep it open.
- **Data integrity and clocks:** Raw telemetry/poses/controller states, derived metrics,
  and plots are separate under ignored `data/`. No raw recording is committed. Source
  hashes for both DATA-02 cases were verified after running; no source was overwritten.
  World X/Y metres, CCW yaw radians about +Z, no additional transform; cm plots use m*100.
  Physics 1/60 s, control 0.1 s, row zero is settled observation; each later row describes
  its ending execution interval. Feedback uses the previous complete control interval.
  Protocol/metadata record source activation/observation/switch times and UTC execution
  times; no new inference or readiness event and no invented waypoint timing.
- **Validation and incomplete attempt:** Related tests passed 27; full suite passed
  `468 passed in 20.12s`; compileall, launcher `bash -n`, whitespace checks, the existing
  66-trial validator and all 36 additional trial/raw-hash validations passed. First
  additional output `controller-current-20260913T062219Z/` stopped on missing required
  diagnostic metadata after its first nominal trial. It was preserved as incomplete,
  excluded from results, and fixed by supplying the existing validators' semantic fields
  before the exclusive `-r2` rerun. Runner emits an explicit failure/completion marker
  because Isaac shutdown can obscure an exception's exit status.

## 2026-09-13 — Physically execute frozen EXP-02D candidates and show success/failure

- **User authorization and boundary:** User explicitly requested driving Jackal on EXP-02D
  corrected paths and showing success and failure in GUI. Added a separate post-primary
  execution diagnostic. Original EXP-02D candidate-only metadata, raw/derived paths,
  objective, solver, LightNav, controller and follower remain unchanged. Initial branch
  `main`, HEAD `0b57f45`, origin `https://github.com/minju5054/se-3-reconciliation.git`;
  fetched origin and confirmed no divergence. Preserved/excluded the existing user's
  Stage 0-B camera and Stage 0-C playback-speed config edits.
- **Implementation:** Added `configs/exp02d_physical_execution.yaml`, a Hospital runtime
  using unchanged Stage 0-E apply/reset/closed-loop execution, a headless runner and
  launcher, strict result/plot summarizer, physical-outcome bookkeeping, a live GUI and
  launcher, and a small paused-render helper. Added synthetic bookkeeping/display-clock
  tests; these fixtures are not experimental evidence. New Korean report is
  `docs/EXP_02D_PHYSICAL_EXECUTION.md`; README and historical EXP-02D documentation link it
  explicitly as a later diagnostic rather than rewriting past claims.
- **Frozen inputs and setup:** Source
  `data/exp02d_lookahead_direction/exp02d-lookahead-primary-20260910T171139Z/`;
  selected saved S1/S2/F1 RAW suffix and M3 arrays are hash-verified by the existing loader,
  copied unchanged, not reoptimized/resampled/translated, and B is not inserted. Source
  DATA-02 Hospital/Jackal assets and authored collisions are used without an extra ground
  plane. Each execution requests the saved switch B world XY/yaw, world/wheel reset and
  1-second settling with new follower/PI state. Original velocity, contact history and
  online OLD execution are not restored. No new inference, planner or gain tuning.
  Source observation/readiness/switch and new UTC/execution clocks are explicit.
- **Current controller:** Stage 0-D `pi_strong`, model SHA-256
  `40821584e14a3f444fdd19ff27acc03e752ec3d0c5f2e8635b824a1419a81464`, controller code
  `0bc4a97b4dac6407fe9bedd9d37451ab2d53a19c8efc396d84f496b8115c04a5`.
  Physical wheel radius/separation are `0.097999997437/0.375589996576 m`. Protocol freezes
  the entire source system, acceptance criteria, candidate, source/result/code hashes and
  config snapshot. Source and primary execution code hashes verified after all trials.
- **Completed primary:** `data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/`,
  generated with `run_exp02d_physical_execution.sh --run-id exp02d-physical-20260913T072000Z`.
  S1/S2/F1 x RAW/M3 x 3 reset repetitions = 18 valid new physical trials. Stop at existing
  follower goal (8 cm / 0.08 rad) or 18 seconds. Existing Stage 0-E absolute scenario
  gates classify each method; none were relaxed. Historic S/F labels do not force physical
  outcomes; absent outcome classes would remain null. Selection prefers S2/S1/F1 among
  actual M3 passes and F1/S2/S1 among actual failures, yielding S1 success and F1 failure;
  S2 is additionally displayed because it times out.
- **S1 physical success:** `v1:episode_000016_transition_00`. RAW/M3 position RMS
  `0.008246637/0.008105629 m`, yaw RMS `0.052102921/0.056479296 rad`, goal `3/3` for both,
  times `4.1/4.2 s`; both PASS. M3 maximum spatial deviation `0.022538873 m`, zero
  saturation. This shows an executable M3 example, not a substantial benefit over RAW.
- **S2 physical failure:** `v1:episode_000027_transition_01`. RAW reaches goal `3/3` at
  `16.8 s`, but yaw RMS `0.479679241 rad` fails. M3 position RMS is only `0.003200385 m`
  and yaw RMS `0.041648037 rad`, yet goals are `0/3`, endpoint error `0.750296464 m` at
  `18 s`; goal/final-position/progress gates fail. GUI shows the body stopped in front of
  a storage cart while the desired command remains forward. Obstacle interference is
  a plausible interpretation of the visible geometry, not a separately measured contact
  force/root-cause result. Small path distance/first-command score is not navigation success.
- **F1 physical failure despite reaching goal:** `v1:episode_000044_transition_04`.
  RAW/M3 position RMS `0.355985038/0.055295602 m`, yaw RMS `0.709716325/0.469072528 rad`.
  RAW times out `18 s`; M3 reaches goal `3/3` in `0.7 s` but fails yaw RMS and saturation
  (`0.714285714`, versus existing maximum `0.20`). The M3 path is only `0.151576376 m`.
  This is corrected FRESH execution after B, not re-execution of original case 44 OLD.
- **Figures:** Strict summarizer validates all 18 trials, raw hashes, references, protocol
  links and recomputed classifications. Generated per-case physical comparison PNGs with
  path, distance, yaw and numerical verdict; additional `--gui-plots` creates small primary
  XY insets with separate provenance. All plots preserve world XY metres; cm=m*100 and
  nearest-polyline distances are spatial, not waypoint-time errors. Three repetitions agree
  at reported precision; repeated reset fixtures are not independent generalization data.
- **GUI semantics and verification:** GUI performs new follower/controller/physics execution,
  with magenta M3, green new actual, gray RAW context, yellow B; optional RAW comparison
  records orange new actual. All path lines now share a floor overlay at visual Z=0.035 m
  to avoid apparent separation from perspective. A separately labeled primary XY inset
  reveals short F1 geometry obscured by the robot. GUI source initialization now uses the
  same first-case construction pose as the primary even when directly selecting F1/S2.
  It records its own telemetry, metrics, hashes, camera plan, captures, and comparison with
  primary repetition 00; equality is checked, never presumed. Initial comparison GUI
  `gui/2026-09-13T072233.212977_0000/` has exact primary pose equality for both methods of
  S1/F1. Direct-case development GUIs with a different construction pose differed; retained
  separately and not included in the 18-trial primary. Final delivery GUI is
  `gui/2026-09-13T075333.852691_0000/`, with S1/S2/F1 automatic executions and case buttons.
- **Display clock handling:** Captures strictly assert an unchanged physics clock. A
  render-only helper reasserts pause; editor Stop resets the clock to zero rather than
  advancing physics. An earlier strict hold treated that reset as an error; its actual
  `19.03333432599902 -> 0` log established the distinction. Interactive and between-case
  holds now report editor Stop and permit another explicit run; capture checks remain
  strict. No saved trial is overwritten. GUI slowdown changes wall pacing only.
- **Checks and incomplete outputs:** Full pytest `488 passed in 20.62s`; targeted
  bookkeeping/render tests passed 20, earlier relevant suite passed 50. Compileall,
  launcher shell syntax, report links, raw validators and whitespace checks passed.
  Initial headless attempts `exp02d-physical-20260913T070000Z` and
  `exp02d-physical-20260913T071600Z` were preserved incomplete (sensor display argument,
  then required scenario metadata); both excluded. Finished primary has explicit
  `EXP02D_PHYSICAL_COMPLETE`, complete summary and strict validation. All generated
  data/captures remain under ignored `data/`; no raw recording is staged.
- **Final delivery verification:** All three final GUI M3 pose arrays exactly match their
  primary repetition 00, with maximum absolute difference zero. All three strict GUI
  telemetry validators and source/candidate hash rechecks passed. Visually reviewed S1,
  S2 and F1 whole-window captures; correct body, case identity, verdict, current/primary
  metrics, legend and labeled primary XY inset are visible. Final F1 full-window image is
  `02_F1/presentation_window_reviewed.png` to avoid a transient partially updated text
  frame in the first whole-window capture. `gui_validation.json` records review and hashes.
  Whole-window PNGs are unmodified XComposite reads of only Isaac, not image composites.
  Native input attempts did not establish button callback activation, so no physical
  button-click test is claimed; `--all-cases` verified the shared GUI execution function
  for all three cases. The final GUI was left at the completed F1 view with case buttons.
- **Subsequent close:** Final status inspection found a normal GUI shutdown after an
  editor Stop, with `completed.json` recording all three cases and closure at
  `2026-09-13T07:56:51.012348+00:00`. Saved runs and reviewed screenshots remain available;
  the report includes commands to reopen each case.

## 2026-09-13 — matched pre-Exp02D objective execution and real GUI movies

- **Request:** User clarified they wanted actual driving video, questioned collision/
  threshold wheel-spin attribution to FRESH, and correctly requested same-episode
  historical-objective versus Exp02D comparison. Prior RAW/M3-only evidence does not
  isolate the objective change. Starting commit `88e9f0e`, branch main. Preserved the
  unrelated camera and playback edits in the two Stage0 configuration files.
- **Frozen matched protocol:** Added `configs/exp02d_objective_execution.yaml`; used the
  existing unmodified physical runner, runtime, calibrated controller and follower.
  Source remains `exp02d-lookahead-primary-20260910T171139Z`. All existing S1/S2/F1
  representatives × RAW/M1_HISTORICAL_M4/M3_LOOKAHEAD × 3 repetitions = 27 new trials.
  No positive rescue selection, optimization, VLA inference, obstacle removal, controller
  tuning, threshold change or source modification. Hospital authored collisions, original
  world XY metres/CCW yaw radians, reset at B, 1 s settling, physics 1/60 s, control .1 s,
  goal or 18 s timeout. Original online velocity/PI/contact history is not restored.
- **Command/output:** `./scripts/isaac/run_exp02d_physical_execution.sh --config
  configs/exp02d_objective_execution.yaml --run-id exp02d-objective-matched-20260913`.
  Output `data/exp02d_objective_execution/exp02d-objective-matched-20260913/`;
  `/tmp/exp02d-objective-matched.log` has COMPLETE and 27 valid trials. Source/candidate
  hashes and all primary execution-code hashes were rechecked after recording.
- **Honest outcomes:** S1 M1 and M3 both PASS (goals 3/3, position RMS 1.075/0.811 cm,
  duration 4.4/4.2 s). S2 both FAIL/goal 0/3 (position RMS 16.944/.320 cm, remaining
  endpoint distance .6428/.7503 m). F1 both FAIL despite goal 3/3 (yaw RMS .2505/.4691
  rad and saturation .7500/.7143). There is NO M1-fail/M3-pass representative here.
  These do not demonstrate a physical success-rate improvement. Repeated reset states
  are not independent episodes; success refers to each candidate's own endpoint.
- **Collision interpretation:** S2 RAW reaches its endpoint but fails yaw quality. M3
  does not reach its shifted endpoint (35.10 cm away from RAW's endpoint). Final desired
  v .3581 m/s, measured v -.000253 m/s, left wheel speeds 7.25/7.01 rad/s show wheel
  motion without body progress. GUI shows the storage cart obstruction. Contact-force
  attribution and obstacle-removal controls were not run; FRESH alone is not established
  as the cause. Explain original-path feasibility, correction displacement, tracking,
  and terrain/contact as distinct possibilities. Exp02D adds no obstacle/terrain factor.
- **Actual recording:** Added `exp02d_objective_video_gui.py` and its launcher. A runtime
  subclass adds only actual-state render/read operations, with unchanged parent reset,
  step, apply and loop. XComposite reads only the Isaac client window; no input injection
  or saved-pose playback. Capture every 3 physics steps (20 simulation fps), pause and
  assert unchanged clock while rendering/reading, then resume. Same camera for methods
  in a case. Original observation/readiness/switch times, new UTC, physics indices,
  timestamps, poses, wheel speeds, raw telemetry/frames and hashes are separate outputs.
- **Recorded runs:** S1 at `video_gui/2026-09-13T081351.078563_0000` (M1/M3);
  S2 at `video_gui/2026-09-13T081502.048785_0000` (RAW/M1/M3);
  F1 at `video_gui/2026-09-13T082056.943061_0000` (M1/M3).
  All 7 entire pose arrays exactly equal primary repetition 00. All 1,265 sampled frame
  times/poses match telemetry and hashes were validated before encoding. No failed or
  divergent physical recording is hidden. S1 pilot was usable as the final S1 recording.
- **Movies:** Installed imageio-ffmpeg 0.6.0 into `/tmp/exp02d-video-tools` with `uv pip
  install --python .venv/bin/python --target /tmp/exp02d-video-tools imageio-ffmpeg`;
  project environment/dependencies unchanged. The initial attempt with venv `pip` failed
  because pip is not installed. Added `encode_exp02d_objective_videos.py`: validated raw
  PNGs → H264 MP4 at 2x slow motion, 30 fps frame repetition only, no pose interpolation.
  Full-window aspect-preserving resize; side-by-side comparison aligns B/time zero.
  Finished methods show their labeled end-state card, final shared hold 2 s. Outputs
  `movies_S1`, `movies_S2`, `movies_F1`, with 7 individual + 4 comparison MP4s and manifests
  containing raw references, encoder version/hash, processing hash and exact commands.
  Encoding's unchanged frame-index rule was subsequently extracted into a pure function
  to test synchronization and end-card boundaries; manifests retain the actual earlier
  processing hash used for encoding.
- **Verification:** 27-trial strict validator; source/candidate and execution hashes;
  7 recorded-trial validators and exact pose equality; video frame time/pose/hash checks;
  all 11 MP4s decoded successfully. Paired S1/S2/F1/S2-RAW clips have 324/1140/108/1140
  frames at 30 fps (10.8/38/3.6/38 s). Reviewed raw GUI samples and decoded comparison
  samples: correct case, method, same sim time, visible moving body/wheels and paths.
  Opened S2 in Totem; MPRIS confirmed Playing at position 18.881 s with duration 38 s;
  queued other comparisons. Delivery audit is `delivery_validation.json`.
- **Tests/docs:** New synthetic tests cover captured-clock/pose mismatch, missing or
  reordered frames, aligned playback and final holds. Initial pytest auto-loaded a ROS
  plugin missing lark; disabling plugin autoload avoids that unrelated environment issue.
  Restricted full run had only two local socket permission failures; allowed-socket final
  run: `env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q`
  → **503 passed in 20.23 s**. Compileall, launcher bash syntax, report links and diff
  whitespace checks passed. Added Korean matched-video report and links from prior report
  and README. All raw data, frames, movies, dependencies remain outside Git.

## 2026-09-13 — search for an exclusive Exp02D success on a curved path

- **Request/scope:** User requested a non-straight episode where RAW and historical
  objective fail but Exp02D succeeds, shown in GUI. Started on main at `543245b`, inspected
  status/remotes and preserved both unrelated Stage0 configuration edits. Kept frozen
  Stage0E tracking criteria; explicitly distinguished goal arrival from tracking PASS.
  Communicated 30-degree/0.5-m search thresholds before physical execution.
- **Predeclared search:** `configs/exp02d_turning_search.yaml`: M3 reference arc >= .5 m,
  unwrapped waypoint-yaw excursion >=30 degrees, XY tangent-direction excursion >=30
  degrees using reference segments >=.02 m. Final M3 actual arc >=.5 m and yaw excursion
  >=30 degrees. Of the 959 saved transitions, 103 met yaw/arc and 87 also had geometric
  curvature. Rank by descending min(J_RAW,J_M1)-J_M3 then ID; execute ALL 87 with all
  three methods, not just the best-looking command-score cases. One-repeat screening is
  explicitly provisional; same numerical gates, repetition/required-goal count 1. Confirm
  exclusive passes under the original 3-repeat gates; prefer missing-goal baselines.
- **Implementation/provenance:** New hash-verified arbitrary-transition archive leaves
  frozen representatives untouched and matches their candidate arrays/source hashes exactly
  for S1/S2/F1. New search runner uses the unchanged Hospital runtime, current calibrated
  controller/follower and saved candidates; no optimization/inference/tuning. Same saved B,
  world XY metres/CCW yaw radians, 1 s settling, 1/60 physics and .1 s control, goal/18 s
  timeout, prior online velocity/PI/contact history reset. Scene construction stays at the
  historical S1 B and is recorded separately from trial B. Config, all 959 inclusion rows,
  ordering, code/source hashes, timestamps, raw references/telemetry and derived outcomes
  are preserved in new output. No raw or prior result was overwritten.
- **Executed:** `./scripts/isaac/run_exp02d_turning_search.sh --run-id
  exp02d-turning-search-20260913`; output `data/exp02d_turning_search/exp02d-turning-search-20260913/`,
  log `/tmp/exp02d-turning-search.log` has COMPLETE. 261 screening trials plus 9 confirmation
  trials. Screening tracking-pass counts RAW/M1/M3 = 40/58/39; own-endpoint arrival counts
  70/66/67. These describe a selected development subset, not unbiased success rates.
- **Selected:** `v1:episode_000016_transition_02`, GUI alias R1 in `confirmation_00`. This
  differs from earlier straight S1 transition_00. All methods reached their own endpoint
  3/3 times. RAW yaw RMS .10054325398 exceeds the unchanged .1-rad gate by only .000543
  rad (~.031 degrees); M1 .85255364055 also fails yaw. M3 .09244573229 passes all gates.
  Position RMS RAW/M1/M3 = .01502691/.09531314/.01341358 m; durations 4.7/13.2/4.1 s.
  M3 moves 1.1023 m with net yaw change 58.27 degrees, excursion 61.01 degrees; its
  reference tangent excursion is 50.88 degrees. Values repeat to reported precision.
  Report RAW's borderline failure prominently; this is not an exclusive goal-arrival
  rescue or strong general superiority over RAW. M1 travels 2.6418 m with 184.99-degree
  yaw excursion before settling at the endpoint.
- **Excluded alternative:** `v1:episode_000025_transition_02` had only M3 reach its own
  endpoint in screening, but M3 yaw RMS .3350 failed the quality gate. It was not relabeled
  as strict success or promoted to confirmation. All screening outcomes remain available.
- **GUI/video:** Extended the existing recorder to explicit protocol case labels and
  frozen arbitrary transition IDs, recorded scene-initialization pose and fallback indoor
  camera settings; legacy S1/S2/F1 loading remains unchanged. Added primary yaw RMS and
  gate to the GUI, showing RAW .1005 versus .1000 rather than just a FAIL label. Executed
  `--run .../confirmation_00 --cases R1 --methods M0_RAW M1_HISTORICAL_M4 M3_LOOKAHEAD`.
  `video_gui/2026-09-13T090619.141155_0000` records 95/265/83 real sampled frames; all three
  complete pose arrays exactly match confirmation repetition 00. Visually reviewed raw
  GUI and decoded samples at movie 3/9/18 s: same case/time/camera, actual curved motion,
  method labels, yaw gate and completed-state holds are visible.
- **Movie delivery:** Added optional `--three-way` to the existing validated encoder.
  `confirmation_00/movies/` contains three individual, two paired and one RAW/M1/M3
  side-by-side MP4 plus hashes/commands. Same 20-Hz physical sampling, 2x slowdown, 30-fps
  repeated frames, no pose interpolation, labeled end cards, whole-window resize. The
  three-way movie is 3840x778, 28.4 s / 852 decoded frames. All six videos decoded cleanly.
  Loaded the triple in Totem and issued playback; no screenshots are substituted for the
  requested driving video. New detailed Korean report links all movies and qualifications.
- **Validation:** New read-only audit rechecked all 261 screening trials, references,
  raw bytes, geometry, source hashes, gate outputs, search ordering and selection, plus
  the 9 confirmation trials. Primary execution code hashes unchanged after search/GUI.
  `audit/validation.json` records the audit and movie verification. Synthetic tests cover
  +/-pi yaw handling, world rigid-transform invariance, rejecting a straight reference
  with changing yaw, and excluding rotation without travel; no synthetic evidence claim.
  Targeted suite 62 passed; final whole suite **506 passed in 20.22 s** with local sockets
  allowed and unrelated ROS pytest plugin autoload disabled. Compileall, bash syntax,
  report links and diff checks passed. All datasets/frames/movies remain ignored by Git.

## 2026-09-13 — explain tracking gates, terminal rotations and data recollection scope

- User asked who set the gates, how yaw error is computed, why only M1 circles near its
  endpoint despite a shared controller, whether controller changes require fresh data, and
  whether a simplified TurtleBot is a valid research platform. Read-only diagnosis; no new
  physical trial, objective/controller/LightNav modification, or raw-data overwrite.
- Started at `62950e5` on main; inspected status/remotes and preserved both unrelated Stage0
  configuration edits. Located gate history at Stage0B `a5d88ee` / Stage0E `dd9ec20`; these
  are internal engineering criteria. Did not infer direct user authorship of the values from
  Git's author identity. Checked spatial segment projection, wrapped waypoint-yaw interpolation,
  whole-physics-sample RMS and worst-repeat gate semantics in existing implementation.
- Reconstructed all 660 original follower commands across 9 R1 confirmation trials using
  saved control-boundary articulation-root world XY/yaw, unchanged source follower settings,
  1/60-s physics / .1-s control conventions. All match saved desired commands at 1e-12 absolute
  tolerance. Derived script/configuration/input and processor hashes plus full phase rows saved
  separately under `data/exp02d_turning_search/exp02d-turning-search-20260913/audit/terminal_tracking_followup/`.
  Run command: `.venv/bin/python data/exp02d_turning_search/exp02d-turning-search-20260913/audit/terminal_tracking_followup/diagnose.py`.
  Output creation is exclusive; preserve existing `diagnosis.json` when rerunning.
- M1 first approach had distance .0807252 m at 6.1 s, .0797196 m at physics-only 6.1667 s,
  and .0800006 m at control-boundary 6.2 s. The 10-Hz follower missed the brief 8-cm radius
  crossing, specifically an opportunity to enter terminal yaw, not full goal completion
  (yaw was still out of tolerance). At 8.2 s M1 entered terminal yaw with 103.1-degree yaw
  error, left the position radius at 8.7 s, resumed forward approach at 9.3 s, and finally
  completed at 13.2 s. M3 entered terminal yaw at 4.0 s with 6.44-degree error, ended 4.1 s.
  Explained path/state/controller interaction and unlatched terminal mode, not a proven
  unique physical/contact/PI cause. Shared controller does not imply shared commands/state.
- Data guidance: preserve prior frozen observations and inference for fixed-input comparisons;
  rerun all methods with the same new controller for physical claims. New online-system claims
  need a separately collected cohort because actual OLD, B, image history and FRESH can change.
  If follower/lookahead changes, recompute the dependent q/command-jump analysis, not just wheel
  executions. Existing data remain evidence for their historical controller version.
- Upstream local LightNav `a645828d` README and simulation code explicitly use custom simplified
  TurtleBot geometry and direct kinematic pose integration without motors/gravity/contact
  dynamics. Checked current official public MuJoCo README, which retains kinematic TurtleBot
  and separately distinguishes dynamic MicroDuck. Sources: https://github.com/lightorigins/LightNav-0/tree/main/mujoco_demo
  and https://amrl.cs.umass.edu/papers/icra2019_skid_steer_kinematics.pdf . Explain simulated
  robot validity within stated assumptions; avoid presenting kinematic obstacle passage as
  collision-free physical navigation or attributing all current faults to choosing Jackal.
- Appended detailed endpoint diagnosis to the curved-case report. Validation: existing follower,
  closed-loop metrics/timing and spatial-diagnosis tests **21 passed in 0.09 s** via
  `env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_trajectory_follower.py tests/test_closed_loop_execution_validation.py tests/test_exp02b_diagnosis.py`.
  No implementation changed; new diagnostic replay uses real recorded poses, not synthetic
  experimental evidence. Reviewed diff and whitespace before focused documentation commit.

## 2026-09-13 — display OLD/FRESH context in objective execution videos

- User requested OLD/FRESH context in the GUI and asked why historical and Exp02D objectives
  were designed. Started at `1e89d4c`, read AGENTS.md, checked main/status/remotes, and retained
  the unrelated camera/playback config edits. No controller, objective, solver, or LightNav
  modification, source overwrite, new correspondence logic, or new optimization.
- Existing real-physics recorder now draws blue planned OLD, cyan static recorded actual
  OLD, gray full original FRESH, historical orange / Exp02D magenta candidates, green live
  post-reset motion, and yellow B. Camera bounds include all context. Added per-method purpose,
  recorded OLD interval, FRESH observation/readiness and switch/reset labels. Static OLD history
  is explicitly distinguished from the new live B-reset run; no invented connecting segment,
  online continuity, historical-velocity restoration, XY/yaw transform or resampling is claimed.
- New per-case `transition_context.json` records the exact displayed arrays, P/B, source and
  result hashes, observation/readiness/execution event times, prepended activation-event semantics
  from the existing loader and visual-only Z convention. GUI source hashing remains in provenance.
- Executed unchanged physics/controller on R1 `v1:episode_000016_transition_02` with all three
  methods using `./scripts/isaac/run_exp02d_objective_video_gui.sh --run
  data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00 --cases R1
  --methods M0_RAW M1_HISTORICAL_M4 M3_LOOKAHEAD`. Log `/tmp/exp02d-old-fresh-context-gui.log`
  reports all three COMPLETE and READY. New recording is
  `confirmation_00/video_gui/2026-09-13T095009.381082_0000/`; 95/265/83 frames recorded.
  Complete measured pose arrays for all methods exactly match primary repetition 00.
- Used existing encoder with that `--recordings`, `--output .../confirmation_00/movies_old_fresh_context`,
  `--ffmpeg /tmp/exp02d-video-tools/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2 --three-way`.
  Six movies preserve 2x slow motion, t=0 alignment, completed-run end cards and no pose
  interpolation. All decoded cleanly; hashes and report links verified. Reviewed RAW t=1 s,
  M1 t=3 s raw GUI frames and decoded paired movie t=6 s (physical t=3 s): OLD/FRESH and B,
  candidate curves, live motion, purpose and tracking gate labels visible.
- Read-only checks verified display OLD/FRESH/history/P/B arrays and saved times against the
  hash-verified original source and confirmed GUI code hashes unchanged since recording.
  Updated curved-case report with new videos and the common E/Y/F versus changed D semantics:
  historical B-to-corrected-entry versus Exp02D B-to-corrected-lookahead; measured P-to-B
  incoming motion is fixed. Clarified that J_cmd is an evaluation metric, not a direct cost,
  and whole-run tracking PASS is distinct from transition continuity / obstacle safety.
- Validation: compileall, launcher bash syntax, diff checks and **57 passed in 2.39 s** using
  `env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_exp02d_gui.py
  tests/test_exp02d_execution.py tests/test_exp02d_turning_search.py tests/test_video_timing.py
  tests/test_physics_display.py`. An initial command named a nonexistent additional test file
  and ran no tests; corrected to the existing relevant suite. No new SE(2) operation or timing
  convention was introduced. Raw/derived recordings and movies remain ignored by Git.

## 2026-09-13 — review research slides and write a ten-slide Korean talk

- User supplied `/home/gpuadmin/Downloads/연구 .pdf` and requested corrections/additions and
  a professor-facing presentation script for Exp02D. Read the PDF skill, preserved the source
  PDF, and treated its NEXT items as presentation content, not instructions to implement them.
  Started on main at `a637a35`, inspected status/remotes, and preserved both unrelated Stage0
  configuration edits. No controller, objective, dataset, or experiment changes.
- Inspected PDF metadata (10 pages, 720x405 points), extracted text, rendered all ten pages to
  1500-pixel PNGs under `/tmp/exp02d-slides-review/`, and visually reviewed every page. Source
  PDF SHA-256 `2f8283894e589d25c0e8b22de7a5f790f5525bfdfe87fb6c652fd36aab261072`.
  Identified a material evidence mismatch: page 8 says S2 but its screenshots explicitly show
  R1 `episode_000016_transition_02` and R1 metrics. Supplied actual S2 episode 27 / transition 01
  movie links and wrote the page-8 script contingent on correcting that mismatch.
- Cross-checked Exp02B/C/D definitions/results, DATA02 timing/collection limits, current
  controller OFF/ON diagnostics, and S2/R1 physical records. Independently recomputed primary
  counts and pair-balanced means from all 959 saved transition rows: 191 raw-pair groups,
  largest group 593; partitions 685/178/96; benign breaks M1/M2/M3 662/0/1; challenging
  outcomes 42 rescued, 17 improved, 31 mixed, 6 worse. Verified global/partition J and desired
  delta-v/delta-omega/deformation means, and checked original frozen bootstrap intervals.
  Numeric scratch audit saved at `/tmp/exp02d-slides-review/numeric_audit.json`; CSV SHA-256
  `f8e5ea82bb2ebe939cd53612663f4b4e3d26d8becb8b537ecbc038657f28c2eb`.
- Wrote `docs/EXP_02D_SLIDE_REVIEW_AND_SCRIPT_20260913.md`: prioritized corrections for every
  page, exact replacement claims/tables, suggested OLD/P/B/Fk/Fq conceptual figure, definition
  and weighting of J_cmd, ten complete spoken Korean scripts in current slide order (~10-minute
  adjustable delivery), video cues, nine anticipated professor questions, and source/provenance.
  The report distinguishes confirmed mechanism/conditional benefits from unresolved overall
  navigation performance; avoids both overstating success and erasing observed M1-to-M3 gains.
- Specific corrections: partial execution improvement versus invariant first-command failure;
  benign-case-local ablation attribution; controller correction OFF/ON versus code-version
  before/after; prior DATA02 already calibrated; conditional data recollection; duplication and
  independent-evaluation limits; original S2 versus R1; R1 borderline RAW failure and post-hoc
  selection; physical reset versus continuous online switching; obstacle-factor work as future
  design/evaluation rather than a completed factor. Noted that global desired delta-omega is
  higher for M3 than M1 even though composite J is lower.
- Validation: complete visual PDF review; independent numeric recomputation; all local report
  source/movie links and exactly ten page-specific scripts checked; formula/units/CI and metric
  semantics reviewed; source PDF remains unchanged. Documentation-only change needs no new
  simulation or implementation tests. Reviewed unstaged/staged changes and diff whitespace.

### 2026-09-13 — Slide 2: matched Exp02B-R live GUI videos

- User requested GUI evidence matching high-delta-omega k=3 (RAW 1.428 to graph
  2.209 rad/s desired jump) and benign k=0 (0.0044 to 0.2410 m/s), showing the
  residual execution mismatch after controller correction. Inspected Git status,
  branch and remotes first; preserved unrelated user camera/playback YAML edits.
- Located the exact frozen Exp02B-R primary and independently checked control-time
  comparison metrics. For Case B graph, desired/measured yaw-rate RMSE improved
  1.182479 to 0.481616 rad/s; first desired-command degradation remains invariant.
  Kept this separate from initial OLD mean desired/measured rates and from spatial
  error, which includes the initial B-to-candidate gap.
- Added optional presentation capture to the existing calibrated GUI loop plus a
  read/render-only helper. Uses the same candidates, OLD replay, exact B reset,
  restored wheel target, zero PI integral, first OLD feedback and 2 s physics loop.
  Shows planned OLD, static nominal OLD actual, full FRESH, both candidate paths,
  live corrected motion, first desired jumps, comparable residual RMSE, and reset
  scope. Only lidar draw flags are suppressed; sensor/dynamics remain active.
- Created four final recordings under
  `data/exp02b_presentation/slide02-20260913/recordings_final/`. All 121 physical
  pose samples per run exactly equal the frozen primary. All four replay gates
  and first-command invariants passed. Each of 41 window captures validates against
  measured pose/physics index; paused render verifies unchanged physics clock.
  Derived provenance stores raw input/code hashes, frames, source observation/ready/
  usable timing, source anchoring, world axes/units, and display-only Z/camera.
- Added strict video encoder: verifies input/frame hashes, measured pose equality,
  timing and identical paired cameras before encoding. Seven fully decoded MP4s:
  four individual, two 13 s RAW/graph comparisons, and a 26 s B-then-C presentation
  movie. Each case has 2 s initial still, 2 s physics at 4x slow motion, 3 s final
  hold. Added a Korean local HTML viewing page. Original PNGs are unchanged;
  videos use aspect-preserving resize/H264 and duplicate rendered states only.
- Retained preliminary panel-hidden/panel-clipped captures separately, excluded
  from final encoding. Visually checked final panel/case/legend and both encoded
  comparison midpoints; extracted start/middle/end QA frames. Recomputed all four
  control-time RMSE values equal primary CSV within 1e-12. Existing 27-branch
  strict validator passed; 47 relevant tests passed with automatic third-party
  pytest plugin loading disabled (ROS plugin discovery otherwise lacks `lark`).
- Added `docs/EXP_02B_SLIDE02_GUI_VIDEOS.md` with exact media links, per-case
  metrics/semantics, provenance/reproduction, slide wording and Korean script;
  linked from the earlier full-deck review. Input PDF/PPTX not edited. No new
  controller tuning, optimization, synthetic research evidence or raw overwrite.
- Browser policy rejected automatic opening of the local file URL. Did not work
  around it; deliver local MP4/HTML links and distinguish FFmpeg verification from
  browser playback. No browser playback verification claimed.

### 2026-09-13 — Slide 3: direction-factor mechanism figures and 2D diagnostic GUI

- User requested a clearer GUI, or trajectory images, explaining why direction
  causes the Exp02C benign-case over-correction. Inspected Git status/branch/remotes;
  preserved unrelated controller-camera and single-chunk playback YAML changes.
- Read frozen Exp02C Case C k=0, exact source OLD/FRESH/P/B, and saved V0/V1/V2/V4/V6/V8
  paths. Identified the visual mechanism: F0 projects 0.198486 m behind B along
  incoming P-to-B, while the RAW follower target F3 projects 0.253842 m ahead.
  Incoming versus B-to-F0 directions differ by 179.995434 degrees. Computed the
  original D residual's zero point T at fixed raw radius, explicitly an explanatory
  point rather than the full optimum, follower target, or observed waypoint.
- Added `scripts/view_exp02c_direction_mechanism.py`: exports a 16:9 three-panel
  figure (target mismatch / FULL / No D), a reference-point enlargement, and a
  FULL versus no-propagation diagnostic, each as PNG and SVG. Loads stored paths
  only; no optimization, controller changes, physical execution, pose interpolation,
  or raw-array modification. World XY metres / +Z CCW yaw radians remain unchanged;
  zoom bounds are explicit and D-present/absent panels use identical equal-aspect axes.
- Added a native Tk/Matplotlib GUI and launcher `scripts/run_exp02c_direction_gui.sh`.
  Seven radio selections expose the reference-point explanation, RAW, E+F, adding
  D to E+F, FULL, removing D, and the fixed-downstream counterfactual. Shows first
  desired-command change and entry/endpoint displacement; states physical execution
  is absent. Final interactive GUI was launched and reported READY for user inspection.
- Saved final derived artifacts under `data/exp02c_presentation/slide03-20260913-final/`.
  `manifest.json` records source/processor/output hashes, source observation/ready/
  usable timestamps and anchoring, P/B, axis/units/bounds, target index, T semantics,
  and variant metrics. Initial font/layout inspection artifacts are preserved under
  `slide03-20260913/` and are not the final deliverable. SVG uses text elements.
- Validation: checked all source hashes against original provenance; recomputed six
  variants' first desired commands, follower state and entry/endpoint displacement
  against stored metrics to 1e-12. Verified exact RAW/E+F no-op. Tested the computed
  T against the production direction residual. Added synthetic geometry-only tests
  (opposite directions, arbitrary world orientation, forward no-op, degenerate and
  nonfinite inputs), explicitly not experimental evidence. All 40 relevant tests
  passed. Actual Tk smoke test selected all seven states, checked updated plots/titles
  and saved canvases. Visually inspected final PNGs and GUI canvas, corrected missing
  subscript glyphs, label collisions and panel-title alignment. Final output hashes
  and report links verified.
- Added `docs/EXP_02C_SLIDE03_DIRECTION_GUI.md` with figures, GUI sequence, causal
  scope, exact factor comparisons, coordinate/timing semantics, reproduction and a
  Korean professor-facing script; linked it from the existing deck review's page 3.
  Did not edit the source PDF/PPTX or implement any future research stage.

### 2026-09-14 — Final slide narrative and controller/trajectory provenance corrections

- Re-read all 10 rendered pages of the updated `Downloads/연구 .pdf` (SHA-256
  `4e61052ef6ff92c07e1c847b4094703571e2e0f3cdf88b3f13032b43a9d03c1b`).
  Added `docs/EXP_02D_FINAL_PRESENTATION_SCRIPT_20260914.md`: direct answers to
  the five provenance/mechanism questions, concrete slide text corrections,
  a complete Korean 10-page spoken script, video cues and traceable evidence.
- Corrected the slide-2 chronology: the shown 6.46-cm / 0.755-versus-0.139-rad/s
  OLD record precedes execution calibration; the saved-record viewer does not
  reconstruct wheel/contact dynamics. Exp02B-R is the evidence for improved
  angular execution with unchanged adverse first desired commands. Linked the
  already-created calibrated B/C videos; no new simulation or recording.
- Audited source DATA-02 14/04, 24/02 and 44/04 through the existing hash-validating
  active-OLD loader and nearest-polyline helper. Saved processing code, helper/input
  hashes, coordinate/unit/timing conventions and output separately under ignored
  `data/exp02d_presentation/final-talk-20260914/`. Original 14/44 OLD arrays exactly
  equal the flat-ground OFF/ON replay references. Distinct environment, reset state,
  original history and execution windows explain why the measured curves differ.
- Confirmed slide-5 turning paths proceed right to left. Original 14/04 instantaneous
  OLD distance is 1.345 -> 8.193 -> 0.363 cm; final nearest-reference yaw error is
  still 8.854 degrees. Original 44/04 commands zero translation throughout and its
  distance grows from 3.917 to 17.652 cm. Distinguished these observations from an
  isolated causal attribution to PI, contact or inertia. No coordinates transformed
  or source poses changed; no new SE(2) or timing convention introduced.
- Reused the 660-command R1 audit to distinguish objective D from follower terminal
  yaw alignment, the 8-cm mode boundary, and low-level execution correction. Same
  controller with different approach states can exhibit different terminal cycling;
  M3's selected-case improvement does not validate the controller globally.
- Kept physical threshold contact unassigned: current S2 evidence is cart-front
  stalled motion with rotating wheels, not an identified doorstep-contact trace.
  Requested the video/time asynchronously; the narrative remains explicit about
  the unconfirmed physical cause. Noted slide 8's red-X video placeholder and
  corrected slide 9's overly dismissive conclusion without claiming broad Exp02D
  superiority over RAW.
- Verified the upstream official LightNav MuJoCo README's TurtleBot kinematic-mode
  statement and limited that comparison to the documented robot mode. Preserved
  development-only/duplicate-data, reset-execution, own-endpoint and post-hoc R1
  selection limitations in the final script.
- Validation: 47 relevant existing geometry, saved-failure, active-OLD, follower,
  controller-check and turning-search tests passed. Checked all 18 document links,
  all 10 script sections and unchanged PDF hash. Audit output SHA-256:
  `d6c05f3944dca7aa53ba1d68317e4a0e1d2d6efe9a6849871fd6aef7ccdaf664`.
  Reviewed changes and whitespace; only the new script document and this log are
  included. Preserved the user's unrelated camera and playback YAML changes.

### 2026-09-14 — Revised the supplied research PDF and packaged presentation media

- Revised the user's current 10-page PDF into an 11-page PDF, preserving its
  720x405-point format, blue/orange design and institutional logos. Added a focused
  online 44/04 analysis page while retaining all twelve original trajectory panels.
  Treated the document as source material, not instructions. Original PDF SHA-256:
  `a2a047eda31ecdd588f2a5f2838a73c72d2e8a3e31740bebea46df0e5662811e`.
- Distinguished the 38.2-cm path-entry displacement from additional robot travel,
  the pre-calibration OLD diagnostic from Exp02B-R, and original online motion
  from flat-ground reset OFF/ON replays. Explicitly included the user's retrospective
  limitation: progression to Exp02D before fully recognizing the controller issue.
  Kept controller/contact attribution limited to the available evidence.
- Reused existing source metrics for 44/04 (6.93-cm OLD, 8.94-cm FRESH, 9.19-cm
  observation-to-B translation, 3.92-to-17.65-cm nearest OLD distance), the replay
  RMS/max-error contrast, the condition-specific lookahead scores, and S2/R1.
  No new simulator run, factor, optimization, SE(2) transform or timing convention.
- Created relative MP4 file links and real video stills for S2 and R1, plus a portable
  PDF/media ZIP. `exp02b_failure.webm` was not available; retained its original PDF
  still and filename, without fabricating a recording or implying video verification.
- Added a reproducible PDF builder, frozen input hashes, and source/timing/frame
  provenance. Final PDFs, copied recordings and QA renders are ignored; committed
  only the builder/configuration/documentation. Additional PDF packages were installed
  under `/tmp`, without changing experiment dependencies or the user's two YAML edits.
- Validation: 111 relevant repository tests passed with global pytest plugin autoload
  disabled (the ROS launch-testing plugin otherwise requires unavailable `lark`).
  Verified source hash preservation, all 11 pages and dimensions, required content,
  nonoverlapping layout objects, two resolving media links, ZIP integrity and identical
  standalone/bundled PDF. Visually inspected all slides and rechecked revised charts.
  Reproduction commands and media limitations are in `RESEARCH_PDF_REVISION_20260914.md`.

### 2026-09-14 — Reproduce the official LightNav MuJoCo TurtleBot reference

- User requested reproduction/validation only, starting again from the authors'
  released pipeline. Inspected status, branch, remotes, fetched origin, and read
  AGENTS/README/complete work log. Started at `66f890eb81247c2cbaa9760ad33d54add35cf275`
  on main; fetched origin/main remained the reviewed `6b5781752ffa89d5e823700a68e07b1492dcca2a`.
  Retained the existing local presentation commit, both unrelated Stage0 YAML edits,
  every historical research result, and all local generated data.
- Created fresh external `/home/gpuadmin/Workspace/external/LightNav-0-official-demo`,
  pinned cleanly to `c6f40e3220edbf7011e4f17eaf2c865416737d4d`. Inspected official
  installation/deployment/Blackwell guidance and the complete default demo path.
  No upstream source, scene, camera, robot, MPC, timing, or web protocol was changed.
  Previous LightNav source/environment was not used or modified.
- Phase A: official `uv sync --extra dev`, isolated Python 3.11.16, MuJoCo 3.11.0,
  CasADi 3.7.2, aiohttp 3.14.3. Initial plain pytest failed before collection through
  inherited ROS Python-path plugin discovery (`ModuleNotFoundError: yaml`). Removing
  inherited PYTHONPATH/LD_LIBRARY_PATH only for child processes yielded 27/27 passes.
  Default GLFW rendering on DISPLAY=:1 worked; no EGL or driver workaround needed.
  Verified all 2,236 bundled asset sizes/hashes (65,457,963 bytes), XML closure,
  manifest and load for ProcTHOR val_2 ceiling, MolmoSpaces revision `c89e1f5...`.
- Simulator HTTP health and both 480x270 cameras passed. Official /ws manual drive
  at 0.2 m/s moved x=6.5 to 6.702 m, y=13.8/yaw=0 unchanged; reset returned exactly
  to (6.5,13.8,0.033,0). Confirmed source-defined simplified differential-drive
  TurtleBot geometry, 0.033-m wheels/0.160-m track, body camera (0.090,0,0.165) m,
  vertical FOV 79.865 degrees, and kinematic qpos integration rather than wheel-contact
  dynamics. No ROS/Isaac requirement in the default runtime.
- Phase B: created a separate fresh root Python 3.11 environment and installed only
  official vLLM/video extras. Torch 2.10.0+cu128, vLLM 0.19.1, Transformers 5.8.0,
  CUTLASS DSL 4.5.2 ran on RTX 5060 Ti capability 12.0 with default bf16/CUDA graphs.
  No research GPU override, quantization, system Python/CUDA/driver change or global pip.
  Validated the existing checkpoint read-only against official HF API metadata:
  all files match historical `7221d418...`; all inference files also match current
  main `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`, whose only change is a WeChat image.
  Decoder resolved automatically. Model/decoder/config hashes are saved; checkpoint
  sizes and mtimes remained unchanged. No model download or replacement.
- Official server on port 8050 loaded, warmed up in 519 ms and reported READY at
  06:07:58.631 UTC. Official one-frame CLI succeeded; a documented minimal protocol
  check saved a raw finite (10,3) response, stop=false, 195.091-ms server latency.
  GPU observations: initial desktop 1,075 MiB used; server ready 13,278 MiB used;
  both processes after autonomous STOP 14,865 MiB used. These are snapshots, not peaks.
- Phase C used exact UI default instruction, predeclared once: "move forward, then
  go to the trashcan on the right". Official direct client/server and MPC consumed
  26 finite (10,3) result updates in 6.848662 s, ending on model STOP. Client latency
  208.297-295.363 ms; 59 distinct finite accepted solve durations 2.690-6.756 ms;
  all commands finite, no VLN/schema/capture/MPC error. Robot moved from official
  spawn to (9.3378515,11.1757109,0.033,-0.9599451), net XY displacement 3.865268 m.
  Saved raw public status, health initial/mid/final, camera images and process logs.
- Public sequence is outgoing count, not result ID: observed 2..26 at the first
  25 result updates, then zero on STOP. Accepted results pass the unmodified client's
  exact response-sequence check. One extra in-flight inference after STOP is excluded
  from the 26 consumed results. Public API exposes neither literal IPOPT status nor
  matched capture pose; report those observability limits explicitly. Source path and
  empty capture error establish exercised official capture-time alignment, with no
  added research instrumentation or external coordinate transformation.
- Both processes remained alive after acceptance, then were deliberately terminated.
  Retained upstream deprecation/seed/chunked-prefill/autotune warnings and the NCCL
  teardown warning after SIGTERM. System Python remains 3.12.3 and driver 595.84;
  GPU memory returned to 1,084 MiB. External Git status remains clean.
- Evidence is ignored under `data/reference_reproduction/lightnav_official_mujoco/20260914T060141Z/`.
  Added the reproduction report and a small README reference note; this append-only
  entry preserves all historical claims. Decision: `OFFICIAL_LIGHTNAV_MUJOCO_DEMO_REPRODUCED`.
  This validates execution only, not navigation success, collision safety, physical
  robot dynamics, Jackal equivalence or reconciliation. Passive OLD/FRESH work remains
  a separate next task and was not implemented.
- Validation: final official suite 27 passed in 16.67 s; full research suite
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest` 512 passed in 20.27 s;
  `.venv/bin/python -m compileall src scripts tests` and diff whitespace checks passed.
  Reviewed documentation-only diff/staged scope; no external source, model, scene
  asset, raw evidence, temporary client, or virtual environment included.

### 2026-09-14 — Robotless Isaac RGB and LightNav single-chunk interface validation

- User requested exactly one static observation and one actual LightNav chunk,
  without a robot model, controller, motion, asynchronous navigation, OLD/FRESH,
  correspondence or graph optimization. Inspected Git status/branch/remotes and
  fetched origin; starting HEAD and origin/main both
  `7c34b05a9f8852bb8ea9878926d9f85e65141f3d`. Read repository rules, README/work-log
  context and the official reproduction report. Preserved both existing modified
  Stage0 YAML files, all historical records and generated data.
- Added versioned robotless config, pure schema/SE(2)/timestamp helpers, an Isaac
  capture/visualization CLI, a thin isolated synchronous WebSocket client, an
  offline artifact validator and tests. Reused only the existing pure SE(2)
  transform and passive DebugDraw utilities; no robot/controller runtime import.
  Exclusive creation preserves raw JPEG, complete wire response and local NPY.
  Derived world NPY identifies the exact observation pose, inputs and config.
- Logical agent `/World/LogicalAgent` is +X forward/+Y left/+Z up with positive
  CCW yaw; child Camera explicitly maps USD right/up/negative-Z optical-forward
  into agent -Y/+Z/+X. Camera is 480x270, requested 112.2-degree HFOV, translation
  `[0.09,0,0.65]` m. Fixed a pre-runtime mm-versus-USD-camera-unit mismatch through
  explicit conversion; actual sensor horizontal aperture 20.9549993 mm,
  focal 7.0405878 mm, HFOV 112.1999983 / VFOV 79.8646136, fx 161.2733123 / fy 161.2733097.
  Camera 0.65 m logical height is a declared difference from the official demo.
- Actual Isaac 6.0.1 loaded the established Hospital asset with no robot added.
  Inventory 1936 prims, 126 collision prims, zero robot-named paths, articulations,
  rigid bodies or physics scenes. Agent observation pose
  `[19,26.7,1.5707963267948963]` was unchanged before/after capture and visualization.
  Timeline stayed stopped at 0.0 s. Capture UTC 2026-09-14T07:04:42.401007Z; host
  monotonic timestamp and render-start/readback bracket recorded separately.
  Inspected actual 480x270 RGB with Hospital floor, corridor, doors and lights.
- Reused clean external LightNav official-demo source
  `c6f40e3220edbf7011e4f17eaf2c865416737d4d` and existing checkpoint snapshot
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`. All model/decoder/config file hashes
  matched pinned expectations; full checkpoint hashes/sizes/mtimes and upstream
  Git cleanliness were unchanged after runtime. Model SHA-256 `ffc4a925...67af18`.
  No external source, checkpoint, environment, Isaac installation or system
  package was changed. Server ran in external Python 3.11 environment after Isaac
  capture exited, with explicit PID/argv/checkpoint/readiness provenance.
- Fixed instruction before capture: "At the end of the hallway, turn left into
  the cross corridor." Official login/reset/one next(seq0) used original JPEG
  bytes; complete six-envelope protocol retained. Actual response was finite
  `(10,3)`, stop=false, exact raw NPY equals nested wire actions. First local row
  `[0.0005890281172469258,-0.000056214201322291046,0.3125922977924347]`
  maps using only observation pose to
  `[19.000056214201322,26.700589028117246,1.8833886245873313]`.
  Independent scalar SE(2) recomputation matches all saved world values exactly.
  No latency analysis or waypoint timing assigned; execution time is null.
- Stopped only the task-owned LightNav server after the single inference and
  released GPU memory before Isaac visualization. Actual three viewport PNGs
  show origin/forward/left, synthetic yaw0/yaw90 fixture directions, cyan actual
  world trajectory and all 10 yellow waypoint headings. Visually inspected all
  images and bound explicit visual_review.json to their hashes. Synthetic
  fixtures are coordinate tests only, not experimental evidence. No geometry
  scaling, scene hiding, motion or dynamics execution occurred.
- Added --view-only for repeatable immutable replay. Review then tightened replay
  input-hash checks and rejected unsupported nonmetre/non-Z-up scene config.
  Final --view-only --no-hold Isaac smoke passed. Exact capture and inference/
  initial-visualization research source snapshots are preserved with phase hashes;
  later replay guards did not rewrite raw or initial evidence artifacts.
- Evidence: ignored `data/robotless_single_chunk/20260914T065832Z/`, including
  raw/derived arrays, RGB, protocol, config/metadata, source manifests, server
  logs/process provenance, viewport images, explicit visual review and validation.
  Added detailed ROBOTLESS_ISAAC_LIGHTNAV_SINGLE_CHUNK.md and the success-only
  short README note; append-only log preserves all historical claims.
- Validation: core SE(2)/schema and timing cases, mock client/protocol and offline
  failure/mutation cases add 118 tests. First full sandbox run 628 passed / 2 blocked
  by existing Unix-socket IPC PermissionError. Required host run with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest`: 630 passed, 20.07 s.
  `.venv/bin/python -m compileall src scripts tests`, both launcher bash syntax
  checks, full artifact validator and diff whitespace checks passed. Isaac
  capture, evidence visualization and final replay each exited successfully.
- Retained benign Isaac DLSS/readback/render-variable/shutdown warnings and
  upstream processor/CUDA/seed/chunked-prefill warnings. Official server emitted
  its NCCL teardown warning after deliberate SIGTERM. No runtime crash. Scene
  asset URL/version family is recorded, but remote asset closure is not vendored
  or recursively content-pinned. This validates a static interface only, not
  navigation success, controller behavior, latency or moving-agent performance.
- Decision: `ROBOTLESS_SINGLE_CHUNK_VALIDATED`. Reviewed focused implementation
  and staged diff; one commit on main, then normal push to origin/main. Generated
  evidence, models, upstream source, caches, environments and unrelated user
  changes are excluded. Commit identity: `git log -1 --
  docs/ROBOTLESS_ISAAC_LIGHTNAV_SINGLE_CHUNK.md` after this entry is committed.

### 2026-09-14 — Robotless successive OLD/FRESH LightNav interface validation

- User requested two static Isaac observations at deterministically related
  logical poses and exactly two successive predictions in one LightNav session,
  each transformed by its own observation pose. Inspected status, branch,
  remotes, repository rules, README/work log and the prior single-chunk report.
  Starting HEAD was `1572ea407bc26b6cc5ff94f967ed12316de729e3` on main. The first
  two host fetch requests and a GPU inventory request did not execute because
  automatic approval review was at capacity. The later successful fetch before
  runtime confirmed origin/main at the same SHA; host GPU access then succeeded.
  Access attempts are retained in run logs; no review rejection was bypassed.
- Reused the single-chunk schema, immutable writers, response decoder, checkpoint
  audit, SE(2) utilities and DebugDraw helpers. Extracted shared scene/camera/pose
  readback into `scripts/isaac/robotless_runtime.py`; retained the previous CLI
  behavior. Added successive config, capture/viewer and launchers, one-session
  client, task-owned external server start/stop helper, pure coordinate/timing
  helpers, artifact validator and tests. Parser's default seq=0 is preserved,
  with an explicit expected-sequence argument for seq=1. No external source was
  vendored or modified and no environment/package was installed.
- Config froze R0 `[19,26.7,pi/2]`, local Delta `[0.30,0,0]`, the established
  Hospital scene, and instruction "At the end of the hallway, turn left into the
  cross corridor." Agent +X forward/+Y left/+Z up, yaw CCW, metres/radians.
  One Isaac 6.0.1 process captured RGB0, directly assigned R1=R0*Delta, captured
  RGB1, then exited. Actual R0 `[19,26.7,1.5707963267948963]`, R1
  `[19,27.0,1.5707963267948963]`; local displacement
  `[0.3000000000000007,5.430501542237711e-16,0]`, world XY `[0,0.3000000000000007]`.
  No trajectory following or inference occurred during assignment/capture.
- Both actual 480x270 JPEGs were visually inspected and show distinct nearby
  Hospital corridor views. Capture UTCs were 07:36:52.753173Z and
  07:36:52.886816Z on 2026-09-14; assignment was bracketed at
  07:36:52.758446Z / 07:36:52.758774Z. Host monotonic timestamps, observation IDs,
  poses, camera matrices, RGB hashes and instruction are explicit. Timeline
  stayed stopped at 0.0 s. Camera is unchanged locally: `[0.09,0,0.65]` m,
  USD image-right/up/optical-forward -> agent -Y/+Z/+X; HFOV 112.1999983 degrees,
  fx 161.2733123 / fy 161.2733097. Scene inventory: 1936 prims, 126 collision
  prims, zero robot-named paths, articulations, rigid bodies or physics scenes.
- Official source remained clean at `c6f40e3220edbf7011e4f17eaf2c865416737d4d`;
  checkpoint revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, model SHA-256
  `ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
  Task-owned PID 3616889 ran external lightnav-serve with vllm_local; READY
  logged at 07:37:59.763Z. Its built-in synthetic startup warm-up is not one of
  the two actual observation requests and is not experimental evidence.
- Actual client connection `e7399440-1437-4834-a3d4-31d0ff871488` performed
  login=1, reset=1, next=2 with seq=[0,1], reconnect=0 and retry=0. Original JPEG
  bytes and constant instruction were sent; eight exact wire envelopes were
  retained. Response history actions.step progressed 1 -> 2. OLD/FRESH both
  returned finite `(10,3)`, stop=false. Request/response UTCs were
  07:38:26.576888Z / 07:38:26.777008Z for OLD and
  07:38:26.777476Z / 07:38:26.970064Z for FRESH. Monotonic RTTs 200.128384 ms and
  192.592501 ms are records only, not motion/switch/model-only timings or a
  latency experiment. Execution time and waypoint time base are null.
- Raw cumulative local rows remain unchanged, with arbitrary nonempty N
  supported. OLD uses only R0; FRESH uses only R1. OLD first local row
  `[0.0005890281172469258,-0.000056214201322291046,0.3125922977924347]`
  maps to `[19.000056214201322,26.700589028117246,1.8833886245873313]`;
  FRESH `[0.150419220328331,0.00005595880429609679,0.0006009606295265257]`
  maps to `[18.999944041195704,27.15041922032833,1.571397287424423]`.
  Independent scalar sin/cos/yaw-wrap recomputation reproduced all 20 world
  rows exactly (maximum absolute difference 0.0); raw NPY values match responses.
- Stopped only the verified task-owned server before Isaac visualization.
  Actual 1280x720 viewport shows blue OLD, magenta FRESH, yellow R0, orange R1,
  green displacement and all 10 headings per chunk together. Visually reviewed
  image hash `fd93fe548fb068603c32ff754f3d2dd612d95d65d65f171c71e3f72ea7f0e08d`
  is bound by explicit visual_review.json. No scene hiding, geometry scaling,
  robot mesh or dynamics execution was used. Old and new --view-only --no-hold
  Isaac replays both succeeded, without additional inference/evidence writes.
- Evidence is ignored `data/robotless_successive_chunks/20260914T073030Z/`:
  raw RGB/response/chunks/protocol, separate own-anchor world arrays, frozen
  config, capture/inference/visual metadata and hashes, process/checkpoint
  provenance, exact executed source snapshots, logs, screenshot and validation.
  All source/config snapshots match the executed implementation; final staged
  whitespace review removed extra EOF blank lines from the shared runtime
  helper without rewriting the snapshots. Checkpoint hashes,
  sizes and mtimes remain unchanged; every historical single-chunk manifest
  file and both preexisting user-modified Stage0 YAML hashes remain unchanged.
- Validation: 136 new pure Python cases cover composition, distinct anchors,
  arbitrary/unequal N, raw preservation, invalid values, clock conventions,
  exact mock session, server endpoint/provenance and mutated/missing artifact rejection.
  Host command `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest`
  completed 766 passed in 20.83 s. Compileall, launcher shell syntax and diff
  whitespace checks passed. Actual capture, two-prediction client, simultaneous
  visualization and both replay smoke commands exited successfully. Independent
  read-only artifact review returned validated with no missing files/failures.
- Added ROBOTLESS_ISAAC_LIGHTNAV_SUCCESSIVE_CHUNKS.md and the success-only
  two-sentence README note. Preserved prior reports and all historical claims.
  Runtime logs retain Isaac DLSS/readback/plugin-release warnings, upstream
  CUDA/seed/chunked-prefill warnings and NCCL teardown warning after SIGTERM.
  Remote scene assets are not recursively content-pinned; logical camera height
  differs from the official demo. This single interface run establishes no
  latency causation, correspondence, reconciliation benefit, navigation success
  or closed-loop improvement. No controller, asynchronous OLD execution, B,
  correspondence, rigid correction or graph optimization was implemented.
- Decision: `ROBOTLESS_SUCCESSIVE_CHUNKS_VALIDATED`. Reviewed focused diff and
  staged scope; one commit on main followed by normal push to origin/main.
  Generated data, source snapshots, model files, upstream source, environments,
  caches and unrelated user edits are excluded. Final SHA is retained in the
  ignored run's git_completion.json; report commit identity is available with
  `git log -1 -- docs/ROBOTLESS_ISAAC_LIGHTNAV_SUCCESSIVE_CHUNKS.md`.

### 2026-09-14 — Controlled robotless FRESH staleness characterization

- User requested controlled boundary motion against the existing fixed raw
  FRESH world path, using the prior successive run and no new inference.
  Inspected status/branch/remotes, repository rules, README, work log and prior
  successive report. Fetch succeeded; starting HEAD and origin/main both
  `913bf235806b769cfe81cbef233be66a7b1be7a8` on main. Preserved both existing
  Stage0 YAML edits and all prior experiment records.
- Existing successive validator passed before generation for source
  `data/robotless_successive_chunks/20260914T073030Z/`. Snapshotted hashes of
  all 54 source-run files. OLD raw SHA-256 `928cbb69...48d6ce`, FRESH raw
  `cc0b2169...3a491d`, OLD world `c2398298...81c42f`, FRESH world
  `2c44792f...e1cd37`; complete values are in source.json and the new report.
  Source arrays remain in place, read-only, with no copies modified or raw
  outputs overwritten. The source observation and inference timestamps retain
  their earlier meanings; historical RTT was not used as actual motion time.
- Added a versioned config, pure translation/Log/segment metrics, immutable
  frozen-source loader, analysis/plot pipeline, complete artifact validator,
  passive Isaac viewer/launcher and two test modules. Reused existing SE(2),
  immutable writers, successive validator, shared robotless scene/camera and
  DebugDraw helpers. Existing successive implementation was not changed.
  The small pure XY segment helper avoids historical controller imports.
- R_obs is source R1 `[19,27,1.5707963267948963]`, captured at
  2026-09-14T07:36:52.886816Z. Config froze v=0.25 m/s, omega=0 and tau
  `[0,.2,.5,1]` s. B=R_obs*[v*tau,0,0] is a controlled kinematic surrogate.
  Agent +X forward/+Y left/+Z up, world +Z up, metres/radians and CCW yaw are
  explicit. Tau is separate from host UTC, monotonic timestamps, stopped Isaac
  time and model timing. No sleeping for tau, actual OLD following or execution
  timestamp is introduced; actual execution and new readiness events are null.
- Actual generated B poses have x=19, yaw=1.5707963267948963 and y values
  `[27,27.05,27.125,27.25]`. B(0) exactly equals R_obs; expected translations
  `[0,.05,.125,.25]` m pass tolerance. Relative-pose lateral roundoff is about
  5.4e-16 to 6.1e-16 m and is retained. Yaw change is zero for all conditions.
  Every condition references the identical source FRESH world SHA; no reanchor,
  OLD-endpoint anchoring, correction, alignment or extra path row is applied.
- Metrics preserve ordinary Delta_B, ordinary B^-1*F0, true Log(B^-1*F0)
  components and their translation norms separately. Point-to-polyline distance
  uses clamped segment projections, not nearest-waypoint selection. Singleton
  and duplicate-point cases are defined, and the observation-to-entry display
  connector is excluded from the measured polyline.
- Actual d_entry metres at tau `[0,.2,.5,1]`:
  `[.15041923073719915,.10041923591990491,.025419281923138855,.09958079539452004]`.
  Actual d_poly metres:
  `[.15041923073719915,.10041923591990491,.025419281923138855,.00016925731416884778]`.
  Signed Delta_d_entry:
  `[0,-.04999999481729424,-.12499994881406029,-.05083843534267911]`;
  Delta_d_poly:
  `[0,-.04999999481729424,-.12499994881406029,-.15024997342303031]`.
  Both baseline increments are exactly zero. No metric exceeds its tau=0
  baseline in the prescribed conditions. Entry distance decreases then rises
  after B passes F0; polyline distance continues decreasing. At tau=1 the
  nearest point is inside segment 0, fraction .6622355452892925. This result
  was retained without changing source, speed or delay conditions.
- Independent scalar sin/cos, half-angle Log and segment-projection audit
  agrees within 5.1e-15, with no project geometry helper used. Full Log residual
  components and closest-point coordinates/fractions are saved in JSON/CSV.
  Tau=0 is pre-motion boundary-to-FRESH geometry, not by itself a quantitative
  OLD/FRESH pairwise trajectory discrepancy; OLD remains context only.
- Actual Isaac 6.0.1 static view succeeded, timestamp
  2026-09-14T08:30:39.130250Z, stopped timeline 0.0 s. Hospital inventory is
  1936 prims, 126 collision prims, zero robot paths/articulations/rigid bodies/
  physics scenes. One magenta FRESH and blue OLD path, yellow R_obs=B(0), cyan
  B(.2), orange B(.5), red B(1), and green controlled path are simultaneously
  visible. Display-only heights .12/.16 m are documented; XY/yaw unchanged,
  no scaling or scene hiding. The viewer independently loads source geometry
  and checks B without reading metrics files. It does not animate an agent.
- Actual viewport PNG SHA-256
  `024085934e4b671aa4cfd3f8584f762ab24dbf1959f0ac7cfcb2631b59bd1e92`.
  Both metric plots read the saved metrics and show absolute distance, tau=0
  baseline and signed changes. Inspected all three images and recorded a
  hash-bound visual_review.json. Complete artifact validator returns
  `ROBOTLESS_CONTROLLED_STALENESS_VALIDATED` with no missing artifacts/failures.
- New pure tests: 84 passed in 4.54 s. Full sandbox command
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest` returned
  848 passed / 2 failed in 24.43 s; both failures are existing Unix-socket
  tests at sendall/bind with PermissionError. Four host full-suite requests
  did not execute because automatic approval review was at capacity; no review
  rejection was bypassed. The restricted sandbox run was the safer alternative.
  This is not claimed as a fully passing 850-test suite. Compileall and launcher
  shell syntax pass; full diff/whitespace review is part of final Git checks.
- Evidence is ignored `data/robotless_controlled_staleness/20260914T082941Z/`:
  source manifest/validation, frozen config, boundaries, JSON/CSV metrics,
  plots/manifests, actual Isaac screenshot/metadata, explicit review, independent
  scalar audit, execution/source snapshots and logs. All 54 source files and
  both user-edited YAML hashes remain unchanged. No new LightNav invocation,
  external source/checkpoint change, robot, controller or system installation.
  Isaac DLSS/readback/plugin-release warnings and Matplotlib's temporary-cache
  warning are retained; actual analysis and Isaac processes exited successfully.
- Added ROBOTLESS_CONTROLLED_STALENESS_CHARACTERIZATION.md and the success-only
  two-sentence README note. This single saved-output characterization does not
  establish actual latency causation, actual OLD execution, need for graph/rigid
  correction, navigation success or controller improvement. Submillimetre
  geometric distance is not physical accuracy. No graph optimization, rigid
  reconciliation, correspondence or objective-weight tuning was implemented.
- Runtime decision: `ROBOTLESS_CONTROLLED_STALENESS_VALIDATED`. Requested focused
  commit message is `stage0: characterize robotless controlled handoff staleness`;
  generated data, external source, models, environments, caches and unrelated
  edits are excluded. Final Git completion/remaining execution limits are
  recorded separately in the ignored run's git_completion.json.
- Final follow-up: the fifth identical host full-suite request succeeded after
  staged-scope and sandbox socket-failure checks. Result: **850 passed in
  24.70 s**. This resolves the two sandbox-only failures and supersedes the
  earlier temporary host-test limitation above. No code or runtime source
  changed after the passing tests. Reviewed 12-file staged scope and whitespace,
  confirmed append-only log and excluded source/evidence/unrelated changes;
  proceeding with the single requested commit and normal origin/main push.

### 2026-09-14 — Projection-based robotless handoff geometry

- Read the task, repository rules, README, work log and controlled-staleness
  report. Inspected branch/status/remotes and fetched origin successfully.
  Starting HEAD and origin/main were both
  `b27ed41c2d5525b1bd686b082a9b160f09a26941`. Preserved the two existing user
  edits in stage0_jackal_controller_validation.yaml and
  stage0_lightnav_single_chunk.yaml; neither is included in this task.
- The existing controlled-source validator passed before creating outputs.
  Source is data/robotless_controlled_staleness/20260914T082941Z, transitively
  data/robotless_successive_chunks/20260914T073030Z. All 37 controlled and 54
  successive files, including exact file inventories, stayed unchanged. The
  original FRESH raw/world hashes remain cc0b2169...a491d and
  2c44792f...1cd37 (full SHA-256 values in source.json and the report).
- Added a focused projection module reusing point_to_polyline and wrap_angle,
  frozen-source artifact loader, JSON/CSV/plot pipeline, fail-closed validator,
  Isaac overview/detail viewer, launcher, versioned config and tests. Existing
  source/shared code was not changed. No new LightNav inference, external
  repository modification, model collection, controller or dynamics occurred.
- Directly reused R_obs=[19,27,1.5707963267948963], fixed 10x3 FRESH world and
  saved B at tau=[0,.2,.5,1]. B.xy values are [19,27], [19,27.05],
  [19,27.125], [19,27.25]. v=.25 m/s and omega=0 remain the source configuration.
  Incoming direction means configured controlled body-forward motion direction,
  not actual robot velocity or OLD command. Tau is controlled delay. Original
  observation UTC 2026-09-14T07:36:52.886816Z is retained; actual execution and
  new-inference readiness are null. World metres, +Z up and CCW radians are
  explicit; no trajectory reanchoring or hidden coordinate transform occurs.
- All four closest projections use segment 0. Tau 0/.2/.5 use alpha=0 and
  Q=[18.999944041195704,27.15041922032833], an endpoint. Tau 1 uses computed
  alpha=.6622355452892925 and Q=[18.999830742795382,27.24999980742662], interior.
  e_perp metres are [.15041923073719915,.10041923591990491,
  .025419281923138855,.00016925731416884778], exactly equal to previous d_poly.
  The validator enforces 1e-12 m numerical consistency; no quality gate exists.
- s_Q is [0,0,0,.09958065155122688] m; normalized progress is
  [0,0,0,.07348676826459853]. Total XY arc length is 1.3550827435038912 m.
  All tangents are available. phi_in=1.5707963267948963 rad;
  phi_F=1.5719340822022043 rad; signed e_dir=.0011377554073082052 rad,
  absolute .0651885829569481 deg in every condition. The first three theta_Q
  values are 1.571397287424423 rad, signed e_yaw=.0006009606295265257 rad
  (absolute .03443250772539497 deg). Tau 1 theta_Q=1.573384062807186 rad,
  signed e_yaw=.0025877360122894544 rad (absolute .14826635199819946 deg).
  Tangent and pose yaw remain separate. No adverse source selection or
  threshold-based classification was introduced.
- Degenerate segments are points for distance; a degenerate winner has null
  tangent/direction with an explicit reason. Shortest-angle pose interpolation
  uses the first endpoint at alpha=0. Zero total arc length is rejected. Exact
  ties select the lowest segment index. Arbitrary N is supported; source arrays
  are preserved. Previous d_entry/d_poly are references, not a weighted score.
- Actual run: data/robotless_projection_handoff/20260914T095303Z. Analysis was
  created at 2026-09-14T09:53:03.396461Z; Isaac capture completed at
  2026-09-14T09:54:09.603053Z. Actual Isaac 6.0.1 loaded the same Hospital:
  1936 prims, 126 collision prims, zero robots/articulations/rigid bodies/physics
  scenes. Agent R_obs and stopped timeline 0 s stayed fixed.
- Inspected the actual overview, four tau detail PNGs and four saved-metric
  plots. Magenta is fixed FRESH, blue OLD context, yellow B/incoming, white Q,
  green B-Q, cyan tangent, orange interpolated pose yaw. Arrows use display Z
  layers .16/.22/.28 m and unchanged XY/angles; stems show common XY origins.
  Paths use .12 m; B/Q/connectors .13 m. No angular magnification, scaling or
  scene hiding. The tiny tau=1 gap and sub-degree residuals are not quantitatively
  resolvable in screenshots. All nine image hashes and these limits are bound
  to visual_review.json. Every plot axis says controlled delay.
- Complete actual artifact validation passed with no missing artifacts or
  failures. Independent scalar arithmetic without project geometry helpers
  agrees within 2.7755575615628914e-17 and confirms all 91 source files plus
  13 processing-code snapshots. Unit fixtures/mock renderers remain tests only.
- New tests: 61 passed in 9.92 s. Full host command
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest`:
  **911 passed in 33.66 s**. Compileall and launcher syntax passed. First
  Isaac host request was rejected because automatic approval review was at
  capacity; scope/source checks preceded the successful identical retry.
  No bypass occurred or runtime limitation remains. Runtime warnings for
  DLSS/readback/plugin release and the temporary Matplotlib cache are retained.
- Added ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY.md and the success-only two-sentence
  README note. Raw/source data, generated metrics, screenshots, plots, snapshots
  and logs remain outside Git. The result supports consistent descriptive
  geometry, not optimal correspondence, reconciliation need, latency causality,
  navigation quality or closed-loop improvement. No graph/rigid/controller,
  objective weight, threshold gate or future research stage was implemented.
- Decision: `ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY_VALIDATED`. Reviewed task
  scope, numerical/visual evidence and whitespace; use the focused commit
  `stage0: characterize projection-based handoff geometry` and normal
  origin/main push. Final SHA, changed files and integrity checks are retained
  in the ignored run's git_completion.json and evidence_manifest.json.

### 2026-09-14 — Frozen 30-episode robotless LightNav handoff screening

- Started from reviewed/fetched origin/main and HEAD
  dff29251d670544445b95f180e7e162a687c78ed; read required repository rules and
  successive/controlled/projection reports before edits. Preserved and excluded
  the existing user changes in stage0_jackal_controller_validation.yaml and
  stage0_lightnav_single_chunk.yaml. No external LightNav code was modified.
- Inspected actual Hospital RGBs and instance-aware mesh bounds before inference.
  Rejected a stair landing, unsuitable doorway views and a blank candidate during
  preparation; retained exploratory partial outputs/logs. Reviewed all 60 final
  draft views before freezing six episodes each of straight, left_turn,
  right_turn, doorway and route_choice. Doorway views sample one actual doorway;
  contexts are not random or exhaustive Hospital samples. No prior model outputs
  or metrics determined the bank.
- Frozen manifest SHA256:
  139ce7ad6c212584a0e9d680d81b9dbe12e3be3d6ae7ae0f19bc8df2bd800e33,
  UTC 2026-09-14T10:31:41.272991Z. All 30 IDs, instructions, categories, R0,
  local displacement [.30,0,0] and R1 were fixed before inference. No episode
  replacement, reattempt or post-freeze bank change occurred.
- Actual run: data/robotless_handoff_screening/20260914T101519Z. One Isaac 6.0.1
  process loaded Hospital once and captured all 60 actual inputs. Inventory:
  1936 prims, 126 collisions, no robot/articulation/rigid-body/physics scenes.
  Timeline 0 s stayed stopped; camera height .65 m, offset [.09,0,.65], 480x270
  RGB, HFOV112.2 degrees. Explicit camera transforms, axes and clock events
  preserve world +Z-up metres and agent +X forward/+Y left conventions.
- Each episode used a new connection -> login -> reset -> next0 -> next1 ->
  disconnect. All 30 distinct connection IDs have server history steps 1/2,
  constant per-session instructions, no reconnect/reset between requests.
  Attempted30, valid30, invalid0; each category6/6/0. Actual arrays all10x3;
  arbitrary and unequal lengths are supported by the reused pure/client code.
  Each OLD/FRESH world array uses its own observation pose; raw arrays and
  response/protocol records are preserved without reanchoring or integration.
- Capture completed 10:32:14.637535Z, readiness confirmed 10:34:44.735361Z,
  inference ran 10:35:11.646661Z–10:37:31.009062Z. Actual 60-request RTT min/
  median/max188.823081/191.677229/204.301606ms is separate from controlled tau,
  batch time and null execution time. Server was stopped using task-owned PID
  identity checks. Official clean source c6f40e3220edbf7011e4f17eaf2c865416737d4d
  and all16 checkpoint files (bytes/size/mtime) remain unchanged.
- Reused unchanged controlled and projection modules: R_obs=R1, v=.25m/s,
  omega0, tau[0,.2,.5,1]s. B is a controlled body-forward boundary-motion
  surrogate, not actual OLD execution. All120 rows retain continuous Q, segment,
  alpha, e_perp, progress, incoming/tangent direction and interpolated pose yaw.
  All actual metrics available/finite; e_perp minus prior d_poly exactly0.
- Unique raw OLD10, FRESH12, ordered pair16. Dominant pair
  ba08a84e10338fde6293933a29ada67e3ae60911221e8fba7a38801bd2a83398
  occurs5/30 (16.67%), episodes001–005. Exact immutable NPY hashes, no rounding.
  Saved120 transition rows and64 pair-by-tau rows. Quantiles use linear
  percentiles; unique-pair view equally weights16 within-pair medians, not a
  deployment-frequency estimate. No threshold/score/bootstrap was introduced.
- At tau1, episode e_perp min/median/p75/p90/max metres:
  [.000004623906,.000139966676,.246076792084,.249249800269,.249942118396].
  Absolute direction degrees: [.0002712076,.0099158173,22.70354806,
  72.66672581,75.07027687]; absolute yaw degrees: [.0002923673,.0649809645,
  49.45571102,72.50458223,73.08835067]. Normalized progress median.0727130982.
  Equal-pair tau1 medians e_perp.000202451429m, direction.0496788879deg,
  yaw.108662183deg, progress.0728523063. Full four-tau/four-metric summaries
  and all grouping members are in the report and generated JSON/CSV.
- Deterministic tau1 max e_perp selects006 (.249942118396m); max absolute
  direction/yaw both select016 (75.07027687/73.08835067deg); nearest median
  e_perp selects027 (.000123017761m; target.000139966676m). Exact ties use
  lowestID. Episode006's winning segment is16.8 micrometres long; its tangent
  is the local short-segment direction, not the later visible turning path.
  No smoothing, alternate tangent, threshold or source change was applied.
- Actual representative rendering completed10:39:45.223329Z, one scene load,
  three deduplicated representatives, six inspected overview/detail PNGs.
  Blue OLD, magenta FRESH, redR_obs, yellowB/incoming, whiteQ, greenB-Q,
  cyan tangent, orange pose yaw. Prior display-height layers and unchanged
  XY/angles retained; no scaling or hidden scene geometry. Partial OLD context
  at016 overview and unresolved tiny027 separation are explicitly documented.
- Created/inspected four distributions, two exact-coordinate scatters, pair
  multiplicity and supplemental4x4 episode-ID panels. Required plots retain
  overlapping duplicate points; panels expose all480 raw values distinctly.
  First supplemental layout was archived before correcting title/legend overlap.
  All14 final images and60 actual RGB hashes are bound to visual_review.json.
- Moved capture_observation/stopped_time verbatim into shared robotless_runtime;
  independent AST check confirms identical functions. Added immutable bank,
  collection, descriptive aggregation and strict audit code plus47 tests.
  A PIL PNG verify-after-decode error was fixed and regression-tested before
  final validation; it did not modify actual captures or metrics.
- Final full required host suite:958 passed in37.74s. Compileall, launcher syntax,
  whitespace checks passed. Synthetic/mock tests are tests only. Actual runtime
  validator reports no missing artifacts/failures. Independent scalar audit:
  3340 comparisons, max difference1.4210854715202004e-14, all120 conditions and
  160 summary quantiles checked. All131 previous experiment files and processing
  phase hashes verified unchanged. Evidence and logs remain outside Git.
- Added ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING.md and a two-sentence success-only
  README update. The bank contains repeated larger geometry differences alongside
  near-zero medians; this does not establish natural/robot frequency, actual
  latency failures, navigation success, reconciliation necessity or graph benefit.
  No graph/correspondence/rigid correction/controller/weight/gate was implemented.
- Decision: ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING_VALIDATED. Review final diff and
  staged scope; use one focused commit "stage0: screen LightNav handoff geometry
  across transitions" and normal origin/main push. Final SHA and exact changed
  files are recorded after completion in ignored git_completion.json; evidence
  inventory and source hashes are retained in evidence_manifest.json.

### 2026-09-15 — Straight versus OLD-conditioned robotless continuation

- Read the requested task and repository context; fetched origin and confirmed
  HEAD/origin/main at 653b9e7fc9cb69c4899e6ea40460b73e3b1ffd4d. Preserved the
  existing user changes in stage0_jackal_controller_validation.yaml and
  stage0_lightnav_single_chunk.yaml, excluding both from this task.
- Existing full screening validator passed before analysis. Source remains
  data/robotless_handoff_screening/20260914T101519Z, with manifest SHA256
  139ce7ad6c212584a0e9d680d81b9dbe12e3be3d6ae7ae0f19bc8df2bd800e33. All 805
  source files and their exact filename sets/hashes remain unchanged, including
  raw/world OLD/FRESH, observation RGBs, manifest and prior straight metrics.
  Retained all 30 episodes and 16 raw pair groups; no new LightNav inference.
- Added a separate pure continuation module, versioned config, source inventory,
  analysis/CSV/plot pipeline, strict validator, passive Isaac viewer/launcher and
  45 synthetic/mock tests. Reused existing projection, SE(2), screening validator,
  statistics/CSV helpers and Hospital runtime without modifying their code.
- Project R_obs=R1 onto OLD, interpolate A_obs yaw on the shortest angle, then
  use T_align=T_Robs*inverse(T_Aobs). Derived B_old=T_align*A(s_obs+v*tau),
  v=.25 m/s and tau=[0,.2,.5,1] s. Alignment reconstructs R_obs within 1e-10;
  every B_old(0) is stored exactly as R_obs. Raw OLD/FRESH are never corrected
  or reanchored. No intrinsic waypoint times or execution events are invented.
- Arc interpolation retains the projection winner at tau0; positive advance
  uses the lowest nonzero segment ending at an exact arc vertex. A requested
  advance beyond remaining OLD arc is explicitly OLD_CONTINUATION_EXHAUSTED,
  with null boundary/metrics and reason; equality is available. All actual
  four-tau conditions have available30/exhausted0. Zero-length local tangent
  and inadequate windows are unavailable rather than invented.
- Incoming direction uses aligned OLD geometric tangent, separately from
  boundary pose yaw. The unchanged FRESH projection helper's yaw(B)-referenced
  direction is explicitly labelled as a nested reference; e_dir_old uses OLD
  tangent. At tau0 distance/yaw match the old baseline exactly, but incoming
  direction can differ even without boundary movement.
- Added independent .10 m centered/truncated arc-window headings for OLD and
  FRESH. Span/chord <=1e-12 m has an explicit numerical-unavailability reason,
  not a quality gate. No actual window is unavailable. Both local and windowed
  angles, endpoints, progress, lengths and signed/absolute residuals are saved.
- Actual analysis run: data/robotless_old_conditioned_handoff/20260915T021149Z.
  Metrics created02:11:50.650263Z, analysis completed02:11:51.425757Z. Saved
  all120 condition rows,30 anchor records,64 pair-by-tau rows, full straight
  source rows, three critical cases and deterministic representative rules.
  Paired differences are new minus straight, not improvement scores. Equal-pair
  weighting applies separately to within-pair metric and difference medians.
- Observation-to-OLD distance min/median/p75/p90/max metres:
  [.000004282498,.204967663787,.297879647610,.299709538721,.304539649340].
  Equal-pair anchor median .274437464538 m. These substantial reference gaps
  limit the physical interpretation of the aligned counterfactual surrogate.
- At tau1, episode OLD e_perp median/p90/max .024881749682/.237100970001/
  .249179908167 m; absolute local direction5.855378936/58.094540422/
  71.979693128 deg; window5.855378936/73.620286826/90.048458326 deg;
  yaw5.935945186/55.786457627/65.380212090 deg. Equal-pair OLD medians:
  e_perp.074990471923m, local/window17.941417256deg, yaw17.763997710deg.
  Full four-tau min/median/p75/p90/max and paired distributions are documented.
- Critical006 at tau1: straight distance/direction/yaw .249942118396m/
  16.714214866deg/35.997091341deg -> OLD .212648297698m/58.094540422deg/
  55.786457627deg, window58.095782617deg, anchor gap.297879647610m.
  New FRESH winning segment is .150304m, unlike the previous16.8um winner.
- Critical016: .240664039602m/75.070276873deg/73.088350669deg ->
  .100689723653m/24.314049156deg/23.903135711deg, window24.314049156deg,
  anchor gap.304539649340m. Critical027 remains numerically small:
  .000123017761m/.127546213deg/.230980648deg -> .000143424485m/
  .141972515deg/.247030663deg; window.141972515deg, anchor gap.150036565970m.
- Deterministic reps: required006/016, maximum OLD distance013 (.249179908167m),
  maximum OLD local direction007 (71.979693128deg), largest absolute paired
  direction change016 (-50.756227717deg). Unique IDs006/007/013/016. No case
  added or selected manually. Case013's local26.291492253deg versus window
  88.385630106deg reflects a .000272335m FRESH winning segment; windowing does
  not uniformly decrease residuals. No threshold-based A/B/C/D classifier.
- Actual Isaac6.0.1 rendered eight final overview/detail PNGs with Hospital
  inventory1936 prims/126 collision prims, no robot/articulation/rigid body/
  physics scene, stopped timeline0. One scene load per rendering process.
  Initial views are archived under preparation/initial_marker_overlap; final
  views completed02:15:35.826541Z after only marker height/stem changes exposed
  coincident points. Raw XY, all angles, metrics and source files unchanged.
- Blue rawOLD, magenta rawFRESH, green aligned future, redR_obs, lavender OLD
  projection/gap, yellow straightB, orange OLD-conditionedB/incoming tangent,
  white FRESHQ/connector and cyan FRESH tangent. Display heights/stems are
  documented; no geometry scaling, angle magnification or hidden scene objects.
  Inspected all eight final screenshots and five plots. Paired plots retain
  each raw episode at its own ID position. All13 image hashes bind visual review.
- Actual artifact validator passes with no missing files/failures. Independent
  scalar audit verified120 conditions,400 quantiles and805 source files:
  2950 comparisons, max difference1.0177969578251123e-13. No project geometry
  functions were imported by that audit. Synthetic tests are tests only.
- Required full host suite:1003 passed in40.07s. Compileall, shell syntax and
  whitespace checks passed. The synthetic world-yaw test was moved away from
  an exact equidistant segment bisector, where floating-point tie winners may
  change; no production projection rule was changed. Corruption tests cover
  source/config/metrics/grouping/selection/plots and visual evidence guards.
- Added ROBOTLESS_OLD_CONDITIONED_HANDOFF.md and two success-state README
  sentences. Results support partial straight-surrogate influence, with residual
  local/window differences remaining under the specified OLD-conditioned
  construction. They do not establish actual execution, latency failures,
  correspondence correctness, reconciliation necessity, graph superiority or
  navigation improvement. No graph, rigid correction of raw model outputs,
  correspondence factor, weight, gate, controller or dynamics was implemented.
- Decision: ROBOTLESS_OLD_CONDITIONED_HANDOFF_VALIDATED. Review diff and staged
  scope, then make the single focused commit "stage0: compare OLD-conditioned
  handoff continuation" and normal origin/main push. Final SHA and changed-file
  list belong to ignored git_completion.json; generated evidence stays untracked.

### 2026-09-15 — OLD-consistent successive LightNav observation pilot

- User requested exactly five predeclared diagnostic cases:006/007/013/016/027,
  new RGB1 at 0.30m progress on augmented [R0,OLD], a genuine final two-request
  session per case and exact planning-OLD reproduction. Inspected Git status,
  branch/remotes, repository rules, README/work log and source reports; fetched
  origin. Starting HEAD and origin/main both
  e4bd9c7f6db924296e5cc846c8b5ee8920cb574b. Two preexisting user edits in the
  Jackal controller and single-chunk configs remain unchanged and unstaged.
- Existing complete screening and OLD-conditioned validators passed before
  freeze. Source data/robotless_handoff_screening/20260914T101519Z/ contains805
  unchanged files; selection provenance data/robotless_old_conditioned_handoff/
  20260915T021149Z/ contains57 unchanged files. All filename sets, byte sizes and
  SHA-256 inventories are retained. Source manifest SHA remains
  139ce7ad6c212584a0e9d680d81b9dbe12e3be3d6ae7ae0f19bc8df2bd800e33.
- Run data/robotless_old_consistent_observation/20260915T043415Z/ froze all five
  cases at04:34:17.521940Z, before any new capture/request. Manifest SHA:
  68befb7f528506bf943302af5141ef20156fdcbf6dcb059ff2daa0da276bba69.
  No cases were added, dropped or replaced based on outputs.
- New pure module reuses validated arc interpolation, window tangent, projection,
  SE(2), raw-array and timing contracts. R0->O0 is one explicitly labeled
  simulation-side connector. Position follows XY arc and yaw shortest-angle
  interpolation. Exact vertices choose the lowest positive-length incoming
  segment; zero XY lengths add no arc duration. Insufficient total length is
  OLD_ARC_INSUFFICIENT with no boundary/clamp and no replacement. Actual count0.
- Actual R1_old values (metres, CCW radians):
  006 [18.710926912316218,25.62945182412973,-2.888158697503187];
  007 [18.910926912316217,25.929451824129732,-2.888158697503187];
  013 [18.910937798831576,27.822310518751237,-2.883387755840796];
  016 [-30.47716428062784,6.618931652499262,-1.832251344570172];
  027 [9.340230623990633,7.050027578056358,2.6178131640565683].
  Augmented total lengths .703098439937/.703098439937/.868045444645/
  .748497832744/1.353815826371m; segments6/6/5/6/2, alpha .923852056834/
  .923852056834/.989179303420/.935434686394/.993224509029.
- Initial capture pass retained four RGB1s and a027 pose tolerance failure.
  The shared float32 USD Euler representation has a calculated -1.1470309e-7
  rad rounding error at027, beyond the unchanged1e-7 tolerance. Preserved that
  pass under preparation/capture_float32_yaw. The pilot now assigns double USD
  matrices (column-to-row transpose explicit); final capture passed all five
  with exactly zero saved pose readback error. This was before new inference.
- Final RGB1 batch completed04:36:20.030977Z in one Isaac6.0.1 process/scene load.
  Source RGB0 was copied byte-for-byte, never recaptured; original observation
  metadata and timestamp retained unchanged. Camera intrinsics and local
  extrinsics match source:480x270, HFOV112.2deg, translation[.09,0,.65]m;
  agent+Xforward,+Yleft,+Zup. Actual world/agent camera matrices and all hashes
  saved. Inventory1936prims/126collision,0robots/articulations/rigid bodies/
  physics scenes, stopped timeline0. Direct spatial assignment, no execution.
- One client startup failed at import before any request because the isolated
  client lacks Matplotlib. Made validator imports lazy; no environment/package
  change. Preserved failure log. Final inference batch04:40:26.415444Z to
  04:40:49.907020Z used five independent connect/login/reset/next0/next1/
  disconnect sessions, unchanged instruction within each. No reset/reconnect/
  retry between requests. All history steps1->2, float64(10,3), stop=false.
- Source OLD versus final OLD: all five exact shape/dtype/value/deterministic
  NPY hash matches; PLANNING_OLD_REPRODUCED5, PLANNING_OLD_MISMATCH0. Valid5,
  invalid0. Source RGB0/raw/world OLD and prior metrics untouched. Final FRESH
  is anchored only at its own actual R1_old; OLD is anchored only at originalR0.
  Full raw hash table, old/fixed/new poses and relative SE(2) transforms in report.
- Primary tau=0 only, B=plannedR1_old exactly. No extra0.25m motion or latency
  condition. Geometric incoming OLD tangent is separate from pose yaw. Retain
  both local and0.10m centered/truncated window diagnostics. Prior tau0 saved
  direction uses yaw(fixedR1), so paired direction changes include this explicit
  incoming-reference change as well as the observation protocol. Never paired
  previous tau1 against new tau0; former is selection/context only.
- New e_perp/local/window/yaw residuals (m/deg/deg/deg), all cases:
  006 .000035240467/90.110654262/89.924631874/72.000527929;
  007 .000110123546/1.182352010/74.506081545/18.025794710;
  013 .000310137952/.552479068/74.538388537/17.997232567;
  016 .000462856647/90.520649538/90.139008308/72.066661315;
  027 .151385084978/.006820173/.005773425/.003617659.
  Previous tau0 baselines and new-minus-old differences saved for every case.
- The006/016 FRESH winning segments are .150304/.150026m; both local/window
  differences remain about90deg. In007/013 the FRESH winners are58.1/224.8um,
  giving small local differences while0.10m windows expose about74.5deg. No
  threshold-based category or weighted score. At tau0 the four turn cases have
  small spatial distances in both protocols; no causal comparison with prior
  tau1 spatial residuals is asserted.027 direction/yaw is small but immediate
  cross-track .1514m reflects an ahead-of-observation FRESH start.
- Actual Isaac saved-geometry overview for all five, final images inspected
  alongside all five RGB1s. Initial013 far FRESH tail was cropped; wider trial
  occluded007. Both full trial sets preserved separately, including black image.
  Final007 uses its clear closer view; others use wider framing. No scene hiding,
  XY scaling or angular magnification. BlueOLD/magentaFRESH/yellowR0/lavender
  fixedR1/green augmented advance/orangeR1_old=B andOLDtangent/whiteQ/cyanFRESH
  tangent. Only display Z layers/stems differ. Exact camera offsets recorded.
- Research source versions retained by hash under processing_source_history
  where import/CSV/viewport changes occurred after earlier phases. Final code
  snapshots retained too. A post-freeze CSV change ensures invalid-first-row
  results keep all columns; it changes no planning geometry or metrics.
- Final actual validator passed with five distinct final sessions, no missing
  artifacts/failures and862 unchanged frozen input files. Independent scalar
  audit imports no project geometry helpers:380 values checked, max difference
  1.1102230246251565e-16; every requested/actual new pose and OLD array exact.
  Source/selection validators passed again. All16 checkpoint files rehashed
  with unchanged bytes/sizes/mtimes; official LightNav clean at
  c6f40e3220edbf7011e4f17eaf2c865416737d4d. Stopped only task-owned server.
- Full host pytest:1044 passed in40.57s, including41 new pure/mock tests;
  compileall, launcher bash syntax and diff checks passed. Earlier1040-test
  pass preceded extra provenance corruption tests. Synthetic tests are not
  research evidence. New report ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT.md
  includes all five rows, timings, failures and interpretation; README gains
  two sentences only after actual runtime validation success.
- Interpretation: OLD-consistent observations do not uniformly remove direction/
  yaw disagreement, and local tangent can understate the window-scale difference.
  This five-case pilot establishes genuine successive predictions at spatial
  surrogates, not actual robot/controller execution, latency failure, correct
  correspondence, graph necessity/superiority or navigation improvement. No graph,
  rigid reconciliation, factor weights, gates, controller or dynamics added.
- Decision: ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_VALIDATED. Review complete
  diff and staged diff, then one focused commit "stage0: validate OLD-consistent
  LightNav observations" and normal origin/main push. Final SHA/changed files
  recorded in ignored git_completion.json; generated RGB/raw arrays/screenshots,
  model weights, external code and environments remain uncommitted.

## 2026-09-15 — Persistent saved OLD/FRESH handoff problem GUI

- Request: demonstrate the actual saved OLD-consistent handoff geometry in an
  interactive Isaac GUI; no new inference, robot execution or reconciliation.
  Read AGENTS, README, prior work log and the OLD-consistent pilot report. Fetched
  origin before edits. Starting HEAD/origin/main were both
  e7f2f6a295f8937b0d5fcd3f823a08cfb283d3a3 on main. The two preexisting user
  controller/single-chunk configuration edits retain their task-start hashes.
- Primary source is robotless_old_consistent_observation/20260915T043415Z.
  Existing validator passed before implementation and during final validation.
  Independently compared all 208 source files against the task-start file set,
  sizes and SHA-256 values: unchanged. No source raw/derived/RGB/metadata writes,
  LightNav modifications, new inference, checkpoint changes or synthetic fallback.
- Added a pure geometry/state module, a frozen-source gate and evidence helper,
  explicit display configuration, Isaac GUI/launcher, fail-closed GUI validator,
  43 pure/mock contract tests and ROBOTLESS_OLD_FRESH_PROBLEM_GUI.md. README gets
  one short success sentence. Allowed cases remain exactly 006, 016 and 027.
- Reconstructed the augmented path [R0, OLD] directly from source arrays, with
  the connector included once and the pilot's shortest-angle/arc convention.
  Reconstructed B at 0.30 m equals saved R1_old exactly; marker starts exactly
  at R0 and ends exactly at B. Checked OLD/FRESH raw-to-world transforms using
  each actual saved observation pose. No FRESH re-anchoring or source scaling.
- Reconstructed closest FRESH projection, local tangents, centered/truncated
  0.10 m window tangents and shortest-angle FRESH pose yaw independently of the
  saved metrics lists. Compared every reconstructed metric/diagnostic field.
  Panel values are loaded from saved metrics, never hardcoded research truth.
  Cross-track/local/window/yaw (m/deg/deg/deg):
  006 .0000352404667 / 90.1106542623 / 89.9246318737 / 72.0005279288;
  016 .000462856647 / 90.5206495380 / 90.1390083079 / 72.0666613149;
  027 .151385084978 / .00682017277 / .00577342531 / .00361765864.
  027 is labeled BENIGN DIRECTION / YAW REFERENCE and its ahead-of-anchor first
  spatial reference is explained, without failure/success classification.
- GUI controls: all three case buttons; Replay OLD, Pause at Handoff, Reveal
  FRESH, Replay Full, Reset, RGB1 toggle, local/window overlay toggle, Overview
  Camera and Handoff Camera. Case switching resets marker/reveal/window state.
  Display-only timeline is 3 s motion, 1 s pause, 4 s static comparison; default
  hold remains open after eight seconds. Label visibly states OLD-CONSISTENT
  SPATIAL SURROGATE REPLAY. Original observation/request/readiness timestamps
  remain separate from host display/capture timestamps; execution time is null.
- Actual Isaac 6.0.1 loaded Hospital once: 1,936 prims, 126 authored collision
  prims, no rigid bodies, physics scenes, articulations or robot-named paths.
  Timeline stayed stopped at zero. Only a logical Xform/camera and DebugDraw
  marker/lines were used. Source XY/headings remain unscaled; display Z layers
  and annotation lengths/colors, both camera transforms and UI DPI are recorded.
- Retained three ignored layout trials. Initial inherited widget margins and
  desktop DPI clipped the lower/right panel. Removed cascading margins and
  fixed UI DPI/dock width. Final full application screenshots show every case
  button, all metrics/legend and real RGB1 together. The no-hold trial exited
  after its documented observation period. No scene geometry was hidden.
- Primary GUI evidence: robotless_old_consistent_problem_gui/20260915T064814Z.
  Produced six required before/after screenshots plus three window overlays and
  three overviews, each with source OLD/FRESH/metrics hashes, camera/reveal state,
  timestamp, research SHA and geometry audit. All 12 images manually inspected;
  027 after also inspected at original resolution. White Q can overlap B in the
  two tiny-distance cases; its distance is never visually enlarged.
- Actual renderer exercised the same callbacks used by every button for all
  three cases. Verified replay/reset/case changes, exact endpoint, automatic
  handoff/reveal order, RGB/window toggles and both cameras. Validator checked
  1,974 actual display marker samples. Persistent run rendered 1,248 updates
  over 12.030951 s after readiness and remained after the full replay. The primary
  tool-owned process later ended with exit143 after about340 s; this does not
  establish the cause of closure. A separate terminal-owned persistent GUI was
  launched at gui_native_controls to remain independent of tool lifetimes.
  Optional XTest native mouse checks did not establish interaction while other
  desktop windows were foreground; no native mouse success is claimed. The
  actual Isaac callback/control checks and screenshot review are the evidence.
- The separate terminal GUI also passed its twelve-second observation and later
  closed after about268 s; the final integrity audit records its process absent.
  Do not claim an open window at delivery or successful native mouse testing.
- Final GUI validator: ROBOTLESS_OLD_FRESH_PROBLEM_GUI_VALIDATED, zero failures;
  208 unchanged source files, 12 inspected screenshots, exact saved geometry.
  Full host pytest: 1,087 passed in42.18 s, including43 new pure/mock tests.
  Compileall, launcher bash syntax, source/config preservation and diff checks
  passed. Synthetic tests remain software checks, not research evidence.
- Claims remain trajectory-level and descriptive. No robot failure, collision,
  executability, graph necessity/improvement, general LightNav defect or natural
  deployment frequency claim. No graph/rigid/correspondence/factor/controller/
  dynamics implementation. Review diff/staged diff, make one focused commit
  "gui: demonstrate OLD-FRESH handoff disagreement", and normally push origin
  main. Final SHA recorded in ignored git_completion.json; generated media/raw
  evidence and external/model/environment data excluded from Git.

## 2026-09-15 — Genuine online robotless collector implementation and freeze

- User requested actual new live RGB → successive official LightNav predictions →
  fixed official MPC → finite-rate logical SE(2) execution, all60 frozen Hospital
  episodes, immutable per-event data/PNG coverage, validation and saved replay.
  Starting HEAD and fetched origin/main were both
  e56c0fe320309c2c4ffd8488fb3abdac42da8bfe. Preserved unrelated user edits in the two
  stage0 config files; hashes remain48b4d635… and0b0aed65… respectively.
- Implemented isolated serial LightNav JSONL worker, full-episode SlowFast history
  reconstruction, read-only official asynchronous MPC worker, exact unicycle
  executor, clock/activation journals, bounded JPEG writer, frozen30×2 schedule,
  resume/raw hash checks, every-event plots, independent full-stream validator,
  live collection GUI and separately labelled recorded-sample replay GUI.
- Official source remains clean at c6f40e3220edbf7011e4f17eaf2c865416737d4d;
  checkpoint revision7221d418bfff55cfcbadd09f7a26aaab81e1f8a6 and all10 configured
  file hashes verified. Persistent warmed GPU server started under
  data/robotless_online_handoffs_v1/smoke_20260915_01, with source-supported
  gpu_memory_utilization=.55, unchanged bf16 model/history. Upstream explicitly
  reserves2.29GiB KV cache and warns that this overrides fraction-based profiling;
  actual GPU process/memory coexistence is recorded, rather than inferred from
  the fraction. Isaac and CPU/model environments remain separate and unmodified.
- Technical preparations are retained individually, without replacing conditions
  based on geometry: smoke01 refused a changed source hash before acquisition;
  smoke02 exposed inherited CUDA library path contamination, fixed by the existing
  clean-environment launcher pattern; smoke03 caught render-triggered extra
  physics; smoke04 caught a second timeline-commit physics tick through independent
  stream reconstruction. Neither03nor04 qualifies as valid online execution.
- Native playSimulations guards now prevent automatic render/timeline physics;
  only explicit World physics steps advance execution. Every loop checks the
  previous recorded clock; exact resolved float32 dt drives integration.
  Smoke05 validated3handoffs but visual review found renderer-wide DebugDraw in
  model RGB. Subsequent captures clear annotations before rendering and copy RGB
  before restoring the display. Smoke06/07 inputs were individually inspected
  clean. Earlier technical records and images remain preserved.
- Smoke08 adds strict10Hz solve submission (no off-grid install solve):3moving
  successive handoffs,316states/315integrations, zero reconstruction error,
  41accepted fixed-grid solves,2stale results rejected,21real captures,4predictions,
  6/6required160dpi event PNGs. Exact client-inflight updates are11/11/11; capture
  counts1/0/1 reflect capture deadlines, not a stopped executor. Timing-valid2/3;
  history-full0 in this short smoke. Request-to-ready-seen coarse timing is stored
  separately from exact request-to-client-receipt timing and never silently mixed.
- Recorded replay verification in
  data/robotless_online_replay/smoke_20260915_07_replay02 inspected4actual application
  PNGs and callback/sample selection, preserved all source hashes, and performed no
  inference/integration. A first roof-occluded overview is retained as partial
  visual evidence; orthographic below-ceiling framing fixes display geometry only.
- Full host pytest after implementation:1286passed in42.40s. Earlier sandbox-only
  suite exposed expected Unix-socket permission failures; rerun on host passed.
  Final display-only framing receives actual Isaac smoke09 plus compile/bash/diff
  checks. Synthetic fixtures and CPU solver smoke remain software checks, not
  experimental evidence. All primary episodes and final counts are still pending
  at this implementation commit and will be appended after collection.
- No graph, rigid correction, correspondence optimizer, controller tuning, robot
  dynamics, prompt search, external source edits, generated-data commit or deletion.
  Review implementation/staged diff, commit the collector, freeze the full60
  schedule with its SHA/source snapshots, then execute every frozen episode.
- Final live GUI display check (smoke10) also shows the actual green executed path,
  blue active reference and clean latest saved JPEG. Display refresh now follows
  RGB readback, uses guarded file URIs, and settles before swapchain capture.
  This corrects a display-phase/asset-loading issue without changing physics,
  MPC settings, instructions or prediction inputs. Smoke09/10 and their final
  validation remain separate immutable technical evidence. Primary acquisition
  uses the same collector headless to reduce viewport overhead.


## 2026-09-15 — Genuine online robotless primary collection and results

- Collector freeze commit eed5f2c68f9bfc2b62d70a00a6d784647c32a971 was pushed before
  primary acquisition, following starting SHA e56c0fe320309c2c4ffd8488fb3abdac42da8bfe.
  Run data/robotless_online_handoffs_v1/primary_20260915T091900Z froze all 30 Hospital
  conditions × two repeats before inference. Schedule SHA-256 d5c3238100b13dfb25350b90b7ed13a70dd5d55e5545edf7cfe8dcc317a19081;
  config SHA-256 adc70b2f8d68b6228cda69b0a44e4de0c1753252601d6df50ef17e3fbf95e049.
  Only prior R0/instructions/category metadata were reused. All RGB and predictions
  were newly acquired in one uninterrupted session per episode.
- Executed every one of the 60 frozen episodes; collector exited 0 with
  COLLECTION_SCHEDULE_FINISHED. No replacements or count/residual-based early
  stopping. Terminal outcomes: 59 MODEL_STOP, one predeclared 40-attempt limit.
  Requested 1,000 predictions: 60 initial C0 plus 940 handoff attempts. Retained
  881 actual valid activations (880 moving, one stationary) and 59 STOP attempts;
  no model/protocol/controller/technical-error episodes occurred.
- Saved 4,639 live JPEGs: 4,537 delivered frames, 3,537 buffer-only uploads and 102
  explicitly indexed unsent terminal tails. Saved 69,125 states and 69,065 applied
  command integrations. All observed raw outputs were 10×3; arbitrary-N support
  remains software-tested. No raw array, old evidence or user change was overwritten.
- All 881 valid events have 10–43 exact client-inflight updates, 1–3 overlapping
  captures, newly solved official MPC commands and actual FRESH execution afterward.
  Full stream audit reconstructs all 69,065 integrations with zero error, verifies
  10,740 accepted fixed-grid 10 Hz submissions, first FRESH pre-integration B/P,
  current generation and successive FRESH→OLD lineage. Translation-threshold moving
  count is 804 plus 76 rotation-dominant events; stationary episode_000_repeat_01 /
  handoff_020 retains 27 inflight updates, two captures and 1.3333 s post-switch.
- Exact client-inflight pacing passes 838/881; 18 RTF values fall below 0.8 and 25
  exceed 1.2, with no inflight stall above the 0.25 s ceiling. Median RTT 0.3801 s,
  observation→ready-seen 0.4667 sim s, observation→actual switch 0.5667 sim s,
  matched-sample RTF 1.0010 (range 0.6092–1.6248), translation 0.2000 m and actual
  FRESH lifetime 1.1333 sim s. Whole-episode RTF 0.9795–1.0000 does not erase local
  pacing limits. Raw request-seen→ready-seen flags pass 554/881; 334 flags differ
  from the separately derived exact interval and both definitions remain explicit.
- Full-episode SlowFast nominal 64 is not a ring. Valid FRESH delivered histories
  range 8–205 (median 36); 246/881 satisfy nominal history_full. Reconstructed
  selected unique frames / processor slots range 8–82 for valid FRESH events.
  All 60 sessions preserve chronological frames, exact JPEG bytes, actions.step,
  capture anchors and source-pinned sampler reconstruction without future leakage.
- Official bf16 LightNav and Isaac coexisted on RTX 5060 Ti 16 GB; a saved snapshot
  records 15,796/16,311 MiB total, model 13,062 MiB and Isaac 1,959 MiB. No OOM,
  model reload, reduced history, quantization or environment/source changes. The
  source-supported 0.55 memory fraction is not a cap when upstream explicitly
  reserves 2,457,600,000 KV bytes. Task-owned server was identity-checked and stopped
  after all 60 episodes; the primary Isaac process also exited.
- After collection, three disjoint 20-episode plotting shards used the frozen
  functions and source hashes, followed by canonical whole-run aggregation. Saved
  1,762/1,762 valid-event PNGs plus 118 STOP diagnostics; no unplottable attempts.
  Added 60 overviews, 60 timelines, 99 contact sheets, all-event HTML/index CSV and
  aggregate distributions. All 3,041 index links resolve; 19 images directly reviewed
  with fixed category-first selection and separate stationary/STOP examples.
  Archived orchestration source, commands and hashes under logs/postprocessing.
- Raw valid-event diversity: OLD 145, FRESH 151, ordered pairs 357; dominant pair
  178/881 (20.20%). Episode medians and ordered-pair medians are weighted separately.
  No case was filtered for residual size; this is a dependent formulation-development
  corpus, with unknown collision validity and incomplete nominal-history coverage.
- Actual primary recorded replay in data/robotless_online_replay/primary_20260915T091900Z_replay01
  passed renderer/callback and four-screenshot checks, preserving all 97 source
  hashes. Displayed states 0/128/158 use saved records; no new inference/integration.
  It exits after verification. Final smoke10 live GUI PNG is independently hashed
  and archived by link; its end-of-execution nature and asynchronous null screenshot
  sidecar are explicit. Replay displays raw coarse timing flags, explaining the
  first pictured event's false flag versus the exact plot's true pacing flag.
- Full no-plot validator passed all 60 episodes, with errors=[] and causal_valid=true.
  Final pytest passed 1,286 tests in 44.44 s on the host; compileall and both new
  launcher bash syntax checks passed. Final external audit rehashed all 16 checkpoint
  files against startup and verified all 10 configured hashes; pinned upstream
  checkout remains clean. All 14 frozen collector source hashes and unrelated
  user config hashes remain unchanged. Full PNG validation and final Git completion
  are recorded in the following entry/bullets.
- Final full PNG validator also exited 0: valid=true, causal_valid=true,
  schedule_complete=true, errors=[], 60 episode validation records and 1,762 valid
  PNGs. Root validation SHA-256 a525bf52748573d13ceaaa71d31e763feacbe8fef362c27e98d07f4e4b10c1e2.
  Final status ROBOTLESS_ONLINE_HANDOFF_DATASET_COLLECTED_WITH_LIMITATIONS: 43 local
  pacing limits and 635 below nominal history-full length; no runtime blocker.
- Updated README and the full dataset report with actual counts, timing-window
  distinctions, raw diversity and weighting, image paths, exact commands, smoke
  and primary replay evidence, interpretation limits and verification. Independent
  documentation review recomputed counts and checked all headline claims/paths.
  Reviewed final diff and staged diff; commit only these three documentation files
  and normally push main. Final commit/push SHA is recorded in the ignored run
  git_completion.json. Generated media/raw data, external source, model weights,
  environments and unrelated user changes are excluded from the commit.


## 2026-09-16 — Straightness and challenging-geometry follow-up

- User asked how much of the online trajectory dataset is straight and whether
  difficult trajectories exist. Reanalysed all 881 valid FRESH events without new
  inference, source mutation or relabelling. The first previously linked example
  is straight; it was an order-based example, not a difficulty representative.
- Added a read-only reproducible trajectory-mix analyzer with explicit capture-local
  yaw/forward conventions, saved-row-only XY, minimum 5 cm motion, chord/arc .995,
  angle sensitivity 1/5/10/15 degrees, and separate actual execution classification.
  At 5 degrees: raw straight447/881 (50.74%), executed segments591/881 (67.08%).
  Equal-episode raw fraction48.30%; raw duplicate removal gives12/151 (7.95%).
  Short and rotation-only paths remain in the denominator, not called straight.
- Actual active-command composition uses |v|>=.05m/s and |omega|<=5deg/s, stored
  simulation durations and actual state displacement: straight65.67% time/76.27%
  distance; turning translation21.29% time; low-translation rotation5.58%; low
  motion7.47%. Includes initial OLD but excludes67.62s bootstrap; FRESH lifetime
  statistics separately exclude initial OLD. No intrinsic waypoint time assumed.
- Raw accumulated yaw >30/60/90deg occurs161/66/12 times; actual FRESH yaw travel
  exceeds those thresholds103/33/8 times. B-to-FRESH distance >10cm in138/881,
  >20cm in17/881. Direction maxima with near-zero incoming or tangent motion are
  explicitly flagged as poor hard-case examples, never deleted. All10 local
  angle>60deg cases use raw segments<=7.6mm; the largest164.37deg has7.57nm incoming
  displacement. The separately declared stable window subset has689 events.
- Inspected existing world/zoom plots for episode008repeat01/handoff013 (26.26cm,
  49.99deg window mismatch), episode013repeat01/handoff024 (27.91cm,36.54deg), and
  episode014repeat01/handoff025 (endpoint gap20.92cm,32.73deg). First two have
  substantial incoming motion and interior projection; all are timing-valid with
  unique raw ordered pairs. These measurements do not prove navigation failure.
- Derived outputs live separately at
  data/robotless_online_handoff_shape_analysis/primary_20260915T091900Z_v1,
  with2,884 input hashes, processing hash, configuration, per-event measurements,
  supplementary geometry-tail audit and test log. Original dataset and images
  remain unchanged. Independent analysis reproduced every headline fraction.
- Added13 synthetic software tests for raw preservation, no inserted XY origin,
  observation-local reorientation, yaw wrapping, pure rotation, chord/arc, command
  units, unequal recorded time weights, bootstrap exclusion and overwrite refusal.
  Focused87tests passed; full host suite1,299passed in42.76s. Synthetic checks are
  not runtime evidence. Compileall/diff checks and final/staged diff reviewed.
- Append the documented definitions, sensitivity, weighting and representative
  paths to the dataset report. Commit the analyzer, tests and two documentation
  files, then normally push main; preserve both unrelated user configuration edits.


## 2026-09-18 — Retire unrelated legacy generated research records

- User requested removal of old files/folders unrelated to current robotless
  research, then explicitly approved deleting previous final raw Jackal, EXP and
  DATA-02 records after being told that they were not recoverable from Git.
  Inspected branch/status/remotes first: main at
  5ea2bbf6008fe8133e06b5c2a7bafd8b0be175ac, with two unrelated configuration edits.
- Audited current data references, shared-code imports and presentation captures.
  Kept all robotless_* roots and the official reference reproduction, including
  primary 881-event evidence, trajectory-shape analysis, replay, smoke01 server
  provenance, smoke10 GUI evidence and predecessor robotless research. Retained
  all tracked source/configuration/tests/reports because historical names do not
  establish that current code no longer imports their shared modules.
- Deleted the explicit 37-target allowlist: 26 legacy roots under data/exp01*,
  exp02*, data02_*, stage0 and controller_effect_check; output/pdf and tmp/pdfs;
  two generated synthetic result files; seven Python/pytest cache roots. Removed
  91,181 regular files, 3,392 symlinks and 18,893 directories, then the two empty
  output/tmp parents. Removed allocation was 8,695,209,984 bytes (8.70 GB).
  The original GUI/physics captures and final PDF/video package were deliberately
  retired historical evidence, not mislabeled as regenerable caches.
- Wrote per-file deletion/preservation inventories with hashes, exact sizes,
  metadata and symlink targets to ignored data/cleanup_audits/20260918_robotless_scope.
  A one-off allowlisted script rejected tracked targets/unknown roots and checked
  complete inventories before deletion; it did not follow nested symlinks. All
  36,684 retained data files (2,504,027,893 content bytes) matched SHA-256 and
  filesystem metadata afterward. Both user configuration hashes are unchanged.
- Full read-only online validation including PNG checks exited 0, matching the
  preserved original validation JSON: valid/causal/schedule complete, 60 episodes,
  881 valid handoffs, 1,762 valid-event PNGs, errors=[]. Existing timing/history
  limitations remain unchanged. Independent post-deletion checks passed all
  2,884 analysis input hashes, the processing-source hash, 97 replay input hashes
  and 3,041 HTML index links. No inference, simulation or raw regeneration ran.
- The initial full suite reported 1,281 passed, seven skipped and 11 failures
  from removed historical corpora. Adjusted only three test files to explicitly
  skip those absent-corpus comparisons while retaining mathematical tests and
  always-running G2 configuration/hash/negative checks. Partial corpus restoration
  still fails rather than hiding missing/corrupt records. Focused checks passed
  50 tests and skipped 11. Original failure and follow-up logs are retained.
- Updated README to explain the current robotless runtime, retained evidence and
  unavailable legacy recordings. Added the dated cleanup audit, superseding the
  old local-retention decision without rewriting historical work-log entries.
  External LightNav source/weights, simulator installations and environments were
  untouched. The commit excludes ignored data and the two user config edits.
- Final full host suite passed 1,282 tests, with 18 historical-corpus-dependent
  skips and no failures, in 34.84 seconds. The additional separate G2 availability
  test accounts for the total changing from 1,299 to 1,300. Tests ran with bytecode
  and pytest cache writes disabled. No production source was changed. Final
  commit/push and audit hashes are recorded in the ignored cleanup directory.

## 2026-09-18 — GP-SE2-01 pre-primary implementation and environment freeze

- User explicitly authorized this offline GP-SE(2) formulation pilot, including
  actual optimization, unchanged official MPC comparisons, independent collision
  checks, complete images and actual Isaac GUI replay. Started from main
  ada33e145a412899ef5d33b05e9757fe4a780db7, matching fetched origin/main; preserved
  both unrelated Stage 0 configuration edits.
- Revalidated all 881/60 source events/episodes and 1,762 PNGs; rehashed all
  36,684 retained data files without changes. No inference, new collection,
  external LightNav source/checkpoint edit or deleted legacy-data assumption.
- Implemented locally linear SE(2) GP prior/interpolation, right-Jacobian math,
  constrained rigid/GP methods, immutable reference preparation, frozen-state
  official MPC rollout, direct geometry evaluation, failure-first aggregation,
  full plots/validator and Hospital comparison replay. M2/M3 differ only by
  obstacle inequalities. Added independent math, environment, formulation,
  timing, execution, corruption/coverage and failure-accounting tests.
- Validated actual Hospital export retry04 after preserving technical retries:
  2,058 meshes; correct transforms/instances/height clipping, solid interiors,
  thin walls and real doorway retained; unresolved 5.471 m² interiors excluded.
  Direct geometry is separate from conservative 0.05 m optimizer grid. All 13
  actual environment checks pass, with actual scene screenshots and 293 layer
  hashes. Historical per-layer byte identity is unavailable beyond same URL/config.
- Fixed pre-primary small-angle Exp/Log cancellation using existing series through
  1e-4 rad and independent matrix-exponential regression tests. Verified paper
  interpolation body velocity uses forward J_r, with finite-difference identities.
  Synthetic numerical qualification is explicitly separate from dataset evidence.
- Froze common radius/margin/height, 3 s horizon, goal/dwell, two identical starts,
  200 iterations/30 s per start, design weights and deterministic scarce-group-first
  selection rules. Environment screen finds only one defensible shortcut-sensitive
  corner passage; no threshold relaxation or outcome-driven case replacement.
  Implementation freeze precedes all primary optimizer/new rollout results.
- Pre-freeze full host suite passed 1,430 tests, with 18 explicit absent-historical-
  corpus skips, in 46.08 s. The preceding sandbox run had only two Unix-socket
  permission failures; no implementation workaround or test suppression was used.
  Compileall, both launcher syntax checks and staged diff checks passed. Final
  review additionally tests nested rollout-hash coverage before committing.

## 2026-09-18 — GP-SE2-01 primary comparison and actual GUI completion

- Froze implementation at 25d65ffab091c8155a1ace4ddef89a2a5739e911 and primary
  data/robotless_gp_se2_01/primary_20260918T054000Z before optimization. From
  881 events, 730 were eligible; selected ten distinct episodes/raw pairs:
  A3/B3/C3/D1. No relaxed thresholds, post-outcome replacements or new inference.
- Executed all 50 case/method attempts with 60 actual solver starts. Rigid generated
  ten candidates (eight independently plan-valid); both GP methods generated none.
  M2's 20 starts timed out; M3 had 19 timeouts and one solver failure. M2/M3 starts
  and configuration match in float64 bytes. Saved latest GP iterates violate dense
  nonholonomic/acceleration checks; this is finite-budget numerical failure, not
  an infeasibility proof. No RAW fallback or method-specific rerun was allowed.
- Ran unchanged official MPC on 30 independent accepted references, 900 solves,
  zero controller failures. All ten historical solve audits and every held-command
  state reconstruction reproduced exactly. Actual primary successes: native 6,
  adapter 7, rigid 7, M2 0, M3 0. M3 loses six native and seven adapter successes
  through no-candidate availability; GP paired-success secondary metrics are N/A.
  D native violates 0.05 m clearance and the usable gate; adapter/rigid pass.
- Saved all 460 required 160 dpi method plots, ten common overlays and linked
  index, retaining failures and separate prediction/execution traces. Actual Isaac
  verified 15 screenshots for regression/D/benign representatives and every
  method, plus reset/play/end/next callbacks and saved USD pose readback. GUI is
  explicitly OFFLINE COUNTERFACTUAL; no new solves/inference/states during replay.
- The first post-primary validator had one bookkeeping mismatch: reconstructed
  paired rows omitted already-saved optimizer wall time and deformation. Preserved
  its error report and archived code; fixed only the validator and a regression
  test in distinct verification_20260918T060500Z. No numeric primary or MPC rerun;
  all 1,097 method/result hashes remain unchanged. Corrected independent validation
  passes 42,512 checks, all 50 method records/460 images/943 links and actual GUI.
- Final preservation rehashed all 36,684 retained data files and all 63 checkpoint
  tree files with no changes. External LightNav remains clean and pinned, original
  validation unchanged, and both user configuration edits unchanged/uncommitted.
- Final host suite: 1,432 passed, 18 explicit absent-historical-corpus skips in
  49.22 s. Compileall, launcher syntax, diff/staged-diff checks and representative
  image inspections pass. Documentation includes every case/method failure,
  regressions before paired metrics, compute times, exact commands and correction.
- Operational GP_SE2_01_COMPLETED_WITH_LIMITATIONS; research interpretation
  NO_ADDITIONAL_BENEFIT_OBSERVED under this frozen formulation/budget. The final
  interpretation is appended separately from the preserved pre-GUI summary flags.
  Initial implementation push was rejected by automatic approval review; read-only
  verification established PUBLIC origin minju5054/se-3-reconciliation and ADMIN
  permission. Final normal-push handling is recorded in ignored git_completion.json.

## 2026-09-18 — GP-SE2-DIAG-01 feasibility diagnosis and single-seed recovery

- User explicitly requested a bounded diagnosis of the 40 historical GP failures,
  3-second known-feasible/perturbed fixtures, one fixed actual event and one
  evidence-selected change without physical acceptance relaxation. Started from
  main f0cd9de405f4b03cd6871ffdffd1dffeb74475c8, matching fetched origin/main.
  Preserved the two unrelated Stage 0 config edits. Historical numerical core
  remains byte-identical to the 25d65ff implementation freeze.
- Created data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z with original
  source/config hashes and per-phase diagnostic source snapshots. Before/after
  hashes match all 37,984 retained historical files and 63 checkpoint-tree files;
  external LightNav/MPC remains clean/pinned. Independently reloaded the fixed
  event from raw context, execution, commands and controller logs. No inference,
  collection, environment export, controller change or new MPC execution.
- Audited all 40 saved attempts before new optimization. All latest and all
  initial vectors are COLLOCATION_INFEASIBLE; collocation-pass/dense-fail count
  is zero. Reproduced 80/80 saved reports. Terminations remain 39 TIMEOUT and one
  SOLVER_FAILURE. Missing intermediate/history evidence remains unknown. Lateral
  midpoint failures and linear-acceleration knot-side failures occur in all 40;
  only 2/20 M3 latest iterates violate obstacle constraints.
- Independently verified S0/S1/S2 3-second GP reconstruction to roundoff. S3 has
  small nonzero reconstruction error but passes the original tolerance. The
  prescribed blind-spot interpolant passes support/midpoint lateral checks but
  reaches 0.00023094 m/s between them against 1e-5 tolerance. This synthetic
  example is separate from the actual rejection counts.
- Ran six 150-variable synthetic solves with original 30-second budgets. S0/S1
  known seeds are returned unchanged; S2 known seed improves while feasible.
  All three perturbed seeds recover original dense feasibility; only S2 perturbed
  converges, while S0/S1 perturbed time out with retained feasible callbacks.
  Synthetic results are solver diagnostics, not research performance evidence.
- Reproduced M2/M3 on episode_001_repeat_01/handoff_002 using both exact original
  starts: four timeouts, no candidate. Recorded actual callback histories, unique
  vectors/function calls and disjoint timing components. Environment queries,
  including M2 workspace queries, account for about 61–62% of baseline solve
  time. Instrumentation defers heavy callback validation; original selection and
  acceptance are preserved and timing differences disclosed.
- Froze decision.json before the one allowed B variant. Changed only the second
  seed to X=B Exp((t-t²/(2T))*actual_initial_twist), T=3 s; no parameter fitting,
  RAW seed, objective/derivative/constraint/tolerance/horizon change. Kept first
  seed and both 200-iteration/30-second budgets; charged seed creation inside
  the corresponding budget. Both M2/M3 return the same full feasible initial
  seed unchanged, objective improvement zero. All four variant solves time out.
  Full original dense/direct-geometry acceptance and 6,018 additional query times
  pass: lateral9.12e-12 m/s, peak linear acceleration0.266667 m/s², original goal
  position error0.026316 m and minimum footprint clearance1.326945 m.
- Saved 26 PNGs with numeric/hash provenance, separated synthetic/actual plots,
  and a 23-member 2.66 MB review ZIP without raw RGB/data/checkpoints. Actual XY
  context/zoom displays Hospital geometry, original OLD/FRESH/B and accepted or
  rejected GP curves; no execution trace is invented. A 243-file scientific
  manifest freezes all results and phase snapshots.
- First independent validation found 14 validator metadata errors: six short vs
  qualified initialization labels and eight relative vs absolute source paths.
  Preserved root validation.json. Corrected only validator comparisons with 14
  new regression cases; verification/validation.json passes 47,216 checks with
  zero errors. All 243 scientific hashes remain unchanged; no optimizer/MPC rerun.
  Archived corrected validator/test and exact scope in verification/.
- Final full suite passes 1,534 tests with 19 explicit skips in 51.11 s; 18 skips
  are retired historical corpora and one excludes S3 from an exact-reproduction
  assertion already covered by its dedicated error test. Compileall and diff
  checks pass. Tracked report retains key numbers, limitations, correction and
  reproduction commands without requiring local images.
- Operational GP_SE2_DIAG_01_COMPLETED_WITH_LIMITATIONS; diagnosis
  PARTIALLY_LOCALIZED; actual-event FULL_FEASIBLE_CANDIDATE_FOUND. The candidate
  is a retained feasible initialization, not solver recovery from an infeasible
  actual seed or evidence of better execution. Verified derivatives are the one
  proposed next element; no second remedy was implemented. Final focused commit
  and normal push are recorded in the new run's git_completion.json.

## 2026-09-19 — GP-SE2-DIAG-02 implementation and pre-comparison verification

- Started from reviewed main 311bf75c2d499a6de907580ce491ebcab875e847, matching
  fetched origin/main. Preserved both unrelated Stage 0 config edits, all 37,984
  retained historical files, the 256-file DIAG-01 run and 63 checkpoint-tree files.
- Prepared the fixed episode_001_repeat_01/handoff_002 and verified exact saved
  FRESH/deceleration vectors for both M2/M3; no seed fitting or original input
  change. Environment copy/export provenance and every original hash are checked.
  Three preparation-only technical errors are preserved: copy-vs-directory identity,
  a primary-only GUI mesh index, and a missing new seed output directory. No solve
  occurred during these corrections; original files were never written.
- Added CPU JAX 0.7.2 float64 AD for the original right-local chart, full GP
  acceleration/objective/motion/goal formulas and analytical original environment
  derivatives, with all three supplied SLSQP callbacks. Both modes retain the
  original GPProblem primal/cache, callback selection, physical conditions and
  independent full checker. No numerical finite-difference fallback is used.
- Frozen derivative protocol and 20 exact points before checking; 10 receive full
  150-column central differences and all receive fixed multi-step directional
  probes where meaningful. The smooth checks passed without changing tolerances.
  Exact relative Log cuts exposed an unsupported objective derivative: pre-guard
  attempts remain saved, and the provider now rejects such cuts explicitly before
  any actual comparison. Final authoritative verification is separate from the
  preserved pre-guard summary. Synthetic checks remain correctness diagnostics.
- Actual comparison is gated on the final authoritative verification and a source
  freeze, then exactly eight sequential starts with identical 30-second/200-iteration
  budgets. Cold setup and optional post-solve optimality diagnostics are separate.
  Implementation commit and measured comparison/final validation are recorded in
  subsequent append-only entries and local run provenance; no navigation claim.
- Final pre-solve gate: attempt_03, 19 supported states verified and the exact
  Log-cut state explicitly rejected; 1,360,500 coordinate entries compared with
  no exclusions, maximum primal error 1.39e-12 and maximum required scaled
  derivative error 0.17417 (<1). Original protocol/points/tolerances unchanged.
  Initial full suite: 1,620 passed/19 skips (69.06 s); subsequent focused suites
  cover the final wrap guard (37 provider tests), validator (13), and plotting (4).
  Compileall and diff checks pass before implementation freeze.
- The first comparison-freeze command stopped before any solve because the gate
  publisher shadowed its output filename with a source-loop variable. Preserved
  the valid JSON written under derivative_checks/se2.py, corrected only the
  publication function, and added an exact-filename regression test (14 validator
  tests pass). AST comparison proves every numerical validation node unchanged;
  the authoritative gate records old/new publisher hashes. A small follow-up
  commit fixes publication and the unsupported-cut coverage-table label before
  any actual solve. The comparison freeze additionally requires source-hash
  agreement with the checked provider and declared publication-only correction.

## 2026-09-19 — GP-SE2-DIAG-02 frozen comparison and reporting

- Implementation commits 06983f7 and 298f81e precede all actual starts; the latter
  is the authoritative comparison implementation. Executed exactly eight fixed
  starts sequentially once, with the same saved FRESH/deceleration arrays,
  150-variable chart, original primal evaluator/cache, SLSQP settings, physical
  constraints, callback selection and full acceptance. No inference, environment
  export, hard-case solve, MPC execution, new seed, weights or restoration.
- All four FD starts time out at 30 s: I0 retains no candidate; I1 retains its
  initial full-feasible seed unchanged. All four supplied-Jacobian starts
  converge in 1.744–2.129 s with changed full-feasible candidates. Both I0 starts
  recover feasibility. Both I1 starts decrease the original objective from
  3.963365592 to 0.194335067, raw reduction 3.769030525 (95.0967%). Changed I1
  support poses reach 0.272962 m displacement; body-speed change reaches
  0.366138 m/s. Final terminal speed remains nonzero because no original terminal
  stop constraint exists. This is numerical optimization evidence on one benign
  event, not navigation or execution evidence.
- Original dense/direct-geometry checks and the independent 6,018-time offset
  grid accept all four supplied final/selected candidates, including M2 obstacle
  checks. Max lateral velocity is 3.60e-8 m/s, max linear acceleration 1.094 m/s²,
  goal position error below 4.23e-5 m and minimum footprint clearance 1.326945 m.
  All four FD final iterates remain infeasible, including M2 I1's lower-cost
  iterate; no invalid lower-cost candidate replaces the selected seed.
- Prepared solve time falls 92.90–94.19%, objective calls 94.20–95.02%, and
  original primal cache misses 98.02–98.31%. Derivative AD/analytic geometry
  overhead, compilation, environment load, seed preparation, dense/full checking
  and optional optimality diagnostics are separately reported. Supplied cold
  setup + solve + checks totals 4.899–10.664 s; full-feasible history checking
  costs up to 7.657 s and is not hidden in solve time. Timing is one observation
  per cell, not a repeated runtime distribution.
- Local constrained stationarity diagnostics improve from 1.830836 at I1 to
  0.000104895 at the supplied I1 final, while I0 final is 0.00640364. Fixed initial
  speed bound row 90 has zero derivative and explains final active rank 30/31.
  Internal QP multipliers and these mixed-unit residuals do not certify a global
  or exact nonlinear optimum. Unsupported exact relative Log/goal-yaw cuts are
  explicitly rejected; no nonsmooth or invalid-domain derivative branch was
  encountered in the four actual supplied starts.
- Saved 18 figures with exact numeric/hash sidecars, all eight outcomes in the
  index, and a 74-file 7,616,326-byte review ZIP. Two presentation attempts and
  the minimal bundle remain preserved. Corrections only change axes/legend;
  all 18 numeric payloads and 40 numerical per-start files remain unchanged.
  The compact review excludes raw RGB/datasets, environment, weights, external
  source and full solver vectors. Independent scientific review passes 224
  checks, including unchanged seeds/boundaries/configs and identical supplied
  M2/M3 vectors for each initialization; these are not independent events.
- Operational GP_SE2_DIAG_02_COMPLETED_WITH_LIMITATIONS; derivative validation
  VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS. Supplied outcomes: infeasible
  starts recovered, feasible seeds improved, convergence with full feasibility,
  no unchanged-seed return, lower compute cost. The single proposed next
  experiment applies the unchanged provider/settings to predeclared existing
  hard cases; it is not performed in this task.
- Final reporting-source full suite: 1,650 passed, 19 explicit skips, 70.11 s;
  compileall and diff checks pass. Preserved the sandboxed attempt with 1,648
  passes and two existing local-IPC PermissionErrors; the same suite passes with
  UNIX sockets permitted, without source changes or relaxed tests. Earlier
  implementation/full-suite logs remain separate. No actual solve was rerun.
- Final source/artifact freeze contains 794 files. Independent validation passes
  48,785 checks with zero errors, reconstructing all eight starts, original
  primal/full acceptance, derivative error arithmetic, multiplier diagnostics,
  all figure numbers and the strict review ZIP allowlist without optimization.
  The authoritative report is the new run's validation.json; the separate CLI
  duration was not instrumented. Final rehash preserves 37,984 historical,
  256 previous-diagnostic and 64 external files, external Git state and both
  unrelated user config edits. Original numerical/acceptance source is unchanged.
  Final reporting commit and normal push are recorded in git_completion.json;
  generated data/ZIP/PNG and external/environment files are not staged.
- The staged diff check first included two new reporting helpers and found
  excess EOF newlines. Removed only those newlines, verified identical Python
  ASTs, and preserved the first PASS/source freeze/manifest under
  verification/pre_whitespace_correction with old/new hashes. Final compilation
  and staged diff checks pass. The unchanged independent validator passes
  48,889 checks, zero errors, against 846 final artifacts; its whole-run audit
  takes 48.21 s measured externally. No numerical code, optimization result,
  acceptance, figure or review ZIP changes and no optimizer rerun occurred.

## 2026-09-19 — GP-SE2-02 implementation and transfer preparation

- Started from reviewed main 1462e81d9c8ebba04466a07d10898284d0fccf2e, matching
  fetched origin/main. Preserved both unrelated user config edits; rehashed
  37,984 historical, 256 DIAG-01, 851 DIAG-02 and 64 external files. Resolved the
  four prescribed cases through the original manifest and reused the validated
  environment copy/export. All four have zero required gates; no case replacement.
- Added a separate fixed-case runner/protocol, evaluation wrapper and plots,
  with only an optional backward-compatible GP02 schema/SEED_ONLY extension to
  existing Isaac replay. Original GP mathematics, supplied derivative provider,
  SLSQP harness, full checker, reference preparation and official MPC stay
  unchanged. Copied original configuration and input contexts/arrays exactly.
  Physical u_minus and controller memory remain distinct, particularly the
  straight hard case's 0.8 versus 0.619266 m/s values.
- Froze 32 derivative-transfer points before evaluation: each case/method/seed
  and a deterministic perturbation. All pass using original DIAG-02 criteria;
  maximum primal difference 4.89e-12 and required scaled derivative error 0.157844.
  Required smooth directional comparisons number 170,686; two finite stencils
  crossing an obstacle-grid cell boundary are classified separately. No provider
  changes, fallback, new seed, or outcome-based case selection.
- Initial full checks: every I0 is infeasible; I1 is valid only for benign and
  invalid in all three hard cases. These inputs are retained, not fitted. The
  next primary phase contains 16 GP starts and eight original rigid starts,
  followed by original-policy independent MPC counterfactuals. No actual primary
  optimization or rollout has occurred at this implementation entry.
- Pre-primary full regression suite passes 1,723 tests with 19 existing skips in
  73.35 s; compileall and diff checks pass. Independent saved-transfer audit
  passes 1,935 checks. The two excluded obstacle stencils are duplicate support
  rows crossing from grid cell [1345,578] to [1345,579]; the finest h=2e-6 probes
  stay in the original branch and pass. Reporting-only artifact validators are
  explicitly outside the execution-source freeze and will be archived separately;
  no frozen execution code may change during primary.

### 2026-09-19 — GP-SE2-02 fixed primary results and review

- Ran primary_20260919T024000Z once from experiment commit
  bd273b22ed9ddcb35a3f2eadcf550895adee46b9: all 16 GP starts and eight original
  rigid starts, sequentially, with unchanged provider/formulation/config/acceptance.
  All four cases passed derivative transfer; none required gates or FD fallback.
  Twelve GP starts converge, five retain full-valid candidates, four have valid
  final iterates. Hard position/direction converges but fails between-point motion;
  large-turn M2/I1 retains valid callback_0115 while its final is invalid; all four
  straight starts fail SLSQP (iteration/LSQ/incompatible-inequality termination).
  No timeout, retries, extra seed or outcome-driven policy change occurred.
- Ran 16 fresh official-MPC counterfactuals (480 controller solves), preserving
  physical u_minus, controller memory, capture-frame inverse and 60 Hz integration.
  All six methods succeed on benign. Hard native succeeds once; adapter/rigid/M2/M3/
  seed-only have zero hard successes. Large-turn native-success → M2 yaw/dwell
  failure and M3 candidate-unavailable regression; no native-failure → GP-success.
  Hard candidate/plan availability is M2 1/3 and M3 0/3. Three invalid hard seeds
  are not executed. Two original-policy rigid diagnostic rollouts are plan-invalid
  due to endpoint errors just above 0.15 m. No collision, clearance, workspace or
  controller failure occurred among performed rollouts. Straight baseline motion
  violations preserve the recorded physical-speed/controller-memory discrepancy.
- GP prepared solves total 47.133004 s; compilation/warmup 12.263738 s; cold GP
  start costs including checks 80.712326 s. Entire optimization phase 87.411890 s,
  isolated MPC batch 2.239165 s, outcome evaluation 0.880615 s. These nested costs
  are reported separately and make no online-latency claim. Numerical objective
  improvement on benign is separate from the NO_ADDITIONAL_EXECUTION_BENEFIT result.
- Produced 48 frozen-source plots and four reporting-only full-time feasible-history
  supplements. Original autoscaled graphs remain preserved; supplemental early-N/A
  intervals and common axes are explicit. Actual Isaac replay validates 12 renderer
  captures for benign/first hard, all four cases selectable, zero new inference,
  optimization or execution during playback. Captures live in separate
  gui_primary_20260919T024000Z, not the original experiment or dataset.
- Added independent artifact/transfer validators, reporting matrix/cost aggregation
  and compact completion packaging. Original interim summary remains unchanged.
  Failed first validator preflight (heterogeneous/JSON CSV bookkeeping), initial
  cost-label report and first supplemental legend rendering are retained; only
  reporting/checker presentation was corrected, never primary scientific results.
  Corrected numerical/report preflight passes 17,720 checks; final package audit
  follows artifact freeze. Full pytest passes 1,767 with 19 existing skips in
  84.78 s; compileall and diff checks pass. No shell launcher changed.
- Documented all 24 start/method outcomes, controls, regression and compute costs
  in GP_SE2_02_HARD_HANDOFF_TRANSFER.md; README now reflects actual execution.
  Next single experiment proposed: isolate native/common reference preparation
  on the same large-turn event with unchanged MPC and original goal. Not executed.
- Final authoritative artifact audit: validation.json PASS, 64,135 checks, zero
  errors, 1,202 hashed artifacts in 53.854036 s. It independently verifies all
  numerical starts/outcomes, 52 plots, 12 actual renderer captures, 88-file final
  review ZIP and preservation. All 39,155 original/historical/diagnostic/external
  file hashes and both unrelated user configuration hashes match before/after.
  Review ZIP is 37,664,656 bytes, SHA256
  90f27aa197a95e17d8a8d4f39bf3d679afe2cd61382d92481582f703e348dae1.
  Final status GP_SE2_02_COMPLETED; numerical hard availability M2 1/3, M3 0/3;
  execution NO_ADDITIONAL_EXECUTION_BENEFIT. Generated artifacts remain ignored;
  only scoped code/tests/docs are committed. Exact final Git and push confirmation
  follow in local git_completion.json, excluded from the artifact-manifest cycle.

## 2026-09-19 — GP-SE2-REF-01 factor-isolation implementation

- Started from reviewed/fetched main 47668b868e84173fab4516ab7d5edb65ef75b841.
  Preserved unrelated Stage 0 config edits and immutable GP-SE2-02/01 sources,
  results/images, original environment and official external MPC. New run is
  robotless_gp_se2_ref_01/primary_20260919T062000Z; no synthetic replacement.
- Added separate two-case/four-variant input lineage, selector instrumentation,
  original-rollout wrapper, evaluation/contrasts, plots and static review package.
  Large-turn k=1 with row counts 10/9/30/30; benign k=2 with 10/8/30/30. R00 and
  R11 exactly reproduce source F_native/F_common; suffix applied once only.
  Original goal/frame, physical command, controller memory and source rows remain
  unchanged. No GP/rigid optimizer, controller/gain/selector change or new VLA call.
- Prepared 240 same-state official-selector probes without MPC optimization,
  followed by eight independent official counterfactual rollouts/240 solves in
  fixed order, all still pending at this implementation entry. Code/input freeze
  precedes primary. Reproduction tolerances and descriptive metric floors are
  fixed beforehand; physical acceptance is unchanged. Static PNG/index evidence
  is primary; optional GUI is not implemented for this diagnostic.
- Pre-execution validation: 116 relevant tests passed in 3.91 s; new execution
  sources compile and git diff --check passes. The official isolated Python
  imports the worker and the prepared request resolves all eight frozen inputs.
  Independent artifact checker is separate from the frozen execution provider.

## 2026-09-19 — GP-SE2-REF-01 actual factor isolation and report

- Froze implementation 48a8b47267b0d31bf1794e9dbba6a47ff10619f2 and all inputs,
  then completed the prescribed 240 same-state selector probes and eight
  independent rollouts/240 official MPC solves exactly once. No numerical retry,
  extra solve, GP/rigid optimization, new inference or controller change occurred.
  R00/R11 in both cases reproduce all compared original values bitwise, including
  reference selections, predictions, controller memory, commands and states.
- Large-turn R00/R10 succeed; R01/R11 fail original yaw/dwell criteria only, with
  yaw errors 0.332939/0.265866/37.303553/38.827543 deg. Resampling alone suffices to
  reproduce failure; suffix alone does not. Large-turn yaw-error interaction is
  +1.591064 deg. Benign all pass, with goal times 1.280/1.280/2.185/2.590 s.
  All eight pass environment/motion checks and have zero controller failures.
- Same-state large-turn final-row inclusion is 0.9/0.9/1.7/1.8 s; closed loop is
  0.9/0.9/N/A/N/A. Final yaw remains in the input, but resampling changes the
  fixed five-row target window. Stored logs connect different targets to weaker
  early rotation and incomplete rotation at 3 s. This is fixed-event diagnosis,
  not population evidence, a GP-feasibility fix or navigation improvement.
- Saved all 22 required figures/sidecars, static index and 5,081,835-byte review
  ZIP (SHA256 36138db62462c4ff48c7ffd183e47b45eb43bd297697741b33b18a4008dc0085).
  Original input-overview annotations were crowded. Preserved those files and
  added eight readable row/geometry detail figures in separate rendering-only
  presentation_20260919T063000Z with source/hash audit, no numerical rerun. Its
  3,683,258-byte input_details.zip passes all member hash/CRC readback checks.
  No Isaac GUI was run for this optional diagnostic presentation.
- Preserved the first non-authoritative validator preflight with six checker
  metadata-key omissions. Corrected only the separately excluded validator;
  second preflight passes 668,477 checks. Original numerical/code/plot files were
  unchanged. Initial full pytest has 1,837 passes/19 skips and two sandbox IPC
  PermissionErrors; the original log remains. Final unrestricted-local-IPC
  suite and final authoritative artifact check follow below.
- Final full suite: 1,843 passed, 19 existing skips in 94.27 s with required local
  IPC allowed. Compileall and diff checks pass. Authoritative validation.json:
  709,970 checks, zero errors/deferred checks in 8.005008 s. All 40,564 preserved
  file hashes and both unrelated user config hashes match. Frozen primary source
  is unchanged; supplemental rendering is separately versioned and validated.
  Operational GP_SE2_REF_01_COMPLETED; all five diagnostic observation/reproduction
  flags true, mechanism INPUT_SELECTION_COMMAND_OUTCOME_MEASURED. These are not
  navigation-improvement flags. Final Git SHA/push recorded in ignored
  git_completion.json; no generated data or external code enters the commit.

## 2026-09-19 — GP-SE2-REF-02 fixed-geometry selector implementation

- Started from reviewed/fetched main 9a615cee443848e79fff8805e9a038bbbcb2c341;
  no duplicate REF-02 implementation existed. Preserved both unrelated user
  config edits and checked 41,033 prior source/result/environment/figure paths.
  Authoritative REF-01 validation is the final 709,970-check PASS, not its earlier
  metadata-checker preflights. New primary_20260919T081000Z has both fixed cases.
- B/C reference and lineage copies are byte-identical REF-01 R11, float64 30x3.
  A keeps REF-01 R00. C changes only post-nearest selection: q=min(s_j+h,s_last),
  strict float64 searchsorted(left), fixed stride 1.0, existing rows only. No new
  interpolation, suffix recut, GP/rigid solve, VLA call or source modification.
- Added per-instance selector injection preserving official submit/integration
  bytecode, _solve/poll/path-installation and delegated MPC calculation. Actual
  submitted world and controller-local references are captured; C has an
  independent scan audit, A/B retain the original audit. MPC computation is
  identical; reference selector differs. No global monkey-patch or vendoring.
- Prepared exact installed-path matched probes on the same historical Native
  poses and six once-only rollouts/180 MPC solves. All probes/rollouts pending
  until implementation commit and code/input freeze. Pre-execution relevant
  tests: 151 passed in 1.73 s; compile and diff checks pass. No expected C success
  assertion; failures and missing metrics remain valid scientific outcomes.

## 2026-09-19 — GP-SE2-REF-02 fixed-input execution and report

- Froze execution at 1801da1603439182bb95ed38d706fe776ab88e92. Performed the
  planned 180 selector-only matched probes, then six independent primary
  rollouts/180 MPC solves once in the fixed large-turn/benign A/B/C order.
  No other MPC solve, retry, GP/rigid optimization or VLA update occurred.
  All four A/B baselines reproduce REF-01 references, commands, predictions,
  states and compared metrics bitwise. Both B/C installed dense arrays and
  lineage bytes remain identical; all 60 paired nearest/cost probes agree.
- Large-turn A/B/C success is PASS/FAIL/PASS; yaw errors are
  0.332938812/38.827543097/0.255007664 deg, goal times 1.67/N/A/1.58 s.
  B fails original yaw/dwell, while C passes every original condition.
  Matched final-goal inclusion B/C is 1.8/0.9 s; actual closed-loop inclusion
  is N/A/0.9 s. Targets already differ at t=0; significant actual commands
  diverge at 0.3 s. Actual solver arguments match the audited selected rows.
- Benign A/B/C all pass; goal times 1.28/2.59/1.28 s. C's linear command TV
  increases over B (0.539192505 to 0.799988233 m/s), with motion still valid.
  All six have valid footprint/clearance/workspace/motion/route and no solver
  failure. Minima are 0.302121251 m (large turn), 1.326944970 m (benign).
  No safety/motion regression. This is fixed-event selector recovery; no
  sampling invariance, GP improvement or general navigation claim.
- Preserved discrete source-progress overshoot: maximum 0.241379310 and
  0.206896552 original-row units. C does not reproduce Native targets exactly.
  Original goals, 3 s horizon, integration, controller calculation, source
  environment and all evaluation tolerances are unchanged.
- Measured 180 MPC solves total 0.705236 s inside the 1.319402 s worker process;
  selector-only probes take 0.147308 s inside that same worker. Preparation,
  worker, environment load and evaluation subtotal is 8.299599 s, excluding
  reporting/tests. Nested times are not added together. No online latency claim.
- Saved all six methods, 22 figures/sidecars and the primary 6,467,042-byte ZIP.
  Two frozen input tables had overlapping headings; retained them and created
  additive presentation_20260919T_ref02 with two corrected layouts and twenty
  byte-identical figures, exact unchanged numbers and no new numerical run.
  Its ZIP is 6,500,087 bytes. Independent presentation checks: 1,017 PASS,
  plus separate 340-check source/ZIP/numeric audit. No Isaac GUI run.
- Independent primary preflight passed 1,061,371 checks with zero errors;
  only final preservation and artifact-manifest checks were deferred. The
  final authoritative validator and full-suite result are recorded below.
  Report: docs/GP_SE2_REF_02_SOURCE_PROGRESS_LOOKAHEAD.md. Next single study:
  predeclared additional existing-hand-off B/C transfer with the same selector
  and acceptance frozen; no tuning or GP-deformation generalization assumed.
- Final required suite: 1,972 passed / 19 existing skips in 93.89 s, with no
  REF-02 skip. Compileall and diff checks pass; no shell launcher changed.
  Authoritative primary validation: 1,102,800 checks PASS, zero errors/deferred
  checks, 8.058177 s and zero new MPC solves. All 41,033 source/result hashes,
  both unrelated user config hashes and the official checkout are preserved.
  Operational GP_SE2_REF_02_COMPLETED; baseline/fixed-input/nearest/selection/
  large-turn recovery/benign-preservation flags true, new safety-or-motion
  regression false. Mechanism level is
  FIXED_GEOMETRY_MATCHED_NEAREST_SELECTION_COMMAND_OUTCOME. Execution success
  is limited to these fixed offline events. Final report SHA and normal push
  are recorded in ignored git_completion.json; generated artifacts stay local.

## 2026-09-19 — GP-SE2-REF-03 source-only cohort and execution freeze

- Started at reviewed main 22c655ea421c77c1bc7bd7c70da5fec7c9ae7620,
  fetched origin and preserved both unrelated local configuration edits.
  Reused the authoritative online/GP01/GP02/REF01/REF02 results and validated
  environment. No historical arrays, figures, source or external MPC changed.
- Authenticated all 881 source records and original eligibility. Of 730
  eligible records, the 10-event prior-selection union leaves 720 additional
  eligible. O/R/P/S pools contain 35/16/95/378 flagged records, with overlap
  recorded. Frozen greedy diversity/hash selection chooses six per group,
  all 24 in distinct episodes and ordered raw pairs, without shortfalls.
- Added exactly the two REF02 reproduction controls and original route-sensitive
  obstacle stress. All original state/frame/physical-command/controller-memory,
  goal, gate and reference inputs are preserved. B/C share byte-identical dense
  arrays and lineage. Independent prepared-input audit passed 594 checks.
- REF02 selector, rollout and numerical/evaluation core remain hash-pinned.
  New REF03 code only selects the source cohort, runs the existing per-instance
  wrapper and aggregates/plots/validates saved records. No GP/rigid/VLA, tuning,
  reanchoring, extra horizon, failure filtering or scientific retry is allowed.
- Primary primary_20260919T141000Z is prepared for 27 events / 81 rollouts /
  2,430 MPC solves / 1,620 selector-only calls, still unexecuted at this entry.
  Per case the order is A, B/C probes on current-A solve input poses, then B, C.
  Related pre-execution tests: 155 passed; compileall and diff checks pass.
  Documentation records frozen rules and pending outcomes before first solve.

## 2026-09-19 — GP-SE2-REF-03 frozen transfer execution and regression report

- Execution frozen at 343f1d2e57753e2ff99c4197c9f9bd20f6b2e5e7. Completed all
  81 independent rollouts, 2,430 primary MPC solves and 1,620 selector-only
  calls once, with zero technical method failures, missing results or retries.
  Per-case order remains A/current-A B-C probes/B/C. All 810 same-state pairs
  preserve nearest/progress/costs exactly; 481 change selected targets.
- Regression first: known obstacle stress episode_017_repeat_00/handoff_007
  changes from B success to C clearance and route failure. B/C minimum edge
  clearance is 0.063068847/0.036086666 m; C crosses outside the original gate
  interval. A also fails original clearance/route. No footprint overlap occurs.
- Additional O/R/P/S A/B/C success counts are 6/6/6, 5/4/5, 6/5/6, 6/6/6.
  Overall additional counts are 23/21/23: 21 all-success, two A/C-only successes,
  one all-fail. Additional success-to-failure counts are zero, but C introduces
  a motion reason relative to B on episode_014_repeat_00/handoff_029: physical
  initial angular acceleration 5.088190932 > 5 rad/s². A has the same source-
  memory-associated violation; B instead fails yaw/dwell. Original physical
  predicate remains unchanged and the event remains a full failure.
- Additional recoveries: episode_009_repeat_01/handoff_007 fixes B yaw/dwell
  failure; episode_018_repeat_01/handoff_006 fixes late goal entry with only
  0.18 s final dwell. Both goal-horizon inclusion and actual command records
  support the selection mechanism. No additional success beyond Native was
  observed. Controls are separate; all eight prescribed historical comparisons
  (six REF02 A/B/C and two GP01 stress A/B) reproduce bitwise.
- Among 21 jointly successful additional B/C pairs, mean C-minus-B goal time
  is -0.816667 s, linear/angular command TV +0.059418 m/s/+0.148864 rad/s,
  and minimum clearance -0.001202 m. Do not pool failed pairs or known controls;
  source episode/pair memberships and equal-weight summaries are retained.
- Preparation/worker/evaluation subtotal is 29.138591 s. Official MPC solves
  sum to 8.178146 s and selector probes to 1.307833 s inside the 14.964786 s
  worker process; inclusive times are not double-counted. No online claim.
- Saved all 193 primary figures with numeric/hash sidecars and a 12,134,883-byte
  review ZIP containing 76 PNGs and every 27 case overlay. Static diagnostics,
  no GUI run. Visual audit found correct data/axes and some clipped long
  boundary-zoom titles; original images are retained and the limitation disclosed.
  Additional-only aggregate figures do not replace the separate stress report.
- The first reporting validator preflight used obsolete planned-count request
  field names; preserved its failed report. Corrected only validator lookup.
  Preflight v2 passed 7,519,948 checks, zero errors, with final preservation/
  manifest/ZIP deferred. This is not a numerical rerun or authoritative result.
- Full required pytest: 2,102 passed / 19 existing skips in 104.88 s; no REF03
  skip. Detailed final validation and Git completion are recorded below.
  Research interpretation: MIXED_TRANSFER_WITH_REGRESSIONS. Next single task:
  audit the known stress's selected targets, MPC predictions and executed swept
  path at the original directed gate before proposing any selector modification.
  GP continuous-time feasibility and GP-deformed-path applicability remain open.
- Final authoritative validation: 7,565,567 PASS, zero errors/deferred checks,
  20.689802 s, no new MPC/GP solves. It verifies all 193 plots and compact ZIP.
  All 41,505 historical file hashes, both unrelated user config hashes and the
  official checkout are preserved. Compileall and diff check pass; no launcher
  changed. Operational GP_SE2_REF_03_COMPLETED; scientific interpretation
  MIXED_TRANSFER_WITH_REGRESSIONS. Execution-stage pending-review metadata is
  preserved and explicitly points to the completed research_interpretation.json.
  Final report SHA/normal push are recorded in ignored git_completion.json.

## 2026-09-20 — GP-SE2-REF-04 saved audit implementation freeze

- Started from `fc51724ececc29ad00e1d62b832ce097821d16b7`; fetched origin/main and inspected branch/remotes. Preserved the two existing Stage-0 configuration edits.
- Added a bounded saved-record audit, direct-geometry helper, runner/validator, static plot/review entry point and tests for the one REF-03 known obstacle stress event.
- Separated five actual selected targets, six Euler prediction states and exact reconstruction of already applied commands. Original full acceptance and gate geometry stay unchanged; missing future controls and out-of-horizon actual records remain explicit.
- Froze the <=1 ms swept-segment refinement and representative-interval selection rules before generating the new audit output. New VLA/GP/rigid/MPC solves and new rollout count are zero.
- Focused tests pass; full repository verification and the numerical report follow in the result entry. No historical output, numerical/selector core or external source is edited.

## 2026-09-20 — GP-SE2-REF-04 stress failure localized and reported

- Audited the fixed `episode_017_repeat_00/handoff_007` from REF-03 using source commit `0638cffb9b4cdb5d58521693bde969fd8435aae1`. New VLA inference, GP/rigid/MPC solves and new closed-loop rollouts are all zero. Used all 60 saved B/C predictions and original command/state records; preserved 44,929 original/core/environment/result files plus the two unrelated user configuration edits.
- B/C original evaluator outputs reproduce literally. B remains full-success (minimum footprint-edge clearance 0.063068847 m); C remains clearance/route failure (0.036086666 m), with no physical overlap and successful final goal/dwell. All selected target points and target-to-target polylines are clearance-valid. C's first prediction, issued at t=0, already forecasts an unsafe tail; the first predicted unsafe applied interval is issued at t=0.2.
- Actual C first clearance violation refines to [0.261875, 0.262500] s; minimum checked-polyline clearance is near t=0.371938557 s. C crosses the finite gate 15.158 mm outside its safe center interval at t=0.397130489 s. Euler/exact differences do not flip any observed first-interval clearance classification. Future optimized/pre-clipping controls are unavailable; inherited failures and after-horizon N/A records remain explicit.
- Primary output: `data/robotless_gp_se2_ref_04/audit_20260919T162300Z/`. Independent source recomputation/artifact validation passes with no errors. Seven static figures and a compact review bundle are preserved there.
- Visual QA led to an additive presentation-only correction, not a numerical retry: `data/robotless_gp_se2_ref_04/presentation_20260919T163000Z/`. Three figures expose previously occluded geometry and negative clearance ticks; four PNG/sidecars are byte-identical. All seven numeric datasets and primary authority hashes match. Preferred review ZIP is 3,190,627 bytes, SHA-256 `aecc5dd26823142a4b087856528ddf29ba063085a03063d8251320ecf96632fe`; all ZIP member bytes are validated. No GUI runtime was used.
- Verification: final full pytest 2,160 passed, 19 skipped in 114.60 s; compileall and diff checks pass. An initial sandboxed suite encountered two local Unix-socket permission failures; the full suite passed with local-socket access. Existing historical artifact skips remain disclosed. No shell launcher, system Python, external MPC environment or checkpoint changed.
- Result: `GP_SE2_REF_04_COMPLETED_WITH_LIMITATIONS` / `FAILURE_LAYER_LOCALIZED`. This is a saved-record failure diagnosis, not navigation improvement. Next single experiment: audit existing hard-case GP interpolation between support/collocation points for lateral/acceleration feasibility under the original checker, without making controller redesign a prerequisite.

## 2026-09-20 — GP-SE2-DIAG-03 saved interpolation audit freeze

- Started from `da5b93151aec62ce88d4fa6bda58ca8ae5b7dd86`; inspected status, branch, remotes, fetched origin and preserved both existing Stage-0 edits.
- Added a bounded nine-record audit, interval/time mapping and refinement, independent cubic-coefficient/temporal-derivative checks, static plot/review output and synthetic tests. No numerical core, checker, controller, selector or original source artifact was changed.
- Frozen scope is four hard initial/final pairs plus one benign final reference from the same GP-SE2-02 run. Physical acceptance is unchanged; derivative/reproduction tolerances and supplemental query/refinement rules are specified before actual numerical analysis.
- New inference, optimization, MPC solves, rollouts and GUI runtime remain prohibited. Frozen audit results and full repository validation follow in a separate append-only entry.

## 2026-09-20 — GP-SE2-DIAG-03 saved hard-GP interpolation failure localized

- Audit freeze SHA `d43cf9e5e37bee4a4f859686fd9b95349cdd74c0`; source numerical run SHA `bd273b22ed9ddcb35a3f2eadcf550895adee46b9`. All nine planned records are available (six unique vectors). Original support reconstruction, objectives, collocation/dense reports and available historical full outcomes reproduce literally. Hard final full-offset arrays did not exist after historical dense rejection; the unchanged full checker was additionally applied here and confirms failure.
- Four converged hard finals pass original collocation but fail lateral velocity, negative forward speed and linear body acceleration inside intervals. All original goal/route/workspace/obstacle checks pass. Benign saved final passes collocation/dense/full. Hard I0 initial is motion-invalid; hard I1 initial is goal-invalid.
- First observed acceleration violation refines to [0.002371,0.002449625] s. Sampled worst lateral speed is approximately 1.302e-4 m/s at 0.479 s; forward speed reaches approximately -0.003042 m/s at 0.478 s; linear acceleration reaches approximately -2.000078366 m/s² at 0.029 s. All final violations are off-collocation; no hard-final knot-side tolerance violation was found. I0 initial context does have knot-side violations and is not covered by that final-only flag.
- Independent cubic-coefficient and multi-epsilon temporal derivatives pass for all records. The evidence supports finite enforcement gaps; no implementation inconsistency was found at audited probes. G1 quarters detects all observed 28 lateral/1 speed/3 acceleration violating runs per hard final. G2 witnesses detect all violating intervals but only 14 of 28 lateral pockets; neither grid was used in a new optimizer.
- Primary output `data/robotless_gp_se2_diag_03/audit_20260920T010000Z/`; authoritative saved-record/artifact validator passes with zero errors. 1,275 original/core/environment/result files and both unrelated Stage-0 edits are preserved. Environment loading is 0.0441 s, nine-record analysis 4.2149 s, independent validation 12.7189 s; audit plus plots/package is 19.3434 s.
- Seven static figures, tables, index and compact ZIP are saved. Additive `presentation_20260920T011000Z/` corrects only crowded worst-interval labels and clipped-excess wording; all seven numeric payloads and six copied PNG/sidecars are unchanged. Preferred ZIP is 11,443,730 bytes, SHA-256 `4f36945b1c3cefa23f581a2a39d94a9c062f4ba96a210f7e3d01c27cd292f56a`. No GUI runtime was launched.
- Operational `GP_SE2_DIAG_03_COMPLETED_WITH_LIMITATIONS`: finite sampling and shared primitive algebraic checks are explicit limitations. New VLA/GP/rigid/MPC solves, rollouts and Isaac runtime are all zero. Proposed next single experiment is bounded quarter-point constraint refinement under unchanged physical tolerances; feasible solve recovery and execution benefit are not claimed or attempted here.
- Final verification: 2,225 passed / 19 existing skips in 119.71 s; no DIAG03 test skip. Compileall and diff checks pass, no shell launcher changed. Primary saved-record validator and additive presentation validator both pass with zero errors. Official MPC source hash and clean pinned checkout are confirmed unchanged. Normal current-branch report commit/push and final SHA are recorded in ignored `git_completion.json`; only code/tests/docs are committed.

## 2026-09-20 — GP-SE2-DIAG-04 implementation and protocol freeze

- Added a separate original/quarter motion constraint view and supplied-Jacobian
  extension; historical GP math/evaluator/provider/full checker remain unchanged.
- Froze ten paired starts using original GP-SE2-02 seed files: hard M2/M3 ×
  I0/I1 × G0/G1 and benign M3/I1 × G0/G1. No MPC/rollout/VLA/GUI is planned.
- Added original-prefix parity, extra-row derivatives, 15-point actual derivative
  gate, dense-first retention, post-solve rank diagnostics, plots and saved-record
  validation. Extra equalities/inequalities are 60/480; environmental rows unchanged.
- Preserved the two unrelated Stage-0 configuration edits. This entry records
  implementation only; actual verification and solve outcomes will be appended.

## 2026-09-20 — GP-SE2-DIAG-04 completed with no feasibility recovery

- Execution revision `e3776fa30a161ff361ba5f1d321a8caa788e729d`; primary
  `data/robotless_gp_se2_diag_04/primary_20260920T015400Z/`.
- All 15 frozen actual derivative records passed the unchanged tolerance gate;
  original primal/Jacobian prefixes are literal-exact. Added rows are 60 lateral
  equalities and 480 motion inequalities; original environment/goal rows remain.
- Completed exactly ten new sequential GP/SLSQP starts, no retries. All five G0
  latest vectors are bitwise equal to GP-SE2-02. Hard G0 converges but fails the
  original full motion checks. Hard G1 recovers zero of four paired starts.
  G1/I0 exits with status 4 or 8 and invalid motion; G1/I1 stops at its goal-invalid
  seed. Benign G1 exits with status 6 and retains the original full-valid seed,
  without objective improvement; G0 benign converges to objective 0.1943351.
- Post-solve equality spectra expose local row dependence/conditioning: benign
  G1 seed is rank 60/90 at every frozen cutoff; hard ranks are cutoff-sensitive.
  This is not an infeasibility proof and did not trigger row removal or tuning.
- Primary saved-record validator passes (24,699 checks); source/core/config/seed,
  official MPC and unrelated Stage-0 edits remain unchanged. New VLA/MPC solve,
  rollout and GUI counts are zero. Prepared solve total 16.3404 s; compile/warmup
  10.6903 s; candidate post-check 14.6758 s. Early G1 failures are not a speed win.
- Added isolated presentation-only rendering and tests; final review output is
  `data/robotless_gp_se2_diag_04/presentation_20260920T020100Z/`. All ten PNGs were
  visually reviewed and all plotted numbers match primary exactly (46 checks).
  Intermediate T020000Z is preserved; no numerical records or solves were redone.
- Operational `GP_SE2_DIAG_04_COMPLETED_WITH_LIMITATIONS`; numerical
  `NO_FEASIBILITY_RECOVERY`. Next single proposed comparison is quarter motion
  inequalities with original midpoint lateral equalities and unchanged full
  lateral acceptance, to isolate the additional equality effect. Not executed.
- Final validation: full pytest 2,281 passed / 19 existing skips in 149.62 s;
  targeted implementation 53 passed, presentation 3 passed; compileall and
  `git diff --check` passed. No shell launcher changed. Final normal commit/push
  contains only code/tests/docs; generated artifacts and user config edits excluded.

## 2026-09-20 — GP-SE2-DIAG-05 protocol and implementation freeze

Started from `070f2ca5cff44fcd3027e86fd198e8ebb1addd57`. Implemented a separate
inequality-only view/provider that imports DIAG-04 quarter mathematics and solver
policy unchanged. Original equality values/Jacobians are returned directly;
exactly 480 motion inequality rows are appended. Five G2 starts are scheduled;
ten DIAG-04 G0/G1 results remain historical. Synthetic parity, directional,
retention, rank, taxonomy, hash/no-retry and rendering tests pass (17 tests).
Actual derivative verification and five-start execution are pending this freeze.
No historical numerical files or user Stage-0 config edits were changed.

## 2026-09-20 — GP-SE2-DIAG-05 completed with limitations

Execution revision `b84d81598591eb3fc9ba017a0c251cc59bc528df`; sole primary
`data/robotless_gp_se2_diag_05/primary_20260920T064500Z/`. All fifteen frozen
actual derivative records pass, with literal G0 equality/base-prefix parity.
Exactly five new G2 starts ran once; G0/G1 remain historical. All five converge
with SLSQP status 0. Hard full recovery is 0/4: each grid-valid final fails
between-point lateral velocity, negative forward speed and linear acceleration.
The smaller residual inequalities are still failures at unchanged tolerance;
this is not a lateral-only result. Benign selects a new full-valid candidate,
J=3.963365592→0.1943350668. Equality Jacobians retain rank 30/30 and literal G0
identity at initial/latest/selected. Interpretation: MIXED_INEQUALITY_ONLY_RESULT.
Prepared solve cost totals 17.204 s; no VLA/MPC/rollout/GUI was executed.

Original/core/environment and 1,605 preserved source hashes remain unchanged.
Authoritative saved validator passes (22,025 checks); ten PNGs, numeric/hash
sidecars, static index and 21,794,527-byte review ZIP were generated and visually
reviewed. Full suite: 2298 passed, 19 existing skips; compileall and diff-check
pass. No solve was retried and no further refinement was implemented. The next
single uncertainty is whether interval-extremum motion-inequality enforcement
can remove residual speed/acceleration failures without changing lateral
acceptance. User Stage-0 config edits remain untouched and unstaged.

## 2026-09-20 — GP-SE2-DIAG-06 frozen witness refinement implementation

Started from `f86adffc4d54ae09a3581f557a9fc3eb77c07ed4`; inspected status,
branch/remotes and fetched origin. Both unrelated Stage-0 edits are preserved.
Added separate witness extraction, G3 value/Jacobian composition, runner,
static plotting and tests. Historical DIAG-04/05 numerical files are unchanged.
Source precheck finds 12 hard G2 violating motion runs and a three-row common
union on the unchanged 6,041 interval-side sample trace. No lateral witness or
extra equality; all five G3 starts use original saved seed bytes and the same
union. The unchanged full checker and retention policy remain authoritative.
Derivative verification must pass before the single five-start primary; no
adaptive round, retry, controller/VLA/MPC/rollout/GUI is authorized here.

- Before primary optimization, all 15 actual derivative records pass at the
  unchanged multi-step tolerances; equality and all G2 primal/Jacobian prefixes
  are literal-exact. New implementation tests: 18 passed; related unchanged
  DIAG-04/05 tests: 65 passed. Source preparation pins 1,806 files and three
  witness rows. Implementation/protocol commit precedes all five G3 solves.

## 2026-09-20 — GP-SE2-DIAG-06 non-lateral sampled gap closed

- Freeze/execution SHA `ca53a0744a98a5bbce1d153b5d056273d21b4fb1`; sole
  numerical primary `data/robotless_gp_se2_diag_06/primary_20260920T083200Z/`.
  Exactly five new G3 starts, all status 0 (141/144/143/140/122 iterations),
  no retry or historical reoptimization. Equality stays 30 rows; inequalities
  are 1296 M2 / 1386 M3. All 15 derivative records pass literal G2-prefix
  parity and unchanged multi-step tests. One three-row union is shared by all.
- Hard full-valid recovery remains 0/4. All three frozen witness sites repair;
  no above-allowance speed/acceleration pocket is observed on the entire
  original full/supplemental grid or either knot side. Lateral alone remains
  invalid: max |v_y|=1.217865e-4..1.218883e-4 m/s at .479 s versus 1e-5.
  Tiny nominal negative speed (-5.332e-6..-4.974e-6 m/s) and acceleration excess
  (about 9.2826e-6 m/s²) remain inside the original 1e-5 numeric allowance.
  No physical/numerical threshold is relaxed; finite sampling is not proof.
- Benign selects callback_0122, full-valid and changed from its seed;
  J=3.963365592→0.1943350667. Equality rank remains 30/30 at every available
  initial/latest/selected vector. No VLA/MPC/rollout/GUI was executed.
- Prepared solve total 17.384706 s; cold per-start sums 39.535801 s; five-start
  stage elapsed 46.776123 s. Source preparation, derivative gate, analysis,
  reporting and independent validation costs are separate in the report.
- The frozen runner's first aggregate CSV write failed because aggregate/
  was not created. Saved all numerical/analysis records and failed console
  log unchanged. Added a narrowly scoped report-completion helper: directory
  creation, saved aggregation and rendering only, no reanalysis/reoptimization.
  The frozen execution code hash remains unchanged. Ten PNGs with numeric/hash
  sidecars and static index are complete and visually reviewed.
- Authoritative saved validator passes: 23,270 checks, no errors, 50.953056 s.
  All 1,806 original/core/config/environment/result hashes and both unrelated
  Stage-0 user edits remain unchanged. Compact review ZIP contains all ten
  original PNGs, tables and links/hashes for full curve sidecars; 3,014,569 bytes,
  SHA256 ac050e8bf88af22d55eaf6e601b6e33b4363ea1e6c6dbb8c4bbb99e66743ee3e.
  Original complete-sidecar ZIP remains preserved; compact package validator passes.
- Full pytest with local IPC access: 2318 passed / 19 existing skips, 159.34 s.
  The initial sandbox run's two existing Unix-socket permission failures are
  separately preserved. Compileall/diff checks pass; no launcher changed.
- Operational GP_SE2_DIAG_06_COMPLETED_WITH_LIMITATIONS; research interpretation
  NONLATERAL_GAP_CLOSED_LATERAL_REMAINS. Next single uncertainty is plan-level
  lateral hard-feasibility semantics at the fixed pose-reference MPC interface,
  not another recursive witness round or an automatic tolerance relaxation.
  Only code/tests/docs are committed and normally pushed; data and user edits
  remain excluded. Final SHA/push completion is recorded in the ignored run.

## 2026-09-20 — GP-SE2-DIAG-07 execution protocol freeze

Starting main f454a3de86a18160ec182cccab3f4857f1b92bf0, equal to fetched origin/main.
Implemented an isolated fixed-reference official-MPC diagnostic, without changing
GP/selector/controller/acceptance code. Source verification checks 30,937 files
and preserves two unrelated Stage-0 config edits. All five G3 latest support
reconstructions and full checks reproduce literally: hard four fail lateral only,
benign optimized latest passes. Five exact unique references, no aliases merged.
Two historical solves in the existing pinned official MPC environment pass.
56 relevant tests pass (including 29 new DIAG-07 checks); no primary rollout yet.
Frozen intended primary: five rollouts / 150 solves, no retry or GP/VLA/GUI.
Protocol and commands: docs/GP_SE2_DIAG_07_LATERAL_PLAN_VS_EXECUTION.md.

## 2026-09-20 — GP-SE2-DIAG-07 fixed-reference execution result

- Frozen execution SHA: 666fd14ac6fa2ba14b8467e1e889cd9faec76760. Five exact
  unique references ran once: four hard G3 latest plans, one optimized benign
  control. 150 primary MPC solves + 2 historical audits = 152 total. No new
  GP/rigid/VLA/RGB/GUI/online runtime or retry; no frozen scientific code change.
- Hard plans remain lateral-only invalid, deployment_candidate=false. All four
  executions fail original goal position/dwell (position ~0.16993 m >0.15 m,
  yaw ~2.966 degrees passes); safety/workspace/route and actual command motion
  pass. Benign full plan and execution pass (0.0163534 m, goal entry 2.595 s).
- Interpretation LATERAL_INVALID_REFERENCES_EXECUTION_FAIL: no execution-success
  counterexample observed, not evidence that planned v_y caused the failures.
  Actual unicycle [v,omega] has no lateral command channel. Goal-region targets
  appear at 1.9 s; final GP row at 2.7 s. Plan endpoint has ~1.685 cm original-goal
  margin, with ~3.846 cm final tracking displacement. No unsafe saved prediction.
- Source inventory preserves 30,937 transitive files, original checker/core/MPC,
  environment, inputs and unrelated Stage-0 config edits. Historical baselines
  remain context only; no historical rollout was rerun.
- Original saved-record validator's only failure was the plotter's redirected
  stdout hash (empty at inventory time, final summary written afterward).
  Preserved the failed validator, manifest and complete log. New isolated report
  completion rejects all other errors and recomputes plans/executions/sidecars;
  verification/validation.json is valid=true and authoritative. Zero new solves.
- Added separate per-reference presentation for overlapping original titles:
  50 readable figures, presentation/index.html and review_bundle_readable.zip.
  All numbers/source hashes preserved. No GUI. Protocol, complete tables,
  commands, compute, limitations and next single planning-margin uncertainty are
  in docs/GP_SE2_DIAG_07_LATERAL_PLAN_VS_EXECUTION.md.
- Final required tests: 2,350 passed / 19 existing skips (163.35 s), compileall
  and diff checks pass. An initial sandbox run's two existing local IPC errors
  were resolved by the same full command with IPC permission, not a code change.
  Three existing test-only official historical audits also ran (relevant set +
  two full suites); task-wide actual MPC count is 155, distinguished from the
  frozen experiment's 150 primary + 2 protocol audits. No extra rollout or
  experimental condition; no GP/VLA/GUI runtime. Temporary pytest retention
  removed the first test-only audit artifact; both full-suite audits are retained.

## 2026-09-21 — GP-SE2-DIAG-08 implementation/protocol freeze

Starting main 1fa1e1e83074218b44456e4a00c64947a2591760 matches fetched origin.
Added isolated G4 endpoint-margin view: G3 plus one .11²-distance² row; fixed
.04 m reserve, unchanged original .15 m execution criterion and all other core.
Reuses original AD goal-position Jacobian (constant radius derivative is zero).
15 actual seed/G3-latest/fixed-perturbation gates pass; all primal/Jacobian
prefixes literal, M2/M3 dimensions 150/30/1297 and 150/30/1387. Maximum endpoint
primal error 6.4393e-15, fine directional error 2.1746e-9. Preserves 31,457 source,
core, environment and historical files plus two unrelated config edits.
Source authority explicitly uses DIAG-07 verification/validation.json.
Nominal endpoint <=.11 admission is separately frozen from the unchanged 1e-5
m² solver-grid allowance. Execution always uses saved latest poses[1:], with
original retention separately reported; lateral-only invalid stays diagnostic.
No primary solve yet. Exactly five original-seed G4 starts, no retry or margin
search, then only eligible exact-unique official-MPC rollouts. Detailed protocol:
docs/GP_SE2_DIAG_08_ENDPOINT_MARGIN_EXECUTION.md. No historical solve/rollout
reoptimization, VLA/RGB/GUI or online collection is included.
Pre-freeze relevant set: 84 passed (32 new DIAG-08 tests); compile and diff checks
pass. No actual MPC or primary GP invocation in these implementation tests.

## 2026-09-21 — GP-SE2-DIAG-08 result and preserved technical completion

- Numerical freeze SHA 5e2da93927196fb65cb5a260b2f0c40adbf7cc0d. Exactly five
  original-seed G4 solves, no retry; all status 0, grid pass. Hard retained/full
  candidates 0/4: lateral plus negative forward speed fail original full checks.
  New minima at .482 s are -1.688e-5 to -1.773e-5 m/s, beyond unchanged 1e-5
  allowance; frozen .480 s witness passes. Benign stays full-valid and optimized.
- Endpoint radius reaches .11 within original squared solver allowance. Strict
  predeclared nominal check records 4.7e-12–7.6e-11 m excess for hard; independent
  speed failure blocks all four even apart from this numerical boundary issue.
  Hard G4 execution/tracking/recovery N/A, not four failed executed trajectories.
- Exactly one eligible unique benign reference newly executed: 30 primary MPC
  solves + one protocol historical audit. Original goal/dwell/safety/motion pass;
  final position .01635339835 m, yaw .00322856 deg, sampled dwell .405 s. Reference,
  states, commands and selections reproduce G3 literally; historical G3 not rerun.
- First MPC worker import pulled absent SciPy into official MPC venv and failed
  before any actual call. Logs preserved. Added import-isolated bridge/worker;
  no environment edits or GP rerun. Separate runtime_20260920T153000Z with
  pre-call code hashes and explicit parent output links; actual additive runtime
  was uncommitted at execution, now included in report commit. Gate/parity/import
  tests pass. Numerical G4 source and original failed worker remain unchanged.
- Initial saved-record validator recomputes all numerical checks, but four
  descriptive reason-list comparisons fail due its += shared-list mutation.
  Preserved root validation.json and source. Additive completion admits only
  those exact errors, verifies nonmutating reason parity and rechecks execution,
  source/figures. verification/validation.json is authoritative, valid=true.
- Preserves 31,457 source/core/environment/historical files and both unrelated
  configs. 79 retained source labels checked. Prepared solve sum 20.298129 s;
  cold per-start sums 44.058008 s; no compilation/validation hidden in budget.
- Full tests: 2,393 passed / 19 existing skips (177.34 s), plus two later report
  regression tests. Full suite includes one actual historical MPC audit, saved
  before temporary retention. Task total actual MPC calls=32, distinct from
  30 primary + one protocol audit. GP=5; no VLA/RGB/GUI/online or historical rerun.
- 50 comparison PNGs + one explanatory speed-gap PNG, numeric/source sidecars,
  all-condition index and compact ZIP. No fabricated hard execution. README
  Current research state now includes verified DIAG-08 and DIAG-07 results.
- Operational GP_SE2_DIAG_08_COMPLETED_WITH_LIMITATIONS. Interpretation
  ENDPOINT_RESERVE_NOT_PLAN_FEASIBLE, not solver failure or infeasibility proof.
  Next proposed single test: fixed 11 cm reserve plus one predeclared speed
  witness at .482 s, unchanged tolerances/interface, checking for relocation.
  No such additional refinement is implemented in this task.
Visual QA covers all ten plot types and the speed-gap panel. Added five readable
tracking-panel copies to wrap a clipped legend, with exact numeric sidecar parity;
original images remain unchanged. Final packet: 51 figures + 5 readable copies.
Final new-task regression set: 45 passed in 8.58 s (no real MPC); compileall/diff
checks pass. Review ZIP is 5,631,435 bytes, 144 members, validated byte-for-byte,
SHA256 d5960adf4dcc879ac3d5beb6df2f5fbeb1d7843a4bc02ce42fea2a0c9f1c2900.
Pre-commit verification again preserves all frozen GP/runtime/source hashes.

## 2026-09-21 GP-SE2-JOIN-01 pre-primary freeze

- Freshly fetched main/starting HEAD 7864ab26bf0c16be0b8575338c6c269874246a34.
  Read the current GP01/DIAG07/DIAG08 results and acquisition/reference/MPC path.
  Preserve both unrelated Stage-0 YAML edits; no historical numerical core,
  external source/checkpoint or environment changes.
- New research line: controlled static obstacle reveal, genuine official OLD and
  FRESH, continued OLD during request, application-time B; no synthetic source.
  Three ordered source-only placements (1.5/1.3/1.1m actual OLD arc) and strict
  source qualification are frozen before inference. No qualifying source means
  no method comparison, rather than an invented avoidance path.
- Add explicit optional collector intervention (default None), runtime box and
  actual semantic visibility records; separate original Hospital + exact box
  optimization distance and independent geometry checker. GPU verified through
  isolated read-only host query; no runtime dependency/driver modifications.
- M4 reuses original GP chart/prior/motion/goal and supplied Jacobians; post-join
  original-FRESH cost, <=30 time/index choices, four support tube sites, original
  two seeds and solver budget. No DIAG06 witnesses or DIAG08 endpoint reserve.
  Add original-FRESH forward projection and sampled .30s attachment metric,
  exclusive provenance/qualification/solve/rollout/plot/validation entry points.
- Focused synthetic tests cover correspondence/frames/variable N/no fallback,
  sustained join/forward projection, exact primitive geometry, unchanged base
  constraints and verified changed-cost/tube derivatives. Tests are not primary
  evidence. Relevant regression suite also includes one explicitly historical
  official-MPC reproducibility audit, separate from primary execution counts.
- This entry is pre-primary: no JOIN01 source/result or claimed recovery yet.
  Infrastructure commit precedes the first actual obstacle-reveal attempt.

## 2026-09-21 GP-SE2-JOIN-01 actual obstacle-reveal outcome

- Execution freeze 0de41faebe9a9bb662b0f6773123e4c6975a8d1e; run
  data/robotless_gp_se2_join_01/primary_20260921T083025Z/.
  Actual Hospital Isaac + pinned persistent official LightNav ran all three
  predeclared placements once. Six real outputs (3 OLD/3 FRESH), 22 RGB frames,
  source MPC solve results 9/7/7. Recorded physical commands/states, observation,
  request/receipt/ready/application clocks and observation-anchored raw/world
  arrays remain immutable. Runtime box pose/dimensions/UTC/USD matrix preserved.
- Qualified=0/3. All B points remain valid (clearance .686667/.653333/.463333m),
  but original FRESH remains nearly straight and intersects each box (reference
  footprint clearance -.2m; not an actual executed collision). Cross-track
  disagreement .000160226/.000047310/.000047310m. No safe future FRESH exists
  under the frozen checks. Therefore no GP/rigid solve or common-B method
  rollout is performed, no M4 time/index or measured join is reported, no
  fourth placement or synthetic replacement. All five method ledger rows are N/A.
- Request RTT .290080/.216171/.199467s, FRESH observation-to-B translation
  .293333/.266667/.256667m. B follows continued OLD execution, not a reanchor.
  P01 inflight RTF1.903869 fails the frozen timing gate; P02/P03 pass.
- Visibility limitation: same-camera masks report only BACKGROUND/UNLABELLED.
  Visual inspection confirms the orange box in all triggering FRESH RGBs and
  its absence in OLD. Six wire images decode byte-identically to the saved
  triggering RGB, with matching hashes/response sequences. Do not interpret
  zero semantic pixels as absence of obstacle in the input. Preserve visibility
  gate failure; all three independently fail safety/disagreement anyway.
- Initial artifact validator incorrectly includes pre-OLD stationary bootstrap
  in the 'OLD during FRESH inference' predicate. Preserve root validation.json
  (3 errors); additive completion admits only those exact errors, rechecks all
  source manifests/qualification/plot values and the actual inflight-to-B OLD
  commands. verification/validation.json is authoritative valid=true. No
  scientific result or frozen numerical/acquisition source is changed.
- Final tests: 2,420 passed, 19 existing skips, 168.48s. New reporting tests cover
  bootstrap exclusion and missing/wrong chunk rejection. Compileall src/scripts/
  tests, shell bash -n, git diff --check, all 513 frozen source/env/config hashes,
  image numeric parity and review ZIP byte checks pass. Four historical real
  MPC test audits from the relevant/full test invocations are separately saved;
  total task recorded MPC calls=27 (23 source +4 tests), comparison calls=0.
  Source MPC reported solve wall sum .136773794s; optimizer cost N/A. Service
  warmup is separately logged and not counted as an OLD/FRESH prediction.
- Static review/index.html contains qualification table, actual before/after
  RGB links, all three source overlays and summary figure. All four PNGs visually
  inspected. review_bundle.zip=370,820 bytes/27 members, SHA256
  66d916183f7469b5d336a0f66b676c8161b8c8c95cd729b9ef446df82ca96bea.
  No fabricated GP/execution comparison trace or comparison GUI. Large/raw data,
  generated figures/ZIP and external assets remain untracked. Server shut down
  through recorded process-identity check; unrelated Stage-0 edits preserved.
- Operational GP_SE2_JOIN_01_COMPLETED_WITH_LIMITATIONS; research interpretation
  UPSTREAM_QUALIFICATION_FAILURE / NO_QUALIFYING_OBSTACLE_REVEAL_HANDOFF.
  The one remaining prerequisite to this frozen comparison is a genuine safe,
  obstacle-responsive FRESH under a separately declared upstream observation
  protocol with validated visibility measurement. No next experiment implemented.

### GP-SE2-ATTACH-01 — source-only and pre-primary protocol (2026-09-21)

Freshly fetched origin/main equals44e204499cb18855de2236780a4244903cd35d8a.
Preserved JOIN-01 and both unrelated Stage-0 config edits. New scope fixes
correspondence and treats T as an external condition, with no acquisition or
join-index/time search. Source-only full881 ledger has9 eligible events; the
predeclared ATTACH01-v1 hash order selects episode_021_repeat_01/handoff_003.
Original suffix/prepared clearance .09842650/.09842521m, e_perp .14120145m.
Record is moving under original collector displacement criterion but B velocity
is nearly zero; raw local OLD/FRESH identical, different observation transforms.
These limitations are preserved without replacing the selected source.

Fixed common value hash c67a128205a70383fbc9b89fdb21bd3d0d377287e571e52204f855500e6f3a7f.
Six external durations .4,.6,.8,1,1.2,1.4; original support k→common row k−1.
M4 masks only preservation before T and appends12 nominal tube inequalities.
Original30 equalities/903 inequality prefix and full checker remain unchanged.
14 seed/condition derivative gates pass; no GP optimization/MPC primary yet.
Pre-primary implementation must be committed/pushed before frozen14-start run.
Protocol/source ledger: data/robotless_gp_se2_attach_01/primary_20260921T103000Z/.
Details: docs/GP_SE2_ATTACH_01_FIXED_CORRESPONDENCE_DURATION.md.

Pre-primary verification: full suite 2436 passed/19 skipped; two existing IPC
socket tests were blocked by sandbox permissions and then passed unchanged with
local socket access. Latest focused ATTACH suite:21 passed. Compileall and
whitespace checks pass. Full suite's one historical official-MPC audit is saved
separately in preflight/repository_test_historical_MPC_audit.json; zero primary
MPC/GP calls before the freeze commit. No scientific retry occurred.

### GP-SE2-ATTACH-01 — frozen duration comparison completed (2026-09-21)

Pre-primary implementation c540533bdf72b0a7ea31b19fd1c0819697a5596e was pushed
before14 scheduled GP starts. No source replacement or retry. Six fixed external
T conditions retain one exact common reference and original acceptance. M3
retains I1/callback_0117; M4 T=1.4 retains I0/callback_0114. Later final iterates
fail original between-point speed checks and remain recorded. T=.4/.6/.8 fail
solver grid and physical/tube checks; T=1.0 converges and passes original full
acceptance but exceeds the strict nominal tube by5.55e-12–1.69e-11m. No tolerance
change or execution of that invalid result. T=1.2 also retains speed/tube failures.

Four independent actual counterfactuals /120 primary official-MPC solves:
Native/Adapter/M3/M4(1.4) all pass original safety/motion/goal/dwell. Sustained
original-FRESH attachment .155/.155/.165/.170s; GP planned attachment .203/.278s.
M4 is slower to attach/reach goal, with lower linear command TV and slightly
higher angular TV. Minimum clearances all about.098m. One valid transition is
positive constructibility evidence with a trade-off, not attachment-speed or
navigation improvement. Shortest TESTED full-valid condition1.4s is not optimal
or a robust lower physical feasibility bound, especially given T1.0 roundoff.

Source limitation is explicit: selected B nearly stopped, raw OLD/FRESH identical
in local frame, world curves aligned. Existing e_perp .1412m is a finite-polyline
endpoint gap; infinite-line perpendicular error only1.12e-5m. Large lateral
mismatch with moving B remains the single largest uncertainty; not implemented.

Prepared solve total103.403s across14 starts; optimization phase143.872s including
warmup/check/serialization; official primary MPC total.433468s. Simulation time
is separate. No new LightNav, Isaac GUI, reveal or controller changes. Complete
881-event ledger and73 unique retained GP vectors independently revalidated;
artifact valid. Frozen input/core/environment/JOIN-01 hashes still match.

Full final suite2441 passed/19 skipped,168.22s with local IPC access; compileall
and diff checks pass. Two repository-test historical MPC audits (one pre-primary,
one final) are separately preserved; not counted as primary rollouts. Static
primary plots preserved. A separate reporting-only review_v2 fixes legend/spacing,
with no numerical rerun or overwrite. Authoritative review:
data/robotless_gp_se2_attach_01/primary_20260921T103000Z/review_v2/index.html
and review_bundle_v2.zip. Operational COMPLETED_WITH_LIMITATIONS; original
physical acceptance, source records and unrelated Stage-0 edits preserved.

### JOIN-SOURCE-02 — pre-development source protocol (2026-09-21)

Fresh fetch confirms fb181dbf4553aa91b4c864f2c61c6fea71ba2cc4. Preserve JOIN-01,
ATTACH-01, original Hospital/export/LightNav/MPC and both Stage-0 user edits.
New source-only line: bounded paired obstacle-off/on/sham development, then only
if qualified a separately frozen three-episode online confirmation. No GP/rigid,
correspondence choice, attachment-duration sweep or downstream comparison.

Actual Isaac inspection of two declared poses confirms target shelf/doors.
Existing natural cart internal-reference mesh parity passes; same-product
instance-ID off/on/off pixels0/14682/0 at identical camera/pose/time. Original
semantic-label failure is not interpreted as invisible input. Actual mesh slab
projection area .270833411m²; no box replacement (AABB overfills .351985296m²).
Render presence and direct-oracle occupancy are separate explicit records.

Read-only official contract audit retains source c6f40e3/checkpoint7221d418,
task vln/vlnce, actual camera, unmodified frame preprocessing/RVQ/SlowFast.
Object-target instructions fit official Prompt Guide. History is genuine live
approach, no copied frames to fill counts; development suffixes end at latest
activated OLD observation and paired render stays fixed there. At most80 live
captures/location allow16/32/64 delivered suffixes when available. Two geometry-
only prop placements per location use short cart axis and .60s latency design
reserve. Empty placement/history remains unavailable; no threshold change.

Run data/robotless_join_source_02/source_20260921T112111Z. Technical preflight
has zero model/MPC calls. Full repository tests2500 passed/19 skipped172.67s;
latest67 focused tests pass. One historical official-MPC test audit is separate
from source acquisition. Compileall/diff checks pass. All524 initial preserved
hashes match. Development protocol/code freeze precedes first actual model
source request; commit and normal push required by runtime verifier.

### JOIN-SOURCE-02 — bounded source screening result (2026-09-21)

Pre-development code/protocol 3e082228f666ba2a0aad758a7de37693c54cb420 was pushed
before actual source execution. Run source_20260921T112111Z records all 12 declared
conditions. Two location01 H16 conditions completed independent-session A/B/A';
both were CHANGED_BUT_UNSAFE. Six entries lacked authentic H32/H64 history; four
others lacked the frozen geometry/latency placement reserve. No replacement,
extra condition, model setting change or threshold relaxation occurred.

Both off/sham arrays are identical. Both on arrays differ from off and equal each
other. Interior lateral separation is 0.006775 m, reliable yaw difference 31.046266
degrees, and returned arc length changes from 1.356219 to 0.848790 m. On raw paths
overlap the actual cart (-0.20 m footprint-edge clearance). Off/sham are safe without
the cart (0.935417 m) and unsafe with it. On endpoints remain 0.487315/0.508862 m
before the bypass plane. Off/on instance pixels are 0/15779 and 0/15194. Paired
JPEG/wire parity and actual mesh/environment checks pass. Target-visible=false,
STOP=false; pointing/token changes do not identify a decoder/perception cause.

Live history captured 33/59 frames, with 28/56 usable causal prefixes. Location01
ended at the target-approach boundary. Location02's oracle abort preceded unsafe
integration: last actual edge clearance 0.050146128 m, proposed endpoint 0.047051410 m.
No collision, teleport or fallback occurred. Original resolved-dt updates reproduce
481+871 applied steps exactly; one attempted command remains unapplied. The abort
concerns an original Hospital MopSet, not the development runtime cart.

Actual model predictions: 28 = 22 history + 6 screening; synthetic startup warmup 1
is separate. Buffer-only requests: 159 = 69 history + 90 screening. MPC submitted
205 attempts, with 204 saved completions and one unknown result; observed solve
sum 0.802302 s. Sixteen stale results remain recorded. One late location01 result
is in location02's journal; accounting uses its own episode/solve identity. History
RTT sum 7.146927 s and server latency 7.041935 s overlap. Screening elapsed 12.554 s;
server startup 17.012 s and lifetime 148.204 s are separate. Two historical MPC
test calls are not source evidence. GP/rigid/reconciliation calls remain zero.

NO_QUALIFYING_DEVELOPMENT_SCENARIO: 0 of 3 planned confirmations attempted, with
all three ledger entries N/A. Zero qualifying online sources; no downstream source
bundle. No further acquisition or optimization was implemented. Limits: H16 only,
two placements 2.15 cm apart, uncertain target recognition, no online/traversal claim.

Full suite: 2508 passed, 19 skipped in 171.95 s; compileall and diff checks pass.
Saved-only validation preserves scientific failures. Supplementary geometry audit
v1's ideal 1/60 clock mismatch is preserved; authoritative v2 uses the original
float32-resolved dt and passes 58/58 checks. Review v1 is preserved; a new layout-only
wrapper moves the world legend without changing frozen report code or numbers.
Final review_v2/index.html and review_bundle_v2.zip contain actual RGB, pointing,
whole-path geometry, all conditions and numeric/hash sidecars. Exact source/status
information is in JOIN_SOURCE_02_OBSTACLE_RESPONSE.md. Historical raw/core files
and unrelated user configs remain unchanged.

Authoritative saved validation_v2.json passes all 1507 checks. The initial checker
attempt's string-ID schema error and validation.json's inclusive/exclusive bbox
mismatch are preserved; only the new saved-record validator was corrected to
the frozen producer's contracts. Eight focused validator tests pass in addition
to the full suite. No source/model/MPC rerun occurred. review_bundle_final.zip
(1,841,152 bytes) adds protocol/counts/authoritative audits to the same review_v2
figures. It excludes the full dataset, environment and checkpoint.

### JOIN-SOURCE-03 — MPC provenance and bright-input protocol (2026-09-22 KST)

Fresh origin/main matches97ba41aab0ec846f9e6ddc0fc913dc3277cccb2e. Preserve the
SOURCE02/JOIN01/ATTACH01 evidence and both unrelated Stage0 config edits.
Six historical provenance chains plus current official module import match the
pinned MPC file/settings; no tracker/solve is needed for this audit. REF selectors
remain offline per-instance experiments, not the online collector default.

Actual Hospital technical renders add one fixed neutral downward ceiling fill,
without touching original USD/lights/materials/camera. Mean sRGB-code luma rises
12.271445→77.479331, cart median12.2006→81.3702; unchanged instance pixels and
protected geometry. No model outcome was used to choose brightness. The first
technical snapshot's timeline-settling failure is preserved; setup rendering
resolves it before the same strict synchronization check, not by loosening it.

Freeze maximum7 independent-session A/B/A' conditions/21 terminal predictions:
core DARK/BRIGHT H16, conditional clearly visible target pose H16, same-pose H32,
then same-core near/medium/far cart distances. Existing authentic chronological
DARK histories are preserved, with only terminal RGB re-rendered; this limits
lighting causality to the current image. H32 uses the exact H16 terminal images.
No new history collection, MPC execution, GP/rigid/reconciliation, or optional
receding-horizon rollout. Later-chunk recovery remains untested. Pre-prediction
code/config/input freeze and normal push precede all diagnostic model calls.

### JOIN-SOURCE-03 — bounded diagnosis completed (2026-09-22 KST)

Execution revision1904bd2031ffa14cf86399ffdd1277a49bd4cc92 was committed and pushed
before the first new diagnostic prediction. Run remains
`data/robotless_join_source_03/cause_20260921T153120Z/`. All seven frozen conditions
completed once, each OFF/ON/sham in independent official sessions. No retry,
threshold change, new controller, avoidance policy or reconciliation occurred.

LIGHTING_CONTRIBUTES_BUT_NOT_SUFFICIENT. Same-pose terminal-image relighting
increases mean sRGB-code luma12.271445→77.479331 and cart median12.2006→81.3702,
with matching protected scene/camera geometry and instance pixels. Core ON minimum
edge clearance improves−.20→−.111383570m, but both first unsafe segments are row2→3.
BRIGHT shortens the nearly straight path rather than producing a safe detour.
First returned points remain safe; the claim that this core path starts inside
the cart is not supported. Stored history remains authentic DARK RGB, so the
lighting intervention concerns the terminal image only.

Target-pose ON minimum−.080925625m still fails, although29,221 shelf pixels are
visible in the renderer. All21 model responses retain target-visible=false and
OPOS not_visible. H32 at the identical final JPEG worsens clearance to−.20m.
Near/medium/far ON minima are−.20/−.192836088/−.20m. Near visibly curves right but
collides before clearing the cart; all three remain unsafe. OFF/sham are literally
equal in every condition. These observations weaken a short-history, distance-only
or random-sham explanation within the tested range; target grounding versus
upstream spatial/action generation remains NOT_ISOLATED. Root cause remains
UNRESOLVED_AFTER_BOUNDED_DIAGNOSIS. H64, all-bright history and later chunks were
not tested. No optimization-ready online source, new B or traversal was obtained.

Official MPC clean-source/hash/settings match six historical lineages and current
import. New scientific calls:21 model predictions,363 buffer-only restores; server
startup warmup1 is synthetic and separate. Terminal RTT sum6.781750s, worker sum
37.372727s, startup17.261760s and server lifetime118.809556s are nested and not
additive. Diagnostic MPC/GP/rigid/rollout calls0. Existing full-suite historical
MPC test calls1 (.041430s), exact saved command/reference parity, separately
preserved. Server stopped cleanly using its recorded identity.

Frozen reporter's GeometryCollection drawing failure and partial review remain
preserved. A new presentation-only adapter handles that type and writes review_v2;
original scientific source, arrays, summary and validator remain unchanged.
Authoritative validation_v2 passes2019 checks, including1899 preserved historical,
core and unrelated-user-file hashes. Additional review validator checks all210
rows, three paired figures and ZIP member bytes, without inference. All21 final
figures were visually inspected. Root index.html and review_bundle_final.zip
(5,302,134 bytes) link actual RGB, lighting/history/distance comparisons, pointing,
clearance and evidence matrix with numeric/hash sidecars.

Full suite2536 passed/19 skipped (174.14s); final SOURCE03/presentation tests23
passed. compileall and diff checks pass; no shell launcher changed. README and
JOIN_SOURCE_03_BRIGHT_CAUSE.md record every completed condition, source hashes,
exact commands and interpretation limits. Previous JOIN01/SOURCE02/ATTACH01 and
both unrelated Stage0 configuration edits remain preserved. Stop at upstream
diagnosis; no next experiment implemented.

### JOIN-SOURCE-04 pre-prediction protocol, 2026-09-22

Fetched main24a0209 and preserved the two unrelated Stage0 config edits. Read
SOURCE02/03 source/protocol/history/official pointing and RVQ decode paths.
Added a separate fixed H16/K0,1,2,4,8 persistence diagnostic. Re-rendered exact
SOURCE03 moving-pose lineage in Isaac as32 all-BRIGHT OFF/ON frames with the
same natural cart mesh/transform and camera. No model/MPC/GP call during capture.
Cart visibility671..15779pixels; final RGB visually checked. Frame/pair geometry,
camera and source hashes are frozen before independent-session predictions.
Clamped APOS is censored; depth/instance first-hit and nominal-ground ray are
explicit evaluator proxies, never model inputs or exact intended waypoints.
Maximum7 terminal predictions; no retry, online execution or next-stage work.

SOURCE04 pre-request technical correction: pushed freeze5f06f07 attempted seven
worker processes, all failing at import before any session/model request because
the official venv has no Shapely. Preserve the entire044703 run and its one
separate synthetic server warmup. Remove only the worker's evaluator dependency,
read declared IDs/scope from protocol, verify import in actual external Python,
and add regression test. New050237 run reuses the exact32 bank files with no
rerender, K/threshold/settings change or response-driven retry. Freeze/push again
before the first actual scientific prediction; no external environment changes.

### JOIN-SOURCE-04 completed bounded diagnosis, 2026-09-22

Pushed corrected freeze7682acd before7 scientific predictions in050237 run.
All premodelK0/1/2/4/8 available; all32 paired BRIGHT frames preserved, exact
SOURCE03 cart/camera/target/instruction and clean official source/checkpoint.
Seven independent sessions each delivered15 buffer-only frames then1 prediction:
105 buffer restores,7 terminal calls,0 diagnostic MPC/GP/rigid/online rollouts.
The original044703 pre-request failure and all its logs remain preserved.

K1 clearance-.143249985m, K2-.080954592m, K4/K8-.111383570m: all overlap; gain
.062295392/.031866415m, nonmonotonic, same first unsafe segment2/row3. Max lateral
<=.006376111m. Improvement is shorter near-straight extent, not bypass. K0 was
safe OFF(.935417382m) and blocked under hypotheticalON(-.2m). Both shams exact;
K4/K8 action tokens/arrays identical. AllON APOS bottom-clamped; [235,265] for
K1/K2/K4, [245,265] forK8. Raster hits floor, but nominal-ground footprint proxies
remain unsafe; censored points establish no clear-side or direction mismatch.
Target visible=false/OPOS not_visible persists despite rendered shelf pixels.

Result PERSISTENCE_IMPROVES_BUT_REMAINS_UNSAFE; APOS
AFFORDANCE_REMAINS_AMBIGUOUS_DUE_TO_CLAMPING. Persistence contributes but does
not resolve failure; target grounding vs spatial/action realization remains
unisolated. No promising safe bypass or moving online source. Stop after diagnosis.

TerminalRTT sum2.326373s; worker sum32.142204s; corrected server startup16.761073s,
lifetime106.828761s; renderer187.221508s. Two separate synthetic startup warmups
(287/277ms), including failed run. Full-suite historical MPC1 solve(.043629s)
reproduces saved command/reference exactly and is not SOURCE04 evidence.

Full tests2564 passed/19 skipped(169.38s), focused147 before freeze, corrected58.
Saved validator4686 checks valid; review validator295 checks valid, including
prior failure hashes, actual calls, CSV, ZIP bytes. Frozen analysis preserved;
report-only finalizer adds full-range APOS proxy geometry and compact index.
All12 static figures visually inspected, raw input images never annotated.
Output050237/index_final.html and review_bundle_final.zip. Source/core/environment
and unrelated Stage0 configs preserved; no external source/dependency change.

Final SOURCE04 saved-only recheck validation_final.json passes after the full
suite; final focused regressions148 passed(2.26s), compileall/diff pass. Both
unrelated Stage0 config bytes match original preserved hashes. Final review ZIP
9,416,422bytes,12 PNGs with numeric/source sidecars; only code/docs are committed.

### SOURCE03 curved FRESH saved GUI, 2026-09-22

On user request, added a small saved-only Isaac display for distance_near_H16.
It reconstructs the exact saved Hospital/cart/BRIGHT scene, shows original ON
RGB and raw world paths, and provides previous/next prediction-row footprint
inspection. RedON bends right to local lateral-.552365m/yaw-.890956rad, but saved
clearance remains unsafe. No movingB, new inference, MPC/GP, physics or rollout.
WorldXY unchanged; annotations raised toz1.35m for visibility only, explicitly
recorded. Actual renderer screenshot visually checked with source-hash sidecar
under data/robotless_join_source_03_gui/view_20260922T052500Z. The GUI exited
normally after display/capture; screenshot and relaunch command preserved.
Six focused loader tests cover arbitraryN, observation anchoring, read-only data,
invalid-source rejection and no solver calls. SOURCE03/presentation regressions
included:29 passed. Historical raw/results and unrelated Stage0 changes untouched.

Expanded saved-viewer regression check: 42 passed, including comparison-replay
tests; compileall and git diff --check passed. These checks made no model/MPC calls.

### Genuine source-only moving/mismatch inventory, 2026-09-22

User requested existing-corpus evidence only. Fetched main/starting HEAD
3a2734ba10ec07e7fc1b3d9cd9056ff8cc838f4f; unrelated Stage0 edits preserved.
Protocol fixes physical v_minus>.20m/s, obs-to-B travel>=.02m and lateral>=.10m
or reliable direction/pose-yaw>=20deg. Full raw FRESH/common/B and recorded
pre-B geometry use original .20m footprint/.05m clearance/direct Hospital
checker. Endpoint distance is separated into normal/along-track components.

881 records ->680 base ->673 whole-FRESH safe ->670 past-safe ->548 moving
->13 mismatched candidates,7 episodes/13 ordered raw pairs/10 distinct FRESH.
11 interior projections;8 lateral AND orientation (7 interior). Two endpoint
cases have nearly exhausted raw progress. All13 have changed raw OLD/FRESH.
Zero meet the previous retained-suffix obstacle-near<=.20m criterion. Full raw
paths of008/01/023 and014/01/025 are near obstacles (.1313/.1306m), but their
retained suffixes are .2335/.4341m. These scopes are reported separately.
No automatic next-experiment case choice or correspondence/plan was produced.

Final output data/genuine_source_moving_mismatch/scan_20260922T073107Z
contains complete ledger CSV/JSON, source/config hashes, static index and seven
PNG/numeric sidecars. Earlier saved-input passes retained; adding B clearance
and moving legends outside axes did not change predicates or candidate IDs.
All calculations use saved source; new VLA/GP/rigid/MPC/rollout/GUI calls zero.

Final scan wall time 14.700s; saved-only validator recomputes all881
records and verifies 3920 source hashes, CSV and plot numbers.
Relevant tests134 passed(4.26s), no real model/controller calls; compileall and
diff checks passed. Raw/core/environment/previous results remain unchanged.

### Genuine source inventory Isaac GUI, 2026-09-22

User requested GUI presentation of the source-only scan. Added a saved-only
viewer with all13 candidate cases, original observation JPEG, raw OLD/FRESH,
optional unchanged common reference and recorded observation-to-B playback.
Default display008/01/023 is a visual example, not a new experiment selection.
The GUI shows physical speed, travel, mismatches, full/suffix/B clearance,
projection location and raw arc remaining; endpoint caveats are preserved.

Actual Isaac screenshot visually inspected at
`data/genuine_source_moving_mismatch_gui/view_20260922T081500Z/source_inventory_gui.png`.
WorldXY/yaw unchanged; explicitly declared display-only neutral fill and raised
annotations improve visibility without altering the original input JPEG or USD.
13 cases/104 display files authenticated, plus original scan input/core hashes.
No new VLA/GP/MPC/rollout or physics/integration; replay selects saved states only.
GUI process verified running after capture; left available for user interaction.
Focused26 tests passed, compileall/diff checked. Prior results and unrelated
Stage0 edits preserved. Only viewer/tests/docs are tracked.

### Recorded FRESH post-switch GUI clarification, 2026-09-22

Fetched main and started at f716522a697e06ec3924434d40f0b806d6295929.
User asked to see rotation/chunk-transition mismatch and whether FRESH was ever
executed. Added opt-in post-switch playback to the existing inventory GUI.
The genuine corpus already contains actual FRESH execution; the preceding
inventory display stopped at B and obscured that distinction. SOURCE03/04
remain prediction-only counterfactual diagnostics, not online traversal.

Authenticated all13 original FRESH command lifetimes (1,011 historical applied
steps) and111 display source files. Next-chunk commands are excluded at each
endpoint. Original B/timing/command-change metrics reproduce to1e-12. Actual
Isaac screenshots of013/01/024,008/01/023 and001/01/013 were visually inspected;
their remaining same-FRESH distances are .182382/.115583/.077264m at the next
activation. These are descriptive tracking errors, not sustained-attachment or
full safety/failure classifications. No new source event or outcome was chosen
for optimization. No model/GP/MPC/rollout calls or state integration were made.

Evidence: data/genuine_source_moving_mismatch_gui/post_switch_20260922T085623Z/
contains three PNGs and numeric/source sidecars, all13-row execution summary,
saved-only validation, scene/runtime provenance. GUI dropdown and replay
controls expose the original OLD-to-B and subsequent FRESH execution. Added
tests for next-reference exclusion, exact B, missing/mixed lineage and saved
sample playback. Relevant source/timing/replay tests91 passed; compileall and
diff check passed. All raw/core/environment/external sources and unrelated
Stage0 edits preserved. Clarified native no-reconciliation, adapter preparation
and rigid comparison roles in the source report; no new comparison implemented.

### Saved native handoff execution loss audit, 2026-09-22

User asked what actual loss the proposed optimization should reduce. Started
and fetched main at cb45aa91f1e81c141257671b4f265362d9ecc04a. Audited all13
previously screened sources without new selection or model/GP/rigid/MPC/rollout
calls. Protocol preserves original .10m/15deg/.30s sampled sustained-join
semantics and forward-only original-FRESH projection. A common54 saved steps
(nominal.9s) permits equal-exposure error-area and command-TV descriptions;
full reference lifetimes and endpoint caveats remain separate.

One sustained join observed; seven never enter, four enter too late for full
dwell before next switch, one enters then exits. Interior-only11 counts are
1/5/4/1 respectively. No N/A is changed to3s or full navigation failure.
Hard013/01/024 distance grows .279113->.376956m at.500000s; common-window
mean.343891m and AUC.309502m s quantify post-boundary cost. All13 post-switch
paths pass direct swept clearance with the original held-arc bound. One
nominal10Hz dv flag in008/01/023 is accompanied by source evidence of .2s
application spacing and an intermediate computed-but-unapplied command; it
is not attributed to optimizer or physical dynamics failure.

Generic GP objective and ATTACH masked objective were read, not changed.
They constrain/score plans, not actual MPC attachment cost. Report distinguishes
fixed B offset, subsequent tracking loss, secondary command costs, hard safety
requirements, and unproven task/navigation benefit. No new formulation added.

Final saved-only output data/saved_handoff_execution_loss/audit_20260922T091708Z/
contains all13 records/CSV, summary, source/protocol/core hashes, three PNGs
with numeric sidecars, index and validation. Initial091255 pass retained; final
adds censoring and timing explanation without changing any previous CSV value.
Audit/plot/recomputation2.155s. Saved-only validator independently reproduces
all metrics/geometry/commands, source hashes, CSV and plotted numbers.
Relevant tests138 passed; compileall/diff checks passed; no real model/MPC test
calls. Source/core/history and unrelated Stage0 config edits preserved.

## 2026-09-22 — PROJECT-AUDIT-01 native LightNav and reconciliation

User requested a SAVED-ONLY + SOURCE-ONLY audit of unsafe native obstacle
FRESH generation and whether successive-chunk reconciliation has a defensible
research contribution. Started on main at freshly fetched origin/main
6b6cb01d8d44b890dbd60845e139f86de74874ac. The unrelated two Stage0 config edits
remain unchanged and excluded from staging. No new scientific model, MPC,
optimizer, simulator or rollout call; no controller/model/formulation change.

Native is explicitly scoped to official inference/RVQ/MPC inside the custom
robotless Isaac scheduler/camera/SE2 integration harness, not the entire official
robot deployment. Direct external checkout verification is clean at
c6f40e3220edbf7011e4f17eaf2c865416737d4d. Sixteen checkpoint files match all five
historical launch manifests, revision 7221d418bfff55cfcbadd09f7a26aaab81e1f8a6.
Official MPC SHA256 remains
2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1.
Scoped hashes/diffs span original eed5f2c, JOIN01, SOURCE02, SOURCE03/04 and
current HEAD; online model/MPC workers and adapter remain unchanged. Original
inherited generation environment is incompletely archived, a stated limitation.

Authoritative output:
data/project_audit_native_lightnav/audit_20260922T_project01_v3/.
42 token/response/raw/observation-anchored-world traces reproduce. JOIN01's three
raw/suffix clearances are -.2m while B edge clearances are .687/.653/.463m;
obs-to-B travel .293/.267/.257m cannot cause an already unsafe observation-anchored
model output. No dynamic detour impossibility is inferred. SOURCE02–04 geometry
and paired/persistence input checks reproduce; brightness/persistence change
severity without a bypass. Internal neural cause remains unresolved. Raw
generation, later attachment, reference preparation and MPC safety are separated.

Unchanged source scan reproduces all881 rows; original13 metric/geometry results
reproduce literally. New frozen outcome-independent feature matching yields8
pairs from4 hard episodes and6 controls,5 unmatched, without caliper relaxation.
Paired first54-interval position-AUC difference is +.152209m s (also episode-equal).
This is retrospective association, not causal/IID or held-out inference; source
selection conditions on mismatch, route intent is unobserved and realized lifetime
can be post-treatment. Prior published outcomes and control reuse are disclosed.

Safe entire raw FRESH:765 events/60 episodes,637 inside tube at B,740 sampled
sustained attachments. Original13:1 attachment,7 no entry,4 right-censored dwell,
1 transient; all13 saved executions safe. Broad safe-raw post-execution clearance
passes758/765, with7 exceptions retained. Most reported tiny initial growth is
not practically large (median .0000216m). Full7-episode sequences cover189 handoffs;
hard013/01/024 jumps OLD/new error .022870→.279113m at identical B, then the next
FRESH starts .000352m away. Among12 available next handoffs,10 start in the tube
and11 attach. No endpoint completion test is extended past actual replacement.

Audited Native/Adapter/Rigid/GP/REF01–04/DIAG01–08/M4 evidence and primary RTC,
TIC-VLA, AsyncVLA, ACT, Apollo stitching, reference-governor and GP/Lie-group
literature. Current optimization has no demonstrated moving-hard attachment
benefit; M4's selected nearly stationary example reduces linear TV but attaches
later. Existing selector/preparation explanations and absent simple-splice
comparison prevent a necessity claim. Primary verdict PROMISING_BUT_UNPROVEN.
Exactly one next method-comparison experiment is proposed, not run.

Deliverables: comprehensive report
docs/PROJECT_AUDIT_NATIVE_LIGHTNAV_AND_RECONCILIATION.md; frozen matching YAML;
saved-only core/report/validator scripts; tested audit metrics; six CSV matrices,
event/episode metrics, integrity report, HTML index and14 scientific figures with
numeric/source-hash sidecars. Geometry uses equal XY axes. Figures explicitly
separate host and simulator clocks and original-FRESH attachment from optimized
path agreement. Historical docs and all raw/derived experiments are preserved.
Two partial audit attempts remain: acquisition-only worker_exit_code comparison
and duplicate end_reason serialization were corrected without scientific calls
or outcome/threshold changes. The v3 end reasons agree before merging.

Validation: core before/after12,073 input hashes; complete audit12,425 checks,
zero failures,14 figures/6 tables. Independent original corpus validator passes
60 episodes/881 valid handoffs. SOURCE02/03/04 validators pass1,507/2,019/4,686
checks. Focused final audit/SE2/timing/loss tests70 passed. Full regression before
three final audit tests:2,619 passed,19 existing fixture/diagnostic skips,1
deliberately deselected real-official-MPC test,2 AF_UNIX permission failures.
Those exact2 IPC tests pass outside sandbox restriction (2 passed). With the
three additional passing audit tests, all2,624 exercised tests pass;2,644 total
tests collected. Synthetic solver fixtures are software validation only.
Compileall and diff whitespace checks pass. Exact logs are retained in the run's
metrics/test_logs directory. No numerical/controller code was altered.

Core README/WORK_LOG hashes describe the initial 6b6cb01 source snapshot; this
required append is the sole later change to an already-manifested documentary
input. Prefix preservation and final source/test hashes are recorded separately
in completion evidence. New report/validator files have their own hashes.
One focused code/config/test/doc commit and normal push are recorded after Git
completion in the ignored run's git_completion.json; no force push/history rewrite.

## 2026-09-22 — JOIN-SOURCE-05 pre-prediction instruction freeze

Started/fetched main dd138d72adb80a05019d40b2f8f57d651de23d54. New bounded
instruction-only counterfactual diagnostic reuses SOURCE04's immutable all-BRIGHT
32-frame bank and all historical I0 outputs. Complete SOURCE04 saved validator
passes; all JPEG/mask/depth hashes and original K premodel gates pass. No new
render or scientific model call before this freeze. Two unrelated Stage0 edits
are hash-preserved and excluded from staging.

Exact I1/I2 and twelve ordered independent sessions (K0/1/1-sham/2/4/8 each)
are frozen. First15 requests remain empty-instruction; comparison against
historical wire payloads permits only final instruction to differ. Existing
SOURCE04 worker, geometry/APOS helpers and official model files are unchanged.
New code records actual official-server process generation environment/argv and
checkpoint parity before predictions. I0 is not rerun. Shams, K0 text control,
whole raw swept clearance, partial versus complete bypass and goal-semantics
limitations are explicit. No B, MPC, GP, rigid, traversal or next-stage work.

Implementation protocol: docs/JOIN_SOURCE_05_INSTRUCTION_AVOIDANCE.md.
Prepared run: data/robotless_join_source_05/instruction_20260922T103000Z/;
run directory suffix is an identifier; actual host event clocks are recorded
separately. Focused SOURCE02–05 geometry/history/protocol tests168 pass(2.26s),
with no real model/MPC calls. Compileall and diff whitespace checks pass.
This commit freezes infrastructure, not a positive scientific result.

### SOURCE05 pre-image technical correction

Pushed freeze230e393 opened12 sessions in instruction_20260922T103000Z, but
all failed before the first image request: the live history guard compared
historical SOURCE04 capture monotonic values with a different current host
clock epoch. Scientific predictions0, buffer requests0; one official synthetic
server warmup. Every attempt and error is retained and hashed in the new run.
The server was identity-checked and stopped normally.

A SOURCE05-only saved-history adapter authenticates exact frozen frames and
preserves the original sequence/sampler/state machine, replacing only the
invalid cross-boot age comparison. Source capture values remain unchanged;
real request/receipt clocks and RTT remain current; no clock rebasing or fake
capture timestamp. Both domains and N/A capture-to-request age are recorded.
Official model/source, existing online worker, RGB, instructions, K, settings
and acceptance are unchanged. External worker import preflight passes.
New exclusive run instruction_20260922T125204Z; all12 original conditions remain
scheduled with zero prior scientific responses. Relevant171 tests pass, including
same-boot parity and cross-boot preservation. No result-driven retry or tuning.

## 2026-09-22 — JOIN-SOURCE-05 completed: safe shortening, no lateral bypass

Pushed execution freeze51d6d39d33600bfa24fbdc78178b5de55ac885d8 ran exactly12
terminal scientific predictions /180 buffer restores in independent official
sessions. Historical I0 was not rerun. SOURCE04's32 JPEGs and all masks/depth
match their frozen hashes; all192 next payloads match historical inputs after
removing only instruction. Actual server argv/environment confirm VLN,
temperature0/top_p1/top_k0/traj_top1=0 and the pinned source/checkpoint. Current
request clocks remain separate from unmodified historical capture clocks.
SOURCE04's full inherited process environment is unavailable historically.

K0 trajectories are literally identical across I0/I1/I2; both K1 shams match
raw arrays/text exactly. Six of eight non-sham cart-ON conditions pass original
raw-path clearance, including I2_K8 STOP. The five safe non-STOP conditions are
short forward chunks ending before the cart; all new ON lateral magnitudes
are below1.1mm. I1_K1/K2 remain unsafe; I1_K4/K8 and I2_K1/K2/K4 are safe-short.
No lateral detour or complete bypass was generated. Frozen safe-non-detour
classification: INCONCLUSIVE, with both INSTRUCTION_EFFECT_INCONCLUSIVE.
This does not mean measurements are missing or that safety never improved.
No bypass-qualified source for subsequent online confirmation was obtained.
No inference-driven threshold/input change or additional prediction followed.

Counterfactual-only run:
data/robotless_join_source_05/instruction_20260922T125204Z/.
All earlier pre-image failures remain in instruction_20260922T103000Z,
with zero next/prediction requests; no scientific-output retry. Total task
sessions24, scientific predictions12, buffer requests180, separately counted
synthetic server warmups2. Scientific terminal RTT sum3.961543s; schedule start
to last session close62.158958s. Nested timing fields must not be summed.
MPC/GP/rigid/reconciliation/render/rollout calls0. Tests/validators model calls0.
Both official server processes were stopped; external source/checkpoint unchanged.

Saved-only validator passes219 checks; SOURCE04 revalidation passes4686.
Presentation completion validates13 PNGs with numeric/source sidecars and final
ZIP members. K0 actual OFF safety is clearly separated from hypothetical ON
clearance. Final HTML/ZIP: index_final.html / review_bundle_final.zip. All figures
were visually inspected; raw RGB/checkpoint/environment are excluded from ZIP.
Final focused SOURCE02–05/history/SE2 tests193 pass(2.57s); compileall and diff
whitespace checks pass. No actual model/controller calls in tests. Detailed
commands/results and timing provenance are in the report and ignored run.

README records the bounded result; the new report includes every historical/new
condition, APOS/action tokens, source/call identity, safe-short versus bypass
interpretation and remaining next-chunk/internal-mechanism uncertainty. No
online confirmation or optimization was added. Two unrelated Stage0 edits
remain hash-preserved and excluded from staging. Final Git completion is saved
in the ignored run after focused result commit and normal push.

## 2026-09-23 — JOIN-ONLINE-02 pre-primary native approach protocol

Fetched origin/main `a02b79750e22cd64b68351992fb82a2ce852141a`; preserved two unrelated
stage0 config changes. Reused SOURCE04/05 actual cart/BRIGHT/Hospital/camera and
pinned official MPC. Geometry-only [5,4.5,4] m candidates all pass; frozen R0 is
5 m from cart. Actual Isaac OFF/ON preflight passes (0/372 cart pixels), exact
mesh/static layers/camera/state and official controller provenance; zero new
LightNav predictions/MPC solves. Added optional abort-only command hook to the
existing collector, saved-only analysis/validator/static report and Isaac replay.
Four independent OFF/ON repeats, native scheduler, classification and source
selection are frozen before scientific execution. No GP/reconciliation.
See `docs/JOIN_ONLINE_02_FAR_APPROACH.md` for exact protocol and commands.

## 2026-09-23 — JOIN-ONLINE-02 actual successive chunks and saved replay

After normal push of freeze `f3170c76042e9a2fd62dfa515f74723a7fbeaafd`, executed
OFF00→ON00→OFF01→ON01 once. Actual LightNav responses20/7/21/7; applied chunks
19/6/20/6. All episodes ended by MODEL_STOP, no oracle abort. ON raw arrays match
bitwise across repeats: near-straight1.356m chunks →1.056m →.511m →STOP, with no
bypass onset/full bypass. Actual ON minimum edge clearance .355180979/.354753227m.
The frozen spatial SAFE_SHORTEN label does not cover the .510854m C5; this is
explicitly reported without threshold tuning. OFF references would intersect
hypothetical cart ON, but actual OFF execution passes cart-OFF geometry.

55 terminal predictions,183 buffer requests and one separate server warmup.
557 official MPC submissions;556 unique saved result records,2.342283712s saved
solve time (one final unapplied completion timing unavailable). Four-episode
acquisition including Isaac startup83.422505s. No GP/rigid/reconciliation
experimental solve. All14 ON request-local RTF checks are1.34–1.39, above prior
1.2 gate; ON loop maxima.2548/.2574s also exceed.25s. Thus no timing-qualified
bypass source is claimed. No moving bypass candidate exists.

Saved-only validator passes raw/wire/history/anchor/selection/memory/applied
command/exact states, direct guard, visibility and plots. Reporting-only bool
serialization, call-accounting and figure/GUI layout fixes retain frozen code,
failed-development logs and derived versions; no scientific retry. Final
relevant test set276 passed, compileall/diff checks pass. An overbroad full-suite
run was interrupted in unrelated diagnostic fixtures and is not claimed complete.

Static index/review ZIP in `data/robotless_join_online_02/primary_20260923T001500Z/`;
actual saved-only Isaac GUI verified and left open at ON00 C5, evidence in
`data/robotless_join_online_02/replay_20260923_final/`. Four-episode dropdown,
chunk slider/previous/next, play/pause, untouched RGB, OLD/FRESH/B and numeric
fields are present. Outcome **PROGRESSIVE_SHORTENING_OR_STOP**, with pacing and
last-unapplied-solve record limitations; no next experiment implemented.

## 2026-09-23 — Saved JOIN-ONLINE-02 STOP/target-point cause audit

Fetched origin/main and HEAD both `ababf8518f5365a30b42bd1596cc3e8de66ddc25`.
Preserved the two unrelated Stage0 changes. Added a saved-only auditor/tests
and report, no changes to collector/model/MPC or historical artifacts.
Rechecked all55 responses,14 ON,4 STOP, raw/wire/anchor/mask hashes and source.
All STOP responses carry explicit RVQ tuple[6,122,174]; APOS also indicates STOP.
Official demo and research collector terminate on this signal. Actual STOP
wire chunk contains10 zero rows, correcting an earlier report's single-row
description without changing raw data.

New observation: both ON C6 OPOS cells are100% cart pixels, centre(235,155);
none of12 earlier ON OPOS centres hits cart. Pre-STOP APOS samples hit floor,
approach image bottom, then become STOP. This supports a target-grounding
hypothesis but cannot distinguish target confusion from occlusion/blocked-route
stopping. OFF final OPOS hits a wet-floor sign/floor, so actual hallway-end
completion is also unverified. Original timing limitations remain.

Output `data/robotless_join_online_02_stop_audit/audit_20260923_final/`;
two visually checked figures, numeric/hash sidecars, HTML and55-record ledger.
Initial layout-only output preserved separately. New and original saved-only
validators PASS,95 relevant tests PASS,compileall/diff checks PASS. All new
model/MPC/GP/render/rollout calls0. A destination-grounding contrast is proposed
only; no future experiment or avoidance/reconciliation policy implemented.

## 2026-09-23 — JOIN-ONLINE-03 destination-first instruction freeze

User requested an actual rerun with exactly: "Go to the far end of the hallway.
Pass around the supply cart without touching it, and stop only when you reach
the end of the hallway." Fetched HEAD/origin main68ebd7e; two unrelated Stage0
edits preserved. New config/wrapper reuse ONLINE02's exact resolved settings,
saved5m start/scenario/side geometry, original collector/checkers and official
LightNav/MPC. Only instruction and experiment label differ. Four fresh online
OFF00/ON00/OFF01/ON01 episodes, same budgets/guard/STOP rules, no retry or tuning.
New live histories may diverge, so not an identical-input terminal contrast.
Pre-primary tests100 pass. See JOIN_ONLINE_03_DESTINATION_INSTRUCTION.md and
new run primary_20260923T031300Z; historical outputs remain immutable.

## 2026-09-23 — JOIN-ONLINE-03 completed instruction rerun

Executed the four frozen episodes once after pushed freeze1bd1be8. The requested
destination-first wording produces partial lateral response in both ON repeats,
but no complete bypass. Unlike historical native STOP, both now end at the
unchanged oracle pre-command guard. Actual minimum edge clearances .0532953 /
.0537319 m remain valid; proposed next-step lower bounds .0424345 / .0462442 m
would violate .05 m and those commands are unapplied. ON C5/C6 raw paths are
unsafe; positive-but-insufficient clearance is distinguished from overlap.
OFF continues past the absent cart region, then emits STOP; hallway-end arrival
has no independent oracle. No GP/rigid/reconciliation or controller changes.

All43 terminal predictions,136 buffer requests,1 technical model warmup and
409 accepted MPC submissions preserved;408 saved completions,1 final unapplied
completion unavailable. Saved MPC time1.679291s is a lower bound; terminal RTT
sum15.402936s, collection68.147044s including Isaac startup. All14 ON local RTFs
exceed1.2 and ON maximum stalls exceed.25s; no timing-qualified bypass source.
Existing source_bundle retains early safe moving onset only, not full bypass.

Saved-only validator passes all4streams, exact integration, guard reconstruction,
raw/wire/source hashes and numeric figure parity. 293 relevant tests pass:
initial sandbox2IPCpermission failures resolved by same9testmodule with local
socketpermission, no model calls. Compileall/diff check pass. Static figures,
index.html/instruction_review_bundle.zip preserved under the new run, visually
checked. New report distinguishes changed behavior from successful avoidance
and different live histories from identical-input inference. Both unrelated
Stage0 configuration edits preserved. No follow-up experiment implemented.

## 2026-09-23 — JOIN-ONLINE-03 saved turn-back visual/history audit

Fetched origin/main and starting HEAD both
`0dc3e80677e4cf13d5bf16068e8bb2acc6173433`. Preserved both unrelated Stage0
config edits and all historical artifacts. Audited ON_REPEAT_00/01 C0–C6:
14 chunks, 56 unique captures, 224 reconstructed sampled-image slots across
requests. Exact wire JPEGs, source/checkpoint/sampler, observation transforms,
camera/mask/cart render-state provenance, pointing/raw arrays and original
clearance/classifications agree. No new model/MPC/GP/rigid calls, rollouts,
scientific renders or reconciliation.

C4→C5 current cart pixels grow 2375→8576 and 2668→8162. Every sampled frame
remains cart-visible: 20/20→24/24→28/28 for C4/C5/C6; latest preceding visible
frame remains 0.25 s old. Preceding-only pixel sums also increase. C6 cart is
partially clipped but occupies about 20–22% of current RGB. Official sampler
IDs are source-reconstructed from wire-confirmed history, not internal server
telemetry; original pixel area is not model attention or post-pooling salience.

C5 contains inward internal path segments in both repetitions; endpoint shifts
toward hallway/cart centre by 0.116083/0.109080 m, final yaw changes +33.410/+17.217
degrees, and raw minimum clearance falls 0.494527/0.452860 m. Repeat01 C5 retains
positive 0.005718 m clearance but violates the required 0.05 m; other late failures
include raw overlap. C5/C6 APOS/OPOS centres hit floor outside cart mask/bbox,
not cart. Actual original guard prevents unsafe execution; no collision is
newly executed or reclassified.

Interpretation: **CART_EVIDENCE_REMAINS_STRONG_DURING_TURNBACK**. Simple loss of
available visual evidence is not a sufficient explanation. No memory-loss,
grounding or decoder cause is established. Route-side persistence/action
generation remains unisolated; no next-stage implementation.

Authoritative output:
`data/robotless_join_online_03_turnback_audit/audit_20260923T044600Z_r02/`.
Seven reviewed figures with numeric/source sidecars, full ledgers, static index
and review ZIP. First derived directory remains preserved with a development
validator failure: optional absent OPOS CSV fields were blank. Explicit N/A
serialization and a focused regression fixture fix the representation; all
measurements are unchanged. No scientific retry.

New saved-only validator and original authoritative JOIN-ONLINE-03 validator
PASS; all 1482 primary files and unrelated edits retain their hashes. Analysis
8.792487 s, validation 20.168549 s. 310 relevant tests PASS (17 new synthetic checker
tests; IPC uses local socket fixtures, no real model/MPC), compileall and diff
checks PASS. Report: `docs/JOIN_ONLINE_03_TURNBACK_VISIBILITY_AUDIT.md`.

### 2026-09-23 — HANDOFF-DELAY-ATTRIBUTION-01 pre-execution freeze

Fetched main718a726; preserve two unrelated Stage0 config edits. Added a bounded
saved-source 2×2 diagnostic for the frozen13 plus ONLINE03 ON C1–C4 (eight).
All21 source state packages reconstructed; all84 conditions available. Reference
interface is prepared-reference plus frozen REF02 selector, not selector-only.
Actual B-time controller memory differs from legacy pre-FRESH input memory in all
21 sources; preserve this explicitly and retain first-step motion failures.
Primary54 intervals/.90000004694s, maximum756 MPC solves, VLA/GP/Isaac0. Source-only
preparation corrected the authoritative saved-loss path from retained development
091255 to final091708 before scientific execution; old preparation retained.
Added exclusive outputs, immutable source/code hashes, no-retry loop, safety abort,
saved-only validation and static figures. Scientific execution awaits freeze push.

### 2026-09-23 — HANDOFF delay diagnostic IPC failure before integration

Freeze c823be8 was pushed. Full suite2718 passed/19 skipped. The first scientific
solve request encountered native CasADi/IPOPT stdout in the new JSON pipe; then
close parsing also failed. Retained run054600 has one issued MPC call, zero saved
solve responses and zero integration steps; no result was available for tuning.
This is a technical failure, not a controller failure or scientific outcome.
Applied the existing online worker's native-stdout/stderr separation to the new
bridge, plus durable request/response journal and close-error preservation.
No source/core/threshold/initial-state semantics changed. Added a subprocess
native-print regression test without CasADi. A separate pushed correction and new
unique run will execute the same full frozen schedule; the failed attempt remains.

### 2026-09-23 — HANDOFF-DELAY-ATTRIBUTION-01 completed with limitations

Corrected/pushed freeze b9d44afd65676d97c890eea99e224af4db60c322; authoritative
run `data/handoff_delay_attribution_01/primary_20260923T055300Z/`. All21 source
handoffs and84 conditions available, all84×54 intervals completed with756 saved
MPC solves. VLA/GP/rigid/graph/RGB/Isaac0. Zero abort, controller failure,
overlap or unknown workspace; minimum conservative clearance.119760m genuine13,
.493842m ONLINE03. Original source, preparation/selector/MPC/evaluation cores and
unrelated Stage0 edits preserve their hashes.

Genuine13 native/prepared+SP mean position delay gaps.130656/.124985m·s
(episode-equal.136697/.127858); positive13/13 and12/13. Sampled attachments
DN/DL/FN/FL=1/1/13/11. ONLINE03 gaps−.010531/−.014327m·s; all8 attach in every
condition. Reference comparison includes preparation and cannot be selector-only.
All26 genuine delayed trajectories violate original physical command-grid
acceleration at interval0 because actual post-poll B memory already contains the
historical first FRESH result; a new synchronous solve advances from that memory.
No later violation. ONLINE03 delayed motion failures1native/3prepared+SP; none
in Oracle. These failures are retained, not reclassified or repaired.

Interpretation MIXED_ATTRIBUTION: genuine state-package delay sensitivity remains,
ONLINE03 is already easy, and memory/application-phase effects prevent a clean
position-delay attribution or motion-valid performance claim. No recoverable
maximum, graph necessity/benefit, obstacle bypass, population or deployment claim.
Report provides all21 event tables, episode-equal/cohort-separated results, Path1/
Path2 decision support,27 PNGs/numeric sidecars, static index, complete review ZIP
and small Git-readable84-row numeric summary. Main loop7.883226s, summed solver
wall2.571845s. Primary saved-only validator PASS7.526908s; final reporting validator
PASS, source authoritative validators PASS. Final suite2719 passed/19 skipped,
compileall and diff checks PASS. Two full suites each ran the existing one-solve
historical reproduction test: total MPC requests756primary+1failed IPC+2tests=759.
This corrects the early focused-preflight zero-real-test-MPC annotation; scientific
VLA remains0. Failed first IPC run and prior source-only preparation remain intact.
No next-stage implementation. Largest uncertainty: reducible same-B attachment
cost under a motion-admissible controller-memory/application boundary.

### 2026-09-23 — OBSTACLE-SOURCE-ACQUISITION-01 pre-prediction protocol

Fetched main1cfffed; preserve two unrelated Stage0 edits and every historical
source. New source-only experiment: exact saved BRIGHT frame pairs at four
geometry-selected SOURCE04 poses, H8, terminal-only cart reveal, one explicit
RIGHT instruction selected from existing direct mesh passage probes. Eight
independent terminal predictions; all four pairs run once, first qualified in
frozen order selected. No optimizer/GP/rigid or method comparison. Separate
short real OFF technical pacing episode (up to4 predictions, official MPC),
unchanged4/10/60Hz collector; [.8,1.2] request-local RTF/.25s stall gate blocks
conditional online acquisition. Two genuine sudden-reveal repeats only after
both gates and a second pushed selected-source freeze. Preparation073500 failed
on YAML OFF/ON boolean enum before any call; corrected quoting/test and preserved
it, using new exclusive073600. Actual inputs and source validators pass; no new
scientific prediction at this protocol-writing stage.

### 2026-09-23 — OBSTACLE-SOURCE-ACQUISITION-01 bounded completion

Execution freeze9190c70 pushed before any model request. Run073600 completed
one separate actual OFF technical pacing episode and all8 independent PhaseA
OFF/ON predictions. Four source geometries/cart relative distances1.983/1.783/
1.583/1.383m fixed by existing BRIGHT moving-pose bank, H8, terminal-only cart
presence difference. Exact RIGHT instruction frozen from direct passage probes.
No source/controller/threshold edits or retries after outputs.

PhaseA2/4 qualify, firstPOSE11 selected. OFF hypothetical-cart edge−.125215m,
ON+.205296m, final lateral+.584514m and arc1.348502m. POSE12 also qualifies,
edge+.100168m/lateral+.644361m. POSE10 OFF remains safe even with cart; POSE13
shortens to.100066m and fails change/progress/future gates. AllON are raw-safe,
nonSTOP; no complete bypass. Positive local y is LEFT despite exact cart-on-your-
RIGHT wording; preserve that distinction, no side-compliance or intent claim.

Technical post-bootstrap requestRTF1.378365/1.393438/1.387402 fails[.8,1.2];
maxstall.216156s passes. No pacing patch and no PhaseB executed. Final
PACING_BLOCKED_ONLINE_ACQUISITION, no genuine moving-B representative source.
Paired candidate bundle is explicitly NOT online/optimization-ready, B/memory
null; technical OFF B is never borrowed. Both planned online repeats remain N/A.

Scientific8terminal+56buffers; technical4terminal+13buffers; one serverwarmup.
Technical31MPCaccepted/30saved; finalsolve000031 no saved completion or applied
command. SavedMPCwall.154098410s lowerbound; PhaseAloop36.962635267s and terminal
RTTsum2.159152857s; technicalcollector14.618949200s excluding SimulationApp startup.
ScientificMPC/GP/rigid/reconciliation0. Source and saved-record validators PASS,
CSV/JSON/PNG/ZIP parity PASS, final480 relevant tests PASS6.74s with zero real
model/MPC/optimizer test calls; compileall/diff PASS. Four pair figures inspected.
Raw/derived remain ignored; tracked summary/report provide all candidates and
pacing limits. Largest uncertainty: genuine sudden-reveal reproduction with a
timing-qualified moving B. No next-stage work.

### 2026-09-23 — OSA02 saved pacing diagnosis and pre-qualification correction

Fetched main1b8f94e; preserved unrelated Stage0 edits and OSA01/earlier raw results.
New exclusive run `data/obstacle_source_acquisition_02/primary_20260923T090000Z/`.
OSA01 authoritative validator passes before collector change. Saved C1–C3 show
~74–75ms deadline debt after ~83ms render and 2–3ms fast outer iterations inside
request windows, explaining localRTF1.378–1.393 despite wholeRTF.99925. Each loop
integrates one step; historical sleep itself was not measured. Old code copied
and hashed; no previous result overwritten.

Opt-in minimum-wall-step scheduler rebases after actual completed step, never
repays blocking debt by burst integration. Absolute remains default. Fixed
simulationdt60Hz/capture15ticks/control6ticks, official MPC/history/pose/activation
semantics unchanged. Buffered diagnostics record deadline/sleep/capture/IPC/guard
and actual step coverage. Predeclared one OFF technical episode; all3 postbootstrap
RTFs must pass[.8,1.2], stall<=.25, exactstates/cadence/guard/no-burst. Failure stops;
no performance retry. POSE11/instruction/cart remain immutable, PhaseA0. Conditional
2 sudden-reveal repetitions only after pass and another pushed selected freeze.
No model/MPC/optimizer calls during this implementation/saved diagnosis stage.

### 2026-09-23 — OSA02 technical PASS and conditional PhaseB freeze

Pushed technical e539a03; single qualified OFF episode completed. RequestRTFs
.990677/.990953/.989740, stall.210615s, zero burst steps, minimumstep.0167743s,
266states/265 exact integrations (error0),18 captures every15ticks, MPC every6.
WholeRTF.721769 disclosed: no catch-up means blocking render reduces wall throughput;
nominal simulation cadence and clocks unchanged, no RTT metric manipulation.
4technical predictions+14buffers,31MPCaccepted/31saved, one serverwarmup; no retry.
Original frozen PhaseA not rerun. All source hashes preserved.

Prepared exactly REPEAT_00/01 at fixed POSE11 minus.40m forward, unchanged yaw,
cart/instruction/camera/BRIGHT/history semantics. Dynamic cart reveal first4Hz
capture pastplane with OLD active; first postreveal only, queuedpre frames buffered.
Both repetitions required, no outcome-driven replacement. B physical command and
actual memory-at-cut independently reconstructed; geometry/RTF gates unchanged.
Separate pushed PhaseB freeze required before these two live acquisitions. No
source outcome yet; no optimization or method comparison.120 focused tests pass.

### 2026-09-23 — OSA02 two frozen genuine repetitions, final source NOT QUALIFIED

PhaseB freeze de6ab7a pushed before exactly REPEAT_00/01, no retry. POSE11 minus
.40m start, same instruction/cart/camera/BRIGHT, no PhaseA rerun. Both reveal first
eligible live frame, continue actual OLD, apply safe turning FRESH at moving B.
Raw FRESH hash equals historical POSE11_ON; world clearance .157818823/.149120888m,
local LEFT .584514499m despite RIGHT instruction, arc1.348502485m; no complete
bypass. Travel .293333349m, physical incoming speed.8m/s; B clearances1.029186438/
1.019186906m. Incoming omega~.000014 rad/s differs from controller memory~.500014
rad/s at application; both preserved with solve/application identities.
Remaining8rows/1.047797574m. FRESHRTF .991010591/.990905369, stalls.192278178/
.169465806s, no catch-up or safety abort, .10s postroll only.

Only frozen OLD obstruction gate fails in both: actual OLD generated fartherback
has1.351820588m finite arc and +.273958726m hypothetical-cart edge clearance.
It ends before cart; cannot substitute PhaseA OFF anchoring, extrapolate it, or
loosen criterion. Final ONLINE_POSE11_SOURCE_NOT_QUALIFIED, representative null,
qualified0/2 and source bundle entries empty. Safe FRESH/movingB measured facts
are preserved separately; no accepted obstacle-induced source claim.

Calls total8terminal (4technical/4scientific),28buffers,1separate serverwarmup;
56MPCaccepted/56saved, summedsolverwall.300798942s; all optimizer/GP/rigid/graph/
splice/reconciliation0. Collectorwall16.078964879s technical/15.538959193s online,
excluding Isaac startup; server stopped. No real model/MPC tests/validator calls.
Independent saved episode recomputation, source/CSV/JSON/plot/ZIP checks PASS;
519 relevant tests PASS6.79s; compileall/diff PASS. Eight static PNGs+sidecars/index/
reviewZIP at `data/obstacle_source_acquisition_02/primary_20260923T090000Z/`.
1,722 preserved source hashes and unrelated Stage0 edits unchanged. Raw remains
ignored. Remaining uncertainty: jointly obtain an actually obstructed finite OLD
and safe turning FRESH/movingB under a separate frozen acquisition; not implemented.

### 2026-09-23 — OSA03 pre-scientific exact-POSE11 OLD protocol

Fetched HEAD/origin84b8d80. Preserve OSA01/02 and two unrelated Stage0 edits.
New exclusive run `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`.
Target OLD now observed exactly at selected_POSE11, stationary new live bootstrap,
cart OFF throughout OLD inference. No historical input/output replay. Reveal at
first normal4Hz capture strictly after actual OLD command application. First
postreveal frame only; OLD continues to first FRESH application B. Exactly two
independent repeats; no retry/PhaseA/pacing search or next-stage optimization.

Unchanged OSA02 collector/scheduler/60-10-4Hz simulation cadence. New config sets
minimum_active_before_prediction_sim_s .5→0 solely to implement prescribed first
post-active request; otherwise that frame would be consumed buffer-only. This
necessary eligibility difference is explicit before inference, not hidden pacing
or geometry tuning. Exact same instruction/cart/camera/light/model/MPC/thresholds.
Saved validator adds exact OLD pose, zero pre-OLD motion, true application/reveal
order and finite OLD nominal obstruction to reused source gates. No extrapolation.
109 focused tests pass, no live model/MPC calls. Freeze commit/push precedes calls.

### 2026-09-23 — OSA03 qualified genuine obstructed-OLD source, 2/2

Scientific freeze 76352cf was pushed before exactly REPEAT_00/01. Both initialized
stationary at authoritative POSE11 [19.20312073159454,24.423334915767065,
-1.5689742328041627], new live bootstrap, genuine OLD with cart OFF. Actual OLD
application sim1.116666725/1.066666722; first strictly subsequent scheduled4Hz
capture sim1.283333400 reveals cart and supplies the only FRESH observation.
No PhaseA rerun/pacing search/retry/POSE12 fallback; OSA02 scheduler unchanged.

Both finite OLD arc1.351820588m, OFF edge1.117514011m, same raw OLD+cart edge
-.125215494m: the previously missing obstruction gate now passes, without
extrapolation/connector. FRESH whole edge.229486890/.221686301m, lateral+LEFT
.676244259m, yaw30.000670deg, arc1.353245520m, hallway progress1.172445m; nonSTOP,
no trimming. Interior mismatch.717343090/.712353270m. Same local OLD/FRESH hashes
across independent repeats were observed, not forced; world anchors differ.
RIGHT wording/localLEFT caveat retained; no complete bypass.

Actual observation→B travel.213333346m, physical incoming v.8m/s; B edge
1.095851731/1.085852164m. u_minus omega~-1.6e-6/-1.3e-6 versus worker memory
omega~.49999845/.49999873 at application are separately preserved. Original
FRESH remains observation-anchored; 9rows/1.202489115m future remains. FRESHRTF
.991909419/.991224685, maxstall.194015202/.151906103s, burst0; 99states/98 exact
integrations each, error0. WholeRTF.712711/.733696 supplemental only. Neither
abort; only.10s FRESH postroll, normal ATTEMPT_LIMIT. All frozen gates PASS.

Classification QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE; representative
REPEAT_00, replication REPEAT_01, both sealed in source_bundle. Exactly4scientific
terminal predictions,10buffers,1separate synthetic serverwarmup;14MPCsubmitted/
14saved, solverwall.106523391s. RTTsum.883671581s, collectorwall14.792618380s
excludes Isaac startup. GP/rigid/graph/splice/reconciliation0; no real test or
validator calls. Owned server stopped. Source acquisition ends here.

Saved-only independent recomputation, raw/bundle/CSV/JSON/plot/ZIP parity PASS;
2,123 historical/external/core/input/unrelated hashes unchanged. 532 relevant
regression tests PASS7.02s, compileall/diff PASS. Seven PNGs with numeric/hash
sidecars visually inspected; static index/reviewZIP and validation_final.json
under data/obstacle_source_acquisition_03/primary_20260923T085200Z/. Small tracked
summary docs/results/obstacle_source_acquisition_03.json. Raw/generated remain
ignored; two Stage0 edits untouched. Remaining uncertainty: actual same-B
transition execution and reconciliation benefit, not tested or implemented.

## 2026-09-23 — OSA03_NATIVE_CONTINUATION_01 pre-primary freeze

- Fetched origin/main and started at565e40a586e5f8081339c44d8f2949c5abffce9a;
  both unrelated Stage0 configs preserved/excluded. REPEAT_00 is the only rollout.
- Saved-only OSA03 validator, sealed hashes and B/command phase audited. Btick92,
  next historical submit96/application97, saved end98, first new submit102.
  u_minus differs from first-FRESH command and memory; no re-solve at B.
- Added small asynchronous official-worker continuation wrapper, saved-result
  prefix replay, existing exact integration/guard/evaluator, numeric/hash plots
  and saved validator. Six historical intervals reproduce with zero pose error.
- Official installation/restoration preflight disables solve:0 MPC/0 VLA calls.
  Protocol freezes one nominal3s continuation with fixed raw FRESH, source clock,
  original nearest/+1, abort-only safety and original .10m/15deg/.30s attachment.
  No reconciliation implementation or future-stage work. Results pending.

## 2026-09-23 — OSA03 Native continuation measured, source/phase preserved

- Pushed execution freeze5d76492653057a8b2e707ac82c54d5a73e53d39b. Exactly one
  REPEAT_00 continuation ran: six historical integration steps/zero pose error,
  then29 new asynchronous official MPC solves; no solve atB, no retries.
- Nominal3s cap reached; no abort/controller error. Initial distance.106649m
  grows to.214078m at.466667s; sustained attachment1.466667s occurs with.04185m
  original arc remaining. RawFRESH clearance.229487m vs execution.133849m
  (curve-adjusted lower bound.133806m), all above unchanged.05m.
- Nominal10Hz motion passes; actual command-application spacing shows6rad/s²
  for the second historical FRESH result, versus5 nominal. No continuous physical
  acceleration claim. Endpoint settles about.0961m from final pose, no full bypass.
- First validator attempt exposed only pre/post-journal guard-command metadata
  schema mismatch. Preserved failure; new saved-only authoritative validator
  permits only3 journal fields to be absent. Original execution/metrics/worker
  code and all258 source/input hashes unchanged; no scientific rerun.
- Added six numeric/hash-linked static figures/index/ZIP; visually inspected.
  Saved validator/figure parity PASS; final relevant suite371 PASS; compileall
  and diff check PASS. All test/validator real MPC/VLA calls0; scientific VLA,
  GP/rigid/graph/splice/reconciliation calls0. Unrelated Stage0 edits preserved.
- Result NATIVE_CONTINUATION_SAFE_SUSTAINED_ATTACHMENT. Same-B improvement remains
  untested; no subsequent reconciliation factor or comparison implemented.

## 2026-09-24 — OSA03 same-FRESH observation-start control, pre-execution

Fetched origin/main and confirmed `0842f5583d9295b436a47c7da9d26ef6ed054143`. Preserved unrelated stage0 config edits. Read the OSA03 source/Native continuation and delay-attribution contracts. Saved-only OSA03 and B-start authoritative validators pass. Observation tick75 has physical/controller .6000000288 m/s, last OLD result applied tick73, no pending solve, first legal FRESH submit78. New independent tracker restoration/install preflight uses zero numerical solves. Added separate observation-start wrapper, frozen two-origin metrics and signed comparisons; no source or historical code/result edits. Availability-origin 180-step cap leaves execution-origin full3s N/A if incomplete. No scientific execution before protocol commit/push. See `docs/OSA03_FRESH_TRACKABILITY_CONTROL_01.md`.

## 2026-09-24 — OSA03 same-FRESH observation-start control, result

Pushed freeze `21bd4f6a53f0e6afd88a83274450ac279e491b08`, then ran exactly one observation-start condition. Source REPEAT_00 and saved B continuation remain hash-identical; saved-only authoritative validators pass. Observation75 / legal submit78 / first FRESH application82: .116666673 s availability delay, .070000007 m incoming OLD travel. Execution-origin initial error .034992303 m/22.825669 deg; maximum .133758332 m at .466666691 s; sustained attachment1.000000052 s at original progress5.581671/9. Compared with saved B, first .9s position-AUC gap is +.067293615 m·s, attachment gap +.466666691 s, clearance gap −.041100181 m (all B minus observation). Both geometry-safe: .174906335 versus .133806153 m swept lower bound, margin .05 m. Observation still has intrinsic/interface tracking difficulty. Cold first MPC47.752507 ms compressed its next application interval to .05s; actual angular-command jump diagnostic10rad/s², while unchanged nominal10Hz diagnostics pass. No additive latency decomposition/recoverable-cost claim.

Availability cap3.000000156 s; execution exposure2.883333484 s. Complete execution-origin3s is N/A; .30/.90s and a common173-interval saved B prefix are independently compared. New MPC30, B-start0, VLA/RGB/Isaac/GP/rigid/graph/splice/reconciliation0; no retry. New solve sum152.355585ms; rollout wall3.112428s. Final relevant regression suite223 passed; compileall and diff-check pass. Static five-figure review, numeric/hash sidecars and ZIP: `data/osa03_fresh_trackability_control_01/primary_20260924T012000Z/review/index.html`. Report: `docs/OSA03_FRESH_TRACKABILITY_CONTROL_01.md`. Frozen execution/analysis code unchanged after outcomes; only report/README/log updated. Unrelated stage0 edits remain unstaged.

## 2026-09-24 — BLIND_CORNER_SOURCE_ACQUISITION_01 pre-scientific freeze

Fetched origin/main at `9bdf775e5b4dd5de2eac00efc1453a100923fefa`; preserved two unrelated stage0 config edits. Inspected two genuine Hospital corridor corners with zero model/MPC calls. C01 fails initial occlusion (1079 cart pixels); C02 passes 12→5655 diagnostic-pixel transition, actual wall ray occlusion, unchanged .20/.05 m footprint/clearance and one free passage. New static hook cannot move/toggle cart after initialization and latches only first scheduled visibility crossing. Frozen scientific order [C02], one OLD/one FRESH maximum, no retry, no reconciliation. Protocol: `docs/BLIND_CORNER_SOURCE_ACQUISITION_01.md`; preflight/result paths therein. Initial 12 focused implementation tests pass; regression checks precede commit/push and scientific calls.


## 2026-09-24 — BLIND_CORNER_SOURCE_ACQUISITION_01 result

Pushed scientific freeze `ee814f307904b878777fcfa56e5d7629e5837795`, then ran only prequalified C02 once. C01 was excluded before model calls (initial cart1079 pixels). C02 OLD saw12 pixels; first scheduled frame_000004→000005 crossing12→237 used exact triggering JPEG. Cart stayed authored-visible/oracle-present at constant transform;113 checks and final exact mesh parity pass. Genuine OLD arc1.283276m/yaw52.190122deg/tangent49.606466deg; rotating moving B has u_minus[.8,.126377241], separate memory[.8,.626377241], obs→B travel.279999892m, Bclearance.344559064m. RTF.991361139/stall.200494433s/no bursts pass.

Result `BLIND_CORNER_LIGHTNAV_SOURCE_NOT_QUALIFIED`: finite OLD remained safe with cart(.160775m overall/.740713m cart-only); raw FRESH violated existing corner wall margin(.015054543m overall/.512886513m cart-only); cart-influence mismatch has no eligible samples and stays null/unestablished. Actual source prefix stays safe(.329437471m), no abort, only.10s postroll. Representative=null; qualified bundle entries empty. No placement/prompt/threshold changes or retries. Added saved-only wall/trim attribution plot after observing failure; frozen scientific acquisition/gates/validator code remains byte-identical.

Calls:2 scientific terminal+5 buffer requests+1 server warmup;7 official MPC solves/results, solve sum.070488088s; acquisition wall12.099965s excluding Isaac process startup. Tests/preflight model/MPC0; GP/rigid/graph/splice/reconciliation/Native-continuation/trackability0. Final relevant regression270 PASS, compileall and diff-check PASS. Saved-only original episode/source validation PASS; static figures and numeric/hash sidecars at `data/blind_corner_source_acquisition_01/primary_20260924T054500Z/review/index.html`, review ZIP at run root. Report: `docs/BLIND_CORNER_SOURCE_ACQUISITION_01.md`. Historical artifacts/external source preserved; unrelated stage0 edits remain excluded.

### 2026-09-25 — BLIND_CORNER_SOURCE_ACQUISITION_02 protocol and geometry freeze

- Fetched origin/main; starting `f30cc25557063a37a0c38700851e8024f9b01f13`. Preserved/excluded the two unrelated stage0 config edits.
- Declared C03/C04/C05 at three different existing Hospital exterior room-block corners, with one common analytic left-turn envelope/cart shadow placement and no C02 retry. Saved-only genuine corpus census fixes the finite spatial prefix at turning-chunk q75=1.3254158228259225 m (80/1000 chunks).
- Rendered 33 static diagnostic viewpoints, not execution/history or model evidence. Exact mesh and same-product RGB/mask checks; all initial cart pixels0. C03 alone passes finite nominal conflict, real wall occlusion, .10 m combined bypass reserve and interaction/reaction-length gates. C04/C05 rejected against Hospital geometry; no replacement. Freeze online order `[C03]`, one OLD/one FRESH maximum, unchanged first scheduled crossing/pacing/abort guard and scientific thresholds.
- New implementation/synthetic plus historical read-only regression: 144 passed; compileall PASS. Zero LightNav/MPC calls during geometry/tests. Protocol/code/tests/eligible bank must be pushed before scientific acquisition. Details: `docs/BLIND_CORNER_SOURCE_ACQUISITION_02.md`; artifacts `data/blind_corner_source_acquisition_02/preflight_20260925T014000Z/`.

### 2026-09-25 — BLIND_CORNER_SOURCE_ACQUISITION_02 completed without qualified source

- Pushed scientific freeze `309a169ffd839a442bbdccf44119f75daccfd8ac`; ran only C03 once, with constant actual cart, exact first scheduled 0→104 pixel crossing, one OLD and one FRESH. No scientific retries or geometry/instruction/threshold changes. C04/C05 remained excluded.
- Saved-only validation PASS; classification `BLIND_CORNER_02_LIGHTNAV_SOURCE_NOT_QUALIFIED`. FRESH whole clearance .168307595 m and timing pass, but OLD remains cart-safe (.411010093 m), reliable tangent excursion28.341658°<30°, actual omega_minus .007337088<.10 rad/s, and no eligible within-influence OLD/FRESH correspondences. Physical u_minus [.8,.007337088] stays distinct from memory/first FRESH [.8,.323743905]. Travel .213333346 m; B clearance .210230734 m; remaining10rows/1.239975930 m. Actual short prefix safe .204006267 m, no abort.
- RTF .991162402, stall .190133362 s, bursts0. Calls: terminal2, buffers5, separate server warmup1; MPC7, solve wall .075395757 s; optimizer/reconciliation/continuation0. Server stopped afterward. All previous blind-corner inputs/results and unrelated stage0 edits preserved.
- Relevant regression462 passed; compileall/diff check PASS. Static review/geometry/clearance/timing figures with numeric/hash sidecars; additional saved-only influence-slab panel does not change primary results. `source_bundle/manifest.json` has no entries. Full report `docs/BLIND_CORNER_SOURCE_ACQUISITION_02.md`; artifacts `data/blind_corner_source_acquisition_02/preflight_20260925T014000Z/`.
- Stop this blind-corner search branch for now. No third search, new regime, method experiment or downstream optimization implemented.

### 2026-09-25 — LOCAL_SE2_RECONCILIATION_FORMULATION_01 implementation freeze

- Fetched latest origin/main; starting `5fa541a22082942da39f01dc809bab754b40b5c2`. Preserved/excluded both unrelated stage0 edits. Read source/native/observation-control and EXP-02C/graph/GP contrast evidence.
- New discrete spatial three-factor SE(2) objective: measured `B A^-1` transport target, original relative edges, downstream original world-FRESH anchor; arc-based `(1-s)^2`/`s^2`, equal block means, .10 m/10° normalization, lambdas1. No waypoint timing, GP or correspondence. Exact raw FRESH initialization and original-A local representation.
- Minimal optional feasibility/telemetry extension to existing right-local central-FD LM; unchanged default configuration and result schema. Pre-edit synthetic numerical goldens preserved. Improving unsafe steps rejected with original damping increase; initial safety required. Existing direct Hospital/cart geometry, .20 m footprint/.05 m margin, no B connector. Full-union final checker separate from proposal acceptance.
- R00 sealed bundle/source validator authenticated saved-only; R01 not evaluated. One preflight response-wrapper read error was corrected before scientific work and preserved separately; no model/controller/planning calls in preflight. Protocol/code/input hashes and tests frozen before the single planned solve. Report: `docs/LOCAL_SE2_RECONCILIATION_FORMULATION_01.md`.
- Freeze verification: 207 relevant tests passed, 1 historical-corpus absence skip; compileall/diff check PASS. Final prepared run `data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/`; scientific planning calls remain 0 until pushed freeze.

### 2026-09-25 — LOCAL_SE2_RECONCILIATION_FORMULATION_01 single planning result

- Pushed freeze `65a16823513709865cb3f3a7c8aa70ee26ccd440`, then exactly one saved-only R00 planning solve at `data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/`. No scientific retry or post-result formulation/weight/schedule/solver/source changes. Raw original FRESH SHA `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521` unchanged; only original R00 qualification revalidated.
- Initial→final costs: E_L 4.551116373→.348589943; E_R≈0→.068480118; E_A≈0→.348193565; total4.551116373→.765263626. Five LM iterations, cost-tolerance termination, four accepted, one non-improving rejection, zero unsafe rejections; wall.071575423s including feasibility/telemetry. No global optimum claim.
- First-node correction.211144018m, endpoint.002186831m; max node yaw change.341394deg. Relative-edge translation RMS.026158510m/max.040260838m, yaw RMS.073043299deg/max.099286219deg. Closed-form XY rigid-fit residual RMS.066834803m/yaw5.950349deg: descriptive nonrigidity, not a rigid baseline comparison.
- Independent full-union Hospital/cart reference-geometry safety PASS: raw.229486890m→optimized.227809208m minimum, both cart-limited at last segment endpoint; required.05m/.20m radius, original1e-7m tolerance. No B→X0 connector or execution safety assertion. Correction tapers downstream in this result without a monotonicity constraint.
- Saved-only independent equations/retraction/accepted-state safety/source/hash/frame/CSV/JSON/plot validation PASS; all four planning PNGs visually checked. Relevant regression207 passed/1 missing-historical-corpus skip; compileall/diff-check PASS. Scientific calls: local planning1; LightNav/MPC/GP/actual execution/rigid baseline/splice0; no R01 evaluation. GP support and waypoint timing none. No downstream method experiment implemented.
- Report and small numeric metadata: `docs/LOCAL_SE2_RECONCILIATION_FORMULATION_01.md` and `_RESULT.json`. Remaining uncertainty: actual common-B execution benefit and no-harm/intent preservation have not been measured.
