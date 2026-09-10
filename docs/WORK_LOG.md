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
