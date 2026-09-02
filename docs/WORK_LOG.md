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
