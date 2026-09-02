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
