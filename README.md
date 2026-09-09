# SE(3) Reconciliation Research

This repository studies how to reconcile successive Navigation VLA action chunks across an
OLD-to-FRESH transition while preserving the intent of the new trajectory. LightNav-0 emits
arbitrary `N x 3 [x, y, yaw]` SE(2) waypoint references; the reconciliation method developed
here is separate from LightNav.

## Current research state

- Stage 0-G resolved the LightNav input contract but the frozen six-scene qualification
  failed left/right/detour reproducibility and diversity; DATA-02 collection is not authorized.
- The execution-platform investigation prompted by feedback item 1 is complete through the
  frozen Stage 0-D/E/F and EXP-02B-R evidence. This does not claim that the platform is
  generally validated beyond the observed LightNav execution envelope.
- EXP-02C attributes the principal current-M4 failure to the incoming-direction transition
  factor. This diagnoses the existing objective; it does not introduce an improved objective.
- The EXP-01B-derived DATA-01 bank is retired from primary formulation use. Its dedicated
  generated bank and pipeline are removed, while the independent frozen EXP-01B source
  evidence is preserved.
- A replacement formulation-development LightNav dataset must be redesigned. No DATA-02
  collector or generated DATA-02 artifact is active in this repository.
- EXP-02D has not started.

## System boundaries

The execution stack keeps four levels distinct:

1. LightNav output: untimed spatial `N x 3 [x, y, yaw]` waypoint reference.
2. `TrajectoryFollower` output: desired body command `[v, omega]`.
3. differential/execution controller output: left/right targets mapped to four Jackal wheels.
4. Isaac Jackal measurement: actual pose, body motion, and wheel velocity.

Isaac Sim 6.0.1 is the simulator, ROS 2 Jazzy is the robotics middleware, and LightNav-0 is
the upstream navigation-VLA baseline. Do not install or modify LightNav inside this repository.
ROS 2 continues to use the machine's system Python; repository tests use the local Python 3.12
`.venv`.

LightNav waypoint rows have no intrinsic timestamps. Any time assignment is an explicit
experiment/controller convention and must not be presented as model timing. A local waypoint
is transformed only with the robot pose captured at that chunk's observation event:

```text
T_world_waypoint = T_world_robot_at_observation * T_robot_waypoint
```

Coordinate frames, units, observation/readiness/execution events, transforms, and source hashes
must remain explicit. Raw VLA outputs are never overwritten.

## Frozen evidence retained locally

Generated experiment data are ignored by Git, but the following primary paths are preserved on
this machine because current claims and provenance depend on them:

| Evidence | Local path |
|---|---|
| Redesigned controlled-latency EXP-01B source | `data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/` |
| Controller-level EXP-02B | `data/exp02b/exp02b-controller-aware-20260906T150400Z/` |
| Execution calibration Stage 0-D | `data/stage0/execution_calibration/stage0d-20260907T102700Z/` |
| Closed-loop validation Stage 0-E | `data/stage0/closed_loop_execution_validation/stage0e-20260908T021433Z/` |
| LightNav envelope Stage 0-F | `data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z/` |
| Calibrated EXP-02B re-evaluation | `data/exp02b_calibrated_reeval/exp02b-r-20260908T054233Z/` |
| Current-M4 factor isolation EXP-02C | `data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z/` |

These are not a new formulation dataset. The complete keep/archive/delete dependency audit is
in [the 2026-09-09 cleanup audit](docs/REPOSITORY_CLEANUP_AUDIT_20260909.md).

## Documentation map

Detailed protocols, commands, schemas, observed results, and claim limitations live in `docs/`:

- Platform: [Stage 0 Jackal](docs/STAGE_00_JACKAL_TRAJECTORY.md),
  [controller validation](docs/STAGE_00_CONTROLLER_VALIDATION.md),
  [single-chunk LightNav](docs/STAGE_00_LIGHTNAV_SINGLE_CHUNK.md),
  [execution calibration](docs/STAGE_00_EXECUTION_LAYER_CALIBRATION.md),
  [closed-loop validation](docs/STAGE_00_CLOSED_LOOP_EXECUTION_VALIDATION.md), and
  [LightNav execution envelope](docs/STAGE_00_LIGHTNAV_EXECUTION_ENVELOPE.md).
- Transition characterization: [EXP-01](docs/EXPERIMENT_01.md),
  [EXP-01A](docs/EXP_01A_LIGHTNAV_LATENCY.md),
  [EXP-01B](docs/EXP_01B_ONLINE_RAW_SWITCH.md),
  [EXP-01B extension](docs/EXP_01B_EXTENSION.md), and
  [redesigned controlled latency](docs/EXP_01B_REDESIGNED_CONTROLLED_LATENCY.md).
- Reconciliation: [EXP-02 pilot](docs/EXP_02_ORACLE_GRAPH.md),
  [EXP-02A](docs/EXP_02A_SPATIAL_ENTRY_RECONCILIATION.md),
  [EXP-02B](docs/EXP_02B_CONTROLLER_AWARE_RECONCILIATION.md),
  [EXP-02B GUI diagnosis](docs/EXP_02B_GUI_DIAGNOSIS.md),
  [EXP-02B-R](docs/EXP_02B_CALIBRATED_REEVALUATION.md), and
  [EXP-02C](docs/EXP_02C_FACTOR_ISOLATION.md).
- Chronology and commands actually run: [append-only work log](docs/WORK_LOG.md).

## Local environment

Keep the repositories separate:

```text
~/Workspace/
├── se-3-reconciliation/       # this repository, Python 3.12 .venv
└── external/
    └── LightNav-0/            # upstream checkout, Python 3.11 .venv
```

Reproduce the research test environment without changing system Python, ROS 2, CUDA, Isaac
Sim, or LightNav:

```bash
cd ~/Workspace/se-3-reconciliation
uv venv --python /usr/bin/python3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

## Validation

Run the repository test suite with external pytest plugin autoload disabled:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

Synthetic fixtures are tests and mechanism demonstrations only. They are never experimental
evidence.
