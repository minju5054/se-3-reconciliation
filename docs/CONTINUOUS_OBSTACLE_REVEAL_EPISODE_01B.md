# CONTINUOUS_OBSTACLE_REVEAL_EPISODE_01B

## Frozen protocol before science

Starting HEAD and fetched origin/main:
`e1e8ba0f3d70241cd2e9f20424daacbad51405d1`.
Run: `data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z`.
This is one separately authorized and frozen attempt following the independently
diagnosed pre-episode infrastructure failure. The failed 01 attempt retains its
original retry=0 record. Inside 01B, retry=0 and the full SimulationApp launch
budget is exactly one. No preliminary full SimulationApp startup is allowed.

The existing scientific collector, request policy, reveal implementation,
controller, safety guard, evaluator and four-figure reporter are reused without
editing their bytes. Three small command-line wrappers manage the separate
namespace, authenticate provenance, launch the original collector and handle a
possible absence of scientific data. They introduce no reconciliation method.

### Preserved evidence and scientific equivalence

Historical run: `data/continuous_obstacle_reveal_episode_01/primary_20261006`.
Historical tracked results: `results/continuous_obstacle_reveal_episode_01/`.
Both trees are preserved by a 39-file SHA256 manifest and checked before/after
execution. Tracked results also match their exact Git objects at the diagnosis
commit. Historical scientific freeze:
`2334b8e528b77395e24bde956f31cc0ee5cb474b`; blocked result:
`a8409a0013d4fab91b9384ac6511bdaa9b3512f3`.

Authority remains OSA03 REPEAT_00 at
`data/obstacle_source_acquisition_03/primary_20260923T085200Z`.
The saved OSA03 validator passes; 2,350 inherited source/input hashes remain
bound. No prior JPEG, model output or executed episode is replayed as new data.

[protocol_equivalence.json](../results/continuous_obstacle_reveal_episode_01b/protocol_equivalence.json)
compares 28 named scientific fields, including the whole runtime configuration.
All compare equal. The resolved runtime YAML, scene/cart, POSE11, schedule,
side-passage and external model/MPC authority files are copied byte-for-byte.
Only the two experiment-name labels in `protocol.json` change to 01B; run/output
paths and labels are administrative differences. The sole intentional runtime
difference is **process_launch_environment_only**.

Exact initial pose, world `[x m, y m, yaw rad]`:
`[19.20312073159454, 24.423334915767065, -1.5689742328041627]`.
Initial command `[0,0]`; cart OFF; one EPISODE_00. Instruction:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

The historical RIGHT-instruction/local-LEFT-output caveat is retained without a
new side-compliance gate. Hospital/cart/camera/BRIGHT, model/checkpoint settings,
official MPC and abort-only guard remain identical to 01/OSA03.

- 60 Hz integration, MPC every 6 ticks, capture every 15 ticks;
  `minimum_wall_step_no_catchup_v1`, no inference pause or catch-up.
- Four terminal predictions maximum: C0–C3, one in flight, no reset or C4.
  C0 uses the fourth live stationary frame; first three requests are buffer-only.
- Reveal once at the first scheduled capture strictly after C0's actual first
  command application. That frame requests C1. Cart stays ON/static thereafter.
- C2/C3 use the first eligible new scheduled frame strictly after the previous
  actual application. No queued substitute; the old chunk runs during inference.
- Each raw chunk is immutable and anchored at its own observation pose A_k.
  B_k is actual first-command application, never readiness/installation. Local
  +x forward/+y left; world XY metres/+Z up/CCW radians; waypoint dt is null.
- Same 4 s active cap, .10 s postroll after C3 application, host/worker timeouts,
  raw-reference abort gate and execution abort guard. Radius .20 m and required
  footprint-edge clearance .05 m remain unchanged.
- Same first-response gates and outcome priority. Later local evolution uses
  symmetric vertex-to-continuous-polyline distance >=.02 m, endpoint lateral
  difference >=.02 m, or shortest-angle endpoint/net yaw difference >=5 degrees.
  STABLE is an allowed qualified source outcome.

### Process-local launch correction

The OSA03 successful command is parsed directly from its historical document;
the ordered list of 11 removed variables and four assignments is asserted equal
to 01B's declaration. No system library, symlink, Python, CUDA package, shell
startup file or external source is edited. The inherited environment dictionary
is unchanged; sanitation applies only to the launched child.

Read-only `ldd -r` checks on the same Isaac `libcusparse.so.12` reproduce:

| Environment | Selected nvJitLink | Unresolved symbols | Exit code |
|---|---|---:|---:|
| Inherited | `/usr/local/cuda/lib64/libnvJitLink.so.12` (CUDA 12.6) | 8 | 0 |
| Exact OSA03 sanitation | Isaac bundled `nvidia/nvjitlink/lib/libnvJitLink.so.12` | 0 | 0 |

Exact selected/real paths, SHA256 hashes, symbols, command and output hashes:
[loader_verification.json](../results/continuous_obstacle_reveal_episode_01b/loader_verification.json).
These checks do not initialize SimulationApp. A failing sanitized loader check
blocks science as TECHNICAL_PREFLIGHT_BLOCKED; no alternate CUDA path is tried.

The launch wrapper reserves an exclusive `launch_attempt.json` before invoking
the original collector. An existing marker prevents a second attempt, including
after exit code 0 without an episode. A read-only parent observer records the
scientific process's relevant `/proc` environment and mapped CUDA library paths;
it never pauses or communicates with the scientific child. Isaac's own launcher
sets its internal LD_LIBRARY_PATH/PYTHONPATH after sanitation; these are reported
separately from inherited variables. Exit code alone does not establish success.

### Validation, outputs and claims

The saved-only 01B audit checks protocol equality, historical preservation,
external/launch/library hashes, actual environment and independent call counts.
It delegates episode geometry, RGB, timing, application, command reconstruction,
safety and evolution checks to the byte-unchanged 01 validator. No new model,
MPC, optimizer or SimulationApp call occurs during validation/reporting.

Only a validated qualified C0–C3 episode is marked as a valid continuous source.
A validated partial episode retains its exact prefix with an explicit partial
qualification record. A technical or validator failure has no qualified source
claim. On startup failure the reporter produces one clearly labeled technical
status PNG; otherwise it reuses the original four figures. No fabricated geometry.

All classification thresholds remain frozen. No result-driven change, retry,
source search or follow-on B_ENTRY/graph/canonical experiment is permitted.
A source may establish successive native chunks and reveal-associated geometry
change; it does not establish causal semantic recognition, complete bypass,
navigation-task success, reconciliation benefit or generalization.

### Commands

```bash
git fetch origin main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_continuous_obstacle_reveal_episode01b.py --mode prepare --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python -m pytest -q tests/test_continuous_obstacle_reveal_episode01.py tests/test_continuous_obstacle_reveal_episode01b.py tests/test_obstacle_source*.py tests/test_join_online*.py tests/test_online*.py tests/test_robotless_online*.py tests/test_data02*.py tests/test_se2*.py tests/test_join_source02*.py tests/test_gp_se2_environment*.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_episode01b.py --mode freeze --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
# Review and stage only 01B code/config/tests/protocol, then:
git diff --cached --check
# Commit and normal push. Verify local HEAD == origin/main before start/launch.
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_episode01b.py --mode verify --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_episode01b.py --mode start --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_episode01b.py --mode launch --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_continuous_obstacle_reveal_episode01b.py --mode stop --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/validate_continuous_obstacle_reveal_episode01b.py --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z --seal
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/report_continuous_obstacle_reveal_episode01b.py --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
```

The `launch` command invokes exactly the following child command (also frozen
as an argument list in `launch_environment.json`):

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 /home/gpuadmin/isaacsim/python.sh scripts/isaac/continuous_obstacle_reveal_episode01.py --run /home/gpuadmin/Workspace/se-3-reconciliation/data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
```

Results and pushed scientific freeze SHA will be appended after the single attempt.

Final pre-science regression: **558 passed, 1 skipped**. The skip is the absent
immutable generated DATA02 v1 corpus. The suite ran with local Unix-socket
permissions needed by the existing worker tests. Initial combined 01/01B focused
suite: 48 passed; final 01B focused suite: 20 passed, included in the 558 total.
The added cases cover saved-request accounting and blocked-report verification.
Compileall and diff checks pass. No new scientific call has
occurred during preparation, loader checks or tests. Unrelated Stage0 config
edits and the GPU-memory plotting script remain excluded.
