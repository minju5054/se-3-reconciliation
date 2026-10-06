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

## Repository-confirmed results

Scientific freeze **`86c54fbd78b66282c4eb6707a8b9f38e5faba12e`** was committed,
normally pushed, and verified equal to origin/main before server startup and the
single Isaac launch. All 39 failed-01 files remain unchanged. There was one new
startup, one initialized/finalized episode and no retry or second startup.

**CONTINUOUS_CHUNKS_NO_QUALIFIED_FIRST_REACTION** is the unchanged classifier's
result. Its name does not imply that C1–C3 were applied: **only C0 was applied**.
C1 was returned nonSTOP but rejected before installation by the existing full
raw-reference safety gate. Runtime status is SCENE_INVALID; policy termination
reason is RAW_UNSAFE. C2/C3 were not requested. No qualified C0–C3 source was
obtained. No model STOP, controller numerical failure, timeout or execution-guard
abort was observed.

### Startup and actual library

The old dependency conflict was resolved. Isaac logged `Simulation App Startup
Complete`; scene setup and actual acquisition followed. The live scientific
process's saved `/proc` mappings identify:

```text
/home/gpuadmin/isaacsim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle/nvidia/nvjitlink/lib/libnvJitLink.so.12
SHA256 0369e6867d44b800437de4e146d72c65afc6c75adf677a15c2ecd8e6a7ac135f
```

This is the bundled library from the sanitized read-only check. The recorded
process environment has the prescribed thread variables and no inherited
CUDA_HOME/CUDA_PATH/ROS/AMENT/CMAKE/COLCON/RMW variables. Isaac's own internal
library/Python paths are recorded separately. No external installation/source
file was edited. Both official model/checkpoint and MPC hashes remain unchanged.

### Chunk availability, geometry and safety

| Quantity | C0 | C1 | C2 | C3 |
|---|---:|---:|---|---|
| Generated / actually applied | yes / yes | yes / no | no / no | no / no |
| STOP | false | false | N/A | N/A |
| N | 10 | 10 | N/A | N/A |
| Raw XY arc [m] | 1.351820588145103 | 1.3660895841364071 | N/A | N/A |
| Endpoint local lateral [m] | 0.000003947936875192681 | -0.5808053612709045 | N/A | N/A |
| Max absolute local yaw [deg] | 0.0009005603795339317 | 29.598703315425713 | N/A | N/A |
| Request cart pixels | 0 | 3992 | N/A | N/A |
| Hospital-only edge clearance [m] | 1.1175140107938666 | 1.123279502284892 | N/A | N/A |
| Cart-ON combined edge clearance [m] | -0.1252154936144262 | 0.00799722992900126 | N/A | N/A |
| Whole raw safety in actual observation scene | PASS, cart OFF | FAIL, cart ON | N/A | N/A |

C0's cart-ON row is the frozen hypothetical obstruction check, not an executed
collision. C1 has no physical overlap in the reference checker, but its clearance
is below the unchanged .05 m requirement, so it is unsafe under this protocol.
The rejected original reference is preserved without repair. Raw local poses
were transformed using their own A only; safety checks include every returned
world segment and no invented A/B-to-first-row connector.

Actual executed-prefix swept clearance lower bound: **1.0697830924553602 m**.
All 89 integrated command intervals passed the abort-only execution guard. Every
non-bootstrap applied command used C0; no C1 command was applied. The cart was
revealed exactly once and remained ON/static until termination.

This C1's negative local Y is a rightward output. Historical OSA03 returned
positive local Y/leftward output under the same instruction. The side caveat is
retained; no instruction or selection rule was changed to obtain this outcome.

Raw local NPY SHA256:

```text
C0 6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae
C1 bee96feb024de770e98f217d60c7262830a76fadd8abf2a29d499e64b94b25be
C2 N/A
C3 N/A
```

### Actual poses and timing

World `[x m, y m, yaw rad]`:

```text
A0 = [19.20312073159454, 24.423334915767065, -1.5689742328041627]
A1 = [19.203242163778583, 24.356668352984542, -1.5689752526514715]
A2 = N/A
A3 = N/A
B1 = N/A (C1 was never installed/applied)
B2 = N/A
B3 = N/A
```

C0's stationary bootstrap application pose equals A0. It is not a moving B1.
Its physical incoming command was `[0,0]`; its first applied command and controller
memory were `[0.20000000979019583, -0.000005957309117577748]` in m/s and rad/s.
C1 has no switch-time command/memory, because it never became active.

| Simulation-clock event [s] | C0 | C1 | C2/C3 |
|---|---:|---:|---|
| Observation | 0.7833333741873503 | 1.2833334002643824 | N/A |
| Ready seen by runtime | 1.0166667196899652 | 1.5166667457669973 | N/A |
| Install | 1.0166667196899652 | N/A | N/A |
| Actual first command application | 1.1166667249053717 | N/A | N/A |

Reveal occurred at the exact C1 observation, strictly after C0 application.
The episode ended at simulation time 1.5166667457669973 s. C0 remained active
through C1 inference. Request send/receipt are host events, not waypoint times.

The following displayed host times use **2026-10-06 KST (UTC+09:00)**; original
UTC stamps and full monotonic nanoseconds remain in the machine-readable records.

| Host event | C0 | C1 | C2/C3 |
|---|---|---|---|
| Observation | 19:11:47.645786 | 19:11:48.293100 | N/A |
| Request sent | 19:11:47.736467 | 19:11:48.384713 | N/A |
| Response received | 19:11:47.942425 | 19:11:48.599240 | N/A |
| Ready seen | 19:11:47.954522 | 19:11:48.602934 | N/A |
| Install | 19:11:47.954720 | N/A | N/A |
| Actual first application | 19:11:48.126951 | N/A | N/A |

C0/C1 host RTTs: .205970441 / .214535369 s. Request-local RTFs:
.9913469415385182 / .9913244868228904; maximum loop stall .21772544202394783 s.
Observed request timing and scheduler checks pass. The four-request aggregate
is false because only two requests occurred; it does not establish a complete
C0–C3 timing record. No host/simulation clock subtraction or waypoint timing.

### First reaction and later evolution

The frozen geometry gates give:

- C0 OFF validity: PASS.
- Same finite C0 obstructed by revealed cart: PASS.
- C1 whole raw safety: **FAIL**, .00799722992900126 m < .05 m.
- Meaningful C0→C1 change: PASS. Interior separation .5808102516166572 m;
  reliable tangent difference 25.948876599808514 degrees; reliable pose-yaw
  difference 26.546080116111007 degrees.
- Returned-path progress/future proxy: PASS. There is no applied-B future gate
  result, because B1 does not exist.
- Reveal, visibility and observed request timing/scheduler: PASS.
- Applied/moving/safe B1 and switch memory: unavailable; required qualification
  booleans are false because C1 was not applied, not because an unsafe B was driven.

The complete first-response qualification therefore fails. C1→C2 and C2→C3
raw/world hash comparisons, geometry differences, observation motion, clearance
and pixel evolution are **N/A**. Neither EVOLVING nor STABLE is assigned.

### Call accounting: separate attempts

| Item | Failed 01 (historical) | New 01B |
|---|---:|---:|
| Full SimulationApp launch attempts | 1 | 1 |
| Scientific episodes initialized / collector finalized | 0 / 0 | 1 / 1 |
| Qualified continuous C0–C3 sources | 0 | 0 |
| Terminal LightNav predictions | 0 | 2 |
| Buffer-only requests | 0 | 4 |
| Server startup warmups | 1 | 1 |
| RGB captures | 0 | 6 |
| MPC submissions / solved results / physical new applications | 0 / 0 / 0 | 5 / 5 / 5 |
| Integration intervals | 0 | 89 |
| Cart reveals | 0 | 1 |
| Episode initialization resets / later resets | 0 / 0 | 1 / 0 |
| Retries / source searches | 0 / 0 | 0 / 0 |
| Graph / canonical / B_ENTRY / Hermite / V2 / GP / rigid / correspondence | all 0 | all 0 |
| Validation/report model / MPC / optimizer / Isaac calls | all 0 | all 0 |

Collector finalization records a bounded termination, not successful C0–C3
acquisition. The owned model server was identity-checked and stopped.

### Parent observer failure and reporting deviation

The passive parent observer raised `PermissionError` while reading
`/proc/3988921/environ`; its exit code was 1. It therefore did not write its final
`launch_result.json`. The unchanged scientific child independently wrote episode
metadata/completion, extension hashes and schedule completion, with the exact
RAW_UNSAFE termination above. No relaunch or scientific code/config change was
made in response to the observer error.

A saved-only reporting script derived the missing supervisor completion record
from the existing launch reservation, child PID/start, two `/proc` samples,
Isaac log and collector completion records. Its origin is explicitly
`DERIVED_SAVED_ONLY_AFTER_SUPERVISOR_OBSERVER_FAILURE`; the **unobserved child
exit code remains null**. Recorded library hashes and collector completion are
independent evidence, not an inferred exit-code success. The script, input hashes
and transcribed tool error are bound in `execution_audit.json`. An independent
saved-only recovery audit passes. This metadata reconstruction is the technical
reporting deviation; it does not replace a model response, command or trajectory.
The newly generated CSV was normalized from CRLF to LF without changing values.

### Validation, partial bundle and PNGs

The byte-unchanged scientific validator passes saved source/scene, raw/RGB/history,
observation transforms, request order, native controller/command integration,
timing, safety rejection and preserved-prefix checks. 01B protocol/environment
and historical preservation checks pass. Saved-only check-again and four-figure
numeric/hash parity pass. All four PNGs were visually inspected; C2/C3 and B1–B3
are absent. World curves denote returned references; only the dark actual path
was executed. C1's displayed reference was rejected.

Partial bundle:
`data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z/source_bundle/`.
`qualification.json` states **valid_continuous_C0_C3_source=false** and
**validated_partial_data=true**. It contains C0 execution and immutable indices
to the generated C0/C1 outputs. It contains no fabricated C1→C2/C2→C3 handoff.

1. [Continuous world episode](../results/continuous_obstacle_reveal_episode_01b/figures/continuous_world_episode.png)
2. [Observation-local evolution](../results/continuous_obstacle_reveal_episode_01b/figures/observation_local_evolution.png)
3. [Application/request timeline](../results/continuous_obstacle_reveal_episode_01b/figures/episode_timeline.png)
4. [Exact request RGBs](../results/continuous_obstacle_reveal_episode_01b/figures/request_rgb_sequence.png)

Additional saved-only commands executed after the observer error:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z/reporting/derive_launch_record.py
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z/reporting/validate_recovered_record.py
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/validate_continuous_obstacle_reveal_episode01b.py --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-01b-mpl .venv/bin/python scripts/report_continuous_obstacle_reveal_episode01b.py --run data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z --validate
```

## Interpretation and claim boundary

The process-local launch correction resolved the observed startup conflict.
One genuine native C0 execution and a changed post-reveal C1 were observed, but
the unchanged reference safety gate rejected C1. A continuous applied C0–C3
source was **not** acquired. This bounded result is preserved without retry or
protocol tuning. It does not support semantic causal recognition, complete
bypass, navigation-task completion, reconciliation benefit or generalization.
No later research stage was implemented or executed.
