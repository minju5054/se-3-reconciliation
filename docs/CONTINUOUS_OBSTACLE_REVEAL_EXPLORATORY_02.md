# CONTINUOUS_OBSTACLE_REVEAL_EXPLORATORY_02

## Frozen protocol before science

Starting HEAD / fetched origin/main:
`0f21559cbac6cda0d58b28870b2d823b4de7e409`.
Run: `data/continuous_obstacle_reveal_exploratory_02/primary_20261006`.
One newly authorized episode, one full Isaac launch, no retries, no source search,
and at most four terminal predictions C0–C3. This is exploratory source
acquisition, not navigation safety validation or a reconciliation experiment.

### Sole scientific intervention

01B generated a C1 with 0.00799722992900126 m footprint-edge clearance and no
physical overlap. Its extra 0.05 m reserve gate rejected installation. This
experiment changes only the required **additional** clearance to **0.00 m**.
The circular robot radius remains **0.20 m**. Both complete native-reference
acceptance and the runtime command guard use this policy. The legacy 0.05 m
checks are retained as descriptive diagnostics.

The isolated `ExploratoryEnvironment` delegates both thresholds to the same
unchanged direct Hospital/cart checker. Numerical tolerance, obstacle projection
and uncertainty, workspace convention, swept-circle geometry and curved-command
sagitta bound are unchanged. Unknown workspace and borderline checks fail closed.
A negative footprint-edge clearance cannot pass the zero-reserve criterion.
No unsafe command is applied, no rejected reference is installed, and no repair,
replacement prediction, clipping or alternative method is permitted.

All historical source files and defaults remain byte-identical. The new Isaac
entry point binds only a different geometry-worker class into the historical
collector process. That worker uses the historical RPC and command guard with
the explicit margin adapter. The collector, request policy, reveal rule, command
computation and model worker are reused. The independent saved-only validator
and four-PNG reporter derive from the historical implementations in this separate
namespace to retain historical hash validation.

The geometry adapter computes both thresholds during collection; that bookkeeping
may consume host time, which remains measured. No simulation pause, catch-up or
logical schedule replacement is introduced.

### Authenticated source and unchanged inputs

01B: `data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z`,
freeze `86c54fbd78b66282c4eb6707a8b9f38e5faba12e`.
Its complete saved validator is reexecuted without scientific calls; its tracked
results are compared with exact Git objects at the starting HEAD. A 140-file
manifest preserves the old run/results, in addition to 2,389 inherited authority
files. OSA03 REPEAT_00 is independently revalidated saved-only.

The POSE11, scene/cart transform, side passages, runtime YAML, episode schedule
and external authority files are copied byte-for-byte. The protocol delta is
limited to namespace, explicitly declared clearance policy and classification.
The instruction, camera/BRIGHT, official MPC and LightNav source/checkpoint,
generation parameters and history processing are unchanged.

Initial world pose `[x m, y m, yaw rad]`:
`[19.20312073159454, 24.423334915767065, -1.5689742328041627]`.
Initial physical command `[0,0]`. Instruction:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

The margin is a downstream acceptance policy. LightNav receives the unchanged
RGB/history and instruction interface: `seq`, encoded `image`, and `instruction`.
It receives no robot radius, clearance threshold, cart coordinates or map. The
saved-only audit checks payload keys, instruction bytes/hash, JPEG bytes/hash,
full ordered history IDs/hashes, reconstructed SlowFast selections and raw
response/action hashes. Reconstructed sampler indices are not server telemetry.
No replay or determinism investigation is part of this task.

### Continuous execution and frames

- One session/login/reset at initialization; no subsequent reset or reconnect.
- Integration 60 Hz; MPC every six ticks; RGB every 15 ticks. The unchanged
  `minimum_wall_step_no_catchup_v1` scheduler never pauses for inference.
- C0 uses the fourth genuine stationary capture after three buffer-only frames.
  Cart is OFF during C0 observation and inference.
- Reveal exactly once before the first scheduled capture strictly after C0's
  first physical command application. That exact frame requests C1.
- C2 and C3 use the first eligible scheduled capture strictly after the preceding
  actual application, with one terminal request in flight. No frame substitution,
  C4, cart movement or second reveal.
- Each immutable raw chunk is anchored at its own measured observation A_k:
  `T_world_chunk = T_world_Ak * T_Ak_chunk`. Local +x forward/+y left; world XY
  metres, +Z up, CCW radians. B_k is actual first-command application, not receipt
  or installation. No B re-anchoring; intrinsic waypoint dt remains null.
- Same 4 s active cap, .10 s postroll after C3, timeouts and official MPC memory,
  generation/stale handling, nearest+1 selector and H=5.

The process-local launch is the exact successful OSA03/01B environment sanitation
(11 removed variables, four assignments). Read-only relocation checks require
Isaac's bundled nvJitLink with no missing symbols before launch. Actual mapped
library paths/hashes are observed via `/proc`. No system or external file changes.
The new passive observer catches OSError, including permission failures, and
records them without stopping or relaunching the scientific child. This fixes
01B's administrative observer failure and does not interact with simulation.

### Descriptors and predeclared outcomes

C0→C1 retains the frozen response geometry descriptors, including cart-OFF C0,
hypothetical cart-ON C0, continuous interior separation, tangent/yaw differences,
progress and cart pixels. An exploratory first reaction requires actual C0/C1
applications, reveal, nonSTOP C1, meaningful change and a passing exploratory
reference. Historical moving-B/future-progress qualification is descriptive only.

For C1→C2 and C2→C3, record raw hash equality, endpoint forward/lateral/wrapped
yaw delta, wrapped net-yaw delta, max absolute lateral/yaw excursion change,
symmetric vertex-to-continuous-polyline distance, observation translation/yaw,
A→B travel, physical and controller commands, pixels and both clearance flags.
Meaningful change uses the frozen OR thresholds: separation >=.02 m, absolute
endpoint lateral change >=.02 m, or endpoint/net yaw change >=5 degrees.

Pairs receive `C1_TO_C2_EVOLVING/STABLE` and `C2_TO_C3_EVOLVING/STABLE`.
Episode intent is `POST_REVEAL_INTENT_EVOLVING` if either observed pair changes;
`POST_REVEAL_INTENT_STABLE` requires **both** observed pairs below threshold.
Missing pairs are N/A and cannot establish stability.

Top-level categories, in order:

1. `CONTINUOUS_EXPLORATORY_C0_C3_ACQUIRED`: all four generated/applied, one reveal,
   no reset, exploratory reference/actual execution checks pass, timing/provenance
   and source-bundle validation pass. Legacy 5 cm failures are allowed and shown.
2. `PARTIAL_CONTINUOUS_EXPLORATORY_SEQUENCE`: C0 and C1 applied, but no C3
   application. Exact overlap/guard/STOP/controller/timeout/cap/failure cause stays
   in the result; missing continuation is not fabricated.
3. `FIRST_POST_REVEAL_REFERENCE_OVERLAPS`: C1 physically overlaps; never install it.
4. `TECHNICAL_EXECUTION_BLOCKED`: restoration, runtime infrastructure, timing or
   validator failure prevents qualified collection.

A behavior-only stop before C1 application, without physical overlap (for example
STOP or borderline clearance), is not covered by the requested first three
categories. Its classification remains null with the exact reason; it is not
relabeled an infrastructure failure. This coverage rule is frozen before science.

### Saved-only validation, outputs and limits

Every guarded interval retains exploratory and legacy results. The report also
rechecks the **actual one-step executed interval** separately from a new command's
nominal .1 s guard lookahead. Limiting Hospital/cart group diagnostics use the
same sweep; no future state enters controller computation.

The bundle preserves raw chunks/world installation, A/B/P, observation/request/
receipt/readiness/install/application clocks, exact requests/RGB/history,
controller previous_control and physical commands, generation/reference versions,
and actual OLD execution segments. A manifest authenticates both raw sources and
derived indices. A partial bundle stays explicitly partial.

Exactly four primary PNGs: world episode (equal axes, separate .20 m physical and
.25 m legacy boundaries), own-local geometry/yaw, timeline/active identity, and
exact RGB sequence with metadata outside images. Missing chunks are N/A. Numeric
plot inputs and large arrays stay under ignored data; compact JSON/CSV, hashes
and PNGs are tracked. Optional fifth figure is omitted as redundant.

No optimizer, graph, B_ENTRY, canonical method, Hermite/spline, transport,
correspondence, repair or follow-on reconciliation is implemented or called.
Even acquisition of C3 establishes only successive native behavior in this one
exploratory episode. It does not establish navigation safety, complete obstacle
bypass, right-side instruction reliability, real-world improvement, graph benefit
or general multi-chunk navigation success. Any overlap-free claim is limited to
the frozen 0.20 m footprint checker with **no additional 5 cm reserve**.

## Commands

```bash
git fetch origin main
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode prepare --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python -m pytest -q tests/test_continuous_obstacle_reveal_exploratory02.py tests/test_continuous_obstacle_reveal_episode01.py tests/test_continuous_obstacle_reveal_episode01b.py tests/test_obstacle_source*.py tests/test_join_online*.py tests/test_online*.py tests/test_robotless_online*.py tests/test_data02*.py tests/test_se2*.py tests/test_join_source02*.py tests/test_gp_se2_environment*.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode freeze --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
# Review/stage only this experiment; check staged diff; commit and normal push.
# Confirm HEAD == origin/main before server warmup or launch.
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode verify --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode start --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode launch --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode stop --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/validate_continuous_obstacle_reveal_exploratory02.py --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006 --seal
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/report_continuous_obstacle_reveal_exploratory02.py --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
# Recheck without --seal and with reporter --validate; inspect all PNGs.
```
