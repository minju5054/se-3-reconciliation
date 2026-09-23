# OBSTACLE-SOURCE-ACQUISITION-02: pacing correction and conditional POSE11 acquisition

Starting/fetched main: `1b8f94e737ac477291936eef01777e13b571c5f1`.
Two unrelated Stage0 config edits remain untouched. This work ends at source
acquisition, with no GP/rigid/splice/graph/reconciliation or method comparison.
OSA01 remains preserved: PhaseA POSE11/12 qualify, POSE11 selected, no moving B
because its original collector fails request-local pacing. No PhaseA rerun,
POSE12 fallback, instruction/geometry/camera/lighting/model/controller tuning.

Run: `data/obstacle_source_acquisition_02/primary_20260923T090000Z/`.
Configuration: `configs/obstacle_source_acquisition_02.yaml`.

## Stage 1A: saved-only diagnosis, before new model calls

Authoritative source: OSA01 `primary_20260923T073600Z/phase0/`.
The original saved-record validator passes before the permitted collector edit.
`diagnosis/OSA01_revalidation_before_correction.json` retains that validation;
`diagnosis/collector_before.py` retains exact old code, authenticated by SHA/git.
Original raw results, original frozen code revision and thresholds are not changed.

The code advances exactly one simulation step per outer loop, then sleeps until
an **absolute episode-origin deadline**. There is no inner multiple-step loop.
After render/readback blocks, remaining time is negative; following outer
iterations execute without a positive pacing sleep until absolute time catches up.
Saved state wall timestamps confirm fast steps during each inference window:

| Request | Observation render/readback s | Debt after rendered step s | Request-local RTF |
|---|---:|---:|---:|
| C1 | .082847 | .073773 | 1.378365284 |
| C2 | .084082 | .075253 | 1.393438269 |
| C3 | .083053 | .074085 | 1.387401764 |

Following state steps occur approximately every .002–.003 s instead of nominal
.016667 s; debt falls toward zero over consecutive outer iterations. The whole
episode RTF .999249833 hides this request-local behavior. Render/readback spans
range approximately .080–.208 s in this saved episode. Geometry guard median
.000922 s, maximum .004035 s is much smaller. The acquisition-loop mechanism is
localized; internal renderer/GPU stall causes are not isolated. Historical sleep,
outer-loop-start and fine-grained blocking timestamps were not recorded; they
are N/A, not invented. `diagnosis/diagnosis.json` binds all state/request rows
and source hashes. Its fast-step label is measured spacing, not measured sleep.

## Frozen technical correction

New opt-in policy: `minimum_wall_step_no_catchup_v1`.
For completed step n, the next completion deadline is
`host_completion[n] + (sim[n+1]-sim[n])/target_RTF`.
If a blocking operation overruns that deadline, no debt is repaid with subsequent
fast steps. Next deadline rebases on actual completion. No integration step is
skipped, coalesced, duplicated, rescaled or timestamp-normalized. This changes
wall scheduling policy, **not the intended simulated cadence**. Blocking may
reduce overall wall throughput; whole-episode RTF is reported without repair.
Historical default policy remains `absolute` for configurations without opt-in.

Unchanged: float32-resolved 1/60 dt; exact held-command unicycle; capture every15
steps (4Hz simulation clock); submit every6 (10Hz); official MPC computation,
async result polling/command application; OLD continues during request; actual
render/observation poses; empty buffer requests and full-episode native history;
observation anchor; readiness/install/first-command-switch separation. New
live inputs will reflect actual corrected scheduling, not replayed old pixels.
This is not a claim of byte-identical new online observations.

Opt-in instrumentation buffers rows in memory and flushes after the episode:
outer-loop start/end, sim state IDs, deadline/error, requested/actual sleep,
number of steps, capture/render duration, model/MPC message activity and submits,
guard/integration timing. Saved-only audit derives per-request no-sleep runs,
fast/burst steps, integration coverage and unchanged RTF metric. Segmentation
and guard remain evaluator-only; no obstacle steering or raw-path correction.

## One technical qualification, frozen before calls

Exact OSA01 Phase0 config and 5m OFF initial pose, exact instruction below,
4Hz live RGB /10Hz native MPC /60Hz integration; three post-bootstrap handoffs,
maximum5s active, .10s postroll. Independent native session, no cart reveal.
One episode only, no automatic retry or outcome-driven tuning.

Pass requires >=3 completed post-bootstrap requests, **every** request-local
RTF [.8,1.2], maximum loop stall <=.25s, exact saved-command state reconstruction,
no safety abort, fixed capture/control simulation ticks, no completed interval
faster than resolved dt minus1 microsecond instrumentation comparison tolerance.
The latter is a descriptive scheduler consistency bound, not a relaxed RTF gate.
RTF remains the historical first/last saved state quotient inside true client
send/receipt; full RTT and observation-to-last-inflight sim progress are separate.
No intervals are dropped to pass. Failure -> `PACING_FIX_NOT_QUALIFIED`, PhaseB0.

## Conditional Stage 2, requires a separate pushed freeze

Only if technical qualification passes: exactly REPEAT_00 and REPEAT_01, no
PhaseA calls. Fixed selected POSE11 observation pose from OSA01, initial pose
.40m backward along saved hallway forward with yaw unchanged. Exact same cart,
BRIGHT lighting/camera, source/checkpoint/VLN/greedy generation and instruction:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

Source-only ON was local +y/LEFT; side-language compliance is not established.
No side reinterpretation, prompt alteration or new side gate follows.

Cart initially absent. OLD must be actually active. Reveal at first scheduled
capture at/past frozen POSE11 longitudinal plane. Queued pre-reveal RGB is only
buffered, never the post-reveal FRESH request. First eligible post-reveal frame
only; no second FRESH retry. Actual OLD commands continue while inference runs.
B is the first actual new FRESH command application. End .10s later, maximum4s
active. Both independent repetitions run irrespective of first outcome.

Existing OSA01 OLD straight/safe/progress and hypothetical-cart obstruction,
whole-raw FRESH safety (.20m footprint/.05m edge), nonSTOP/progress/meaningful
mismatch, actual v_minus>.20m/s, obs→B travel>=.02m, B safety, >=4 remaining raw
rows and >=.60m arc at B, RTF/stall/scheduler gates remain unchanged. No prefix
trimming or B anchoring. Raw FRESH remains observation-anchored. Physical incoming
command, controller memory at B, first solve/input/result and first command are
separate. Guard can only abort before unsafe application and is disclosed.
First qualifying repetition is representative; neither -> no retry/fallback.
No complete-bypass, population navigation or reconciliation claim follows.

## Commands / validation

```bash
.venv/bin/python scripts/run_obstacle_source_acquisition02.py --mode prepare --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q tests/test_obstacle_source_pacing.py tests/test_join_*.py tests/test_gp_se2_join01*.py tests/test_online_*.py tests/test_robotless_online*.py tests/test_se2.py tests/test_obstacle_source_acquisition.py
.venv/bin/python scripts/run_obstacle_source_acquisition02.py --mode freeze --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
# Review/commit/normal push BEFORE server or new qualification calls.
.venv/bin/python scripts/run_obstacle_source_acquisition02.py --mode start --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
# Clean Isaac environment, same as OSA01; stdout/stderr to new logs only.
/home/gpuadmin/isaacsim/python.sh scripts/isaac/obstacle_source02_collect.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
.venv/bin/python scripts/validate_obstacle_source_acquisition02.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Technical source validator uses original raw/history/controller/integration and
geometry helpers, plus scheduler/config parity. Synthetic clock fixtures are
implementation tests only and make no real model/MPC calls. No result exists at
this protocol freeze. Actual qualification and conditional acquisition outcome
will be appended without rewriting historical OSA01.

Pre-qualification regression: 488 tests passed (6.69s), compileall/diff checks pass. No real model/MPC calls from this suite.
