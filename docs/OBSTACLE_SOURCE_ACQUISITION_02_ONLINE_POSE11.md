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

## Corrected qualification result and Phase B pre-execution freeze

Pacing correction commit **`e539a03f300c2ec070e52040831795c837ce35b8`** was
pushed before starting the official server or qualification. Exactly one real
OFF technical episode completed three post-bootstrap handoffs. Saved validator
passes; 266 states/265 exact integration intervals, maximum reconstruction error0;
18 captures at every15 ticks, every controller submission at6-tick cadence.
No burst step; minimum actual wall step .016774260 s. No safety abort.

| Request | RTT s | Request-local RTF | Gate |
|---|---:|---:|---|
| C0 bootstrap | .580370095 | .854139228 | context; within bounds |
| C1 | .278702692 | .990677358 | PASS |
| C2 | .293023314 | .990952774 | PASS |
| C3 | .306310637 | .989739785 | PASS |

Max stall .210615059s PASS. Whole-episode RTF **.721768711** is disclosed:
blocking render lowers wall throughput once catch-up is disabled. This is not
60 wall steps per second through blocking calls, nor post-hoc clock correction.
The nominal 60/10/4 rates refer to the unchanged simulation clock. The original
request-local metric is unchanged and no bad window is omitted. Native full-
episode history semantics persist; new actual RGB is not forced to old hashes.
Technical4 terminal+14 buffer requests;31 MPC submissions/31 saved results;
saved solve wall .154325614s. One separate official server synthetic warmup.
PhaseA0. No optimizer. Technical collector wall16.078964879s excluding Isaac
startup. Original OSA01 files and external MPC/checkpoint remain unchanged.

PhaseB eligibility now established. Separate run-local `phaseB/protocol.json`,
`episode_schedule.json`, config and freeze bind exactly **REPEAT_00, REPEAT_01**.
Start `[19.20222765555384,24.823333918784805,-1.5689742328041627]`; plane is original
POSE11 `[19.20312073159454,24.423334915767065,-1.5689742328041627]`. Same fixed cart
transform, instruction and nominal scheduling. Only maximum handoffs1/active4s
replace the qualification limits; .10s postroll is unchanged. No history cap or
new bootstrap was introduced. The live history may differ from paired H8 and
its resulting FRESH must independently qualify.

An independent official session/reset and execution state is used per repetition.
Both run regardless of first scientific outcome. The dynamic cart hook activates
USD visibility and oracle occupancy before the same scheduled render, records
both host times/state, and refuses queued pre-reveal or later substitute frames
as FRESH. All guards remain abort-only. Saved validation independently checks
reveal plane/cadence, raw/wire/anchor, dynamic guard queries, both clock domains,
actual physical u_minus versus worker memory at command application, and complete
raw paths without trimming. B-memory uses the existing conservative memory-at-cut
helper from the delay audit; ambiguity is unavailable, never reset/copied.

PhaseB adds no model/controller changes to the qualified loop. Original policy,
thresholds and first-repetition selection are frozen before its first model call.
The initial synthetic test's broad `qualified` text check incorrectly matched
the required Phase0 precondition; it was replaced before execution with AST
verification that the repetition loop contains no early break. No scientific
outcome or input was involved. Focused120 tests pass, no real model/MPC calls.

```bash
.venv/bin/python scripts/run_obstacle_source02_online.py --mode prepare --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
.venv/bin/python scripts/run_obstacle_source02_online.py --mode freeze --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
# Review, commit, normal push before the two repetitions:
/home/gpuadmin/isaacsim/python.sh scripts/isaac/obstacle_source02_online.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
.venv/bin/python scripts/validate_obstacle_source02_online.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
```

## Final saved result — neither source meets the frozen OLD obstruction gate

**ONLINE_POSE11_SOURCE_NOT_QUALIFIED**, representative **N/A**, accepted sources
**0/2**. Both required genuine online repetitions ran exactly once after PhaseB
freeze **`de6ab7a00b5588f847eae464efc609264f9f4687`** was pushed. Both produced a
safe turning, non-STOP raw FRESH, continued actual OLD execution during inference,
and recorded a timing-qualified moving B with enough remaining FRESH. The only
failed eligibility item in both is **`B_OBSTACLE_RELEVANT`** (the gate label's B
means gate B, not robot boundary B): the actual stored OLD future remains safe
with cart occupancy. We do not extrapolate OLD, re-anchor it, change the start,
replace POSE11, or rerun a prediction to make this pass.

The frozen 0.40 m backward start matters. OLD was generated at that earlier
observation and has only 1.351821 m of returned XY arc. Its finite world path ends
before the cart: edge clearance with hypothetical cart present is **+0.273959 m**,
above the unchanged .05 m requirement. OSA01's paired OFF path was anchored at
POSE11 itself and had **−0.125215 m** clearance with cart; that historical result
is unchanged. An online OLD anchored farther back is a different world path.
A future extrapolation toward cart is not a returned OLD segment.

### Measured online records

All clearances are footprint-edge metres, radius .20 m, required margin .05 m,
with the unchanged direct environment/swept checker and its allowances. No raw
prefix is removed. Positive local lateral is LEFT; RIGHT wording did not establish
side-language compliance. FRESH stops short of the cart front plane and no complete
bypass is claimed. The actual FRESH postroll is only .10 s.

| Quantity | REPEAT_00 | REPEAT_01 |
|---|---:|---:|
| OLD non-STOP / straight / safe before reveal | PASS | PASS |
| OLD raw XY arc [m] | 1.351820588 | 1.351820588 |
| OLD pre-reveal clearance [m] | 1.024957035 | 1.024957035 |
| OLD hypothetical cart-ON clearance [m] | **.273958726** | **.273958726** |
| OLD obstruction gate | **FAIL** | **FAIL** |
| FRESH whole raw clearance [m] | .157818823 | .149120888 |
| FRESH non-STOP / positive progress / whole-path safe | PASS | PASS |
| FRESH forward progress [m] | 1.210529445 | 1.210529438 |
| FRESH maximum local lateral [m, LEFT] | .584514499 | .584514499 |
| FRESH maximum yaw magnitude [deg] | 29.919087 | 29.919087 |
| FRESH XY arc [m] | 1.348502485 | 1.348502485 |
| OLD→FRESH interior separation [m] | .533550260 | .530257906 |
| Reliable tangent difference [deg] | 19.603912 | 19.603913 |
| Reliable pose-yaw difference [deg] | 19.404949 | 19.661916 |
| Actual observation→B travel [m] | .293333349 | .293333349 |
| B edge clearance [m] | 1.029186438 | 1.019186906 |
| Remaining raw rows / XY arc at B | 8 / 1.047797574 m | 8 / 1.047797574 m |
| FRESH request-local RTF | .991010591 | .990905369 |
| Maximum episode loop stall [s] | .192278178 | .169465806 |
| Cart pixels in actual triggering RGB | 4111 | 3986 |
| Actual executed-prefix clearance lower bound [m] | .949232252 | .939232985 |
| Safety abort / catch-up burst | none / 0 | none / 0 |
| Final source qualified | **NO** | **NO** |

B in Isaac world `[x m, y m, yaw rad]`:

- REPEAT_00: `[19.20358778270297, 24.07666844683044, -1.568971117917399]`.
- REPEAT_01: `[19.20360598068249, 24.06666846238167, -1.5689710699553299]`.

Physical incoming command `[v m/s, omega rad/s]` is respectively
`[.8, .000014019208269887136]` and `[.8, .00001412082965312196]`.
Actual controller previous-control memory at B is respectively
`[.8, .5000140282874762]` and `[.8, .5000141299088592]`. The worker has completed
its first FRESH solve by application; that state is distinct from the command
physically active just before application. Both are preserved, along with the
first FRESH solve input/result/application identity; no memory reset or copy is
used. These measurements are not an assessment of handoff motion continuity.

### Separate clocks and inference overlap

| Timestamp / duration | REPEAT_00 | REPEAT_01 |
|---|---:|---:|
| FRESH observation / reveal simulation time [s] | 1.783333426 | 1.783333426 |
| Ready-seen simulation time [s] | 2.050000107 | 2.050000107 |
| Install simulation time [s] | 2.050000107 | 2.050000107 |
| First FRESH application / B simulation time [s] | 2.150000112 | 2.150000112 |
| Observation→B simulation duration [s] | .366666686 | .366666686 |
| Observation→request host duration [s] | .101860164 | .090477439 |
| Request→full receipt host RTT [s] | .265254772 | .249977953 |
| Observation→B host duration [s] | .583065078 | .598181024 |

Host monotonic/UTC send, receipt, ready-seen, install and application records
are retained individually in `phaseB/validation.json` and the compact tracked
[summary](results/obstacle_source_acquisition_02.json). Receipt and send have no
invented simulation timestamp. Ready and install share one simulation tick but
have different host timestamps. OLD commands, not a frozen robot, cover the
observation→B interval. The first post-reveal live JPEG is used; queued pre-reveal
frames are not substituted. Both episodes end normally after the frozen postroll.

### Source identity and compute

Raw local FRESH SHA256 in both repetitions is
`6893d5f9791c076037216647bba1b589050787cf703640d9440298b33490dfe2`, identical to
historical OSA01 POSE11_ON. This is an observed result from new live RGB/sessions,
not replayed model output or a forced match. World arrays differ with actual
observation poses, so world clearance differs too. Raw OLD SHA256 is
`6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae` in both episodes.
Official LightNav source remains `c6f40e3220edbf7011e4f17eaf2c865416737d4d`,
checkpoint `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, task VLN,
temperature0/top_p1/top_k0/traj_top1=0 verified from actual server environment.
Official MPC hash remains
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
No external source/checkpoint, raw result, scene, camera, light, instruction or
geometric threshold was modified. 1,722 preservation hashes pass; the authorized
collector scheduling change is recorded separately with original source copy.
Unrelated Stage0 local edits remain unstaged and hash-identical.

| Calls / compute | Technical OFF | PhaseB REPEAT_00 | PhaseB REPEAT_01 |
|---|---:|---:|---:|
| Terminal LightNav predictions | 4 | 2 (OLD+FRESH) | 2 (OLD+FRESH) |
| Buffer-only requests | 14 | 7 | 7 |
| MPC accepted / saved completions | 31 / 31 | 12 / 12 | 13 / 13 |
| Saved official MPC wall [s] | .154325614 | .095256179 | .051217149 |

Total 8 terminal predictions (4 technical, 4 scientific), 28 buffers, one separately
counted official synthetic server warmup; PhaseA rerun0. Total56 MPC solves, all56
saved, .300798942s summed solver wall. Collector wall excluding SimulationApp
startup: technical16.078964879s, both PhaseB episodes together15.538959193s.
Warmup/model loading/Isaac startup are not included in these collector durations.
GP/rigid/graph/splice/reconciliation0. Tests, saved validators and reporting add
zero real model/MPC calls. No retries; model server stopped after collection.

### Validation, review and exact completion commands

Saved-only technical and online validators recompute source hashes, genuine
history/wire bytes, observation anchoring, exact integration, dynamic cart guard,
first application B, physical command versus memory, whole-raw geometry, future,
RTF/stall and scheduler invariants. Final independent report validator reruns
those checkers, compares CSV/JSON/raw-world/plot sidecars and ZIP bytes. All PASS;
artifact integrity PASS does not mean source qualification PASS. Final relevant
regression **519 passed (6.79s)**, with no real model/MPC calls; compileall and
`git diff --check` pass. Both world overlays, actual RGB, timing and gate figures
were visually reviewed; figure legends do not conceal the trajectories.

Run root: `data/obstacle_source_acquisition_02/primary_20260923T090000Z/`.

- `diagnosis/diagnosis.json`: saved-only OSA01 pacing audit.
- `phase0/validation.json`: corrected technical qualification.
- `phaseB/episodes/REPEAT_00`, `REPEAT_01`: immutable live RGB, raw outputs,
  transformed arrays, applied commands, exact states, timing/history/worker logs.
- `phaseB/validation.json`: all source gate values and actual B/command identities.
- `source_bundle/manifest.json`: **entries empty, representative null, count0**.
  Failed-source records are preserved, not exported as qualified candidates.
- `aggregate/summary.json`, `aggregate/episodes.csv`: numeric results.
- `validation_independent.json`: saved recomputation and presentation parity.
- [Static index](../data/obstacle_source_acquisition_02/primary_20260923T090000Z/index.html)
  and [review ZIP](../data/obstacle_source_acquisition_02/primary_20260923T090000Z/review_bundle.zip).
  Eight PNGs with numeric/source-hash sidecars under `review/`.

For both Isaac collectors, the actual invocation used the same clean environment:

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 /home/gpuadmin/isaacsim/python.sh scripts/isaac/obstacle_source02_collect.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
# After the separately pushed PhaseB freeze, same env prefix:
/home/gpuadmin/isaacsim/python.sh scripts/isaac/obstacle_source02_online.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
.venv/bin/python scripts/validate_obstacle_source02_online.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
.venv/bin/python scripts/run_obstacle_source_acquisition02.py --mode stop --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
MPLCONFIGDIR=/tmp/osa02_mpl OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/report_obstacle_source_acquisition02.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_obstacle_source02_report.py --run data/obstacle_source_acquisition_02/primary_20260923T090000Z --output data/obstacle_source_acquisition_02/primary_20260923T090000Z/validation_independent.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q tests/test_obstacle_source02_report.py tests/test_obstacle_source_online.py tests/test_obstacle_source_pacing.py tests/test_join_*.py tests/test_gp_se2_join01*.py tests/test_online_*.py tests/test_robotless_online*.py tests/test_se2.py tests/test_obstacle_source_acquisition.py tests/test_handoff_delay_attribution.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Collection/report outputs are exclusive-create. To revalidate existing artifacts,
use the report validator without `--output`; this performs no new execution.
Do not rerun collectors or overwrite the frozen run. Raw/PNG/ZIP stay ignored;
only code/tests/docs and the small numeric summary are tracked.

### Interpretation and remaining uncertainty

**Repository-confirmed facts:** the saved absolute pacing loop had one step per
outer iteration, but recovered rendering deadline debt through 2–3ms consecutive
steps. New scheduling removes these bursts, preserves simulation cadence, and
passes every required request/stall gate. Fine-grained renderer/GPU delay cause
is not isolated; original sleep duration was not stored. Both new online FRESH
paths are safe/turning/progressing and applied at moving B. Neither actual finite
OLD path is obstructed by the revealed cart under the frozen source rule.

**Research interpretation:** the previously observed POSE11 local response
survives genuine sudden reveal and moving-B acquisition in these two attempts.
The full requested obstacle-induced representative source is nevertheless absent,
because OLD obstruction and safe-FRESH conditions were not simultaneously met.
This is a source eligibility limitation, not evidence that FRESH again became
unsafe or that pacing failed. The initial observation/finite OLD extent explains
the discrepancy from the source-only pair without modifying either result.

**Not demonstrated:** reconciliation benefit, complete obstacle traversal,
controller handoff continuity, graph superiority, full navigation improvement,
real-robot feasibility or general LightNav capability. No optimization or next
stage ran. The one remaining uncertainty is whether a separately frozen genuine
acquisition can jointly capture an actually obstructed finite OLD future and this
safe turning FRESH at timing-qualified moving B. No new condition was implemented.
