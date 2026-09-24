# OSA03_FRESH_TRACKABILITY_CONTROL_01

## Frozen question and scope

How difficult is the exact same OSA03 FRESH to execute from its original observation state, compared with the saved delayed B-start Native continuation? This is an offline same-FRESH observation-to-application sensitivity diagnostic, not an additive decomposition or a recoverable reconciliation-cost bound.

Starting revision: `0842f5583d9295b436a47c7da9d26ef6ed054143`. Source: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`, representative `REPEAT_00`. Comparator: `data/osa03_native_continuation_01/primary_20260923T103000Z/`, revalidated saved-only, never re-solved. New unique run: `data/osa03_fresh_trackability_control_01/primary_20260924T012000Z/`.

Original FRESH SHA256: `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`. Original OLD: `6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae`. World coordinates remain observation-anchored Isaac Z-up SE(2), metres/radians, yaw CCW; body x forward, y left. Neither raw array changes. No crops, re-anchoring, resampling, selector intervention, model update, or optimization.

## Saved observation phase, before execution

Observation is state/tick 75, simulation time `1.2833334002643824`; pose `[19.203242163778583,24.356668352984542,-1.5689752526514715]`. Both physical incoming command and independently reconstructed controller memory happen to equal `[0.6000000287746985,-4.733615847732774e-06]`; their provenance remains separate. This differs from B's incoming .8 m/s and already-advanced .5 rad/s controller memory.

OLD solve `REPEAT_00_solve_000002` was submitted at tick 72 and applied at tick 73. Its result was already seen before observation. There is no pending OLD solve at the observation cut. The next historical OLD submit at tick 78 is after the cut, and becomes the first counterfactual FRESH submit; it is not replayed as an obsolete pending solve. Tick 75 is off-grid, so no solve is permitted there.

A fresh independent official tracker is constructed, OLD is installed, and the last accepted OLD controller state is restored before simulated time begins. Official FRESH installation increments generation 2 to 3 and preserves this nonzero memory. No numerical solve is used to restore it. Official generation rejection of stale OLD results is retained and tested; no result exists to discard in this source. Missing or ambiguous observation state fails closed.

The unchanged official source hash is `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`. Nearest/+1 selection, Q/R, limits, horizon, asynchronous worker, hold/failure policy, and exact held-command integration are reused. Original absolute control ticks modulo six and source float32-resolved `dt=0.01666666753590107` remain fixed.

## Pre-execution protocol

Exactly one new scientific observation-start rollout after the code/config/tests freeze commit is pushed. No B-start solves. Availability is counterfactually immediate at observation; physical OLD command persists until an actual new FRESH result is received and applied. Original asynchronous MPC compute/IPC delay is measured, not deleted. The newly constructed solver may have first-call construction/cache costs, unlike an already warm live worker; no extra warmup solve is invented.

The observation cap is 180 integration intervals from **availability**, `3.0000001564621925 s`. It is not an intrinsic duration of the LightNav rows. Two origins are stored: observation availability and first FRESH command application. Primary paired .30/.90 s metrics use the latter. Its complete 3 s window is N/A if unavailable. Each achieved full horizon is labelled, and the saved B trajectory is also evaluated over a common execution-prefix exposure, without another controller solve.

Safety reuses the direct Hospital/cart checker and abort-only guard, physical radius .20 m, required edge clearance .05 m, original uncertainty/curve allowance. Attachment reuses original-FRESH forward-only polyline projection, shortest-angle yaw, .10 m/15 degrees for a full following .30 s of samples. No continuous-time attachment proof or completed obstacle bypass claim.

Spatial descriptors include row0 distance/bearing/yaw, reliable first-chord direction, row spacings/yaw increments, meaningful XY curvature, lateral amplitude and arc. Untimed rows are never converted into required rates or accelerations.

Signed B-minus-observation contrasts cover initial geometry, early growth, .30/.90 s AUCs, attachment time/progress, clearance and commands. No scalar score or percentages of intrinsic versus latency cost. Disagreement between metrics is reported as interaction. Reconciliation would start at B and cannot undo preceding motion; future intent-preservation constraints would need to prevent silently replacing upstream intent or repairing arbitrary source infeasibility.

## Commands

```bash
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode prepare
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode preflight
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_osa03_trackability.py tests/test_osa03_native.py tests/test_online_mpc_adapter.py
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode freeze
# Commit and normal push, then exactly once:
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode execute
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode validate
.venv/bin/python scripts/report_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z
```

Results will be appended after the one frozen execution. Tests using synthetic fixtures are implementation checks only. Source-state, baseline validation and restoration preflight use saved records and zero numerical MPC solves. LightNav, RGB, Isaac, GP, rigid, graph, splice and reconciliation calls remain zero.

## Repository-confirmed results

Execution freeze, pushed before the single rollout: `21bd4f6a53f0e6afd88a83274450ac279e491b08`. The run completed its availability-origin 180 intervals with no safety abort, controller error, hold timeout, stale result or busy rejection. The unchanged Native tracker executed 30 numerical MPC solves. New B-start solves, LightNav, RGB, Isaac, and optimization calls are all **0**. No retry occurred.

### Actual counterfactual control phase

| Event | Absolute tick | Simulation time (s) | Availability-relative time (s) |
|---|---:|---:|---:|
| Original FRESH observation / immediate availability | 75 | 1.2833334003 | 0 |
| First legal FRESH submit | 78 | 1.3333334029 | 0.0500000026 |
| First FRESH command physically applied | 82 | 1.4000000730 | 0.1166666728 |

The first solve input was `[19.203296792427953,24.326668399719377,-1.5689754893322765]`, with previous control `[0.6000000287746985,-4.733615847732774e-06]`. First FRESH applied command was `[0.8,0.4999952731986477]`. The physical switch delta was `[+0.19999997122530155,+0.5000000068144954]`. Execution-origin pose was `[19.203369619582023,24.28666846201238,-1.568975804906683]`. The robot travelled **0.0700000070 m** while waiting for the next legal submit and its result.

First solve host times: input `140492.341543924`, worker submit `140492.341735070`, solve start/end `140492.341970008` / `140492.389731413`, worker receipt `140492.391456549`, main-loop seen/application `140492.408425214`. These are new-process host monotonic timestamps, not clocks to subtract from the historical source's host times. Availability is a counterfactual simulation event; setup wall time is excluded from simulated waiting. The first solve took **47.752507 ms**, including cold first-call costs. No extra warmup solve was performed. This remaining MPC/application latency is measured; the diagnostic removes VLA availability delay, not all controller computation latency.

### Original FRESH spatial compatibility

Actual N=10, supported as generic N in code. Row0 is only **0.0000109648 m** from observation, but its yaw is **18.000774 degrees** from the robot's heading. Its tiny-displacement bearing is 31.589 degrees and is not a robust navigation-direction conclusion. The first reliable chord (row0 to row1, about .150756 m) is **29.992418 degrees** from the robot heading. Row1 yaw is about 30.000670 degrees. Successive row spacings are .149754–.150770 m, and the first yaw increment is 11.999896 degrees; later increments are small. Maximum local lateral is **.676244259 m**, total XY arc **1.353245520 m**. Three-point XY curvature is small (maximum magnitude about .0048371 /m): this is principally an initial directional mismatch followed by an almost straight diagonal path, not a timed curvature-demand calculation. All exact spacings, increments and curvature values are in `spatial_descriptors.json`.

### Same-FRESH paired execution results

Primary time zero is **first FRESH command application** for both trajectories. Positive contrasts below always mean B-start minus observation-start, without absolute-value conversion.

| Metric | Observation-start | Saved B-start | B minus observation |
|---|---:|---:|---:|
| Initial position error (m) | .034992303 | .106648569 | +.071656266 |
| Initial yaw error (deg) | 22.825669 | 30.000556 | +7.174887 |
| Maximum position error in first .5 s (m) | .133758332 | .214078141 | +.080319809 |
| First .30 s position AUC (m·s) | .025729553 | .047738553 | +.022009000 |
| First .90 s position AUC (m·s) | .100928178 | .168221794 | **+.067293615** |
| First .90 s yaw AUC (rad·s) | .151791301 | .191402386 | +.039611086 |
| Sustained attachment time (s) | 1.000000052 | 1.466666743 | **+.466666691** |
| FRESH fractional row at attachment (last row = 9) | 5.581671 | 8.722075 | +3.140404 |
| Minimum execution clearance lower bound (m) | .174906335 | .133806153 | **−.041100181** |
| Angular command TV, achieved horizons (rad/s) | 3.298011613 | 3.622483660 | +.324472047 |
| Maximum absolute omega (rad/s) | 1.499995279 | 1.546038252 | +.046042973 |

Observation-start execution exposure is **2.883333484 s**; saved B-start is **3.000000156 s**. Only the last-horizon descriptors (including TV/clearance) use those explicitly different horizons in this table. The saved B trajectory was additionally truncated *for evaluation only*, not as an input reference or rerun, to the same 173 execution intervals: B angular TV becomes 3.619516614, contrast +.321505001; minimum-clearance contrast stays −.041100181 m. Paired .30/.90 s windows and attachment times are unchanged by this common-exposure check. The observation-start complete execution-origin 3 s result is **N/A**, never padded or extended.

### Window metrics for observation-start

| Origin / achieved window | Mean position (m) | Max position (m) | Position AUC (m·s) | Excess position AUC (m·s) | Yaw AUC (rad·s) | Excess yaw AUC (rad·s) |
|---|---:|---:|---:|---:|---:|---:|
| Execution / .30 s | .085765172 | .123134518 | .025729553 | .001443323 | .105407973 | .029375802 |
| Execution / .90 s | .112142415 | .133758332 | .100928178 | .016641946 | .151791301 | .029375802 |
| Execution / achieved 2.883333484 s | .078243719 | .133758332 | .225602736 | .016962698 | .245440482 | .029375802 |
| Availability / complete 3.000000156 s | .075881349 | .133758332 | .227644060 | .016962698 | .287005481 | .040397537 |

At equal 2.883333484 s execution exposure, saved B position AUC is .379595928 (signed gap **+.153993192 m·s**), and yaw AUC is .329981575 (gap +.084541093 rad·s). These full-prefix differences are descriptive, not a recoverable bound.

Observation-start first tube entry equals sustained attachment onset, **1.000000052 s execution-origin / 1.116666725 s availability-origin**. The complete following .30 s is observed, and it does not exit thereafter. Remaining original FRESH arc at attachment is **.513028471 m**, versus .041848269 m at saved B attachment. Position distance first grows by **.098766029 m**, reaching .133758332 m at **.466666691 s** after first command. Thus immediate availability does not eliminate intrinsic/controller-interface tracking difficulty.

### Safety, endpoint and commands

Raw original FRESH edge clearance remains **.229486890 m**. Observation-start execution's minimum finite dense sample is .174948001 m; the unchanged swept/curve allowance gives **.174906335 m**. The minimum is at the cap (execution time 2.883333484 s, availability time 3.000000156 s), pose `[19.84935308307086,23.141710276398896,-1.045828678411203]`. Cart-relative longitudinal/lateral coordinates are `[-.499964651,+.643309785]` m. Both executions exceed the required **.05 m edge margin**; a reduced clearance is not an unsafe result. This does not claim exact continuous extrema or completed bypass.

Endpoint pose-tube entry occurs at 1.550000081 s. Minimum endpoint distance is .050916257 m at 1.733333424 s. Final error is .054575760 m / .089072755 degrees, final fractional progress 9 with no remaining arc. Unrestricted projection does not move backward. Maximum extension beyond the final tangent plane is .021954005 m. Final command is approximately `[3.39e-9,-.004534898]`: linear motion essentially stops, but the frozen `1e-3` near-zero-both-components descriptor is **false**. This is endpoint settling of an incomplete local bypass, not arrival at the hallway destination.

Linear TV is .999999968 m/s including the physical switch; angular TV is 3.298011613 rad/s. Speed maximum .8 m/s; maximum |omega| 1.499995279 rad/s at execution time .150000008 s. Nominal 10 Hz diagnostics pass unchanged tolerances: max dv/dt 2.000000054 m/s² and domega/dt 5.000000068 rad/s². **Actual application-spacing diagnostic differs:** the next command follows the cold first result after only .050000003 s, so a .500000003 rad/s jump gives **9.999999529 rad/s²**. The first switch itself follows the preceding OLD application by .150000008 s, with actual-interval rates about 1.333333072 m/s² and 3.333333205 rad/s². No physical continuous-acceleration claim is made. The nominal-grid pass must not be described as an actual-application acceleration pass. Saved B's corresponding maximum was about 6 rad/s².

Actual nearest-row sequence across 30 solves:
`0,0,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9,9,9,9,9,9,9,9,9,9,9`.
Each selected reference is the unchanged official next-five-row rule with endpoint repetition. Per-solve input pose, previous command, selection rows, prediction, result and receipt/application timing are saved in `rollout.json`.

Compute: new controller solve time sum **152.355585 ms**, mean **5.078519 ms**, first/max **47.752507 ms**; one rollout wall time **3.112428 s** including worker setup/shutdown. Numerical solve count 30, busy/failure/stale/timeout count 0. Preflight and tests use zero numerical model/MPC solves. No scientific retry, no new B-start solve.

## Research interpretation

The matched-FRESH pattern supports **same-FRESH observation-to-application sensitivity** in this development source: delayed B has larger early position/yaw AUC, later attachment near the endpoint, and smaller still-valid clearance. This remains evidence about the changed state/controller phase under unchanged Native tracking, not an additive latency term.

The observation-start trajectory also separates from FRESH before converging and needs about one execution second to attach. The original 18-degree row0 yaw and 30-degree first reliable chord are compatible with an intrinsic source/controller-interface tracking component. Neither that component nor the delay-sensitive difference has been independently decomposed. Cold first-solve timing and the compressed subsequent application interval additionally limit causal attribution; actual application-rate diagnostics are worse in this control despite lower geometric tracking cost.

**Largest remaining uncertainty:** how much of the measured same-FRESH gap persists under matched asynchronous controller runtime/application timing and a future comparison that starts every method at the same genuine B. This task does not run that comparison or alter its inputs.

## Not demonstrated

No reconciliation benefit, maximum recoverable cost, additive/percentage causal split, graph necessity/superiority, full obstacle traversal, general online navigation improvement, real-robot result, or population effect is demonstrated. Future reconciliation should target additional post-B transition difficulty and preserve original intent, rather than quietly repairing arbitrary upstream FRESH infeasibility.

## Validation and artifacts

Authoritative saved-only validators pass for the sealed OSA03 source, saved B continuation, and new observation control. New validation rechecks exact source hashes, original anchor, observation command/memory, unchanged official source, absolute submit schedule, independent tracker, literal integration, activation/result identity, direct guard, native selections, both metric origins and signed comparisons. It performs zero numerical solves. Historical files and unrelated local stage0 edits retain their hashes.

Pre-execution relevant regression suite: **222 passed, 1 skipped** (new scientific record did not yet exist). Final relevant regression suite: **223 passed**, including the newly available saved-record/figure checks. `compileall` and `git diff --check` pass. No shell launcher changed. The inherited metric flag `reconstruction_only_no_new_rollout` describes the saved metric calculator only; the run-level call ledger correctly records 30 new controller solves.

Local run: `data/osa03_fresh_trackability_control_01/primary_20260924T012000Z/`.

- `observation_phase.json`, `source_manifest.json`, `freeze.json`, `execution_start.json`: inputs, controller reconstruction, code/input hashes.
- `spatial_descriptors.json`: untimed original FRESH geometry.
- `restoration_preflight.json`, `restoration.json`: official install and nonzero memory proof.
- `rollout.json`, `metrics.json`, `validation.json`: immutable new command/state stream, both-origin results and saved-only validation.
- `review/index.html`, `review/world.png`, `review/errors.png`, `review/commands.png`, `review/progress_clearance.png`, `review/paired.png`: static comparison; every PNG has numeric/source-hash JSON. `review/paired.csv`, `review/windows.csv` provide numbers.
- `review_bundle.zip`: static review package.

The world plot uses **cart + .05 m** as the dashed obstacle outline and **.20 m** physical footprint circles. This avoids confusing an already radius-inflated .25 m center-exclusion outline with an additional footprint-edge margin.

Final regression command (no numerical model/MPC solves):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_osa03_trackability.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_online_mpc_adapter.py tests/test_handoff_delay_attribution.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source_pacing.py tests/test_robotless_online.py tests/test_robotless_online_validator.py tests/test_se2.py tests/test_gp_se2_rollout.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

The reused metric calculator retains legacy keys `B_distance_m` and `B_yaw_error_deg`; inside each explicitly named origin these mean the initial pose of that origin, not a substitution of genuine B. Numerical implementation/configuration remained unchanged after the scientific outcome.
