# GP-SE2-REF-03: frozen source-progress selector transfer

**Result: MIXED_TRANSFER_WITH_REGRESSIONS.** The known obstacle stress changes
from B success to C clearance/route failure (minimum clearance 0.0631 to
0.0361 m). Additional 24-event A/B/C success is 23/21/23: C recovers two B
failures, while a separate already-failing rotation acquires a physical angular
acceleration violation relative to B. All 81 planned rollouts completed and
historical controls reproduce bitwise. Controls and additional evidence remain
separate; there is no unqualified safety or navigation improvement claim.

## Question and frozen scope

Does the exact REF-02 source-progress selector recover execution on additional
saved handoffs, and does it introduce goal, route, clearance or motion failures?
This is a bounded diagnostic transfer within the existing development corpus.
It is not a held-out benchmark, a GP experiment, online navigation or deployment
safety evidence. The starting revision is
`22c655ea421c77c1bc7bd7c70da5fec7c9ae7620`.

The primary output is
`data/robotless_gp_se2_ref_03/primary_20260919T141000Z/`.
Execution revision is `343f1d2e57753e2ff99c4197c9f9bd20f6b2e5e7`.
The protocol below was committed before the first primary solve; the executed
outcomes and final authoritative validation are recorded later in this document.

## Source-only cohort, before execution

The original GP-SE2-01 catalog contains 881 events, of which 730 satisfy its
original source/timing/B/goal/future-suffix/route eligibility. The union of the
GP01/GP02/REF01/REF02 selected manifests excludes 10 previously selected events,
leaving 720 additional eligible records. All source integrity checks pass.
Additional prior dataset visual-review usage is recorded, not hidden behind a
claim of complete novelty. Original source data, authoritative validation,
environment exports, results, figures, external MPC and unrelated local edits
remain protected by before/after hashes.

| Stratum | Eligible flagged events | Selected | Shortfall |
|---|---:|---:|---:|
| O: obstacle near | 35 | 6 | 0 |
| R: large rotation | 16 | 6 | 0 |
| P: large mismatch | 95 | 6 | 0 |
| S: original small straight | 378 | 6 | 0 |

Flags overlap, but assignment does not. O uses valid original **future suffix**
footprint edge clearance at most 0.15 m, retaining the original required 0.05 m
and uncertainty rule and excluding an artificial B connector. R uses at least
45 degrees of accumulated absolute wrapped yaw change *inside* the suffix.
Its rotation-dominant priority uses suffix XY arc at most 0.20 m and at least
30 degrees of yaw variation. P uses original `e_perp >= 0.10 m` or reliable
incoming/window direction difference at least 30 degrees. S uses the unchanged
GP01 `A_SMALL_STRAIGHT` predicate.

Assignment order is O, R, P, S. At every pick prefer an unused episode, then an
unused ordered raw pair, then (R only) rotation-dominant, then lexical SHA256 of
`REF03-v1:` plus the full case ID, and finally the ID. Diversity sets start empty
for the additional cohort; controls do not seed them. No replacement, relaxed
threshold or extra stratum fill is allowed. All 24 selected additional events
have distinct episodes and ordered raw pairs. All six selected R events are
rotation-dominant. Repeated underlying raw arrays can still occur across pairs;
these are not independent population samples or an episode train/test split.

Known controls, excluded from new transfer evidence, run first:

1. `episode_014_repeat_01/handoff_026`: REF-02 large-turn reproduction.
2. `episode_001_repeat_01/handoff_002`: REF-02 benign reproduction.
3. `episode_017_repeat_00/handoff_007`: known obstacle/route stress, preserving
   the original directed gate, geometry and ordered crossing predicate.

The frozen manifest/catalog retain every selected and nonselected reason, source
hash, overlap flag, diversity count and source-associated memory discrepancy.

## Frozen inputs, selector and MPC computation

A uses the unchanged original FRESH world rows and official next-row selector.
B uses original `prepare_reference`/REF01 lineage and the same official selector.
C uses byte-identical B arrays and lineage with the exact REF-02 selector:

```
j = original weighted-pose argmin on installed dense rows
q_h = min(s_j + h, s_last), h = 1,...,5
i_h = searchsorted(s, q_h, side="left")
reference = installed_rows[i_h].copy()
```

Stride is exactly 1.0 original-row units; strict float64 search has zero search
tolerance. Each q uses the same unrounded s_j. No new interpolation, accumulated
overshoot, progress memory, suffix recut, native-row substitution or reanchoring
is introduced. Sequential yaw unwrap and authenticated constant-source handling
are unchanged. Original progress denotes row order, not time or distance.

`gp_se2_ref02_reference.py` and `gp_se2_ref02_rollout.py`, eight other numerical
and reference files, official MPC revision/settings and original evaluation
configuration are hash-pinned in `configs/gp_se2_ref03.yaml` and run provenance.
The wrapper only registers tracker construction for partial fault evidence;
all computational methods and the existing per-instance actual-input recorder
remain inherited. MPC computation is unchanged; A/B/C reference selection is
the intervention. HORIZON=5, dt=0.1 s, control rate=10 Hz, original gains/limits.

All candidates are expressed in the original capture frame through its inverse
transform and audited after official world installation. B, physical u_minus
and recorded previous_control remain separate. Original goal, footprint 0.20 m,
required clearance 0.05 m, workspace, route, goal tolerances 0.15 m/15 degrees
and final 0.20 s dwell are unchanged. LightNav intrinsic waypoint_dt is null;
the 0.1..3 s resampling clock is only the existing row-order convention.

## Once-only schedule, outcomes and timing

The frozen 27-event schedule plans 81 independent 3 s rollouts, 2,430 primary MPC
solves and 1,620 selector-only calls. Per event: A executes 30 solves, its actual
30 solve-input poses provide the B/C matched-state probe bank, then B and C each
execute once. All execution uses exact held-command unicycle integration at
60 Hz, yielding 181 states/180 ticks for a complete rollout. No historical path
fallback, VLA update, GP/rigid solve, special stopping rule or retry is allowed.
Scientific failures do not remove later planned cases. Missing computation is
explicit null and does not masquerade as a failed complete rollout or zero error.

The original full success conjunction remains authoritative. Additional spatial
reference checks do not filter methods from this diagnostic rollout. Gate passage
is required where applicable. Physical-command/controller-memory first-step
violations stay in the original success predicate and are separately labeled as
source-associated, without asserting selector causality.

Report B-success/C-failure and A-success/C-failure before recoveries; also inspect
new safety, motion, route and goal reasons when the baseline already failed.
Six requested transition flags deliberately overlap; the eight A/B/C Boolean
patterns are disjoint. Successful-pair quality is conditioned on both methods
passing all original predicates. Nonarrival stays N/A. Event, episode and ordered
raw-pair equal-weight summaries and memberships remain explicit; controls and
additional strata are never pooled into a novel-evidence rate.

Timing separates preparation, environment load, official MPC solves, full rollout,
selector probes, evaluation and reporting/validation. Inclusive components are
labeled and are not summed as exclusive costs. Wall time does not advance the
simulation clock. No online performance claim is planned.

## Evidence and reproducibility

Every case receives all seven required static figures with numeric/source/config
sidecars. B/C shared reference is drawn once, in metres with equal aspect; yaw
wrap cuts are not drawn as physical turns. Actual gate geometry remains visible.
Representative selection includes all new C safety/route regressions, then the
first hash-ordered B-failure/C-success and same-B/C-success-status case in each
additional stratum, plus all controls. Same status does not imply identical
commands or trajectories. The complete index retains all 27 cases and N/A.
The small review ZIP excludes RGB, full dataset/environment and external source.
GUI is not run: these are saved static diagnostics.

Commands (run once per phase; all output writers reject overwrite):

```bash
.venv/bin/python scripts/run_gp_se2_ref03.py prepare --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/run_gp_se2_ref03.py freeze --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/run_gp_se2_ref03.py execute --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/run_gp_se2_ref03.py evaluate --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
MPLCONFIGDIR=/tmp/ref03_mpl .venv/bin/python scripts/plot_gp_se2_ref03.py --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/package_gp_se2_ref03_review.py --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
```

Execution delegates to the existing official isolated MPC environment; no runtime
upgrade or system Python modification. Saved-record validation performs no new
MPC/GP solve. The result sections below record all outcomes, limitations, exact validation
commands, code revisions and the next single experiment.

## Executed result: regressions first

Operational execution completed at `343f1d2e57753e2ff99c4197c9f9bd20f6b2e5e7`: all **81 independent rollouts, 2,430 primary MPC solves and 1,620 selector-only calls** ran once. There were no missing methods, technical method failures, retries or new VLA/GP/rigid experimental solves. Authoritative completion additionally requires the final saved-record validator below.

**Interpretation: MIXED_TRANSFER_WITH_REGRESSIONS.** The known obstacle stress changes from B success to C failure of original clearance and route. In additional transfer, two B failures recover under C, but a different already-failing event introduces a C motion violation relative to B. Zero additional overall-success regressions therefore does not mean zero constraint regressions or proven safety.

### Original-goal and safety regressions

- **Known stress `episode_017_repeat_00/handoff_007`**: A/B/C full success is FAIL/PASS/FAIL. Minimum footprint edge clearance is 0.043270877/0.063068847/0.036086666 m. C misses the required 0.05 m by 0.013913334 m before conservative curve/geometry allowances. There is no physical footprint overlap. A and C cross the gate line outside its valid interval (one outside/endpoint crossing each); B has a valid directed crossing at 0.719965940 s. All three meet terminal goal/dwell and motion conditions. Faster goal entry for C (1.085 s versus B 2.525 s) is **not** successful execution quality because clearance/route fail.

- **Additional R `episode_014_repeat_00/handoff_029`**: A/B/C all fail, for different reasons. B fails yaw/dwell (21.479150 degrees); A/C satisfy goal/dwell but violate the original first-step angular acceleration bound. C reaches 5.088190932 rad/s² versus the unchanged 5 rad/s² limit; raw excess is 0.088190932 rad/s². Original physical omega is 0.016182693, memory omega 0.007363603, and C applies -0.492636401 rad/s. Relative to memory the step remains within the original bound up to the unchanged numerical tolerance; relative to physical u_minus it does not. A shows the same type of violation. This is a preserved source-memory-associated limitation, not proof that the selector uniquely causes it. The event stays a full failure.

- No A-success→C-failure appears. The additional 24 have no new collision/clearance/workspace/route or goal failure under C, but do have the motion failure above relative to B. Across all 81 executions there is no physical overlap, unknown workspace or controller failure. All original position endpoints pass. Gates remain mandatory: goal achievement never overrides a route failure.

### Cohort outcomes and controls

| Scope | Events | A success | B success | C success | Exclusive patterns A/B/C |
|---|---:|---:|---:|---:|---|
| O_OBSTACLE_NEAR | 6 | 6 | 6 | 6 | 111: 6 |
| R_LARGE_ROTATION | 6 | 5 | 4 | 5 | 000: 1, 101: 1, 111: 4 |
| P_LARGE_MISMATCH | 6 | 6 | 5 | 6 | 101: 1, 111: 5 |
| S_BENIGN_STRAIGHT | 6 | 6 | 6 | 6 | 111: 6 |
| ADDITIONAL_TRANSFER | 24 | 23 | 21 | 23 | 000: 1, 101: 2, 111: 21 |
| KNOWN_CONTROLS (separate) | 3 | 2 | 2 | 2 | 010: 1, 101: 1, 111: 1 |

Each additional stratum has six distinct episodes and ordered raw pairs; the combined 24 also have maximum episode/pair degree 1. Event-, episode- and pair-equal means consequently coincide here. Six additional events have a physical-command/controller-memory difference; only the all-fail rotation above yields the source-associated first-step failure. No selected additional event was in the retained prior visual-review example list, but the corpus has a development history and is not held out.

All six REF-02 A/B/C reproduction runs and both prescribed GP-SE2-01 stress A/B runs match reference selections, commands, predictions, states and compared metrics **bitwise**. The original numerical reproduction tolerances were unchanged; no extra audit solve or reproduction retry occurred. Stress C is a new observation, not a reproduced historical result.

| Transition (overlapping, do not sum) | Additional 24 | Known controls 3 |
|---|---:|---:|
| B_success_C_failure | 0 | 1 |
| A_success_C_failure | 0 | 0 |
| A_success_B_failure_C_success | 2 | 1 |
| B_failure_C_success | 2 | 1 |
| A_B_failure_C_success | 0 | 0 |
| all_failure | 1 | 0 |

### Every frozen event

Triples below are A / B / C. P=original full success, F=completed failure. Displayed errors are rounded; full precision remains in the numerical records. Full per-method predicates, route/gate details, memory, motion magnitudes, final commands and N/A are retained in `aggregate/outcome_matrix.csv` and each method’s `metrics.json`. No unavailable result was filled with zero.

| Cohort/stratum | Event | Success A/B/C | Yaw error A/B/C [deg] | Position error A/B/C [m] | Dwell A/B/C |
|---|---|---|---|---|---|
| REPRO_LARGE_TURN | episode_014_repeat_01/handoff_026 | P/F/P | 0.333/38.828/0.255 | 0.0084/0.0256/0.0061 | P/F/P |
| REPRO_BENIGN | episode_001_repeat_01/handoff_002 | P/P/P | 0.000/0.002/0.000 | 0.0001/0.0163/0.0001 | P/P/P |
| KNOWN_OBSTACLE_STRESS | episode_017_repeat_00/handoff_007 | F/P/F | 0.042/2.507/0.044 | 0.0534/0.0203/0.0613 | P/P/P |
| O_OBSTACLE_NEAR | episode_010_repeat_00/handoff_007 | P/P/P | 0.003/0.659/0.002 | 0.0102/0.0434/0.0096 | P/P/P |
| O_OBSTACLE_NEAR | episode_006_repeat_00/handoff_004 | P/P/P | 0.025/0.679/0.025 | 0.0196/0.0292/0.0194 | P/P/P |
| O_OBSTACLE_NEAR | episode_022_repeat_00/handoff_005 | P/P/P | 0.000/0.000/0.000 | 0.0005/0.0001/0.0001 | P/P/P |
| O_OBSTACLE_NEAR | episode_013_repeat_00/handoff_032 | P/P/P | 0.001/2.113/0.003 | 0.0534/0.0547/0.0516 | P/P/P |
| O_OBSTACLE_NEAR | episode_020_repeat_01/handoff_004 | P/P/P | 0.000/0.000/0.000 | 0.0002/0.0002/0.0002 | P/P/P |
| O_OBSTACLE_NEAR | episode_022_repeat_01/handoff_006 | P/P/P | 0.005/0.596/0.006 | 0.0029/0.0179/0.0047 | P/P/P |
| R_LARGE_ROTATION | episode_012_repeat_00/handoff_027 | P/P/P | 4.442/9.430/4.702 | 0.0215/0.0229/0.0215 | P/P/P |
| R_LARGE_ROTATION | episode_014_repeat_00/handoff_029 | F/F/F | 0.260/21.479/0.230 | 0.0738/0.0289/0.0993 | P/F/P |
| R_LARGE_ROTATION | episode_009_repeat_01/handoff_007 | P/F/P | 0.053/17.999/0.050 | 0.0767/0.0527/0.0899 | P/F/P |
| R_LARGE_ROTATION | episode_011_repeat_00/handoff_008 | P/P/P | 0.023/12.504/0.023 | 0.0032/0.0025/0.0067 | P/P/P |
| R_LARGE_ROTATION | episode_015_repeat_00/handoff_017 | P/P/P | 0.030/12.329/0.027 | 0.0277/0.0101/0.0295 | P/P/P |
| R_LARGE_ROTATION | episode_013_repeat_01/handoff_034 | P/P/P | 0.031/7.794/0.028 | 0.0791/0.0499/0.0941 | P/P/P |
| P_LARGE_MISMATCH | episode_004_repeat_01/handoff_014 | P/P/P | 0.008/0.364/0.005 | 0.0006/0.0003/0.0006 | P/P/P |
| P_LARGE_MISMATCH | episode_001_repeat_01/handoff_018 | P/P/P | 0.000/0.654/0.059 | 0.0033/0.0024/0.0045 | P/P/P |
| P_LARGE_MISMATCH | episode_000_repeat_01/handoff_025 | P/P/P | 0.008/0.001/0.008 | 0.0000/0.0001/0.0000 | P/P/P |
| P_LARGE_MISMATCH | episode_002_repeat_00/handoff_024 | P/P/P | 0.015/0.000/0.013 | 0.0058/0.0000/0.0058 | P/P/P |
| P_LARGE_MISMATCH | episode_026_repeat_00/handoff_010 | P/P/P | 0.000/0.019/0.000 | 0.0000/0.0018/0.0000 | P/P/P |
| P_LARGE_MISMATCH | episode_018_repeat_01/handoff_006 | P/F/P | 0.002/0.082/0.002 | 0.0002/0.0816/0.0012 | P/F/P |
| S_BENIGN_STRAIGHT | episode_029_repeat_01/handoff_001 | P/P/P | 0.000/0.021/0.000 | 0.0017/0.0163/0.0017 | P/P/P |
| S_BENIGN_STRAIGHT | episode_003_repeat_00/handoff_009 | P/P/P | 0.000/0.001/0.000 | 0.0032/0.0140/0.0032 | P/P/P |
| S_BENIGN_STRAIGHT | episode_027_repeat_00/handoff_005 | P/P/P | 0.000/0.005/0.000 | 0.0024/0.0024/0.0025 | P/P/P |
| S_BENIGN_STRAIGHT | episode_004_repeat_00/handoff_006 | P/P/P | 0.000/0.000/0.000 | 0.0042/0.0062/0.0042 | P/P/P |
| S_BENIGN_STRAIGHT | episode_029_repeat_00/handoff_003 | P/P/P | 0.000/0.002/0.000 | 0.0046/0.0062/0.0046 | P/P/P |
| S_BENIGN_STRAIGHT | episode_010_repeat_01/handoff_000 | P/P/P | 0.008/0.374/0.008 | 0.0084/0.0197/0.0085 | P/P/P |

### Recoveries and quality tradeoffs

- R `episode_009_repeat_01/handoff_007`: B yaw/dwell failure recovers. B/C final yaw error 17.998756/0.049984 degrees; C goal-entry time 1.02 s. Matched-state B/C final-goal inclusion is 1.2/0.5 s. Closed-loop B never selects the final goal in its five-row horizon, whereas C does at 0.5 s and keeps it thereafter.

- P `episode_018_repeat_01/handoff_006`: B enters the goal only at 2.82 s, leaving achieved final sampled dwell 0.18 s, below 0.20 s. C enters at 1.72 s and dwells 1.28 s. B/C closed-loop goal inclusion is 2.9/1.1 s; matched-state inclusion 1.8/1.1 s. The fixed 3 s horizon is unchanged.

The 21 additional B/C pairs that both fully succeed have mean C-minus-B goal-entry time -0.816667 s, yaw error -2.030347 degrees, position error -0.001289 m, linear command TV +0.059418 m/s, angular TV +0.148864 rad/s, minimum clearance -0.001202 m, and path length +0.006399 m. These descriptive tradeoffs are not a combined navigation score. Failed pairs have separate endpoint diagnostics and no quality contribution.

Among the 23 additional A/C pairs that both fully succeed, mean C-minus-A goal-entry time is -0.007609 s and mean C-minus-A terminal position error is +0.001504 m. A and C succeed on the same 23 additional events. Both B-to-C recoveries already succeed under A, so this experiment observes recovery of two dense-row-step failures without an additional full-success event beyond Native.

| Additional stratum | Both B/C success | Mean goal-time delta [s] | Linear TV delta [m/s] | Angular TV delta [rad/s] | Min-clearance delta [m] |
|---|---:|---:|---:|---:|---:|
| O_OBSTACLE_NEAR | 6 | -0.727500 | 0.064213 | 0.251588 | 0.000096 |
| R_LARGE_ROTATION | 4 | -1.440000 | -0.018086 | 0.376615 | 0.001232 |
| P_LARGE_MISMATCH | 5 | -0.106000 | -0.013838 | -0.001878 | -0.000305 |
| S_BENIGN_STRAIGHT | 6 | -1.082500 | 0.167338 | 0.019924 | -0.004871 |

### Mechanism and limits

All 810 B/C matched-state pairs preserve the installed dense geometry, nearest index/progress and complete nearest costs exactly. Targets change on 481 pairs across all 27 events. Actual submitted world and controller-local references are independently audited. Across all recorded selection diagnostics, maximum ceil overshoot is 0.241379310 original-row units; five near-tie instances retain the original argmin rule. Such counts are selector calls, not extra events.

The stress case demonstrates the adverse mechanism. At the same t=0 pose, B/C have nearest dense row 0 and original progress 3. B selects progress [3.207, 3.414, 3.621, 3.828, 4.034], C selects [4.034, 5.069, 6.103, 7.138, 8.172]. Their selected-horizon XY arcs are 0.124475/0.625460 m and yaw spans -0.058769/-0.506681 rad. Applied omega differs immediately; C reaches minimum clearance near 0.37 s and crosses outside the valid gate interval. Its final goal enters the horizon at 0.3 s versus B 2.5 s. This links frozen stride, changed targets, changed commands and a corner/route shortcut; it does not yet isolate which finite-horizon prediction segment first becomes unsafe.

Only the separate stress control has an applicable gate; additional 24 do not establish gate-sensitive transfer. Increasing progress is not universally better. No selector tuning or new default installation follows this mixed result. GP-deformed-path semantics and GP between-point motion feasibility remain unresolved.

**Next single experiment:** the same stress event’s B/C selected-target/MPC-prediction/executed-swept-path audit around the directed gate, identifying the earliest unsafe prediction or execution divergence before proposing a selector modification. Do not expand to RAW/rigid/GP comparison until this regression is understood.

### Actual cost and saved evidence

Preparation took 10.137018 s, including preservation hashing 6.551254 s and source-only cohort audit 2.464990 s. Worker process took 14.964786 s; its 2,430 official MPC solves sum to 8.178146 s and selector-only probes to 1.307833 s. These are nested components, not additive costs. Evaluation environment load took 0.044830 s, evaluation 3.991956 s. The disjoint prepare + worker + evaluation-load + evaluation subtotal is 29.138591 s, excluding source/code review, reporting, tests and final validation. No online-latency claim.

Primary evidence is `index.html`, all 193 PNGs with sidecars, `review_bundle.zip`, `aggregate/research_interpretation.json` and complete outcome tables under the run root. The ZIP includes all 27 case overlays and the frozen representative details. Static diagnostic only; no Isaac/GUI runtime claim. Some long primary `boundary_zoom` titles clip at the horizontal image edge; coordinate axes, curves, numeric metadata and case identity remain intact. The aggregate regression image covers additional strata; the known-stress failure is reported separately and leads this report.

### Verification history

The first saved-record preflight exposed a validator-only schema mismatch (`planned_primary_mpc_solves` versus the frozen request’s `primary_mpc_solves`). The failed report is preserved as `verification/preflight_validation.json`. Corrected validator-only lookup and retained all frozen inputs, code and numerical outputs. No new scientific run or MPC retry was performed. `verification/preflight_validation_v2.json` passes with zero errors; final preservation/manifest/ZIP checks are deferred there, so it is **not** authoritative.

Full required suite: **2,102 passed / 19 existing skips**, 104.88 s; no REF-03 skip. `verification/pytest.log` and `pytest_result.json` retain the result. Final compile/diff checks and authoritative validator are recorded below. No shell launcher changed.

### Final authoritative validation and Git

The authoritative `validation.json` passes **7,565,567 checks**, with zero errors
and zero deferred checks, in 20.689802 s. It reconstructs source-only selection,
actual target inputs, exact integration, original acceptance, control reproduction,
regression/quality grouping, plot numbers and review ZIP without another MPC/GP
solve. The earlier failed and successful preflight reports remain historical.
All **41,505** protected historical source/result/environment/figure paths,
both unrelated user configuration hashes and the official checkout are preserved.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
.venv/bin/python scripts/run_gp_se2_ref03.py finalize --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
MPLCONFIGDIR=/tmp/ref03_mpl .venv/bin/python scripts/validate_gp_se2_ref03.py --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
```

Compileall and diff checks pass. No shell launcher changed. The authoritative
validation was run once without `MPLCONFIGDIR`; Matplotlib used a temporary `/tmp`
cache and validation succeeded. The command above specifies the writable cache
for future reproduction. Final source snapshots and artifact hashes are saved;
post-validation code changes are limited to this report and README/append-only
work log. The sole validator correction was captured before final validation.

Operational status: **GP_SE2_REF_03_COMPLETED**.
Research interpretation: **MIXED_TRANSFER_WITH_REGRESSIONS**.
The immutable evaluation summary points to the completed post-analysis
`aggregate/research_interpretation.json`; its original pending-review field is
preserved as evaluation-stage history. The review ZIP includes the completed
interpretation. GUI: **NOT_RUN_STATIC_DIAGNOSTIC**.

Starting SHA: `22c655ea421c77c1bc7bd7c70da5fec7c9ae7620`.
Execution SHA: `343f1d2e57753e2ff99c4197c9f9bd20f6b2e5e7`.
The final report commit and normal push are recorded in ignored
`git_completion.json` after the commit, avoiding a self-referential SHA in this
file. No generated arrays, PNG/ZIP, environment, external source or cache is
committed. The scientific result is mixed, not a research-success designation.
