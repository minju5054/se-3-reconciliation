# GP-SE2-DIAG-05: quarter motion inequality-only ablation

Question: does retaining the original midpoint lateral equality and adding only
quarter motion inequalities remove DIAG-04's numerical regression, and recover
an independently full-valid hard-handoff GP candidate? If convergence returns,
does between-point lateral velocity remain the only full-check failure?

Starting revision: `070f2ca5cff44fcd3027e86fd198e8ebb1addd57`.
Historical baseline: `data/robotless_gp_se2_diag_04/primary_20260920T015400Z/`.
Its execution revision is `e3776fa30a161ff361ba5f1d321a8caa788e729d`;
its final report revision is the starting revision above. G0/G1 are saved
historical results, not new solves. Original source and the preceding saved
interpolation audit remain GP-SE2-02 and GP-SE2-DIAG-03 respectively.

## Frozen single change

DIAG-03 found collocation-pass/interior-fail paths with consistent interpolation
and derivatives. DIAG-04 added quarter lateral equalities and motion inequalities
together; full recovery was absent and SLSQP numerical failures appeared.
This ablation separates those additions. It changes no trajectory representation,
objective, initialization, physical tolerance, environment query, or controller.

| Formulation | Equality rows | M2 inequality rows | M3 inequality rows | Evidence |
|---|---:|---:|---:|---|
| G0 original | 30 | 813 | 903 | Historical DIAG-04 |
| G1 quarter full | 90 | 1293 | 1383 | Historical DIAG-04 |
| G2 quarter inequality only | 30 | 1293 | 1383 | Five new starts |

All use 150 variables, 31 support states, 3 s horizon, 0.1 s support spacing,
fixed B and initial body twist. Poses are T_world_agent, with the original
right-local chart; body twist is vee(X^-1 dX/dt). Units are metres, radians,
seconds, m/s, rad/s, m/s² and rad/s². Body acceleration is the time derivative
of twist components. No re-anchoring, support change, intrinsic LightNav time
interpretation, terminal-stop constraint, slack, or soft lateral penalty occurs.

`InequalityOnlyView` delegates objective/equality to GPProblem literally and
appends only the original DIAG-04 `quarter_motion_values` inequality outputs.
At u=.25,.75 of each interval, eight signed margins enforce forward speed,
angular speed, linear body acceleration and angular acceleration. There is no
quarter lateral equality, a_y bound, new goal or environmental row.

The new derivative wrapper directly returns the original objective gradient
and equality Jacobian. It reuses the frozen quarter AD implementation for the
inequality suffix. Its internal graph also calculates quarter lateral outputs;
these unused values and Jacobian rows are never exposed to SLSQP. No FD fallback
or derivative guard removal is allowed. Original numerical modules are unchanged.

## Schedule and verification gate

Exactly one new G2 solve per row, sequentially:

1. `episode_013_repeat_01/handoff_024`, M2, I0_FRESH.
2. Same hard event, M2, I1_DECEL.
3. Same hard event, M3, I0_FRESH.
4. Same hard event, M3, I1_DECEL.
5. `episode_001_repeat_01/handoff_002`, M3, I1_DECEL (benign control).

Initialization files must be byte-identical to both DIAG-04 pairs and the
original GP-SE2-02 saved seed. Historical final vectors are verification inputs
only. The four hard comparisons are one handoff, not four independent events.
No retry, additional start, G0/G1 reoptimization, VLA, MPC, rollout or GUI is used.

The exact saved DIAG-04 fifteen-point manifest and three directions are reused:
five seeds, five historical latest vectors and five fixed perturbations. Required
literal parity covers objective, equality, equality Jacobian, objective gradient,
original inequality prefix and its Jacobian. Appended values are compared with
NumPy interpolation. Family-wise directional checks use steps 2e-4, 2e-5, 2e-6;
both finest steps must pass. Frozen absolute/relative tolerances remain 1e-8/1e-10
for primal values, 2e-4/2e-5 for objective derivatives, 2e-5/2e-5 for constraint
derivatives. Branch-aware original environment verification is retained.
Failure blocks all primary solves; thresholds cannot be relaxed after inspection.

Prepared SLSQP settings: 200 iterations, ftol=1e-7, 30 s per start, supplied
objective gradient and both Jacobians, CPU float64 and single-thread BLAS.
DIAG-04's `run_refined` is imported unchanged. Graph construction, first-call
compilation, prepared solve and post-check costs are recorded separately.
Historical timing is context rather than a newly paired runtime measurement.

## Candidate, rank and analysis policy

The unchanged solver helper retains initial/latest and actual grid-feasible
callbacks. It selects the lowest objective among grid- and original-dense-valid
candidates, breaking ties by source label. Independent full acceptance does not
silently substitute a different candidate. `check_full_candidate` receives the
original base GPProblem. A new grid-pass path remains rejected if original
lateral, speed, acceleration, goal, route or environmental checks fail.

Every retained vector receives original full checks and DIAG-03 interval
analysis; duplicated vectors can share calculations while source labels remain.
Latest and selected are separate. A null selected vector is never filled with RAW.
The latest outcome classes distinguish grid failure, lateral-only full rejection,
other full rejection and full validity. Lateral-only requires all other full-grid
flags, original dense inequalities and pose/body derivative identity to pass.

At initial/latest/selected, G2 and G0 equality Jacobians must be literally equal
at the SAME vector (30×150). Raw and column-scaled SVD use DIAG-04's unchanged
protocol, with no scaling sent to the solver. G0/G2 final vectors from different
optimizations need not have identical spectra. G1 has 90 equality rows.

Quarter margin diagnostics use `abs(margin) <= original inequality tolerance`
for active-within-tolerance counts and `margin <= max(1e-6,10*inequality_tolerance)`
for near-active counts, with violating and feasible-near-active counts separated.
These are descriptive saved-margin counts, not observed SLSQP internal active sets.

Benign numerical objective improvement must exceed
`max(10*ftol,1e-8*max(1,abs(J_seed)))`; raw differences are retained. Seed retention,
solver convergence, numerical objective improvement and full feasibility are
separate. None is navigation evidence. Supplemental sampled extrema are not a
continuous-time feasibility certificate. No next refinement is implemented here.

## Reproduction commands

The runner refuses existing outputs and any second solve-phase invocation.
Use the run path recorded in the completed-results section:

```bash
.venv/bin/python scripts/run_gp_se2_diag05.py prepare --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py derivatives --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py solve --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py analyze --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py validate --run "$run"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

The saved-record validator recomputes checks, interval diagnostics, equality
parity/rank, retention, tables and plot sidecars, without new optimization. All
fifteen historical/new results appear in the static index. Every PNG has numeric
and source/config hash metadata; plans are labelled **PLAN ONLY / NOT EXECUTED**.
Raw data, environment exports, external MPC, historical solver arrays and
unrelated local edits are preserved; only new code/tests/docs are committed.

## Completed primary results

Operational status: **GP_SE2_DIAG_05_COMPLETED_WITH_LIMITATIONS**. Research interpretation: **MIXED_INEQUALITY_ONLY_RESULT**.

Execution/freeze revision: `b84d81598591eb3fc9ba017a0c251cc59bc528df`. Run: `data/robotless_gp_se2_diag_05/primary_20260920T064500Z/`. The final report commit follows this execution revision; its exact SHA is recorded in the completion metadata and final Git report.

Exactly **5/5 new G2 SLSQP solves**, all status 0, were executed once in the frozen order. G0/G1 were not rerun. Hard full-valid recovery is **0/4 starts** of the single hard event. All four hard G2 outcomes are **NUMERICAL_RECOVERY_OTHER_FAILURE**, not lateral-only rejection. The benign outcome is **BENIGN_OPTIMIZATION_RECOVERED**. New VLA, MPC, rollout and Isaac GUI counts are all zero.

### Source and derivative evidence

All 1,605 preserved historical/core/input/environment files retained their hashes. The five saved initialization files match the original and both DIAG-04 pair members byte-for-byte. The original config SHA256 is `15f3f1580f13d437cff0bf88fba59b8526b209e070692bfaab8367f2784863f6`. Both unrelated Stage-0 config edits are unchanged. No historical numerical module, official MPC or environment was modified.

All fifteen frozen derivative points passed: objective/equality/base-inequality values, objective gradient, equality Jacobian and base inequality-Jacobian prefix have literal parity. Actual dimensions are 150 variables / 30 equalities / 1293 M2 or 1383 M3 inequalities. Added equality, goal and environment row counts are zero; added motion inequalities are exactly 480.

Derivative status: **VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS**. Original environment branch classification/guards were retained. Quarter values and the required two finer FD steps passed every family. Maximum errors over those points/steps are below; scaled error is error divided by the frozen absolute-plus-relative allowance (PASS <= 1). Derivative units follow the original mixed-unit chart; these tolerances do not relax physical bounds.

| Quarter family | Primal absolute error | Directional absolute error | Maximum scaled error |
|---|---:|---:|---:|
| linear_speed_lower | 5.92e-14 | 6.54e-08 | 0.0029 |
| linear_speed_upper | 5.92e-14 | 6.54e-08 | 0.0029 |
| angular_speed_upper | 1.78e-15 | 5.72e-09 | 0.0002 |
| angular_speed_lower | 1.78e-15 | 5.72e-09 | 0.0002 |
| linear_acceleration_upper | 1.57e-12 | 1.74e-06 | 0.0430 |
| linear_acceleration_lower | 1.57e-12 | 1.74e-06 | 0.0430 |
| angular_acceleration_upper | 2.84e-14 | 1.53e-07 | 0.0054 |
| angular_acceleration_lower | 2.84e-14 | 1.53e-07 | 0.0054 |

### All historical and new outcomes

G0/G1 below are historical DIAG-04; only G2 is new. Grid means each formulation’s own solver grid. Dense/full are unchanged original acceptance. J is the latest iterate objective; a rejected lower-cost vector is not an accepted candidate.

| Case/method/seed | Grid | Status / iterations | Grid | Dense | Full latest / selected | J latest | Solve s |
|---|---|---:|---|---|---|---:|---:|
| Hard M2/I0 | G0 | 0 / 140 | PASS | FAIL | FAIL / FAIL | 16.34333351 | 2.369 |
| Hard M2/I0 | G1 | 4 / 64 | FAIL | FAIL | FAIL / FAIL | 2301.643253 | 2.533 |
| Hard M2/I0 | G2 | 0 / 140 | PASS | FAIL | FAIL / FAIL | 16.34353411 | 3.420 |
| Hard M2/I1 | G0 | 0 / 144 | PASS | FAIL | FAIL / FAIL | 16.34333353 | 2.430 |
| Hard M2/I1 | G1 | 4 / 1 | FAIL | FAIL | FAIL / FAIL | 128.3616025 | 0.172 |
| Hard M2/I1 | G2 | 0 / 145 | PASS | FAIL | FAIL / FAIL | 16.34353412 | 3.510 |
| Hard M3/I0 | G0 | 0 / 140 | PASS | FAIL | FAIL / FAIL | 16.34333351 | 2.681 |
| Hard M3/I0 | G1 | 8 / 32 | FAIL | FAIL | FAIL / FAIL | 2798.211333 | 1.196 |
| Hard M3/I0 | G2 | 0 / 141 | PASS | FAIL | FAIL / FAIL | 16.34353411 | 3.761 |
| Hard M3/I1 | G0 | 0 / 142 | PASS | FAIL | FAIL / FAIL | 16.3433335 | 2.739 |
| Hard M3/I1 | G1 | 4 / 1 | FAIL | FAIL | FAIL / FAIL | 128.3616025 | 0.184 |
| Hard M3/I1 | G2 | 0 / 143 | PASS | FAIL | FAIL / FAIL | 16.34353415 | 3.918 |
| Benign M3/I1 | G0 | 0 / 121 | PASS | PASS | PASS / PASS | 0.1943350671 | 2.022 |
| Benign M3/I1 | G1 | 6 / 1 | PASS | PASS | PASS / PASS | 3.963365592 | 0.014 |
| Benign M3/I1 | G2 | 0 / 122 | PASS | PASS | PASS / PASS | 0.1943350668 | 2.594 |

Status 4 is incompatible inequalities; 6 is singular C matrix; 8 is positive directional derivative. These terminations do not prove infeasibility. G1 benign remains full-valid only by returning its unchanged seed. G2 benign selects `callback_0122`, identical to its latest vector and different from the seed. Hard G2 selects no candidate; selected objective/path are N/A.

Benign G2: J_seed=3.9633655920316486, J_selected=0.1943350667612168, absolute reduction=3.769030525270432, relative reduction=95.0967%. This exceeds the 1e-6 numerical threshold and is comparable to historical benign G0 (0.1943350671). It is optimization recovery, not evidence of navigation improvement.

### Remaining between-point failures

All hard G2 final iterates pass their quarter motion inequalities and midpoint lateral equalities, but fail lateral velocity, forward-speed lower bound and linear acceleration on the original independent full grid. Goal, workspace, obstacle clearance, angular speed and angular acceleration pass. Full checker tolerances stay at 1e-5 in each corresponding original quantity. No lateral-only outcome was observed.

| Hard start | max absolute v_y (m/s) | min v_x (m/s) | max absolute a_x (m/s²) |
|---|---:|---:|---:|
| M2/I0 | 0.0001204018738 | -0.0001268823453 | 2.000028483813 |
| M2/I1 | 0.0001203018074 | -0.0001286705049 | 2.000028484399 |
| M3/I0 | 0.0001204011678 | -0.0001268408867 | 2.000028481472 |
| M3/I1 | 0.0001203144176 | -0.0001289594818 | 2.000028484569 |

The 1 ms plus 0.371-offset interval audit reproduces these sampled extrema. For all four hard G2 finals: lateral peak is in interval 4 at t=0.479 s (u=.79); negative-speed minimum is in interval 4 at t=.480 s (u=.80); linear-acceleration minimum is in interval 0 at t=.038 s (u=.38). These are all off-collocation interior samples, not knot-side artifacts. First observed violations are respectively t=.102 s, .475371 s and .028 s. They are sampled observations, not exact continuous extrema or certified first-crossing times; per-vector bounded brackets remain in `solves/*/intervals/*.json`.

Compared with G0, sampled negative speed is reduced from about −0.00304 to −0.000127…−0.000129 m/s, and acceleration bound excess from 7.8366e-5 to about 2.8484e-5 m/s². Both remain above the original 1e-5 allowance. Lateral maxima decrease from about 1.3015e-4 to 1.2040e-4 m/s but still exceed 1e-5. Quarter enforcement improves magnitudes without certifying the intervening curve. The full numerical tables preserve each method/seed separately, including G1 failures.

### Equality and inequality diagnostics

At every available G2 initial/latest/selected vector, G2 equality values/Jacobians equal G0 literally; the Jacobian is 30×150 and raw/column-scaled numerical rank is 30/30. For hard latest vectors the raw smallest/largest singular-value ratio is about .053419; benign latest is .033130. Historical G1 has 90 rows and benign rank 60/90. This controlled ablation supports that extra quarter lateral equalities contributed to G1’s numerical regression. It does not establish a unique linear-algebra mechanism or make dropping those equalities a full-feasibility solution.

At each hard G2 latest vector, the only quarter near-active families are forward-speed lower (1 row) and linear-acceleration lower (4 rows); all other families have zero near-active rows. All quarter margins pass their actual tolerance. Benign latest/selected has zero near-active quarter rows. Threshold=1e-4, feasibility tolerance=1e-5; these counts describe stored margins and are not an observation of SLSQP’s internal active set. Initial/latest/selected row norms and counts are saved separately.

### Compute and validation

| New G2 start | Construction s | Compile/warmup s | Prepared solve s | Candidate checks s | Cold per-start sum s | Objective / equality / inequality calls |
|---|---:|---:|---:|---:|---:|---:|
| Hard M2/I0 | 0.160 | 1.431 | 3.420 | 1.313 | 6.330 | 374 / 375 / 375 |
| Hard M2/I1 | 0.050 | 1.289 | 3.510 | 1.344 | 6.199 | 387 / 388 / 388 |
| Hard M3/I0 | 0.050 | 1.308 | 3.761 | 0.956 | 6.079 | 375 / 376 / 376 |
| Hard M3/I1 | 0.051 | 1.294 | 3.918 | 0.963 | 6.232 | 383 / 384 / 384 |
| Benign M3/I1 | 0.050 | 1.330 | 2.594 | 7.986 | 11.966 | 416 / 417 / 417 |

Prepared solves total 17.204 s. Each gradient/equality-Jacobian/inequality-Jacobian callback is invoked once per listed iteration (691 calls per callback type total); actual primal cache misses total 1,935. Per-start cold sums total 36.806 s, including input load, construction, warmup, solve and candidate checking. Environment load is separately .038 s. Five-start execution stage elapsed is 43.808 s, including serialization/orchestration; it is not equal to solve time.

Source preparation 3.016 s; derivative gate 7.434 s; post-solve interval/rank/aggregate analysis 21.906 s (rank .069 s nested inside it); rendering/index/ZIP 16.766 s; independent saved validator 44.851 s. These phase times total 137.782 s, excluding process startup, human review and tests. Do not add nested per-start/profile times again. Provider construction/warmup is charged for each start; no claim of cross-event JIT reuse is made. The frozen helper computes unused quarter lateral AD outputs internally; this cost is included. Inner environment/interpolation counts are unavailable in this unchanged harness and are not zero-filled.

The authoritative saved validator reports valid=true, errors=[], 22,025 checks, and zero new optimization/MPC/rollout calls. Original files and frozen code retain their hashes. Validation recomputes candidate checks and intervals, not just a success flag. All ten rendered figures were visually inspected; legends, units, historical/new provenance and PLAN ONLY labels are present. Small G0/G2 violations have detail or signed-symlog panels alongside G1’s full range.

### Evidence and scope

From the repository root, use `run=data/robotless_gp_se2_diag_05/primary_20260920T064500Z` with the commands above. This is the sole new primary; no failed technical run or repeated actual solve was needed.

- Static index: `data/robotless_gp_se2_diag_05/primary_20260920T064500Z/index.html`.
- Review ZIP: same root, `review_bundle.zip` (21,794,527 bytes).
- Main tables: `aggregate/all_methods.csv`, `paired_outcomes.csv`, `motion_extrema.csv`, `constraint_margins.csv`, `equality_rank.csv`, `timing.csv`.
- Derivative evidence: `derivative_checks/validation.json` and fifteen point JSON/NPZ pairs.
- Authoritative artifact check: `validation.json`.

No hard full-valid candidate exists to execute; no execution is claimed for benign either. Source progress/controller behavior is outside scope. These are one hard event and one benign control from an existing development corpus; different methods/initializations are not independent navigation trials. Frozen G0/G1 timings are historical, not a fresh causal runtime experiment. Finite-grid checking cannot prove continuous feasibility.

### Next single uncertainty

With the original 30-row equality system restored, can an **interval-extremum-based motion-inequality refinement**, retaining the same lateral equalities and physical tolerances, eliminate the residual speed/acceleration failures? That bounded follow-up would isolate the remaining inequality-enforcement gap before attributing all full rejection to lateral velocity. It would still have to report any lateral failure unchanged; it is not promised to recover full feasibility. No extrema constraints, extra refinement, new parameterization or further solve were implemented in this task.

### Repository checks

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest` completed with
**2298 passed, 19 skipped** in 158.76 s. Skips retain existing unavailable retired
corpus and diagnostic-fixture reasons; no DIAG-05 test was skipped. The full suite
includes synthetic solver/geometry fixtures, which are not additional actual-event
starts. `compileall src scripts tests` and `git diff --check` passed. No shell
launcher was changed, so bash syntax checking is not applicable. The final staged
report contains only README/documentation/work-log changes after the execution
freeze; generated arrays/PNGs/ZIP and unrelated user config edits are excluded.
