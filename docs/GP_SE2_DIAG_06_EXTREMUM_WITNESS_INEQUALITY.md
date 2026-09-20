# GP-SE2-DIAG-06: frozen extremum-witness motion inequalities

Question: can one frozen refinement close the known between-point speed and
acceleration gaps without adding lateral equalities or changing acceptance?
A secondary question is whether only lateral velocity then remains invalid.
This is a bounded development-event diagnostic, not an adaptive algorithm.

Starting revision: `f86adffc4d54ae09a3581f557a9fc3eb77c07ed4`.
Historical G0/G1: DIAG-04 `primary_20260920T015400Z`; historical G2:
DIAG-05 `primary_20260920T064500Z`, execution revision
`b84d81598591eb3fc9ba017a0c251cc59bc528df`. Original inputs and seed files
resolve to GP-SE2-02 `primary_20260919T024000Z`. Authoritative validations,
numerical core, environment provenance, historical outputs and unrelated
Stage-0 config edits are hash-preserved. No historical numerical file changes.

## Frozen protocol

DIAG-03 established collocation/interior gaps with consistent interpolation.
DIAG-04 added quarter lateral and motion rows and encountered numerical failures.
DIAG-05 removed only quarter lateral rows: convergence recovered but hard
speed/acceleration and lateral failures remained. G3 adds only observed
motion-inequality extrema from the four saved hard G2 latest vectors.

Exactly five new starts, sequentially, once:

1. Hard `episode_013_repeat_01/handoff_024`, M2 / I0_FRESH.
2. Same hard event, M2 / I1_DECEL.
3. Same hard event, M3 / I0_FRESH.
4. Same hard event, M3 / I1_DECEL.
5. Benign `episode_001_repeat_01/handoff_002`, M3 / I1_DECEL.

Original initialization .npy bytes must match GP-SE2-02 and DIAG-05. No G2
final, witness or perturbation is an optimizer seed. G0/G1/G2 are saved baselines,
not rerun. Four hard starts are one event, not independent handoff evidence.

The original 31 supports / 150 right-local variables / 3 s / 0.1 s, initial B
and physical body twist, F_common, goal, objective, GP interpolation, weights,
environment and all physical/numerical tolerances stay fixed. World poses are
T_world_agent in metres/radians. Body twist is vee(X^-1 dX/dt); body acceleration
is the derivative of twist components, not world acceleration. Downstream
trajectory times are not intrinsic LightNav timestamps. No re-anchoring occurs.

Witness extraction uses the exact DIAG-05 finest trace: 1 ms plus 0.371 offset,
supports/midpoints and both knot sides. It checks literal sample and value
agreement with the saved trace. Per signed motion family, each connected
sequence of margins < -original inequality tolerance contributes one minimum-
margin sample; ties use earliest time then canonical index. Lateral velocity
and feasible near-active rows never generate witnesses. The common union key
is (family, interval, canonical sample index), without float rounding. Each
witness adds only its generating signed inequality, never all eight families.
The same frozen union applies to all five starts, including benign.

The source precheck observes 12 generating runs and three unique rows:

| Signed family | Interval | Canonical index | Time s | Local u |
|---|---:|---:|---:|---:|
| linear_speed_lower | 4 | 965 | 0.480 | 0.7999999999999996 |
| linear_acceleration_lower | 0 | 76 | 0.038 | 0.37999999999999995 |
| linear_acceleration_lower | 1 | 276 | 0.13737100000000002 | 0.37371000000000015 |

These are **observed extremum witnesses**, not exact continuous extrema.
The generated manifest, not a hard-coded expected count, is authoritative.
G3 dimensions are 150 variables, 30 equalities, and 1296 M2 / 1386 M3 inequalities.
No equality, environment, goal or a_y condition is added.

G3 delegates all G2 primal/derivative prefixes literally. A separate CPU
float64 JAX graph uses the frozen Lie-group interpolation primitives only for
the scalar witness suffix, retaining right-chart chain rule, full acceleration
terms and original branch guards. No FD fallback or solver scaling.

The pre-solve gate uses five original seeds, five saved G2 latest vectors and
five unchanged DIAG-04 fixed perturbations. Three original directions and FD
steps 2e-4, 2e-5, 2e-6 are retained; both finer steps must pass. Primal absolute/
relative tolerances: 1e-8/1e-10; constraint derivatives: 2e-5/2e-5; original
objective derivative: 2e-4/2e-5. These diagnostic thresholds do not change
physical acceptance. Failed validation blocks all primary optimization.

Installed SLSQP, supplied objective/equality/inequality derivatives, 200
iterations, ftol=1e-7, 30 s prepared budget and single-thread BLAS stay fixed.
Provider construction/compilation, solve and post-check costs are separate.
The unchanged DIAG-04 harness inspects initial/latest/actual grid-feasible
callbacks, selecting minimum original objective then lexical source label only
among grid+dense-valid candidates. Original check_full_candidate receives the
base GPProblem. Full rejection does not trigger another selection or fallback.

Post-solve queries cover the whole unchanged sampled grid and both knot
acceleration sides, not just witness locations. A repaired witness with a new
motion violation at another canonical sample is classified as relocation;
overlap with the old observed violating run is separately recorded. No second
witness round, restart, tolerance change or new execution is permitted.

## Commands and evidence

From repository root, for the exclusive run
`data/robotless_gp_se2_diag_06/primary_20260920T083200Z`:

```bash
.venv/bin/python scripts/run_gp_se2_diag06.py prepare --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py derivatives --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
# Commit verified implementation/protocol before freeze and solve.
.venv/bin/python scripts/run_gp_se2_diag06.py freeze --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py solve --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py analyze --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py validate --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
```

The source/union/protocol/code hashes are frozen before primary solves. Every
PNG has numeric/hash sidecars. Outputs include twenty historical/new outcome
rows, five paired results, per-vector interval analyses, witness margins,
remaining runs, timing, static index and compact review ZIP. No historical
arrays are bulk-copied; no VLA/MPC/rollout/GUI is executed.

Implementation status at freeze: tests and actual derivative gate precede the
five primary starts; numerical results and interpretation will be appended.

Pre-execution gate: all fifteen actual records passed with original/G2 literal
primal and derivative-prefix parity. The witness union contains three rows,
with 1,806 preserved-file hashes. Eighteen new implementation tests and 65
related DIAG-04/05 tests pass. No primary G3 solve had run at this freeze.

## Completed result

Execution/freeze revision: `ca53a0744a98a5bbce1d153b5d056273d21b4fb1`. Exactly five G3 SLSQP invocations completed, each once, with status 0. No historical formulation was rerun. The operational result is **GP_SE2_DIAG_06_COMPLETED_WITH_LIMITATIONS**; the research interpretation is **NONLATERAL_GAP_CLOSED_LATERAL_REMAINS**. Hard full-valid recovery remains **0/4**. Completion does not mean full algorithm success.

All four hard latest vectors pass the G3 grid but fail original dense/full acceptance only on lateral velocity. The original full offset grid and the supplemental 6,041 interval-side samples detect no speed/angular/acceleration violation beyond the unchanged allowance. All three frozen witness sites are repaired; no tolerance-violating motion pocket relocates on these sampled grids. Benign retains a changed, full-valid optimized candidate. Goal, route, workspace, obstacle, boundary and body-derivative checks pass.

### All historical and new outcomes

G0/G1 are saved DIAG-04 results; G2 is saved DIAG-05; G3 is new. Each row uses its own solver grid and the same original dense/full checker. J is the latest objective, including rejected vectors. Historical timing is context, not a newly paired runtime baseline.

| Case / method / seed | Grid | Status / iterations | Grid | Dense | Latest full / selected full | J latest | Solve s |
|---|---|---:|---|---|---|---:|---:|
| Hard M2/I0 | G0 | 0 / 140 | PASS | FAIL | FAIL / FAIL | 16.34333351 | 2.369 |
| Hard M2/I0 | G1 | 4 / 64 | FAIL | FAIL | FAIL / FAIL | 2301.643253 | 2.533 |
| Hard M2/I0 | G2 | 0 / 140 | PASS | FAIL | FAIL / FAIL | 16.34353411 | 3.420 |
| Hard M2/I0 | G3 | 0 / 141 | PASS | FAIL | FAIL / FAIL | 16.34355243 | 3.460 |
| Hard M2/I1 | G0 | 0 / 144 | PASS | FAIL | FAIL / FAIL | 16.34333353 | 2.430 |
| Hard M2/I1 | G1 | 4 / 1 | FAIL | FAIL | FAIL / FAIL | 128.3616025 | 0.172 |
| Hard M2/I1 | G2 | 0 / 145 | PASS | FAIL | FAIL / FAIL | 16.34353412 | 3.510 |
| Hard M2/I1 | G3 | 0 / 144 | PASS | FAIL | FAIL / FAIL | 16.34355243 | 3.584 |
| Hard M3/I0 | G0 | 0 / 140 | PASS | FAIL | FAIL / FAIL | 16.34333351 | 2.681 |
| Hard M3/I0 | G1 | 8 / 32 | FAIL | FAIL | FAIL / FAIL | 2798.211333 | 1.196 |
| Hard M3/I0 | G2 | 0 / 141 | PASS | FAIL | FAIL / FAIL | 16.34353411 | 3.761 |
| Hard M3/I0 | G3 | 0 / 143 | PASS | FAIL | FAIL / FAIL | 16.34355243 | 3.811 |
| Hard M3/I1 | G0 | 0 / 142 | PASS | FAIL | FAIL / FAIL | 16.3433335 | 2.739 |
| Hard M3/I1 | G1 | 4 / 1 | FAIL | FAIL | FAIL / FAIL | 128.3616025 | 0.184 |
| Hard M3/I1 | G2 | 0 / 143 | PASS | FAIL | FAIL / FAIL | 16.34353415 | 3.918 |
| Hard M3/I1 | G3 | 0 / 140 | PASS | FAIL | FAIL / FAIL | 16.34355243 | 3.870 |
| Benign M3/I1 | G0 | 0 / 121 | PASS | PASS | PASS / PASS | 0.1943350671 | 2.022 |
| Benign M3/I1 | G1 | 6 / 1 | PASS | PASS | PASS / PASS | 3.963365592 | 0.014 |
| Benign M3/I1 | G2 | 0 / 122 | PASS | PASS | PASS / PASS | 0.1943350668 | 2.594 |
| Benign M3/I1 | G3 | 0 / 122 | PASS | PASS | PASS / PASS | 0.1943350667 | 2.661 |

G3 hard selected source/vector/objective are N/A, not a RAW fallback. Benign selects `callback_0122`, identical to its latest vector but different from its original seed: J_seed=3.9633655920316486, J_selected=0.19433506666620115; decrease=3.7690305253654475 (95.0967%). The comparable G2 optimum was 0.1943350667612168: G3 preserves optimization ability, not a meaningful objective improvement over G2.

### Residual magnitudes and locations

| Hard start | G2 min v_x m/s | G3 min v_x m/s | G2 max abs a_x m/s² | G3 max abs a_x m/s² | G3 max abs v_y m/s |
|---|---:|---:|---:|---:|---:|
| M2/I0 | -0.000126882345255 | -5.1288934745e-06 | 2.000028483813 | 2.000009282171 | 0.000121841764019 |
| M2/I1 | -0.000128670504879 | -5.33195013959e-06 | 2.000028484399 | 2.00000928251 | 0.000121888332794 |
| M3/I0 | -0.000126840886656 | -4.97414746597e-06 | 2.000028481472 | 2.000009282044 | 0.000121786499873 |
| M3/I1 | -0.000128959481797 | -5.2092751495e-06 | 2.000028484569 | 2.000009282584 | 0.00012186155988 |

**No physical or numerical threshold changed.** Nominal speed lower bound is 0 and nominal |a_x| bound is 2 m/s²; their original numerical allowance is 1e-5 in the corresponding units. G3 still has tiny nominal negative speed and acceleration excess, but both lie inside that pre-existing allowance. This is not exact satisfaction of the nominal inequalities. All four sampled speed minima are at t=.479 s, interval 4, u=.79; the largest |a_x| moves from the repaired early interval to t=.237 s, interval 2, u=.37, but its nominal excess ≈9.2826e-6 remains below allowance. Moving the worst sample is not the same as relocating a violating sample.

All four lateral maxima occur at t=.479 s, interval 4, u=.79, about 12.2 times the original 1e-5 m/s equality tolerance. There are 28 observed lateral violating runs per hard final. First observed violation is t=.102 s; bounded entry bracket is [0.101764125, 0.101842750] s. This is an interior enforcement gap; no final knot-side tolerance violation was observed. The original midpoint equality still passes, and equality Jacobians at every available initial/latest/selected vector are literally G0/G2-identical, 30×150 with rank 30/30 in the frozen raw and column-scaled diagnostics.

Benign max |v_y|=2.197233237e-8 m/s, min v_x=.3254877763 m/s, max |a_x|=1.093755579 m/s². It passes original full acceptance. Every interval and both acceleration knot limits are preserved in per-vector interval records. `aggregate/remaining_violations.csv` describes signed motion-inequality runs, including initial context; remaining lateral failures are explicitly in the vy rows of `motion_extrema.csv`, full flags and per-vector `violation_brackets`, not silently omitted or marked valid.

### Frozen witness before/after

Margins below are signed inequalities, with PASS when margin >= -1e-5. w0 is v_x at .480 s (m/s), w1 is a_x+2 at .038 s, w2 is a_x+2 at .137371 s (m/s²). Each row adds only that family. Every one of the three sites is no longer the worst sampled point in its family.

| Start | w0 G2 → G3 | w1 G2 → G3 | w2 G2 → G3 |
|---|---:|---:|---:|
| Hard M2/I0 | -0.000126882345 → -1.50390501e-11 | -2.84838133e-05 → 1.15398358e-10 | -1.68258373e-05 → 3.29587468e-11 |
| Hard M2/I1 | -0.000128670505 → -4.75013126e-12 | -2.84843989e-05 → -2.49134047e-13 | -1.68257062e-05 → -3.57536223e-11 |
| Hard M3/I0 | -0.000126840887 → -2.24170766e-11 | -2.84814722e-05 → 7.703016e-11 | -1.68260422e-05 → 7.58237917e-12 |
| Hard M3/I1 | -0.000128959482 → 8.45492316e-12 | -2.84845689e-05 → 1.06357145e-11 | -1.68259905e-05 → -9.82836035e-11 |
| Benign M3/I1 | 0.42457945 → 0.424579326 | 0.947340988 → 0.947341239 | 1.06598611 → 1.06598425 |

### Derivative gate and compute

All 15 frozen actual records pass with literal objective/equality/G2 inequality-prefix and derivative-prefix identity. Witness NumPy/JAX primal maximum error is 6.422e-13. Across the two required finer directional steps, maximum absolute/scaled error is w0: 3.502e-8 / .000976; w1: 2.592e-7 / .002710; w2: 8.505e-7 / .005024 (scaled PASS <=1). Original branch exclusions remain declared; no FD fallback or acceptance relaxation. Derivative status: VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS.

| New G3 start | Construction s | Compile/warmup s | Prepared solve s | Candidate checks s | Cold per-start sum s | Objective / equality / inequality calls |
|---|---:|---:|---:|---:|---:|---|
| Hard M2/I0 | 0.155 | 1.873 | 3.460 | 0.933 | 6.428 | 376 / 377 / 377 |
| Hard M2/I1 | 0.055 | 1.654 | 3.584 | 1.517 | 6.816 | 385 / 386 / 386 |
| Hard M3/I0 | 0.058 | 1.669 | 3.811 | 1.529 | 7.072 | 377 / 378 / 378 |
| Hard M3/I1 | 0.058 | 1.709 | 3.870 | 0.957 | 6.598 | 379 / 380 / 380 |
| Benign M3/I1 | 0.058 | 1.714 | 2.661 | 8.184 | 12.622 | 409 / 410 / 410 |

Prepared solves total 17.384706 s; per-start cold sums total 39.535801 s. Environment load .038802 s and total five-start execution elapsed 46.776123 s include orchestration/serialization separately. Derivative callbacks each run 690 times; primal cache misses total 1,926. Witness AD costs are reported separately within provider stats; top-level inclusive costs must not be added again to nested timings. Providers compile per start; no cross-case JIT reuse claim. Inner GP/environment counts remain unavailable in the unchanged harness rather than zero-filled.

Source preparation 6.293670 s; derivative gate 10.599477 s; per-start interval/rank analyses sum 14.685621 s, with rank cost nested inside. Reporting repair/assembly 5.693485 s, rendering/package 20.185323 s, and independent saved-record validation 50.953056 s. Analysis wall time is reported as the sum of recorded per-start phases because the aggregate writer failed before its normal completion marker. This excludes unrecorded process startup/failed-writer overhead; do not describe the sum as exact total human/offline elapsed. Analysis/testing ran concurrently after all primary solves, so those costs are descriptive.

### Reporting correction, validation and review

All five numerical solves and all saved candidate/interval analyses completed on the frozen revision. The first aggregate write failed because the runner omitted `aggregate.mkdir` before opening `all_methods.csv`. The original code, failed console log and every saved numerical output remain unchanged. The additive `finish_gp_se2_diag06_report.py` creates only the missing report directory, assembles saved analyses and renders/packages them. It performs zero GP/MPC solves, no analysis rerun, no new candidate choice and no overwrite. This is a reporting correction, not a scientific retry.

Actual follow-up commands after that preserved reporting error:

```bash
.venv/bin/python scripts/finish_gp_se2_diag06_report.py --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py validate --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/package_gp_se2_diag06_review.py --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
```

For a future fresh reproduction with the preserved runner, create its empty `aggregate/` directory before `analyze`; do not rerun an existing primary. The repair script is restricted to the documented missing-directory failure with all analyses already saved.

The authoritative validator recomputes the witness union from historical G2, candidate checks, callbacks, full checker, intervals, rank, tables and plot payloads without solving. It passes 23,270 checks with zero errors. All 1,806 preserved source/core/config/environment/result hashes and both unrelated Stage-0 edits remain unchanged. Ten PNGs were visually inspected for labels/legends, units and equal-aspect XY. The original complete-sidecar ZIP is 50,916,294 bytes; an additive compact review packet keeps all original PNGs and result tables, while linking full numeric sidecars by path/hash. Neither changes plotted numbers.

- Primary static index: `data/robotless_gp_se2_diag_06/primary_20260920T083200Z/index.html`.
- Compact review index: same root, `compact_review/index.html`.
- Preferred review ZIP: same root, `review_bundle_compact.zip`, 3,014,569 bytes, SHA256 `ac050e8bf88af22d55eaf6e601b6e33b4363ea1e6c6dbb8c4bbb99e66743ee3e`.
- Authority: `validation.json`; compact member/ZIP byte validation: `compact_review_validation.json`.
- Frozen input/code authority: `source.json`, `protocol.json`, `execution_freeze.json`, `witness_selection/`.

### Interpretation and next single uncertainty

These sampled results isolate the non-lateral enforcement gap: one fixed three-row refinement closes the observed speed/acceleration failures without the dependent quarter lateral equalities or a numerical regression. Full hard feasibility does not recover: lateral velocity alone remains invalid. No full-feasible hard candidate is exported or executed. Finite checks cannot certify the continuous interval; the witness union is development-event-derived, and four starts are not four independent events. No general algorithm, navigation benefit or deployment safety claim follows.

**These residuals motivate a separate evaluation of plan-level hard feasibility semantics.** They do not imply that tolerance should be relaxed. GP body velocities/accelerations are not fed forward to the current pose-reference MPC; LightNav has no intrinsic waypoint timestamps; downstream MPC independently constrains its commands. Those facts make the remaining plan-level lateral/nonholonomic criterion an empirical/modeling question, not permission to change this run.

Next single proposed experiment: evaluate whether plan-level lateral-only rejection predicts loss of feasibility in the fixed official-MPC execution interface, using explicitly plan-invalid diagnostic references and unchanged execution safety/goal predicates. This would inform whether v_y should remain hard, be treated softly, or be guaranteed structurally; it does not preselect relaxation or implement a new parameterization here. No such execution, additional witness, selector change or further refinement was performed.

### Final repository verification

Full required pytest, with local Unix-socket access, completed **2318 passed,
19 existing skips** in 159.34 s. The first sandboxed run had 2314 passes and
two permission failures in existing IPC tests; its log is preserved separately.
The second run includes the two additive report-repair tests. Skips concern
retired unavailable corpora and the existing S3 diagnostic fixture, not new
DIAG-06 evidence. `compileall src scripts tests` and `git diff --check` pass.
No shell launcher changed. All numerical/plot code in the execution freeze
retains its source hash; the later report/compact-packaging helpers and tests
are additive. Generated data, PNG/ZIP, environment/cache and unrelated user
configuration changes are excluded from both commits. The final report SHA,
normal push status and package hashes are recorded in ignored `completion.json`.
