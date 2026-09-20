# GP-SE2-DIAG-04: quarter-point motion constraint refinement

Question: with the same GP representation, objective, saved initializations and
physical acceptance, does adding quarter-point motion enforcement recover a
full-valid candidate for the fixed hard handoff? This experiment performs new
GP solves. It does not execute paths or change the controller/selector.

Starting revision: `9f6031bf46f4b885af91433ab023a1552e222599`.
Historical numerical source: GP-SE2-02
`data/robotless_gp_se2_02/primary_20260919T024000Z/`, execution revision
`bd273b22ed9ddcb35a3f2eadcf550895adee46b9`. The preceding diagnosis is
`data/robotless_gp_se2_diag_03/audit_20260920T010000Z/`.
Their final authoritative validations, original inputs/configuration, numerical
core, environment, historical artifacts and unrelated Stage-0 edits are hashed.
The original external MPC file is hashed and never executed or edited.

## Frozen experiment

The hard event is `episode_013_repeat_01/handoff_024`. The benign control is
`episode_001_repeat_01/handoff_002`. The ten sequential starts are:

1. Hard M2 / I0_FRESH: G0 then G1.
2. Hard M2 / I1_DECEL: G0 then G1.
3. Hard M3 / I0_FRESH: G0 then G1.
4. Hard M3 / I1_DECEL: G0 then G1.
5. Benign M3 / I1_DECEL: G0 then G1.

Each pair uses byte-identical original GP-SE2-02 initialization files. Historical
latest vectors are verification inputs only. There are no extra starts or
outcome-dependent retries. These are four comparisons of one hard event, not
four independent hard handoffs.

The original 31 support states, 150-variable right-local chart, 3 s horizon,
0.1 s support spacing, B, physical initial twist, F_common, goal, objective,
weights and environmental queries remain unchanged. World poses are
T_world_agent; body twist is vee(X^-1 dX/dt). Translation is in metres and yaw
in radians. The downstream time convention is not an intrinsic LightNav
waypoint timestamp. No re-anchoring or new reference preparation occurs.

G0 delegates objective and constraints to the original GPProblem. G1 appends
60 lateral equalities (v_y=0 at u=.25,.75 of each interval) and 480 original
motion inequalities (eight signed margins per added sample). Acceleration is
the derivative of body-twist components, not world acceleration. No new a_y
condition, environment row, goal row, terminal stop, slack or soft penalty is
introduced. Original arrays are exact prefixes.

| Grid | Variables | Equalities | M2 inequalities | M3 inequalities |
|---|---:|---:|---:|---:|
| G0 | 150 | 30 | 813 | 903 |
| G1 | 150 | 90 | 1293 | 1383 |

Both use installed SLSQP with supplied objective gradient and both constraint
Jacobians: 200 iterations, ftol=1e-7, 30 s prepared budget, float64 CPU and
single-thread BLAS. The new adapter preserves callback/dense-first retention;
both grids disable historical module-global inner profiling hooks. Inclusive
primal evaluation and derivative callback costs are measured. Unmeasured
inner GP/environment costs remain N/A. Compilation, input/environment loading,
candidate checks, rank diagnostics and offline cost are separate from solve.
Package versions and source hashes are frozen in the generated protocol.

## Derivative gate and rank protocol

The unchanged DIAG-02 provider supplies all original derivatives. A separate
JAX extension uses the original AD interpolation primitives to append only
quarter rows, including current right-chart chain rule and both body
acceleration terms. No finite-difference fallback is used. Original relative
Log/goal cut guards and analytic environment branch handling remain active.

Before any primary solve, freeze five seed records, five historical latest
records, and five fixed seed perturbations. Method provenance is retained for
duplicates. Random seed is 20260919; perturbation scales are 1e-4 in each chart
column and are never used as optimization initializations. Three normalized
150-dimensional directions use steps 2e-4, 2e-5, 2e-6. Both finer steps must
pass. Reused DIAG-02 absolute/relative tolerances are:

| Check | Absolute | Relative |
|---|---:|---:|
| Primal values | 1e-8 | 1e-10 |
| Objective derivative | 2e-4 | 2e-5 |
| Constraint derivatives | 2e-5 | 2e-5 |

Original branch-aware base verification is repeated at these records; quarter
families are checked separately, including signs/order/shapes and worst errors.
All base primal/derivative prefixes require literal equality. Diagnostic
tolerances never change physical acceptance. A failed gate blocks optimization.

After all solves, evaluate G0/G1 equality Jacobians at initial/latest/selected
vectors only. Report raw and column-scaled SVD separately, using repeated
[1 m,1 m,1 rad,.8 m/s,3 rad/s] scales. Rank cutoffs are max(shape)*eps*s_max
and relative 1e-12,1e-10,1e-8,1e-6; near-zero row norm threshold is 1e-12.
These scales never enter SLSQP. Rank deficiency does not prove physical
infeasibility and does not authorize removing equalities.

## Candidate and artifact policy

Retain initial/latest and actual solver-grid-feasible callbacks. Recompute
their grid residuals and original dense report after the solve. Among eligible
candidates select minimum original objective, then lexical source label.
Independently run unchanged check_full_candidate on the original base problem,
including original environment/goal, offset grid and both knot accelerations.
Full failure does not trigger reselection. Initial/latest/selected, convergence,
availability, full validity and unchanged-seed return remain separate fields.

Full 3 s curves are also queried with the frozen DIAG-03 interval diagnostics,
including .001 s and .371 offset samples. Extremum figures use each new result's
maximum tolerance excess and earliest-interval tie-break, not historical
witnesses. These are sampled observations, not a continuous-time certificate.
All paths are unexecuted plans. Failed results stay in every table and index.

The saved-record validator recomputes grid/dense/full reports, selection,
callback values, interval traces, SVDs, aggregate tables, plot numeric sidecars
and hashes without optimization. Generated data and images are excluded from
Git. Existing run/phase outputs refuse overwrite and retry.

Interface references: [SciPy minimize](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html),
[SLSQP](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html),
[JAX jacfwd](https://docs.jax.dev/en/latest/_autosummary/jax.jacfwd.html).
Installed versions are used without upgrades.

## Execution and results

Execution revision: `e3776fa30a161ff361ba5f1d321a8caa788e729d`.
Primary: `data/robotless_gp_se2_diag_04/primary_20260920T015400Z/`.
All ten planned starts called SLSQP exactly once. New VLA/MPC solves, rollout
and GUI runtime counts are all zero. No retries, fallback or support changes.
The final source check preserves 1,431 source/core/artifact files and separately
checks the unrelated user edits and new frozen inputs. Original configuration
SHA256 is `15f3f1580f13d437cff0bf88fba59b8526b209e070692bfaab8367f2784863f6`.
All five G0/G1 seed pairs are byte-identical, including the original B/twist.
Installed versions: NumPy 2.5.2, SciPy 1.18.1, JAX/jaxlib 0.7.2, Shapely 2.1.2,
Matplotlib 3.11.1. All actual dimensions match the frozen table above.

All 15 actual verification records passed with declared branch limitations.
Maximum primal absolute error was 3.142e-12 for original rows and 1.573e-12 for
quarter rows. At the two required FD steps, maximum scaled errors were
0.1023800 (original) and 0.04303835 (quarter), against the frozen bound 1.
Maximum absolute directional errors were 3.490e-6 and 1.745e-6 respectively;
linear-acceleration rows dominate. Base value/gradient/Jacobian parity is
literal-exact. Added shapes are 60×150 and 480×150. At coarse h=2e-4, one
obstacle directional element crossed a geometry branch and was characterized
separately (closest one-sided scaled error 0.004208). Neither required finer
step excluded any element; quarter rows excluded none. No unsupported wrap
cuts occurred. Roundoff grows at the smallest FD step; there is no claim of
monotonically decreasing error.

| Case / method / seed | Grid | SLSQP termination (status) | Iterations | Latest grid / dense / full | Selected full | Latest objective | Solve s |
|---|---|---|---:|---|---|---:|---:|
| Hard M2/I0 | G0 | CONVERGED (0) | 140 | PASS / FAIL / FAIL | none | 16.34333351 | 2.369 |
| Hard M2/I0 | G1 | incompatible inequalities (4) | 64 | FAIL / FAIL / FAIL | none | 2301.643253 | 2.533 |
| Hard M2/I1 | G0 | CONVERGED (0) | 144 | PASS / FAIL / FAIL | none | 16.34333353 | 2.430 |
| Hard M2/I1 | G1 | incompatible inequalities (4) | 1 | FAIL / FAIL / FAIL | none | 128.3616025 | 0.172 |
| Hard M3/I0 | G0 | CONVERGED (0) | 140 | PASS / FAIL / FAIL | none | 16.34333351 | 2.681 |
| Hard M3/I0 | G1 | positive directional derivative (8) | 32 | FAIL / FAIL / FAIL | none | 2798.211333 | 1.196 |
| Hard M3/I1 | G0 | CONVERGED (0) | 142 | PASS / FAIL / FAIL | none | 16.34333350 | 2.739 |
| Hard M3/I1 | G1 | incompatible inequalities (4) | 1 | FAIL / FAIL / FAIL | none | 128.3616025 | 0.184 |
| Benign M3/I1 | G0 | CONVERGED (0) | 121 | PASS / PASS / PASS | PASS | 0.1943350671 | 2.022 |
| Benign M3/I1 | G1 | singular C matrix in LSQ (6) | 1 | PASS / PASS / PASS | PASS, unchanged seed | 3.963365592 | 0.014 |

All five new G0 latest vectors, iterations, termination and candidate
availability reproduce GP-SE2-02 literally; maximum chart difference is zero.
Thus disabling the inner profiling hooks did not alter these numerical results.
Historical wall time is not used as the paired baseline.

Hard full-candidate recovery is **0/4 paired starts on one event**. There were
no timeouts. Solver messages 4/6/8 describe local SLSQP termination and do not
prove that the nonlinear physical problem has no feasible solution. Hard G1/I1
latest vectors equal their invalid seeds; they have no selected candidate, so
`returned_initial_unchanged` is false for the return rather than a claim that
those latest vectors changed. Their motion is valid but their original goal
is not. Benign G1 selects `callback_0001`, a byte-identical copy of the known
full-valid seed; the lexical tie-break selects that label rather than `initial`.
This is seed preservation with failed convergence, not new optimization gain.
Benign G0 selects `callback_0121`, equal to latest, and lowers the seed objective
from 3.963365592 to 0.1943350671. No terminal zero-velocity requirement is added.

## Motion failure and rank interpretation

Every final was checked over all 30 intervals and the original independent
full grid, including both knot sides. The four G0 hard results retain the
DIAG-03 pattern: lateral maximum 1.30140e-4–1.30196e-4 m/s at 0.479 s,
minimum forward speed −0.00304177–−0.00303663 m/s at 0.478 s, and linear
acceleration about −2.000078366 m/s² at 0.029 s. Original tolerances are 1e-5;
these remain rejected off-collocation violations.

G1/I0 failures are not successful paths with only a new tiny interior defect:
these latest vectors already fail their own solver grid. M2/M3 lateral maxima
are 4.73044/4.79179 m/s at 0.050371 s. Maximum absolute linear acceleration
is 36.68689 m/s² at the 0.1 s right knot side, interval 1 (M2) and 13.06023 m/s² at t=0 (M3);
angular acceleration is 315.8623/388.9641 rad/s² at t=0. M2 minimum forward
speed is −0.586993 m/s. G1/I1 stays at the goal-invalid deceleration seed,
with maximum |a_x|=0.2666667 m/s² and negligible lateral velocity.
Environment/workspace/route checks pass for these latest curves, but this does
not restore their motion/goal validity. No hard result is classified
REFINED_GRID_PASS_FULL_FAIL: all hard G1 final grids already fail. Complete
extrema, knot sides and observed violation brackets are in each
`solves/<id>/motion_analysis.json` and `aggregate/motion_extrema.csv`.

G0 equality Jacobians have rank 30/30 at every frozen raw/scaled cutoff.
For G1:

- Benign seed and unchanged final have numerical rank 60/90 at machine and
  all relative cutoffs; raw smallest/largest singular values are approximately
  4.33e-17/44.00165.
- Hard deceleration seed has machine rank 76 raw / 80 scaled; relative
  cutoffs 1e-10 through 1e-6 give 60 for both. At 1e-12 they give 60/63.
  This is threshold-sensitive numerical rank, not exact algebraic rank 60.
- Hard FRESH seed has machine rank 90, raw condition number about 2.69e10,
  and raw cutoff ranks 90/88/70/64. Column-scaled condition number is 1.49e10
  and cutoff ranks 90/89/75/65. G1/I0 final machine rank is 90; relative
  1e-6 ranks remain 89 (M2 raw) and 88 (M3 raw).

There are no individually near-zero equality rows at the frozen 1e-12 row-norm
threshold. Dependencies involve combinations of rows. These findings support
local equality dependence/conditioning as a limitation and are consistent with
the benign singular-LSQ exit. They do not fully identify the hard I0 failures,
prove infeasibility, or justify silently removing equations. No rank diagnostic
changed the solver, seed, tolerances or equality array.

## Compute and evidence

| Measured stage / count | Five G0 starts | Five G1 starts |
|---|---:|---:|
| Prepared solve, s | 12.2417 | 4.0987 |
| Compilation / first warmup, s | 4.0629 | 6.6274 |
| Candidate post-check, s | 13.3151 | 1.3607 |
| Input/provider setup + warmup + solve + post-check, s | 30.0027 | 12.3281 |
| Objective calls / primal cache misses | 1925 / 1925 | 297 / 297 |
| Equality + inequality calls | 3860 | 604 |
| Gradient/Jacobian calls | 2061 | 285 |

G1 has lower aggregate solve time because of early failed exits; this is not
an efficiency win. M2/I0 G1 individually takes longer than its G0 counterpart.
Each primary provider is constructed/warmed for its own case/method/grid;
measured compilation/warmup costs are retained, including possible JAX cache
reuse. No unnecessary quarter graph is built for G0. Post-solve rank providers
are reused by case/method (three sets) and their 4.6577 s setup/warmup is separate
from the 0.1696 s derivative-query/SVD work. No scaling is passed to SLSQP.

The ten-start execution phase costs 52.6522 s including environment loading
(0.0378 s), checks, serialization and orchestration; sum of the ten per-start
cold measurements is 42.3308 s. Derivative validation costs 7.5015 s. Analysis
through rank/interval tables costs 12.1508 s, containing the rank costs above;
these overlapping measurements must not be summed as independent costs.
Primary plot/package creation spans approximately 16.14 s based on saved
artifact timestamps. Independent saved-record validation costs 33.0037 s.
From protocol freeze 01:53:51.994705 UTC to primary validation output
01:56:59.683340 UTC, offline elapsed is 187.689 s, including between-command
orchestration gaps. It excludes software development, full tests and the later
presentation-only rendering. This was never a cold end-to-end 30 s experiment.

Primary artifact validation passes, with 24,699 checks and zero errors. It
recalculates the original checker and G1 extras, candidate selection, raw/scaled
rank diagnostics, saved traces, aggregate tables, figure data and ZIP hashes.
The final full suite, including presentation tests, reports 2,281 passed /
19 skipped in 149.62 s; skips are existing
unavailable historical datasets or an explicitly inexact fixture. New targeted
DIAG-04 implementation tests report 53 passed and presentation tests 3 passed.
Compileall and git diff --check pass; no shell launcher was changed. Tests are implementation evidence,
not research outcomes.

All ten primary PNGs were visually inspected. A separate reporting-only
presentation fixes long residual labels, overlaid legends and N/A elapsed axes,
and makes small signed violations visible beside large invalid G1 excursions.
It does not overwrite the frozen primary, alter numbers, or rerun optimization.
The first presentation `presentation_20260920T020000Z` is retained as an
intermediate; the final `presentation_20260920T020100Z` also shortens the objective
label and removes crowded acceleration ticks. Only rendering was repeated.
Preferred figures/index/review ZIP:
`data/robotless_gp_se2_diag_04/presentation_20260920T020100Z/`.
The final presentation passes 46 preservation/numeric checks; all ten final
PNGs were visually inspected and exactly retain the primary plotted numbers.
Its ZIP is 16,718,474 bytes (SHA256
`78a8f93d7bd5343694c86a3cc00767b0b3fbe843b282ff74e63679656e4459eb`).
Primary records, derivative reports and tables remain in
`data/robotless_gp_se2_diag_04/primary_20260920T015400Z/`.

Reproduction commands (each phase requires a fresh/not-yet-started output;
these document the already completed primary, not permission to overwrite it):

```bash
.venv/bin/python scripts/run_gp_se2_diag04.py prepare --run data/robotless_gp_se2_diag_04/primary_20260920T015400Z
.venv/bin/python scripts/run_gp_se2_diag04.py derivatives --run data/robotless_gp_se2_diag_04/primary_20260920T015400Z
.venv/bin/python scripts/run_gp_se2_diag04.py solve --run data/robotless_gp_se2_diag_04/primary_20260920T015400Z
.venv/bin/python scripts/run_gp_se2_diag04.py analyze --run data/robotless_gp_se2_diag_04/primary_20260920T015400Z
.venv/bin/python scripts/run_gp_se2_diag04.py validate --run data/robotless_gp_se2_diag_04/primary_20260920T015400Z
.venv/bin/python scripts/present_gp_se2_diag04.py --primary data/robotless_gp_se2_diag_04/primary_20260920T015400Z --output data/robotless_gp_se2_diag_04/presentation_20260920T020100Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

## Outcome and next single experiment

Operational: **GP_SE2_DIAG_04_COMPLETED_WITH_LIMITATIONS**.
Numerical: **NO_FEASIBILITY_RECOVERY**. Hard full-candidate availability stays
zero for both grids. Benign feasibility is preserved by retaining its seed;
G1 convergence and objective improvement are not preserved. Interpolation,
source, acceptance or derivative mismatch was not observed. Finite full-grid
checks remain sampled verification, not continuous-time proof. There is no
MPC execution, navigation or real-robot safety claim.

The next single bounded comparison proposed is an **added-inequality-only
ablation**: retain original midpoint lateral equalities and add the same
quarter motion inequalities, keeping the original full lateral checker,
initializations, budget and all physical thresholds. This isolates whether the
extra lateral equality rows drive the numerical regression. It does not claim
that removing their enforcement will recover full feasibility; between-point
lateral rejection remains possible and must remain a failure. No such new
solve or equality change was performed in this task.
