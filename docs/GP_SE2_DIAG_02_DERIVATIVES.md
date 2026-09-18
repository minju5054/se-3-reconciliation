# GP-SE2-DIAG-02: supplied derivatives under the fixed handoff formulation

Supplying verified derivatives recovers a new fully feasible candidate from the
FRESH start and improves the previously feasible deceleration seed in both M2
and M3 on the fixed event. All four supplied-Jacobian starts converge in
1.744–2.129 s; all four paired FD starts time out at 30 s. I1 objective decreases
from 3.963365592 to 0.194335067 (95.0967%), with independently checked changed
paths and velocities. No supplied start returns the original seed unchanged.
This is a numerical optimization result on one benign event, not navigation or
controller execution evidence.

Operational status: **GP_SE2_DIAG_02_COMPLETED_WITH_LIMITATIONS**.
Derivative validation: **VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS**.
The actual case remains `episode_001_repeat_01/handoff_002`; exactly eight
sequential starts were executed once, without replacement or retry.

Started from `311bf75c2d499a6de907580ce491ebcab875e847` on main. The historical
GP-SE2-01 core, data, validated environment, DIAG-01 results and unrelated Stage 0
configuration edits are preserved. The authoritative previous validation is
`data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/verification/validation.json`
(47,216 passing checks), not its preserved earlier failed validator report.

Run: `data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z` (UTC run ID;
the local date is September 19, 2026 in Korea).

## Frozen comparison

There are exactly eight sequential starts: M2 and M3, FD_BASELINE and
SUPPLIED_JAC, I0_FRESH and I1_DECEL. Their order is versioned in
`configs/gp_se2_diag_02.yaml` and recorded before any actual solve. Each uses the
original float64 150-variable right-local chart, 3 s horizon, 0.1 s supports,
200-iteration limit, 30 s prepared-solve budget, ftol 1e-7 and single-thread BLAS.
The original primal callback and last-vector cache are shared by both modes.
M2 retains workspace rows: 30 equalities and 813 inequalities. M3 has 30 and 903.
All physical bounds, goal, footprint, clearance and acceptance remain unchanged.

I0 measures recovery from an infeasible initialization. I1 measures retention,
improvement or convergence from the exact previously saved feasible seed:
X(t)=B Exp((t-t²/(2T))*eta), nu(t)=(1-t/T)*eta. No parameter is fitted to the goal.
Both saved initialization vectors match their reconstruction bit for bit and are
identical across methods and derivative modes; fixed B and initial twist are
preserved bitwise. No constant-twist second initialization is reintroduced.

## Derivative implementation

CPU JAX 0.7.2 / jaxlib 0.7.2, float64 forward automatic differentiation, supplies
the objective, lateral equality, speed, full acceleration and goal derivatives.
Its separate expression preserves the original Exp/Log Taylor thresholds, yaw
wrapping, right retraction chain at nonzero delta, GP residual/whitening, uniform
FRESH normalization, and both terms of the right-Jacobian acceleration formula.
JAX is installed only in the repository research `.venv`; NumPy 2.5.2, SciPy
1.18.1, Isaac, system Python and the official MPC environment are unchanged.
The optional `derivatives` dependency records the pinned CPU backend.

The source of primal truth remains `GPProblem.evaluate` for both solver modes.
The AD transcription is separately value-checked and only supplies derivatives.
Objective `jac`, equality `jac` and inequality `jac` are all connected in
SUPPLIED_JAC; FD_BASELINE passes no supplied derivative. The [official minimize
interface](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html)
specifies these callbacks, and the [SLSQP documentation](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html)
describes its numerical differentiation and internal QP multipliers. Installed
SciPy source is hashed separately from the currently served 1.18.0 web manual.
The runtime is not upgraded to match that manual.

World-XY interpolation derivatives are chained with analytic environment gradients.
Obstacle gradients differentiate the original bilinear grid with its world
spacing; conservative error, footprint and clearance offsets stay constant.
Workspace gradients select the current nearest actual polygon boundary feature,
including holes and the inside/outside sign. The nearest feature is queried anew.
No bounding box, smoothing, removed constraint or numerical derivative fallback
is used. Gates are explicitly unsupported outside this fixed no-gate event.

At cell boundaries the selected floor cell gives a one-sided derivative. Ties and
polygon vertices use declared active limiting gradients. Invalid grid interiors
retain the original constant -1e6 query penalty and zero derivative of that
constant branch; these are neither free space nor informative guidance. At domain
discontinuities there is no claim of a classical or Clarke derivative of the full
function. Branch metadata records these cases; unsupported/nonfinite derivatives
produce explicit failures. The same limitation applies at wrapping cuts.

## Verification protocol

Protocol and exact numeric points were frozen before evaluating the provider.
There are 20 states, including actual I0/I1 for both methods, deterministic saved
rejected vectors, fixed perturbations, S0/S1/S2/S3 and dedicated chart/rotation/
Taylor/wrap/speed-bound fixtures. Ten states receive all 150 coordinate probes.
Three fixed normalized directions use h=[2e-4,2e-5,2e-6]; both finest smooth
checks must pass. Synthetic fixtures are correctness checks, not navigation data.

Primal tolerance is atol=1e-8, rtol=1e-10. Objective derivative tolerance is
atol=2e-4, rtol=2e-5; constraint derivative tolerance is atol=2e-5, rtol=2e-5.
Reported scaled error is abs(AD-FD)/(atol+rtol*max(abs(AD),abs(FD))). Each family
records absolute/scaled errors and worst row/column. This scaling is for
validation only; solver rows and variables are not rescaled. Geometry branches
crossed by a finite probe are separately characterized with one-sided differences.
No threshold is increased after a mismatch.

The initial verification passed its smooth-point requirements. The exact relative
Log cut was only characterized, and its objective directional derivative could
not be justified. Consequently a runtime guard rejects unsupported exact/near-cut
relative Log or goal-yaw derivatives; it never supplies those Jacobians to SLSQP.
Pre-guard attempts and their earlier summary remain preserved. The authoritative
gate is `derivative_checks/authoritative_validation.json`, which verifies the
smooth points and the explicit cut rejection with the original frozen tolerances
and points. A protocol addendum records this handling before any actual solve.
Actual SUPPLIED_JAC runs are forbidden unless that final gate passes.

## Timing, acceptance and interpretation

Cold environment loading, graph construction, compilation/first-call warmup,
seed loading, prepared solve, dense candidate checking and independent full
checking are measured separately. No cold setup is counted as prepared solve
time, and the primary comparison is not an end-to-end 30 s claim. A one-vector
derivative cache is shared across the three derivative callbacks and cleared
after warmup; original primal cache policy remains identical in both modes.

Both modes save actual callbacks and defer heavy checks until after solve. The
original candidate rule selects the minimum original objective among qualifying
initial/latest/actual callbacks; the unchanged `check_full_candidate` then checks
both methods, including M2 obstacle geometry. An independently invalid selected
candidate is reported without silently substituting another candidate.

I1 numerical improvement requires J_seed-J_selected greater than
max(10*ftol,1e-8*max(1,abs(J_seed))); the raw delta and path/velocity change remain
visible. I0 is judged first by full feasibility, not by decrease from its invalid
initial objective. Termination, initial/final/retained feasibility, selected source,
unchanged seed, recovery and improvement are separate fields.

Optional first-order diagnostics occur only after all eight primary solves.
They fit multipliers to the fixed vector with g>=0, mu>=0 and
L=f-lambda*h-mu*g; no trajectory is optimized. Active g<=1e-5, unscaled singular
values and rank tolerance 1e-9*max(1,s_max), stationarity and complementarity are
reported. Internal QP multipliers, rank deficiency and nonsmooth branches cannot
prove global optimality. A nonzero unconstrained gradient alone is not a failure.

No new inference, RGB, environment export, MPC rollout, GUI, solver, formulation,
initialization, restoration, weights or physical tolerance are introduced.

## Reproduction commands

Use a new output directory; the scripts refuse result overwrites. The preserved
pre-guard cut evidence is an explicit input to the declared rejection policy.
The original datasets, DIAG-01 and validated Hospital export must be available;
missing local evidence is never replaced with synthetic data.

```bash
DIAG_RUN="$PWD/data/robotless_gp_se2_diag_02/NEW_RUN_ID"
CUT_EVIDENCE="$PWD/data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/derivative_checks/attempt_02"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export JAX_PLATFORMS=cpu JAX_ENABLE_X64=true
export MPLCONFIGDIR=/tmp/gp_se2_diag02_mpl
uv pip install --python .venv/bin/python 'jax[cpu]==0.7.2'
.venv/bin/python scripts/run_gp_se2_diag_02.py prepare --run "$DIAG_RUN"
.venv/bin/python scripts/check_gp_se2_diag02_derivatives.py freeze --run "$DIAG_RUN"
.venv/bin/python scripts/check_gp_se2_diag02_environment.py --run "$DIAG_RUN"
.venv/bin/python scripts/check_gp_se2_diag02_derivatives.py cut-policy --run "$DIAG_RUN" \
  --cut-evidence "$CUT_EVIDENCE"
.venv/bin/python scripts/check_gp_se2_diag02_derivatives.py check --run "$DIAG_RUN" \
  --geometry-report "$DIAG_RUN/derivative_checks/environment_derivatives.json"
.venv/bin/python scripts/run_gp_se2_diag_02.py freeze --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_02.py solve --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_02.py aggregate --run "$DIAG_RUN"
.venv/bin/python scripts/plot_gp_se2_diag_02.py "$DIAG_RUN"
.venv/bin/python scripts/package_gp_se2_diag_02_review.py --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_02.py finalize --run "$DIAG_RUN"
.venv/bin/python scripts/validate_gp_se2_diag_02.py --run "$DIAG_RUN" \
  --output "$DIAG_RUN/validation.json"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

The source freeze checks the provider against the authoritative derivative gate.
The historical validation publisher had a filename-shadowing bug: a valid gate
was written to `derivative_checks/se2.py`. That file and the failed freeze record
are retained. The publisher correction changed only its function; an AST
comparison verifies every numerical validation node unchanged. Its correction
record and authoritative gate retain both source hashes. No actual solver had
started and no numerical check was rerun for that publication-only correction.

Implementation history: `06983f7` records the provider/harness, followed by the
pre-comparison publication fix. The **actual frozen implementation SHA is
`298f81eff23790ab9c4477efd3426acdd73a3bd3`**. Reporting, artifact validation and
reproduction helpers added afterward are separately archived; actual numerical
source remains fixed throughout the eight runs.

## All eight measured results

I0 begins infeasible with objective 13.217924210. I1 begins full-feasible with
objective 3.963365592. Values below are original unscaled objectives. `N/A` means
no retained candidate, not zero objective. `Final full` is the last saved solver
iterate; `Selected full` applies to the separately retained candidate.

| Method / mode / start | Termination / iterations | Solve s | Final objective | Selected objective | Final full | Selected full / source |
|---|---|---:|---:|---:|---|---|
| M2 / FD / I0 | TIMEOUT / 48 | 30.000 | 6.021090110 | N/A | FAIL | NO CANDIDATE / N/A |
| M2 / JAC / I0 | CONVERGED / 128 | 1.863 | 0.194335090 | 0.194335090 | PASS | PASS / callback_0128 |
| M2 / JAC / I1 | CONVERGED / 121 | 1.744 | 0.194335067 | 0.194335067 | PASS | PASS / callback_0121 |
| M2 / FD / I1 | TIMEOUT / 47 | 30.001 | 3.623140798 | 3.963365592 | FAIL | PASS / initial |
| M3 / FD / I0 | TIMEOUT / 45 | 30.000 | 7.530553833 | N/A | FAIL | NO CANDIDATE / N/A |
| M3 / JAC / I0 | CONVERGED / 128 | 2.129 | 0.194335090 | 0.194335090 | PASS | PASS / callback_0128 |
| M3 / JAC / I1 | CONVERGED / 121 | 1.950 | 0.194335067 | 0.194335067 | PASS | PASS / callback_0121 |
| M3 / FD / I1 | TIMEOUT / 45 | 30.001 | 5.279112620 | 3.963365592 | FAIL | PASS / initial |

Both I0 supplied runs recover full feasibility; no FD I0 run does. Both I1
supplied runs improve by 3.769030525, well beyond the descriptive delta_num=1e-6.
Both FD I1 runs retain the initial seed unchanged after TIMEOUT. Their final
iterates are infeasible; even M2's lower final objective is not counted as
feasible improvement. M3's final objective is higher than its initial value. The supplied
I0 and I1 final candidates have slightly different objectives (about 2.31e-8
apart); no claim that either is a global optimum follows from this difference.
For each initialization, M2/M3 supplied candidates are identical in saved vector
bytes; these are two formulation checks on one event, not independent episodes.

I1's accepted support poses change by at most 0.272962 m and 0.000225060 rad;
body speed changes by up to 0.366138 m/s, angular speed by 0.000238751 rad/s.
Thus the success is not an unchanged-seed return. Its terminal speed is about
0.325486 m/s: the original formulation requires a terminal goal region, not a
zero terminal speed. No stop constraint was removed or added.

## Full physical acceptance and adverse outcomes

All four supplied selected candidates pass the unchanged original dense checker,
original independent plan checker and additional 6,018-time offset grid (maximum
actual gap 0.000629 s), including both knot acceleration sides. M2 also passes
independent obstacles and known-workspace checks. All queried derivative branches
in these actual supplied runs are smooth and valid: zero runtime ties/grid cuts,
invalid-domain gradients or rejected Log-cut derivatives were encountered.

| Accepted quantity | I0 supplied (both M2/M3) | I1 supplied (both M2/M3) | Original limit |
|---|---:|---:|---:|
| Max lateral m/s | 3.59782e-8 | 2.13281e-8 | 1e-5 |
| Linear speed range m/s | [0.325484,0.8] | [0.325486,0.8] | [0,0.8] |
| Max angular speed rad/s | 0.000138518 | 0.000143142 | 3 |
| Max linear acceleration m/s² | 1.093410 | 1.093750 | 2 |
| Max angular acceleration rad/s² | 0.00138627 | 0.00123757 | 5 |
| Original goal position error m | 0.000042264 | 0.000039533 | 0.15 |
| Original goal yaw error rad | 0.000054037 | 0.000059235 | 0.261799388 |
| Minimum direct-geometry footprint clearance m | 1.326945 | 1.326945 | 0.05 |

No selected candidate fails independent full acceptance. The adverse outcomes
remain visible: four FD final iterates fail, two FD starts provide no candidate,
and two FD starts only retain the seed. GP interpolation feasibility remains
sampled/refined with a swept-polyline environment check, not a continuous-time
safety proof. No candidate is executed through MPC in this task.

## Evaluation counts and compute costs

Each supplied start calls all three Jacobian interfaces 128 times for I0 or 121
for I1. A single-vector derivative cache shares the full computed Jacobian across
these interfaces: 256/242 derivative-cache hits and 128/121 actual AD evaluations.
Every mode retains exactly the original primal last-vector cache.

| Method / mode / start | Objective calls | Equality / inequality calls | Primal cache misses | Primal environment query batches | Core AD evaluations / analytic environment batches |
|---|---:|---:|---:|---:|---:|
| M2 / FD / I0 | 7432 | 7387 / 7387 | 21929 | 21929 | 0 / 0 |
| M2 / JAC / I0 | 370 | 371 / 371 | 370 | 370 | 128 / 128 |
| M2 / JAC / I1 | 409 | 410 / 410 | 409 | 409 | 121 / 121 |
| M2 / FD / I1 | 7242 | 7264 / 7264 | 21437 | 21437 | 0 / 0 |
| M3 / FD / I0 | 7029 | 7076 / 6931 | 20777 | 41554 | 0 / 0 |
| M3 / JAC / I0 | 370 | 371 / 371 | 370 | 740 | 128 / 256 |
| M3 / JAC / I1 | 409 | 410 / 410 | 409 | 818 | 121 / 242 |
| M3 / FD / I1 | 7051 | 6954 / 6954 | 20642 | 41284 | 0 / 0 |

Prepared solve time decreases by 92.90–94.19% and objective calls by
94.20–95.02% across matched pairs. Original primal cache misses decrease by
98.02–98.31%; primal GP interpolation calls equal these misses. These are single
observed runs, not a statistical runtime distribution. The derivative AD graph
also computes its own interpolation, explicitly counted separately above.

Original primal environment time decreases from 18.527–18.873 s to
0.341–0.401 s per start. Primal interpolation decreases from 5.905–6.147 s to
0.111–0.124 s. Supplied AD itself costs 0.258–0.275 s, plus 0.433–0.611 s for
analytic environment derivatives. Caller-inclusive timers and their nested
components are not added to each other. The figures stack primal evaluator,
derivative callbacks and outside-callback solver/bookkeeping time only.

| Method / mode / start | Graph construction s | Compilation + first-call warmup s | Dense post-check s | Independent full checks s | Cold setup + solve + checks s |
|---|---:|---:|---:|---:|---:|
| M2 / FD / I0 | 0.000 | 0.000 | 0.020 | 0.000 | 30.038 |
| M2 / JAC / I0 | 0.147 | 0.866 | 0.064 | 1.943 | 4.899 |
| M2 / JAC / I1 | 0.029 | 0.780 | 0.221 | 7.628 | 10.419 |
| M2 / FD / I1 | 0.000 | 0.000 | 0.012 | 0.305 | 30.334 |
| M3 / FD / I0 | 0.000 | 0.000 | 0.012 | 0.000 | 30.028 |
| M3 / JAC / I0 | 0.029 | 0.770 | 0.065 | 1.966 | 4.976 |
| M3 / JAC / I1 | 0.029 | 0.784 | 0.227 | 7.657 | 10.664 |
| M3 / FD / I1 | 0.000 | 0.000 | 0.012 | 0.303 | 30.332 |

Shared environment loading is 0.038555 s, reported once; case/seed setup is
about 0.015 s/start and included in the cold total. A fresh provider is compiled
for every supplied start in this experiment. Compilation is CPU-only and could
be reused for the same constants/shapes, but such reuse is not claimed in these
cold totals. The first graph construction includes JAX import costs; later
providers run in the same process. The cold table is measured per-start setup,
not repeated fresh-process interpreter launches.

Full checks cost more for successful supplied runs because every retained
collocation/dense-feasible callback is independently checked for the required
feasible-objective history. Initial/latest/duplicate vectors share a post-check
cache. I1 supplied full checking takes about 7.6 s and is not hidden in the
1.7–1.9 s prepared solve. Cold setup + solve + validation remains 4.899–10.664 s
for supplied starts versus 30.028–30.334 s for FD.

Optional optimality diagnostics occur after all eight starts, with separate
provider setup 0.791–1.036 s per record and local fixed-vector diagnostics
0.016–0.042 s. These are not charged to or stacked inside the prepared solve.

## Convergence and remaining uncertainty

At the feasible I1 seed, the sign-constrained multiplier-fit stationarity infinity
norm is 1.830836. At the supplied final candidates it is 0.00640364 for I0 and
0.000104895 for I1, with complementarity zero. Raw active-Jacobian ranks are 30
at the supplied finals and 31 at the I1 seed under the recorded rank rule; the
active systems are rank-deficient. These are unscaled mixed-unit diagnostics,
not universal acceptance thresholds. Final internal SLSQP-QP multiplier
stationarity norms are 0.0109126 and 0.000490781 respectively. Solver convergence
and full feasibility are established; exact nonlinear stationarity or global
optimality is not established. The FD seed return plus timeout establishes
neither convergence nor optimality.

At the supplied finals the active system has 31 rows but rank 30 because
inequality row 90 is the fixed boundary row v_max−v0=0 with zero derivative.
This structural row explains that rank deficiency; it is not evidence of solver
failure. At infeasible initial/final points, the active-set rule also includes
violated inequalities, so fitted multipliers there are descriptive diagnostics,
not KKT certification.

The supported interpretation is that supplying the verified derivative
information substantially reduces repeated primal evaluation and permits
feasibility recovery and lower-cost feasible candidates within the same budget
on this event. Improved derivative accuracy and reduced evaluation cost are
combined by this one supplied-Jacobian intervention; their individual causal
contributions are not separated. This benign event does not establish behavior
on obstacle-active hard cases, other events or real controller execution.

The next single experiment is to apply this unchanged verified provider, seeds,
solver and acceptance to a predeclared subset of the existing hard cases. No
hard-case optimization or controller experiment is performed here.

Numerical flags over SUPPLIED_JAC starts (per-start rows above remain decisive):
`infeasible_start_recovered=true`, `feasible_seed_objective_improved=true`,
`solver_converged_with_full_feasibility=true`, `returned_initial_unchanged=false`,
`compute_cost_reduced=true` for all four matched compute comparisons. These
flags do not classify navigation improvement.

## Figures and review artifacts

The [local index](../data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/index.html)
contains all eight outcomes and 18 figures (nine types for each method). Start
with the [full-feasible objective history](../data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/plots/M3_GP_CONSTRAINED/best_full_feasible_objective.png),
[world XY comparison](../data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/plots/M3_GP_CONSTRAINED/world_xy.png)
and [acceleration traces](../data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/plots/M3_GP_CONSTRAINED/accelerations.png).
Each figure has a numeric-data and source/hash sidecar. Full feasibility is
certified after solving and plotted at the recorded candidate discovery time;
there is no invented execution trace or feasible objective before one exists.

The [review ZIP](../data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/review_bundle.zip)
contains 74 allowlisted files, 7,616,326 bytes, SHA256
`38ab9ded352e37ca3bd932d68c22e042340bf94cb106df8b5ec074359b3fb88e`.
It includes compact tables, figures, numeric sidecars, per-start summaries and
the authoritative derivative gate; no raw RGB, dataset, environment export,
checkpoint, external source or full solver vectors are bundled.

Two earlier render attempts are preserved. Presentation-only corrections remove
spurious negative log-axis padding for positive objectives, use an integer-count
axis and move the XY legend outside the plot. All 18 numeric payloads and 40
per-start numerical artifact hashes, including eight solver results, are unchanged.
The separate transition audit and visual review record these corrections. The
initial minimal bundle is also preserved before adding compact review evidence.
None of these presentation or packaging operations reruns optimization.

The independent scientific review passes 224 checks over 95 hashed inputs and
sources, including original objective recomputation, exact seed/config/boundary
preservation, changed supplied candidates, timing/count accounting and the rank
interpretation. Its saved report retains the document hash at review time.

Final reporting-source pytest passes **1,650 tests, 19 skips in 70.11 s**;
compileall and `git diff --check` pass. Eighteen skips concern retired historical
corpora and one excludes S3 from an exact-reproduction assertion covered by its
dedicated representation-error test. An earlier sandboxed invocation has 1,648
passes and two local-IPC `PermissionError` failures; its log is preserved. The
unchanged full suite passes with local UNIX sockets permitted. No test or
acceptance tolerance is relaxed for that execution restriction.

The final independent [artifact validation](../data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/validation.json)
passes **48,889 checks with zero errors**, covering all eight actual starts and
846 frozen artifacts. It reconstructs original primal values, full candidate
checks, derivative error arithmetic, saved multiplier residuals, plot numbers
and ZIP provenance without an optimizer or finite-difference sweep. Its separate
CLI takes **48.21 s** measured externally with `/usr/bin/time`; this whole-run
reproducibility audit is separate from the per-start full-check times recorded
in the cold-cost table above.

The first artifact validation also passed (48,785 checks, 794 files; elapsed
time uninstrumented). The staged diff check subsequently found excess final
newlines in two new reporting helpers. Only those newlines were removed, with
identical Python ASTs verified. The first PASS, source snapshot and manifest
are preserved in `verification/pre_whitespace_correction/`; a correction receipt
records old/new hashes. The same validator then passes against the final source
snapshot above. Compilation and staged diff checks pass; numerical sources,
solver results, acceptance, figures and ZIP are unchanged.

Final rehashing preserves all 37,984 historical files, all 256 DIAG-01 files,
64 inventoried external files (including 63 checkpoint-tree files), external
Git state and both unrelated local configuration edits. The original eight
numerical/formulation/acceptance modules have no diff from the reviewed start.
Generated artifacts remain local and ignored; only code/config/tests/docs are
committed. Starting, comparison implementation and final reporting revisions,
with normal push status, are recorded in the run's `git_completion.json`.
