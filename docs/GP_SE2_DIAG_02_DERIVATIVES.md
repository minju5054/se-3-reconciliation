# GP-SE2-DIAG-02: supplied derivatives under the fixed handoff formulation

This bounded diagnostic compares the original numerical finite differences with
supplied objective gradients and both constraint Jacobians. The actual case is
`episode_001_repeat_01/handoff_002`. Its two starts are the unchanged FRESH start
and the previously accepted same-curvature deceleration seed. This document is
the pre-comparison implementation record; measured results are appended after the
eight frozen runs, without changing the experiment.

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
