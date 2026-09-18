# GP-SE2-DIAG-01: candidate rejection and feasibility recovery

The fixed benign event now has a fully checked GP candidate for both M2 and M3,
without changing its physical acceptance criteria. Replacing only the second
initialization with a fixed same-curvature deceleration supplies that candidate.
Both methods return the feasible initial seed unchanged; neither improves it to
another feasible candidate within its original budget. This recovers candidate
availability, not optimizer convergence or executed navigation performance.

Operational status: **GP_SE2_DIAG_01_COMPLETED_WITH_LIMITATIONS**.
Diagnosis status: **PARTIALLY_LOCALIZED**.
Actual-event status: **FULL_FEASIBLE_CANDIDATE_FOUND**.

The historical 10-case pilot remains unchanged. This diagnosis uses one actual
event, four optimizer-free mathematical fixtures, six synthetic solver starts,
four actual baseline starts and four actual single-change starts. It performs no
new inference, RGB collection, environment export, MPC solve, rollout or GUI run.

## Provenance and preserved inputs

Started on `main` at `f0cd9de405f4b03cd6871ffdffd1dffeb74475c8`, matching fetched
`origin/main`. The historical numerical freeze remains
`25d65ffab091c8155a1ace4ddef89a2a5739e911`; its core SE(2), GP, formulation,
reference, environment, evaluation and rollout files still match by SHA-256.
The disclosed GP-SE2-01 validator-only bookkeeping correction is separate from
that numerical freeze. No historical method outcome or termination was rewritten.

New run:
`data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/`.
Versioned diagnostic configuration: `configs/gp_se2_diag_01.yaml`. The run copies
this file and records its hash; the physical formulation is loaded directly from
the original immutable config, SHA-256
`15f3f1580f13d437cff0bf88fba59b8526b209e070692bfaab8367f2784863f6`.
The new diagnostic config SHA-256 is
`03f08115ccb906d4a39c65c821f6e8a8cce3cc5d652c5002f00fa30eeae3db97`.

Before/after inventories cover 37,984 retained historical files, including the
original primary, verification and environment export directories, and all 63
checkpoint-tree files. The previously validated 881-event source remains byte
identical; the chosen event is additionally reconstructed independently from its
raw acquisition manifests, execution, command and controller records. External
LightNav remains clean at `c6f40e3220edbf7011e4f17eaf2c865416737d4d`; official MPC
SHA-256 remains `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
The two unrelated Stage 0 config edits remain unchanged and unstaged.

Each diagnostic phase archives its own source hashes and source files. The runner
was extended between phases; phase snapshots distinguish this from the unchanged
core numerical model and instrumented solve harness. Later additions provide
phase guards, reporting, plotting and independent validation. The final commit
and normal-push result are recorded in local `git_completion.json` after commit,
avoiding a self-referential tracked commit hash.

## Reclassification of the 40 historical starts

The audit was completed before any new optimization. It reevaluates exactly the
80 stored initial/latest vectors and reproduces all 80 saved report maxima and
feasibility flags. The 40 absent best-feasible-intermediate vectors are recorded
as insufficient saved evidence. Full historical callback history is unknown;
there is no reconstructed historical convergence curve.

| Diagnostic class | Attempts / 40 |
|---|---:|
| A INITIAL_ALREADY_FEASIBLE | 0 |
| B COLLOCATION_INFEASIBLE | 40 |
| C COLLOCATION_PASS_DENSE_FAIL | 0 |
| D DENSE_FEASIBLE | 0 |
| E NUMERICAL_EVALUATION_ERROR | 0 |
| F INSUFFICIENT_SAVED_EVIDENCE for latest attempt classification | 0 |

All 40 initial vectors are also collocation-infeasible. Class F applies separately
to the missing intermediate records, not to these available initial/latest
vectors. Terminations are independently **39 TIMEOUT, 1 SOLVER_FAILURE**.
Timeout is not itself a constraint family or an infeasibility proof.

Every latest iterate violates lateral velocity at a midpoint and at off-collocation
queries; support lateral velocity is structurally zero and fails in 0/40. Linear
acceleration violations occur at right knot limits in 40/40, left limits in
39/40, midpoints in 18/40 and off-collocation queries in 36/40. These sets overlap.
There are no cases attributable solely to a dense-check blind spot.

Additional overlapping latest failures are linear speed 20/40, angular speed
3/40, angular acceleration 5/40, goal position 7/40, goal yaw 2/40, gate plane
3/4 gate-required starts, gate direction 1/4 and obstacle clearance 2/20 M3
starts. Workspace and pose/body derivative identity fail in 0/40.

`existing_failure_audit/residuals.csv` records family, physical unit, actual min/max,
allowed interval, nominal excess, excess after tolerance, worst time, interval,
support/midpoint/off-collocation and derivative side. The original goal-position
constraint uses squared distance in m²; its numerical tolerance is not silently
treated as a distance in metres. Actual goal distance is recorded separately.

## What the GP representation reproduces

All fixtures have T=3 s, h=0.1 s, 31 supports and a translated/rotated world
boundary. Independent DOP853 world-unicycle integration and five-point world-pose
differences verify the GP pose/body twist/acceleration relationship, including
both sides of knots. Synthetic fixtures are mathematical and solver diagnostics,
not research performance evidence.

For S2, q(t)=t−t²/(2T) has units seconds, q_dot is dimensionless and q_ddot has
units 1/s. With eta=[0.3 m/s,0,0.4 rad/s], X=B Exp(q eta), nu=q_dot eta.
Because every local logarithm and derivative has the same algebra direction,
the Hermite polynomial reproduces this quadratic q exactly up to roundoff.
Its acceleration is [−0.1 m/s²,0,−0.133333 rad/s²]. The GP prior residual is
nonzero, as predicted analytically; feasibility does not imply zero prior cost.

| Fixture | Max position reconstruction error m | Max lateral m/s | Original dense feasibility |
|---|---:|---:|---|
| S0 constant straight [0.3,0,0] | 9.16e−16 | 8.33e−15 | PASS |
| S1 constant turn [0.3,0,0.4] | 9.16e−16 | 7.84e−15 | PASS |
| S2 fixed-curvature smooth deceleration | 7.77e−16 | 7.03e−15 | PASS |
| S3 v=0.3, omega=0.2+0.1t | 2.55e−11 | 5.17e−10 | PASS |

For S3, tightening the ODE tolerances changes position by at most 9.39e−16 m,
well below the observed reconstruction error. The largest linear-acceleration
reconstruction error is 7.65e−8 m/s². Thus true unicycle motion and its sampled
GP reconstruction are distinct, but this specified variable-curvature example
still meets the original tolerance. It is not evidence of general GP infeasibility.
S0–S2 independent interior pose-difference twist errors are below 8.3e−13 in
their component units; the largest one-sided acceleration FD discrepancy is
about 1.2e−8, below the physical acceptance tolerance.

The separate prescribed blind-spot interval has X0=[0,0,0], X1=[0.03,0,0],
nu0=nu1=[0.3,0,0.08], h=0.1 s. Actual lateral outputs are:

| Fraction of interval | Lateral m/s | Within ±1e−5 m/s? |
|---|---:|---|
| 0 | 0 | PASS |
| 0.25 | +0.000224999965 | FAIL |
| 0.5 | −2.14e−20 | PASS |
| 0.75 | −0.000224999958 | FAIL |
| 1 | +8.57e−20 | PASS |

The dense maximum is 0.000230939640 m/s at 0.0789 s. Independent pose
differentiation agrees with body velocity to 9.57e−11. This GP interpolant is an
explanation of a collocation blind spot, not a feasible physical fixture or the
measured cause of the historical 40 failures.

## Solver behavior at the actual 3-second size

The original solver uses 150 variables, 30 lateral equalities and the original
motion, goal, workspace and obstacle rows. The synthetic known/perturbed checks
have 903 inequalities. Gaussian perturbations use seed 20260918 and standard
deviations [0.01 m,0.01 m,0.5 degrees,0.02 m/s,0.02 rad/s] in the original
right-local optimization chart; the first pose/twist stay fixed. The exact
150-vector is saved and is identical across S0/S1/S2.

| Fixture / start | Initial feasible | Termination / iterations | Accepted source | Objective initial → accepted | Recovered from infeasible |
|---|---|---|---|---:|---|
| S0 known | yes | CONVERGED / 1 | unchanged initial | 4.79e−26 → same | no |
| S0 perturbed | no | TIMEOUT / 134 | callback 134 | 88.815456 → 2.38e−7 | yes |
| S1 known | yes | CONVERGED / 1 | unchanged initial | 6.62e−26 → same | no |
| S1 perturbed | no | TIMEOUT / 134 | callback 131 | 89.524709 → 2.46e−7 | yes |
| S2 known | yes | TIMEOUT / 134 | callback 128 | 0.0416667 → 0.0340423 | no, feasible start improved |
| S2 perturbed | no | CONVERGED / 129 | callback 129 | 89.187906 → 0.0340423 | yes |

S0/S1 known solves take about 0.23 s each. S0/S1 perturbations and S2 known
use 30 s; S2 perturbed takes 28.781 s. All six have sampled feasible candidates,
but only three terminate with solver success. Dense maximum equality residuals
of recovered candidates are 2.95e−9, 1.14e−9 and 4.17e−8 respectively; motion
inequality excess is zero. This separates termination, retained initial witness,
objective improvement and recovery. S2 is not claimed to be a global optimum.

The old tests included 3-second exact-constant-twist acceptance, but the
nontrivial curved-mismatch convergence test had only a 0.5-second horizon.
Neither established 3-second recovery from perturbed states. This task explicitly
performs those full-size checks. Synthetic free-space callbacks are constant and
cheap; their convergence speed is not the Hospital baseline speed.

## Fixed actual event and original criteria

The case is resolved through the original manifest, not a guessed directory:
`episode_001_repeat_01/handoff_002`. Its copied input snapshot preserves:

- B=[19.20116864141151 m,19.680000209519797 m,−1.5700932780181693 rad].
- Physical u_minus=[0.8 m/s,−0.00011474391164847372 rad/s]. Controller memory
  happens to equal it in this event but remains a separate recorded field.
- Initial GP twist=[u_minus.v,0,u_minus.omega].
- Original goal=[19.202068577041054 m,18.50631557706402 m,−1.5699926323631952 rad].
- F_native SHA `11cd36c2fb58743d3e16849a4991ab4c2307454313651620bd8fd7cebc0fbc35`;
  F_common SHA `4648cfd98f6fa555824d36b1f96581bcf4fb6f75cc967649a343a2fe95ac55d8`.

Coordinates are fixed Isaac world metres, +Z up, CCW radians; body +x forward,
+y left. Reference rows retain their original evaluation times 0.1,...,3.0 s;
these are not intrinsic LightNav timestamps. Observation/readiness/activation
timestamps and capture transform remain in the saved context. No projection,
reanchoring or resampling occurs in this task.

Original bounds are v in [0,0.8] m/s, |omega|≤3 rad/s, |a_v|≤2 m/s²,
|a_omega|≤5 rad/s², lateral/equality and inequality tolerances 1e−5, goal radius
0.15 m and yaw tolerance 15 degrees. Footprint radius is 0.20 m, required edge
clearance 0.05 m, height band [0.05,0.65] m. The original clear-shortcut route
requires no gate. Both methods retain known-workspace constraints; M3 additionally
uses the original conservative obstacle grid. Final full acceptance for either
method includes independent direct geometry, not just the optimizer grid.

## Baseline, profiling and the single change

The baseline exactly reproduces both historical initialization vectors in both
methods. All four new starts time out with lateral and linear-acceleration
violations. SLSQP still has no supplied Jacobian: SciPy uses numerical forward
differences in the actual 150-variable chart. M2 has 30 equalities/813 inequalities;
M3 has 30/903. All solve processes use one NumPy and SciPy OpenBLAS thread.

| Phase / method / start | Iterations | Objective / equality / inequality calls | Unique vectors | Latest max lateral m/s | Latest absolute linear acceleration m/s² |
|---|---:|---:|---:|---:|---:|
| Baseline M2 FRESH | 47 | 7281 / 7233 / 7233 | 7281 | 0.00496251 | 2.01904901 |
| Baseline M2 constant twist | 47 | 7367 / 7416 / 7326 | 7367 | 0.000510576 | 2.00149850 |
| Baseline M3 FRESH | 46 | 7092 / 7079 / 7079 | 7092 | 0.00403223 | 2.05453444 |
| Baseline M3 constant twist | 46 | 7190 / 7110 / 7110 | 7190 | 0.000716286 | 2.00227694 |
| Variant M2 unchanged FRESH | 47 | 7335 / 7384 / 7360 | 7335 | 0.00496251 | 2.01904901 |
| Variant M2 deceleration | 48 | 7468 / 7419 / 7419 | 7468 | 0.000275800 | 2.00143464 |
| Variant M3 unchanged FRESH | 46 | 7077 / 7079 / 7079 | 7077 | 0.00403223 | 2.05453444 |
| Variant M3 deceleration | 46 | 7133 / 7109 / 7109 | 7133 | 0.00141823 | 2.00224395 |

Every row uses about 30 s. These are the **rejected latest iterates**, including
the deceleration starts; accepted candidates come from their saved initial seeds.
The variant's larger latest residual than another run does not make its preserved
feasible initial seed invalid. All candidate checks and actual callbacks remain
available, with no fabricated intermediate history.

Baseline time per start is approximately 18.345–18.660 s environment queries,
6.04–6.20 s GP interpolation, 1.93–1.98 s GP prior residuals and 0.27–0.29 s
whitening. Environment cost includes workspace geometry in M2; it must not all
be called obstacle cost. Repeated forward-difference calls reevaluate the same
vectors across objective and constraint requests: distinct vectors and evaluator
cache misses are separate counters. Measured instrumentation bookkeeping is
0.133–0.140 s/start, with timer overhead estimated separately. These component
times are subsets of evaluator time; caller-inclusive timings are not added to
them. Profiling did not supply analytic derivatives or change the mathematics.

Heavy dense callback validation is deferred to after solve in the diagnostic
harness. Every actual collocation-feasible callback, initial and latest iterate
is then checked, retaining the original minimum-objective dense-feasible
selection rule. This changes validation timing, which is disclosed; it does not
relax acceptance. Both compared variants use the same instrumentation. The
historical run cannot be expected to end at the same iteration under a wall-clock
budget. Post-solve and independent full-check costs are reported separately.

After saving the audit, all math results, six synthetic solves and four actual
baseline starts, `decision.json` selected **B_FEASIBLE_INITIALIZATION**. Its SHA
is `25ee737ba6000fb5377f1eb34dc9312a603c8b2ec8e6582d6d4b9e5d3a7edaf9`.
Only start 1 changes to the fixed S2 family using the actual initial twist:

```
eta = [actual v_minus, 0, actual omega_minus]
T = 3 s
q(t) = t - t²/(2T)
X(t) = B Exp(q(t) eta)
nu(t) = (1 - t/T) eta
```

No family parameter is fitted to FRESH or the goal. Start 0, solver, objective,
weight, constraints, tolerances and grids remain unchanged. There is no RAW
rollout seed or restoration phase. Each method still has two starts at 200
iterations/30 s each. Seed construction costs 0.0000864 s for M2 and 0.0000685 s
for M3 and is charged inside the respective 30 s budget. Setup, post-checking
and separate seed/full-candidate validation remain visible. Each method's full
validation takes about 0.2 s per candidate/seed check, rather than being hidden
inside the solver cost. No second remedy was tried.

A derivative change was not combined with this initialization change. Collocation
refinement was not selected because the actual historical dense-only count is
zero. The measured computational bottleneck remains a separate hypothesis for
limited progress, not a proven unique cause.

## Full candidate checks and interpretation

Both M2 and M3 change from no candidate to the **same independently valid initial
deceleration seed**. Its objective is 3.963365592; the solver's accepted objective
improvement is exactly zero. Both selected sources are `initial`, both return
that vector unchanged, neither reports recovery from an infeasible start, and
both solver terminations remain TIMEOUT.

| Accepted candidate quantity | Measured value | Original bound |
|---|---:|---:|
| Maximum absolute lateral velocity | 9.12e−12 m/s | 1e−5 m/s |
| Linear speed range | [0,0.8] m/s | [0,0.8] m/s |
| Maximum absolute angular speed | 0.000114744 rad/s | 3 rad/s |
| Maximum absolute linear acceleration | 0.266666667 m/s² | 2 m/s² |
| Maximum absolute angular acceleration | 0.0000382480 rad/s² | 5 rad/s² |
| Original goal position error | 0.026315621 m | 0.15 m |
| Original goal yaw error | 0.000272762 rad | 0.261799388 rad |
| Minimum direct-geometry footprint clearance | 1.326944970 m | 0.05 m |

The original dense checker and original independent `evaluate_plan` pass. An
additional offset grid contains 6,018 times, combines a nominal 1 ms grid with
offset fraction 0.371, and has maximum actual gap 0.000629 s. It checks both knot
acceleration sides and 5,981 independent pose-difference probes at epsilon 1e−5 s.
Maximum pose/body derivative discrepancy is 3.19e−10 m/s and 2.18e−11 rad/s.
Fixed B, initial twist, original goal/route, known workspace, obstacle clearance
and original footprint all pass. M2 success is therefore not merely an
obstacle-free diagnostic. These remain sampled checks with a swept-polyline
environment check, not a continuous-time safety proof.

The actual event has a feasible GP witness under its original conditions, so
the earlier no-candidate result is not evidence that GP cannot represent an
executable motion for this event. Candidate availability depends on the seed
and the solver's ability to restore/retain feasibility within the budget.
Synthetic recovery and the measured timing further narrow the diagnosis, but
they do not isolate derivative error, conditioning or insufficient iterations
as a unique root cause. Generalization beyond this benign event is unknown.

The next single element to test is **verified objective gradients and constraint
Jacobians with the same SLSQP, objective, seeds and budget**. This is a proposal,
not an implemented second variant. SciPy's [SLSQP documentation](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html)
describes its numerical-differentiation settings; the installed runtime is
SciPy 1.18.1. Any future derivative implementation must independently verify the
actual chart and nonsmooth environment/angle branches before comparison.

## Review artifacts and reproduction

The compact [review ZIP](../data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/review_bundle.zip)
contains 23 allowlisted small files (2,664,669 bytes), including tables, decision,
representative PNGs and numeric/source-hash sidecars. It contains no raw RGB,
full dataset, checkpoint or external source. Its SHA-256 is
`691e8d8e3d2934830b6de191bfe5bd30ab583f0c5c68bdb6d14865192523aaba`.

Representative images:

- [Actual M3 OLD/FRESH/B/candidate XY, context and zoom](../data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/plots/actual_event/M3_GP_CONSTRAINED/world_xy.png).
- [Actual constraint history](../data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/plots/actual_event/M3_GP_CONSTRAINED/constraint_history.png).
- [Lateral velocity and linear/angular acceleration](../data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/plots/actual_event/M3_GP_CONSTRAINED/lateral_and_acceleration.png).
- [Profiling](../data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/plots/actual_event/M3_GP_CONSTRAINED/profiling.png).
- [Synthetic collocation blind spot](../data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/synthetic_fixtures/math/collocation_blind_spot/lateral_velocity_vs_time.png).
- [Synthetic S2 perturbation recovery](../data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/plots/synthetic_fixtures/S2/perturbation_recovery_history.png).

There are 13 optimizer-free mathematical PNGs and 13 audit/solver PNGs. Synthetic
and actual-event directories are separate. Rejected latest iterates are labelled
REJECTED. Accepted GP candidates have no invented execution trace. Plot sidecars
retain plotted data and input hashes; the original math plots have supplemental
trace/hash provenance and an integrity inventory.

Reproduce into a **new** run ID, preserving the phase order:

```bash
DIAG_RUN="$PWD/data/robotless_gp_se2_diag_01/NEW_DIAGNOSTIC_RUN_ID"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export MPLCONFIGDIR=/tmp/gp_se2_diag_01_mpl
.venv/bin/python scripts/run_gp_se2_diag_01.py prepare --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_01.py audit --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_01.py fixtures --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_01.py baseline --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_01.py synthetic-solves --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_01.py decide --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_01.py variant --run "$DIAG_RUN"
.venv/bin/python scripts/plot_gp_se2_diag_01.py --run "$DIAG_RUN"
.venv/bin/python scripts/run_gp_se2_diag_01.py finalize --run "$DIAG_RUN"
.venv/bin/python scripts/validate_gp_se2_diag_01.py --run "$DIAG_RUN" \
  --output "$DIAG_RUN/validation.json"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

Output directories/files refuse overwrite; incomplete technical outputs are
evidence, not silently retried. Validation of this existing run can be repeated
without `--output`; it performs no optimization. Decision execution stops for
review if the diagnostic prerequisites no longer support its declared rationale.
Wall-clock iteration counts can vary; no unbounded retry or output-driven event
replacement is allowed.

## Final validation and disclosed validator correction

The authoritative independent result is
`verification/validation.json`: **valid=true, 47,216 checks, errors=[]**.
It reevaluates saved historical and new vectors, candidate selection, original
and additional physical checks, all input hashes, plot numbers/source links,
the review ZIP and final aggregate tables. It invokes zero optimizers, MPC
rollouts or inference calls.

The first report, preserved at the run's root `validation.json`, has 14 metadata
comparison errors: six synthetic summary labels use `known`/`perturbed` while
their solver records use `S0_known`, etc.; eight aggregate source paths are
repository-relative while the validator constructed absolute paths. No physical,
numeric or hash comparison failed. The correction checks the specific expected
fixture label and resolves source paths against the repository root, retaining
wrong-label/wrong-path rejection and exact source-hash checks.

Only the diagnostic validator and its tests were corrected. Fourteen new
parameterized regression cases cover those six label combinations and eight
phase/method/start paths, including working-directory independence. The corrected
validator SHA-256 is
`c2bbf100970ecc84d99fa0208eccb2c7b705ce7d3a610600ea8050d4282b0d91`.
`verification/correction_scope.json` records every initial error, old/new source
hashes and correction boundaries; corrected source and tests are archived there.
All **243 frozen scientific artifact hashes remain identical**. No numerical
experiment, accepted candidate, constraint or plot was rewritten or rerun.

The initial full suite passed 1,520 tests with 19 skips in 52.25 s. After the
validator regression additions, the final full suite passed **1,534 tests with
19 skips in 51.11 s**; exact logs and hashes are recorded in
`verification/test_results.json`. Eighteen skips concern retired
historical corpora; one explicitly excludes S3 from an exact-reconstruction test
because its distinct reconstruction-error test covers it. Test counts are
implementation validation, not research effect sizes. Compileall and diff checks
pass; this task adds no shell launcher.
