# LOCAL_SE2_RECONCILIATION_FORMULATION_01

## Scope and pre-execution protocol

**PLANNING ONLY — NO MPC EXECUTION — NO RECONCILIATION PERFORMANCE CLAIM.**
This is a new spatial discrete SE(2) least-squares formulation, not historical
GP/M4. One saved-only development solve is permitted, on OSA03 REPEAT_00 only.
No new LightNav/RGB/Isaac, MPC, GP, rigid baseline, splice, correspondence,
controller continuation or method comparison. REPEAT_01 is not evaluated.

Starting `main` and fetched `origin/main`:
`5fa541a22082942da39f01dc809bab754b40b5c2`.
Two unrelated stage0 config edits remain unstaged and preserved.

Authoritative source: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`.
Representative `REPEAT_00`, classification
`QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE`.
Raw FRESH file SHA256:
`8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`.
The loader authenticates the sealed bundle, episode local/world arrays and raw
wire response; independently matches A/B to saved execution state IDs and re-runs
only the original R00 saved-record validator. No rounded report states are inputs.
Full source, input-contract and geometry hashes are saved before/after processing.

### Coordinates and objective

Isaac world XY metres/yaw radians CCW; body +x forward/+y left.
`A=R(t_obs)` and `B=R(t_switch)` are fixed. B is the preintegration state at
first actual FRESH-command application. Observation, request, receipt,
ready-seen, install and application clocks are retained separately in the source
manifest. Physical incoming command and advanced controller memory are distinct
source records; neither is a variable or a dynamics factor here.

The raw `F_j=A F_local,j` remains immutable and observation-anchored.
Variables are `X_0,...,X_(N-1)` for generic `N>=2`, initialized exactly at `F`.
The separate latency target is

```
G_AB = B A^-1
Ftilde_j = G_AB F_j
B^-1 Ftilde_j = A^-1 F_j
X_local,j = A^-1 X_j
```

All products/inverses/Log/Exp use the repository SE(2) helpers. The last line is
representation conversion in the original observation frame, not B re-anchoring.

Progress uses the **original** FRESH XY arc only:
`l_0=0`, `l_j=sum_(k<j)||p(F_(k+1))-p(F_k)||`, `s_j=l_j/l_(N-1)`.
Zero total arc fails closed. LightNav rows have **no intrinsic timestamps**;
there is no waypoint dt, support clock, twist node, GP interpolation or time cost.

```
w_L(s) = (1-s)^2                 w_A(s) = s^2
S = diag(0.10 m, 0.10 m, pi/18 rad)
||r||_S^2 = ||S^-1 r||_2^2

r_L,j = Log(Ftilde_j^-1 X_j)
r_R,j = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A,j = Log(F_j^-1 X_j)

E_L = sum_j w_L(s_j)||r_L,j||_S^2 / sum_j w_L(s_j)
E_R = sum_j ||r_R,j||_S^2 / (N-1)
E_A = sum_j w_A(s_j)||r_A,j||_S^2 / sum_j w_A(s_j)
E = E_L + E_R + E_A              lambda_L=lambda_R=lambda_A=1
```

Normalization scales are not physical success thresholds. Weighted residual
vectors use square roots of normalized weights, and the solver cost is `r.T r`
(no half factor). No schedule/scale/weight/initialization search is allowed.

At raw initialization E_R/E_A vanish. At full transport E_L/E_R vanish.
Common-left-transform invariance of the relative factor explains why EXP-02C
could propagate entry correction almost rigidly. The new downstream original
world-FRESH anchor explicitly breaks that degeneracy. These relative edges are
an intent-preservation **proxy**, not a proof of semantic navigation intent.
The transport retains original robot-relative FRESH structure, including any
intrinsic trackability difficulty measured in the observation-start control.

### Solver and safety acceptance

The unchanged numerical defaults from `graph_optimizer.SolverConfig` are frozen:
80 iterations, central FD epsilon `1e-6`, initial damping `1e-3`, rejection factor
10, acceptance factor .3, maximum damping `1e12`, gradient/step tolerance `1e-9`,
cost-decrease tolerance `1e-12`. Solve
`(J.T J + mu I) delta = -J.T r`, retract each node by `X_j Exp(delta_j)`.
No global additive pose updates or analytic Jacobians.

Two optional keyword-only callbacks add feasibility acceptance and copied-state
iteration telemetry. With feasibility callback `None`, historical arithmetic,
termination and `OptimizationResult` fields remain unchanged. Two frozen
synthetic nonlinear solver goldens were captured from the starting revision
**before editing**; these and historical graph callers are regression tests.
Synthetic test optimization is not experimental evidence.

Initial X must be feasible. A proposed step is accepted only when its cost
strictly decreases and the **entire returned X polyline** passes the existing
direct Hospital+runtime-cart mesh checker: circular footprint radius .20 m,
required footprint-edge margin .05 m, known workspace, unchanged environment
numerical tolerance (borderline fails closed). Radius is subtracted once.
Unsafe improving proposals receive exactly the same damping increase as other
rejected proposals. No projection, clipping, suffix deletion or obstacle cost.
No B→X0 connector is constructed or included in the check.

An optimization error preserves the last feasible state and trace with explicit
limitation status; no restart. The result is not called executable. Final
validation separately uses direct **full-union** GEOS distance and records
Hospital-only/cart-only clearance, limiting geometry, first minimum segment,
fraction and XY position. This checks reference geometry, not a future controlled
transition or continuous-time execution safety.

### Diagnostics frozen before solve

Save original/transported/optimized world arrays and optimized A-local array;
A/B/Log(A^-1 B), N, original arc, all factor costs; every LM proposal, decision,
damping, linearization gradient/step, all feasibility calls and accepted states.
Per-node Log corrections to original/transported target, per-edge relative Log
residuals/RMS/max, first-node and endpoint XY displacement.

Rigid-fit diagnostic is **closed-form XY Procrustes**, minimizing summed XY
squared error with one world left SE(2) transform. Translation RMS and the yaw
RMS under that same fit are reported separately. Zero-covariance tie uses zero
rotation. This is a descriptive fit, not an iteratively optimized baseline or a
method comparison. Nonzero fitted residuals describe nonrigidity without a
post-hoc success cutoff or requirement of monotone correction.

Figures: equal-axis world overlay (historical OLD prefix, A/B, raw FRESH, full
transport target, planned X, cart/margin); per-node corrections; fixed weights;
factor costs at accepted iterations. Each has numeric/source-hash JSON, with
per-node/cost CSVs and a static HTML index. No fabricated execution curves.

### Execution order and reproducibility

1. Saved-only `prepare`: authenticate R00, run source validator, freeze config,
   source/geometry/code hashes. No planning solve.
2. Focused and historical regressions; compileall; diff and staged diff review.
3. Commit/push implementation/config/tests/protocol. Scientific runner requires
   HEAD to equal live `origin/main`, committed code bytes to match the prepared
   hashes, and exclusively creates a one-call execution marker.
4. Run exactly one R00 planning solve from F; save results exclusively.
5. Saved-only validator reauthenticates source, reconstructs equations/retraction,
   checks accepted/rejected decisions, full-union safety, local frame, result and
   plot numeric hashes. No optimization, model or controller calls in validation.
6. Append report/work log, conservative README update, focused result commit/push.

The first loader preflight, `primary_20260925T010000Z`, stopped before any solve:
its response reader assumed unwrapped `actions` instead of the sealed
`{"action":"next","data":...}` wire envelope. That partial artifact is retained.
The corrected `primary_20260925T011000Z` passed saved-only authentication; it is
not a scientific result. Final execution directory and freeze SHA are recorded
below after the pushed freeze. No scientific outcome has informed this protocol.

Tests never invoke real LightNav or MPC. GP/support timing none; LightNav/MPC/GP/
actual execution calls all zero. Baseline/common-B execution remains a separate,
explicitly unimplemented task.

## Frozen input ledger

Final prepared directory: `data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/`.
Saved-only source validation PASS. Pre-execution regression **207 passed, 1 skipped**
(the ignored historical EXP-01B/EXP-02B corpus is absent). Compileall and diff check PASS.

- source manifest SHA256: `85a15e8c52499f8358fbcb6ef13c9045f2e2c3d6b7a926d9d77b085fb8898f05`
- config SHA256: `0db9d8b855de129db9016a268fb5fdfbf27bba0b0a0312c79e5c6bb70864b9a4`
- protocol SHA256: `ad2742b4fb7ece77710450615a32fc4eb16a256acad426ef0b200be6313321f2`
- R00 source-validation SHA256: `6735386c1522986bd2215bb1e4f40300c016fa9c1b83c9fc477d9582434627d5`

Exact commands (from repository root):

```bash
.venv/bin/python scripts/run_local_se2_reconciliation_formulation01.py prepare --run data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
# Implementation/config/tests/protocol commit and normal push precede execute.
.venv/bin/python scripts/run_local_se2_reconciliation_formulation01.py execute --run data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z
.venv/bin/python scripts/validate_local_se2_reconciliation_formulation01.py --run data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z
.venv/bin/python scripts/report_local_se2_reconciliation_formulation01.py --run data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z
.venv/bin/python scripts/validate_local_se2_reconciliation_formulation01.py --run data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z --output validation_final.json
```

## Result — one saved-only R00 planning solve

**SAFE_PLANNING_RESULT**; independent final validator PASS. No scientific retry,
weight change, schedule change, new initialization, source change or execution.

- Scientific formulation freeze: `65a16823513709865cb3f3a7c8aa70ee26ccd440` (normal push before solve).
- Source original acquisition freeze: `76352cf89f461df0c47dd2e7bb4011d665df905d`.
- Run: `data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/`.
- Input FRESH SHA256: `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`.
- Input OLD SHA256: `6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae`.
- Input world FRESH SHA256: `9c8ea94f650d823ac3e1ada0a216dc36758432713fd5407cf18324069865bfc4`.
- Input state/timing SHA256: `8803b3dc1da1b310ba69f3aea87142e35638c2999dfdadba961740be2a3a6cb3`.
- Code/config/source hashes, raw arrays, all rejected proposals and actual
  source clock records are preserved in the run. Result metadata:
  [LOCAL_SE2_RECONCILIATION_FORMULATION_01_RESULT.json](LOCAL_SE2_RECONCILIATION_FORMULATION_01_RESULT.json).

### Source state and clocks

```
A = [19.203242163778583, 24.356668352984542, -1.5689752526514715]
B = [19.203630553052257, 24.14333536015282, -1.5689760264727979]
Log(A^-1 B) = [0.21333334637784984, -2.3811842714752392e-08, -7.738213265717775e-07]
```

Observation→B displacement is 0.213333346378 m;
actual saved travel 0.213333346378 m; yaw shift
-4.43366961098e-05 deg. N=10; raw XY arc
1.35324552012 m. A/B are state IDs 75/92.

| Event | Simulation time [s] | Host monotonic [s] |
|---|---:|---:|
| FRESH observation | 1.28333340026 | 80643.0010666 |
| Request | N/A | 80643.0978328 |
| Receipt | N/A | 80643.3248704 |
| Ready seen | 1.5333334133 | 80643.3319576 |
| Install | 1.5333334133 | 80643.3324271 |
| First FRESH application B | 1.56666674837 | 80643.4521231 |

These are original saved clocks, not new execution times. No mixed-clock
subtraction and no timestamp is assigned to a FRESH row.
Physical `u_minus` and controller memory are saved unchanged in the manifest;
they are not used to imply motion feasibility in this spatial objective.

### Objective and solver

| Factor | Initial X=F | Final X |
|---|---:|---:|
| E_L | 4.55111637312 | 0.348589942563 |
| E_R | 2.60325514947e-33 | 0.0684801182302 |
| E_A | 2.54492466144e-28 | 0.348193565422 |
| E_total | 4.55111637312 | 0.765263626215 |

Termination: **cost_tolerance**, 5 iterations,
4 accepted steps, 1 non-improving rejection,
**0 unsafe rejections**. LM wall time including acceptance
and telemetry: **0.071575423 s**; no claim of online latency improvement.
Initial/accepted-state feasibility calls: 5. The safety gate was implemented/tested
but did not bind on this development solve.

Iteration 4 was non-improving at floating-point precision and raised damping;
iteration 5 was accepted with decrease below the unchanged `1e-12` cost tolerance.
Last linearization gradient infinity norm `3.89097706499e-07` and
step norm `3.58114534003e-08` are **not** claims of a global optimum or
an independently certified final stationary point. No monotonic-correction,
attachment, smoothing or percentage-improvement acceptance gate was imposed.

| Iteration | Decision | Damping used | Gradient infinity norm | Step norm |
|---:|---|---:|---:|---:|
| 1 | accepted | 0.001 | 5.77317196495 | 0.420058483023 |
| 2 | accepted | 0.0003 | 0.00106732535996 | 0.000127146184042 |
| 3 | accepted | 9e-05 | 1.26532241641e-05 | 2.38423812907e-06 |
| 4 | rejected_non_improving | 2.7e-05 | 3.89097706499e-07 | 3.58122341182e-08 |
| 5 | accepted | 0.00027 | 3.89097706499e-07 | 3.58114534003e-08 |

### Spatial deformation and relative-edge preservation

All translation corrections below are norms of SE(2) Log translation components;
yaw columns are absolute shortest-angle residuals, in degrees.

| j | Original arc s | w_L | w_A | Original→X [m] | Original→X yaw [deg] | Ftilde→X [m] | Ftilde→X yaw [deg] |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0.00000000 | 1.00000000 | 0.00000000 | 0.21114402 | 0.05061747 | 0.00218997 | 0.05066181 |
| 1 | 0.11140359 | 0.78960359 | 0.01241076 | 0.20547074 | 0.10957651 | 0.00786304 | 0.10962085 |
| 2 | 0.22261421 | 0.60432867 | 0.04955708 | 0.19033738 | 0.20263510 | 0.02299647 | 0.20267943 |
| 3 | 0.33390950 | 0.44367656 | 0.11149555 | 0.16359609 | 0.29575570 | 0.04973782 | 0.29580004 |
| 4 | 0.44532313 | 0.30766643 | 0.19831269 | 0.12683086 | 0.34139417 | 0.08650306 | 0.34143851 |
| 5 | 0.55644902 | 0.19673747 | 0.30963551 | 0.08612582 | 0.31257478 | 0.12720809 | 0.31261912 |
| 6 | 0.66723554 | 0.11073218 | 0.44520327 | 0.04951773 | 0.22631156 | 0.16381624 | 0.22635590 |
| 7 | 0.77806883 | 0.04925344 | 0.60539111 | 0.02290580 | 0.12702534 | 0.19042830 | 0.12706968 |
| 8 | 0.88873140 | 0.01238070 | 0.78984351 | 0.00784166 | 0.05228983 | 0.20549261 | 0.05233417 |
| 9 | 1.00000000 | 0.00000000 | 1.00000000 | 0.00218683 | 0.01470692 | 0.21114779 | 0.01475125 |

First-node XY displacement **0.211144018 m**;
endpoint XY displacement **0.002186831 m**.
Translation correction decreases over these ten nodes. This observed profile
was not an imposed monotonicity constraint. Maximum node yaw correction is
**0.341394°**.
The endpoint is softly recovered, not hard-fixed to the raw endpoint.

Original-relative-edge Log distortion:

- translation RMS **0.026158510 m**, max **0.040260838 m**;
- yaw RMS **0.073043299°**, max **0.099286219°**.

| Edge | Log translation x [m] | Log translation y [m] | Log yaw [deg] |
|---:|---:|---:|---:|
| 0→1 | -0.004919275 | 0.002693305 | 0.058959035 |
| 1→2 | -0.013103193 | 0.007284627 | 0.093058588 |
| 2→3 | -0.023104072 | 0.012934543 | 0.093120604 |
| 3→4 | -0.031719090 | 0.017815320 | 0.045638473 |
| 4→5 | -0.035101614 | 0.019718311 | -0.028819389 |
| 5→6 | -0.031591099 | 0.017682280 | -0.086263221 |
| 6→7 | -0.023008426 | 0.012780557 | -0.099286219 |
| 7→8 | -0.013055075 | 0.007184590 | -0.074735516 |
| 8→9 | -0.004914448 | 0.002660517 | -0.037582912 |

Closed-form single-left-transform XY fit:
`G_fit=[2.6469564300931268, -2.0553547373790764, 0.1068589617025278]` (world translation m, yaw rad).
Residual translation RMS **0.066834803 m** and yaw RMS
**5.950349118°**. Its large translation parameters depend
on the world origin and rotation; the fit residuals and node corrections are the
relevant deformation diagnostics. This result is not an exact common rigid
transport. It is not evidence that a rigid baseline would execute worse.

### Independent reference-geometry safety

| Returned polyline | Hospital-only [m] | Cart-only [m] | Combined minimum [m] | Limiter | First minimum segment/fraction |
|---|---:|---:|---:|---|---|
| Original FRESH | 0.733862404 | 0.229486890 | 0.229486890 | cart | 8 / 1.000000000 |
| Optimized X | 0.733801607 | 0.227809208 | 0.227809208 | cart | 8 / 1.000000000 |

Both workspace and full returned-polyline checks pass the unchanged **.05 m**
edge-margin requirement with .20 m radius and `1e-7 m` numerical tolerance.
Optimized minimum is at XY `[19.881680721352023, 23.183539797358552]`;
nearest cart surface XY `[19.61594947, 22.84826734]`.
Minimum clearance decreases by
**0.001677682 m** but remains valid.
No prefix/suffix was deleted. No B→X0 connector was constructed or certified.

In the world figure, the wide cart+.25 m region is a **robot-center exclusion**
region. The displayed .20 m footprint circle may overlap that wide region;
its edge must avoid the cart+.05 m margin, not another copy of the robot radius.
The numeric direct footprint-edge check is authoritative.

### Artifacts, validation and call accounting

- [Static planning review](../data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/review/index.html)
- [World XY figure](../data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/review/figure_1_world.png)
- [Per-node corrections](../data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/review/figure_2_corrections.png)
- `review/figure_3_weights.png`, `review/figure_4_costs.png`; each figure has JSON
  numeric/source-hash sidecar; `nodes.csv`, `factor_costs.csv` retain numeric data.
- `derived/optimized_world.npy` and `derived/optimized_original_A_local.npy`
  are derived outputs, never replacements for sealed raw data.
- `source_manifest.json`, `freeze_inputs.json`, `protocol.json`,
  `source_validation.json`, `solver_trace.json`, `feasibility_checks.json`,
  `result.json`, `result_hashes.json`, `validation.json`, `validation_final.json`.

Saved-only validation PASS: original hashes preserved, exact R00 state/anchor,
source qualification, independent factor equations, right-local candidate
reconstruction, acceptance/rejection/damping arithmetic, all accepted states
safe, separate full-union final safety and plot/CSV/JSON numeric/hash parity.
All four PNGs were visually inspected. No new scientific solve for validation.

Relevant final regression: **207 passed, 1 skipped** (unavailable ignored historical
EXP-01B/EXP-02B corpus); compileall and diff check PASS. Synthetic solver tests and
historical graph regression fixtures are implementation checks, not scientific
comparators. Real model/MPC calls in tests: **0**.

Scientific saved-only local planning solves **1**; LightNav **0**, MPC **0**, GP
**0**, actual execution **0**, rigid baseline/splice/other-source solve **0**.
GP support/timing **none**; waypoint intrinsic timing **none**. No R01 evaluation.

### Repository-confirmed facts

The three-factor discrete SE(2) formulation and invariance tests pass. One safe
spatial candidate reduces the frozen objective and trades early measured
transport against relative-edge distortion and downstream original-world
recovery. Its first-node correction is about 21.1 cm and endpoint correction
about 2.19 mm, with nonzero nonrigidity measured above. No unsafe proposal was
accepted or needed to be rejected in this run.

### Research interpretation

This is implementation and single-source planning evidence for a localized
spatial correction rather than a full rigid suffix displacement. Original
relative-motion preservation is a soft proxy; the optimizer does change edges.
The observation-relative FRESH heading structure is substantially retained.
There is no evidence here that intrinsic upstream trackability was repaired,
or that actual B-start tracking becomes easier. No additive latency decomposition
or maximum-recoverable-cost interpretation is justified.

### Not demonstrated

Earlier actual attachment, lower execution AUC, smoother commands, controller
feasibility, obstacle traversal, reconciliation/rigid/splice superiority or
inferiority, online benefit and generalization are all untested. The largest
remaining uncertainty is whether this safe reference deformation actually
improves B-start Native execution while preserving intent and execution safety.
The common-B execution comparison is not implemented in this task.
