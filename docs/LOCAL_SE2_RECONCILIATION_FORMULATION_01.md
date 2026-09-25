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

## Result

Pending the pushed formulation freeze and single saved-only planning solve.
