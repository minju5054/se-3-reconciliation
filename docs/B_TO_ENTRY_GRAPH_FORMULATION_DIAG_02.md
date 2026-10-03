# B_TO_ENTRY_GRAPH_FORMULATION_DIAG_02

## Frozen protocol

Starting HEAD and fetched origin/main:
`d60da62eafc864bb915637d70671087f9a7e2b7c`.
**Saved-only planning / numerical formulation diagnostic.** No controller rollout,
MPC numerical solve, state integration, controller-memory update, command
application, LightNav, RGB, Isaac acquisition or source search. No execution claim.
Run identifier: `data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z`.
This name is an identifier; actual processing UTC is recorded separately.

### Source authentication and fixed geometry

| Source | Historical source | Development role |
|---|---|---|
| S1 | OSA03_R00 | Obstacle / straight-to-detour |
| S2 | episode_001_repeat_01/handoff_013 | Mild nearly-straight control |
| S3 | episode_008_repeat_01/handoff_023 | Progressive-left-turn hard case |
| S4 | episode_013_repeat_00/handoff_020 | Progressive-right-turn control |

Authenticate B_TO_ENTRY_BRIDGE_01 result SHA256
`18e79982aca83fae2035d7496883fe40508dfe346d2830285697baf801efdd6e`, its full
result ledger and frozen source/code/input chain, and supplementary failure
trace record SHA256 `fdee7da55be1c33aae6d5c1213ab559513cc3b3f20a8ba9a7d7f521c2773ab4e`.
Run the saved-only bridge validator and its six earlier experiment validators.
Their reconstruction of saved integration records is authentication only; it is
not a new state trajectory, rollout or controller computation in this diagnostic.
No official adapter installation is needed.

World XY metres, yaw radians counterclockwise, +Z up; observation-local x forward,
y left. Raw FRESH remains `F=A F_local`, never re-anchored at B. Copy exact saved
P, B, C3 E*, Native-installed suffix, source geometry and d_F. P is the historical
actual B_tick-1 pose; its timestamp is provenance only. C3 must reproduce the
existing diagnostic exactly through the historical validator; do not select a
new entry. Both planning boundary rows B/E* and every downstream original row
remain bit-exact, and only bridge interior poses are editable.

### Explicit 2×2 design

| Boundary residual | M=2 segments, one editable pose | M=3 segments, two editable poses |
|---|---|---|
| Angle | A2: historical reuse only | A3: new |
| Vector | V2: new | V3: new |

M=2 is `[B, X1, E*]`; M=3 is `[B, X1, X2, E*]`. Append the exact original
suffix after E* to form the full reference. A2 is never re-solved. Reuse its exact
initialization, trace, result, last accepted state, costs, safety and wall time.
All four historical A2 solves reached max80 without convergence.

For M2 use the exact historical `.npy` Hermite sample, including file bytes. For
M3 sample the same continuous Hermite XY curve at three equal XY-arc intervals.
Reuse `hermite_curve` unchanged: endpoint derivatives are chord length times
unit P→B and unit E*→next-original direction. Quadrature epsabs/epsrel=1e-12,
limit200, error estimate<=1e-10; inverse arc uses Brent xtol=1e-14 and
rtol=4*float64 epsilon. Yaw is shortest-angle B→E* interpolation in normalized
spatial arc. Endpoint bits are copied exactly after interpolation. A3/V3 share
one frozen M3 array. V2 starts from the exact A2 initial array, never another
optimized state. No waypoint dt or timestamp. d_F remains the median XY edge
length of `[E*, retained original downstream rows]`, including the first partial
original segment.

### Residuals and unchanged solver

All lambdas are one. Costs are unaveraged sums of squared normalized residuals,
with no half factor. Inherit historical smoothness/spacing and angle factors
without changing historical source files.

For A2/A3:

```
r_in = wrap(direction(B,X1) - direction(P,B)) / 15 deg
r_out = wrap(direction(X_(M-1),E*) - direction(E*,F_next)) / 15 deg
```

For V2/V3, define ell_in_H and ell_out_H from the first and last XY edge lengths
of that variant's own frozen Hermite initialization. Let u_in/u_out be the same
historical unit directions. Replace only the boundary factors:

```
r_in = ((X1.xy-B.xy) - ell_in_H*u_in) / d_F
r_out = ((E*.xy-X_(M-1).xy) - ell_out_H*u_out) / d_F
```

Each is a 2D residual. At a collapsed boundary edge its norm is ell_H/d_F, not
zero. These soft vector targets discourage boundary collapse; they do not impose
a positive minimum edge length or guarantee noncollapse. They never use the
one-tick P→B travelled distance as a target. Rotation of world coordinates rotates
the vector residual and preserves its squared cost.

For every variant:

```
D_j = X_j^-1 X_(j+1)
r_smooth,j = Log(D_(j-1)^-1 D_j) / [d_F, d_F, 10 deg]
r_space,j = (length(edge_(j+1)) - length(edge_j)) / d_F
E = E_in + E_out + E_smooth + E_space
```

These factors represent untimed spatial regularity. No absolute edge-length
factor, controller cost, velocity, acceleration, GP, transport or extra smoothness.
Different boundary costs are different objectives; do not rank their scalar totals
as a method winner or as execution evidence.

Use unchanged right-local `graph_optimizer.solve_least_squares`, source SHA256
`7d595cedc49965cf15cc6e701db371bf7d7cc69e6c3a99deffc304a32855b6f7`.
max80; central FD1e-6; initial damping1e-3; rejection×10; acceptance×.3;
maximum damping1e12; gradient/step tolerance1e-9; cost-decrease tolerance1e-12.
No analytic Jacobian, solver modification, extra budget, restart, sweep or tuning.

### Feasibility and post-solve numerical stability

The historical acceptance gate is unchanged: every improving proposal must pass
full B→bridge→E*→original-suffix direct geometry safety and have every XY bridge
edge strictly >1e-12 m. Circular radius .20 m, required footprint-edge clearance
.05 m, historical Hospital/cart/workspace and numerical reserve. Collapsed proposals
have the same finite atan2(0,0) costing and historical domain rejection. No repair,
projection or smoothing after solve. No new collapse barrier during optimization.

Post-solve only, define numerical collapse as min XY bridge edge <=10*FD epsilon
=1e-5 m. Report every rho_j=final_edge_j/initial_Hermite_edge_j and rho_min without
an additional ratio cutoff. A source/variant is STABLE iff all eight gates pass:
converged, finite, exact B, exact E*, exact suffix, safe full reference, no bridge
polyline self-intersection, and min edge>1e-5 m. Full-reference self-intersection
is also reported separately; the declared stability condition concerns the bridge.
The threshold concerns numerical conditioning, not navigation quality.

Save last accepted states for all attempts. Unconverged states are explicitly
**UNCONVERGED DIAGNOSTIC — NOT A RETURNED REFERENCE**. A converged but unstable
state is likewise a diagnostic, not a STABLE candidate. No reference is executed.
Nonconvergence and numerical solver termination are scientific outcomes here;
implementation/authentication/validator failures are technical blockers.

### Metrics and validation

Per source/A2/A3/V2/V3: convergence/reason, iterations, accepted/rejected steps,
unsafe improving proposals, final damping, last logged gradient infinity norm and
step norm, descriptive wall time. The gradient/step are the last solver
linearization values, not newly computed derivatives at the final accepted state.
The unchanged trace does not retain Jacobian column norms or JᵀJ; these optional
diagnostics are N/A, without changing the solver to obtain them.

Save initial/final four costs and total; M/editable-node count; every initial/final
edge and rho; min/max edge; chord, polyline length/ratio; turning max/RMS; first/last
direction mismatch; bridge/full-reference self-intersection; complete-reference
clearance; min edge/FD epsilon and numerical-collapse flag. For an unconverged
attempt, "final" means last accepted iterate, never an executable solution.

Saved-only validation authenticates files, verifies right-local proposals,
independently reconstructs residual equations, acceptance/damping and safety,
checks exact endpoints/suffix and metrics/classification. An optimizer-call guard
prohibits new solves during validation. The scientific loop has a separate guard
against controller/integration/acquisition calls. Exclusive start markers prevent
retries. Synthetic test solves and renderer fixtures are not scientific evidence.

### Fixed interpretation precedence

1. TECHNICAL_BLOCKED only for authentication, implementation/test, parity,
   artifact corruption or unrelated validator failure. Scientific nonconvergence
   is not this category.
2. MULTIPLE_STABLE_FORMULATIONS if at least two new variants are stable in all four.
3. RESOLUTION_LIMIT_SUPPORTED if A3 is stable4/4, V2 is not4/4, and A2 is unstable4/4.
4. BOUNDARY_DEGENERACY_SUPPORTED if V2 is stable4/4, A3 is not4/4, and A2 is unstable4/4.
5. BOTH_RESOLUTION_AND_BOUNDARY_NEEDED if only V3 is stable4/4 among the new variants.
6. MIXED_FORMULATION_EFFECT if some new variant is stable>=3/4 and none above applies.
7. FORMULATION_STILL_BLOCKED otherwise.

Report each source and every failed stability gate. Uniformly stable variants are
only technically viable candidates for a later authorized execution study. No
winner from objective values, execution/attachment/endpoint benefit, Hermite
superiority, navigation improvement, obstacle generalization, population effect,
real-time feasibility, final method or real-world benefit is established here.

### Freeze, order and outputs

Authenticate → freeze definitions/M/initial arrays/solver/metrics/classification →
focused and relevant regression tests → compileall/diff review → commit/normal
push → record scientific freeze SHA. Only then execute S1 A3,V2,V3; S2 A3,V2,V3;
S3 A3,V2,V3; S4 A3,V2,V3. Exactly twelve new graph solves, zero A2 solves and zero
execution/acquisition calls. A failure is saved without retry or parameter change.

Large traces/arrays remain in ignored data. Tracked results include
`result_summary.json`, `variant_summary.csv`, `factor_costs.csv`, `edge_scale.csv`,
`convergence_summary.csv`, `figure_manifest.json` and exactly three final PNGs:
`formulation_geometry.png`, `convergence_and_edge_scale.png`, `factor_costs.png`.
Numeric sidecars and file hashes accompany each PNG. Equal XY axes; initialization
and overlapping curves have distinct styles/markers. Failed traces and last-state
costs retain explicit diagnostic labels. No HTML dependency or extra final PNGs.

## Commands

From the repository root, using the existing environment without altering it:

```bash
git fetch origin main
git rev-parse HEAD origin/main
git status --short
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/graph-diag02-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_graph_formulation_diag02.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_graph_formulation_diag02.py --run data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z --mode prepare
# Run the relevant regression suite listed in the work log before freeze.
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_graph_formulation_diag02.py --run data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z --mode freeze
# Review/stage only the new namespace and append-only work log.
git diff --cached --check
git commit -m "Freeze saved-only bridge formulation diagnostic"
git push origin main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_graph_formulation_diag02.py --run data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_graph_formulation_diag02.py --run data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/graph-diag02-mpl .venv/bin/python scripts/report_b_to_entry_graph_formulation_diag02.py --run data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_graph_formulation_diag02.py --run data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z --check-only
# Inspect PNGs/sidecars, append results, update work log/README, commit and normal push.
```
