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

## Repository-confirmed facts

Scientific freeze **`e228237fdf8e68df3db6ab27a5c83eb7da6df76c`** was normally pushed before all twelve new solves. Actual planning start UTC: `2026-10-03T06:12:15.503371+00:00`. Frozen classification: **MULTIPLE_STABLE_FORMULATIONS**.

Historical A2 authentication and exact saved residual/cost parity pass. A2 was not rerun. Exactly **12 new graph solves**, in the frozen S1–S4 × A3,V2,V3 order; **0 retries**. MPC solves, rollouts, new state integrations, controller-memory updates, command applications, LightNav, RGB and Isaac calls are all **0**.

V2 and V3 each pass all eight STABLE gates in 4/4 sources. A3 reaches the unchanged 80-iteration limit in all four. A3 nonconvergence is a scientific formulation outcome in this experiment, not TECHNICAL_BLOCKED. All initial/final values are finite, and all B/E*/downstream suffix bits remain exact.

### Explicit diagnostic matrix

| Source | A2 historical, angle M2 | A3 new, angle M3 | V2 new, vector M2 | V3 new, vector M3 |
|---|---|---|---|---|
| S1 | Unconverged / collapsed; 80 iterations | Unconverged / collapsed; 80 iterations | STABLE; 3 iterations | STABLE; 6 iterations |
| S2 | Unconverged / collapsed; 80 iterations | Unconverged / collapsed; 80 iterations | STABLE; 3 iterations | STABLE; 6 iterations |
| S3 | Unconverged / collapsed; 80 iterations | Unconverged / collapsed; 80 iterations | STABLE; 3 iterations | STABLE; 6 iterations |
| S4 | Unconverged / collapsed; 80 iterations | Unconverged / collapsed; 80 iterations | STABLE; 4 iterations | STABLE; 7 iterations |

### Edge scales and boundary geometry

All lengths below are metres. A2/A3 final values describe **last accepted unconverged iterates**, not returned references. `rho_min` is the smallest final/initial edge ratio; it has no additional acceptance cutoff. The numerical collapse threshold is exactly 1e-5 m. Full machine-precision arrays, including every initial/final edge and rho_j, are in `edge_scale.csv` and `result_summary.json`.

| Source | Variant | First edge | Outgoing edge | Minimum edge / FD epsilon | rho_min | Numerical collapse |
|---|---|---:|---:|---:|---:|---|
| S1 | A2 | 2.24005268726e-06 | 0.106649689212 | 2.24005268726 | 4.33577536446e-05 | Yes |
| S1 | A3 | 0.0606462241228 | 2.4452080167e-06 | 2.4452080167 | 6.78434377162e-05 | Yes |
| S1 | V2 | 0.050802668327 | 0.0560035487999 | 50802.668327 | 0.983320432748 | No |
| S1 | V3 | 0.0312494925969 | 0.0356831996566 | 31249.4925969 | 0.981462819894 | No |
| S2 | A2 | 1.85383750795e-06 | 0.127644795462 | 1.85383750795 | 2.95792178428e-05 | Yes |
| S2 | A3 | 0.0370710270924 | 2.15980777064e-06 | 2.15980777064 | 5.1062054422e-05 | Yes |
| S2 | V2 | 0.0622304056792 | 0.0654790357662 | 62230.4056792 | 0.992927761004 | No |
| S2 | V3 | 0.0388827914046 | 0.0417072620272 | 38882.7914046 | 0.980120299962 | No |
| S3 | A2 | 2.3147443132e-06 | 0.184683942079 | 2.3147443132 | 2.56327028072e-05 | Yes |
| S3 | A3 | 0.0865299774714 | 1.86212841925e-06 | 1.86212841925 | 3.02622306046e-05 | Yes |
| S3 | V2 | 0.0895020407675 | 0.0953918664418 | 89502.0407675 | 0.991115605533 | No |
| S3 | V3 | 0.0557829742965 | 0.0606635585883 | 55782.9742965 | 0.98206950166 | No |
| S4 | A2 | 2.54420923961e-06 | 0.160885533191 | 2.54420923961 | 3.26643194118e-05 | Yes |
| S4 | A3 | 0.0903871380392 | 2.36231419342e-06 | 2.36231419342 | 4.33980960224e-05 | Yes |
| S4 | V2 | 0.0765102169725 | 0.0845728682861 | 76510.2169725 | 0.982291128635 | No |
| S4 | V3 | 0.0469671968781 | 0.0539468535099 | 46967.1968781 | 0.980054164965 | No |

A2 collapsed the first edge in all four sources. A3 retained a first edge of 0.0370710270924–0.0903871380392 m but collapsed the **outgoing** edge to 1.86212841925e-6–2.44520801670e-6 m. Its middle edge is not collapsed. Thus adding one interior pose moved the observed degeneracy to the opposite boundary.

V2/V3 minimum edge lengths span 0.0312494925969–0.0895020407675 m, at least 31,249.49 finite-difference epsilons. Their rho_min values span 0.980054164965–0.992927761004. No vector variant approaches the numerical-collapse threshold.

### Solver records

Gradient and step are the final **logged linearization** values; no new final-state Jacobian was computed. Optional Jacobian-column norms and JᵀJ condition numbers remain N/A. Wall times are descriptive host measurements only; historical A2 used its original recorded run.

| Source | Variant | Termination | Accepted / rejected | Final damping | Last gradient infinity norm | Last step norm | Wall s |
|---|---|---|---:|---:|---:|---:|---:|
| S1 | A2 | maximum_iterations | 44 / 36 | 9.84770902e+09 | 2819.94272 | 5.61203446e-09 | 0.194772053 |
| S1 | A3 | maximum_iterations | 45 / 35 | 295431271 | 2063.25176 | 2.47745465e-08 | 0.328036422 |
| S1 | V2 | cost_tolerance | 3 / 0 | 2.7e-05 | 1.10573821e-05 | 4.76419939e-08 | 0.016641939 |
| S1 | V3 | cost_tolerance | 6 / 0 | 7.29e-07 | 6.92646248e-06 | 8.24701259e-08 | 0.036242753 |
| S2 | A2 | maximum_iterations | 44 / 36 | 9.84770902e+09 | 3687.56418 | 5.25505069e-09 | 0.156046084 |
| S2 | A3 | maximum_iterations | 45 / 35 | 295431271 | 2312.56063 | 3.89244935e-07 | 0.28465842 |
| S2 | V2 | cost_tolerance | 3 / 0 | 2.7e-05 | 1.12694504e-06 | 5.25811952e-09 | 0.012924171 |
| S2 | V3 | cost_tolerance | 6 / 0 | 7.29e-07 | 2.10216113e-06 | 2.36994075e-08 | 0.029516291 |
| S3 | A2 | maximum_iterations | 44 / 36 | 9.84770902e+09 | 1180.31443 | 3.52837495e-09 | 0.161854087 |
| S3 | A3 | maximum_iterations | 45 / 35 | 295431271 | 343.968686 | 2.61307786e-08 | 0.289857187 |
| S3 | V2 | cost_tolerance | 3 / 0 | 2.7e-05 | 7.0968806e-06 | 3.54777905e-08 | 0.013282905 |
| S3 | V3 | cost_tolerance | 6 / 0 | 7.29e-07 | 4.98334868e-06 | 6.29429053e-08 | 0.030369207 |
| S4 | A2 | maximum_iterations | 44 / 36 | 9.84770902e+09 | 3647.19234 | 1.23486538e-07 | 0.159479951 |
| S4 | A3 | maximum_iterations | 45 / 35 | 295431271 | 4174.45285 | 3.67257504e-08 | 0.277124387 |
| S4 | V2 | step_tolerance | 3 / 0 | 2.7e-05 | 9.23949818e-08 | 5.29461047e-10 | 0.013842898 |
| S4 | V3 | cost_tolerance | 7 / 0 | 2.187e-07 | 1.04519145e-06 | 1.40839048e-08 | 0.033066581 |

V2 terminates by cost tolerance in S1–S3 and step tolerance in S4. V3 terminates by cost tolerance throughout. These are the unchanged solver stopping rules, not certificates of a global optimum. Unsafe improving-proposal rejections are **zero** for all sixteen historical/new records; the safety gate did not bind in these data.

### Factor costs

Each cell is initial → final. A2/A3 final means **LAST ACCEPTED ITERATE — UNCONVERGED**. Boundary definitions and normalizations differ between A and V; scalar totals cannot rank execution or choose a winner. Costs are sums of squared factor residuals; the saved solver vector dot product may differ in the last floating-point bit due to summation order.

| Source | Variant | E_in | E_out | E_smooth | E_space | Total |
|---|---|---:|---:|---:|---:|---:|
| S1 | A2 | 67.819091 → 2.87587368e-07 | 33.4022771 → 35.998168 | 0.0186784624 → 0.504377389 | 0.000546068841 → 0.504268168 | 101.240593 → 37.0068138 |
| S1 | A3 | 59.2334996 → 0.201344004 | 26.1909011 → 1.68023928e-07 | 0.0352857025 → 2.40232903 | 0.00513748104 → 1.18728492 | 85.4648239 → 3.79095812 |
| S1 | V2 | 0.367420796 → 0.33745477 | 0.254358304 → 0.288181199 | 0.0186784624 → 0.00414343766 | 0.000546068841 → 0.00119926278 | 0.641003631 → 0.63097867 |
| S1 | V3 | 0.128515844 → 0.120487384 | 0.0888171915 → 0.109810816 | 0.0352857025 → 0.0139076399 | 0.00513748104 → 0.00486227246 | 0.257756219 → 0.249068113 |
| S2 | A2 | 50.2382565 → 2.89950366e-07 | 34.954364 → 35.9842151 | 0.00585811271 → 0.719216245 | 0.000239827493 → 0.719191062 | 85.1987185 → 37.4226227 |
| S2 | A3 | 41.4744379 → 0.0743322023 | 27.4660229 → 1.91073012e-06 | 0.0460028361 → 1.94141954 | 0.00583016258 → 1.32167072 | 68.9922938 → 3.33742438 |
| S2 | V2 | 0.444213143 → 0.422357811 | 0.36447891 → 0.387346924 | 0.00585811271 → 0.00127192766 | 0.000239827493 → 0.000465855468 | 0.814789993 → 0.811442518 |
| S2 | V3 | 0.154915098 → 0.153738992 | 0.126761044 → 0.146134568 | 0.0460028361 → 0.0174104306 | 0.00583016258 → 0.00554229822 | 0.333509141 → 0.322826289 |
| S3 | A2 | 54.5756215 → 6.86657434e-08 | 34.600095 → 36.0005433 | 0.0234090686 → 1.39301523 | 0.000708929651 → 1.3928931 | 89.1998345 → 38.7864517 |
| S3 | A3 | 45.7168729 → 1.19489797 | 27.1825395 → 5.323507e-07 | 0.0922085797 → 4.31041881 | 0.012066857 → 2.35407777 | 73.0036878 → 7.85939509 |
| S3 | V2 | 0.902732433 → 0.843685183 | 0.706466193 → 0.770117036 | 0.0234090686 → 0.00470635052 | 0.000708929651 → 0.00141669149 | 1.63331662 → 1.61992526 |
| S3 | V3 | 0.315221434 → 0.305048827 | 0.246042254 → 0.291875079 | 0.0922085797 → 0.0350389918 | 0.012066857 → 0.0114123286 | 0.665539125 → 0.643375227 |
| S4 | A2 | 69.0570085 → 6.70638684e-06 | 33.2910996 → 36.0006218 | 0.0407536096 → 1.1182583 | 0.00126819242 → 1.11821014 | 102.39013 → 38.237097 |
| S4 | A3 | 60.5377793 → 1.56145933 | 26.0975817 → 6.96705003e-07 | 0.0773956837 → 4.57174557 | 0.0115318983 → 2.26096751 | 86.7242885 → 8.39417311 |
| S4 | V2 | 0.822236027 → 0.756445667 | 0.563545767 → 0.637594948 | 0.0407536096 → 0.00935330246 | 0.00126819242 → 0.00280839979 | 1.4278036 → 1.40620232 |
| S4 | V3 | 0.287626251 → 0.269932424 | 0.196809548 → 0.242821923 | 0.0773956837 → 0.0308806739 | 0.0115318983 → 0.0109415044 | 0.573363381 → 0.554576526 |

### Remaining geometry and safety

All full returned V2/V3 references pass the unchanged .20 m radius / .05 m footprint-edge clearance gate. Their minimum clearance is **0.21148985508009666 m**. Failed A2/A3 last iterates also pass that geometric check, but remain unusable under the STABLE definition. All bridge and complete-reference self-intersection flags are false.

| Source | Variant | XY bridge length m | Length/chord | Turning max / RMS rad | Direction mismatch in / out rad | Full reference clearance m |
|---|---|---:|---:|---:|---:|---:|
| S1 | A2 | 0.106651929 | 1.0000315 | 2.09425816 / 2.09425816 | 0.000140395611 / 1.57075636 | 0.22948689016 |
| S1 | A3 | 0.202642528 | 1.90009607 | 2.38012813 / 2.18654378 | 0.117472978 / -0.000107313459 | 0.22948689016 |
| S1 | V2 | 0.106806217 | 1.0014782 | 0.108807776 / 0.108807776 | 2.03732469 / 1.62249027 | 0.22948689016 |
| S1 | V3 | 0.107464405 | 1.00764975 | 0.297424469 / 0.247704861 | 1.94493754 / 1.53364123 | 0.22948689016 |
| S2 | A2 | 0.127646649 | 1.00001827 | 1.83201627 / 1.83201627 | -0.00014097122 / -1.57045192 | 0.757253286777 |
| S2 | A3 | 0.176552708 | 1.38316153 | 2.02484011 / 1.93219281 | -0.0713768087 / 0.0003618831 | 0.757253286777 |
| S2 | V2 | 0.127709441 | 1.0005102 | 0.0638948063 / 0.0638948063 | -1.79938286 / -1.60157235 | 0.757253286777 |
| S2 | V3 | 0.12868148 | 1.00812541 | 0.29284978 / 0.259879544 | -1.69988518 / -1.50896261 | 0.757253286777 |
| S3 | A2 | 0.184686257 | 1.00001661 | 1.9017269 / 1.9017269 | 6.86023021e-05 / 1.57080818 | 0.21148985508 |
| S3 | A3 | 0.293962236 | 1.59171084 | 2.0454655 / 2.02327863 | 0.286176547 / -0.00019101503 | 0.211488992382 |
| S3 | V2 | 0.184893907 | 1.00114097 | 0.0955422807 / 0.0955422807 | 1.85248959 / 1.61704455 | 0.21148985508 |
| S3 | V3 | 0.186235693 | 1.0084063 | 0.308247272 / 0.260014354 | 1.75124422 / 1.52800841 | 0.21148985508 |
| S4 | A2 | 0.160888077 | 1.00002394 | 2.11045115 / 2.11045115 | -0.000677973825 / -1.57080989 | 0.878000743682 |
| S4 | A3 | 0.290906763 | 1.80817455 | 2.2402174 / 2.13639236 | -0.327140238 / 0.000218520956 | 0.878001187341 |
| S4 | V2 | 0.161083085 | 1.00123604 | 0.099513703 / 0.099513703 | -2.05886618 / -1.61806066 | 0.878001187341 |
| S4 | V3 | 0.162069357 | 1.00736636 | 0.289349691 / 0.244877084 | -1.96901319 / -1.52776405 | 0.878001187341 |

### Validation and artifacts

Before freeze: **36 focused tests passed** (48.28 s); **636 relevant regression tests passed, 1 skipped** (257.05 s). The known skip requires the absent ignored EXP-01B/EXP-02B corpus. Compileall and working/staged diff checks pass. Saved-only validation passes the new independent residual/trace/stability checks and historical bridge plus six earlier validators. Tests and validators add no scientific solves.

- [Result summary](../results/b_to_entry_graph_formulation_diag02/result_summary.json)
- [Variant summary](../results/b_to_entry_graph_formulation_diag02/variant_summary.csv)
- [Factor costs](../results/b_to_entry_graph_formulation_diag02/factor_costs.csv)
- [Edge scales](../results/b_to_entry_graph_formulation_diag02/edge_scale.csv)
- [Convergence](../results/b_to_entry_graph_formulation_diag02/convergence_summary.csv)

Exactly three final PNGs, visually inspected against numeric sidecars:

- [formulation_geometry.png](../results/b_to_entry_graph_formulation_diag02/figures/formulation_geometry.png)
- [convergence_and_edge_scale.png](../results/b_to_entry_graph_formulation_diag02/figures/convergence_and_edge_scale.png)
- [factor_costs.png](../results/b_to_entry_graph_formulation_diag02/figures/factor_costs.png)

### Presentation correction and protocol accounting

No scientific formulation, initialization, solver, gate, classification, source or result was changed after freeze. No retry or extra scientific solve occurred. After viewing the real-data plots, a presentation-only wrapper raised the inset axes above parent markers and reduced inset tick density. The frozen renderer is unchanged. Original PNGs/manifests/result summary are preserved in ignored `data/b_to_entry_graph_formulation_diag02/report_layout_01/`; the original run result ledger is unchanged. All plotted numerical sidecars are exactly equal before/after this correction. `layout_audit.json` records original/final PNG hashes, archived hashes and both renderer hashes. This is a disclosed post-freeze reporting-only addition, not a formulation change.

The same wrapper normalizes the four CSV files from CRLF to LF line endings so the staged whitespace check passes. Original CSV bytes are preserved in the archive's `tables/` directory. Parsed cells are exactly unchanged; the audit records both original and final hashes. This serialization correction was applied after the figure correction without regenerating scientific results or plots.

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/graph-diag02-mpl .venv/bin/python scripts/report_b_to_entry_graph_formulation_diag02_layout.py --run data/b_to_entry_graph_formulation_diag02/primary_20261003T000000Z --archive data/b_to_entry_graph_formulation_diag02/report_layout_01
```

## Research interpretation

Increasing resolution from A2 to A3 alone did not remove the observed boundary collapse or yield convergence in any source. At M2, replacing the angle boundary residuals with the prescribed Hermite-length vector targets was sufficient for convergence and all eight stability gates in every source. V3 also passed uniformly. These results support the vector boundary formulation as a sufficient numerical remedy within this fixed diagnostic, with M3 unnecessary for uniform stability here.

The formal category is MULTIPLE_STABLE_FORMULATIONS because V2 and V3 both satisfy the highest applicable precedence rule. This does not establish that all historical failure was caused exclusively by one mathematical singularity. The prescribed boundary replacement changes residual geometry, dimension and normalization as well as discouraging zero-length edges. It also does not guarantee that every source or internal edge will remain noncollapsed.

V2 and V3 are technically viable candidates for a later separately authorized execution experiment. Both remain close in edge length to their own initial Hermite bridges, while their direction mismatches remain substantial. Numerical convergence is not proof of a dynamically useful transition. Neither candidate is selected as a winner from its objective, wall time or iteration count.

## Limitations and not demonstrated

Four fixed development sources; one frozen initialization per variant; one solver and set of scales; no held-out obstacle geometry, population claim, global-optimality proof or real-time benchmark. No new controller execution, attachment/endpoint metric or semantic-intent evaluation. Better execution, faster attachment, improved endpoint completion, graph superiority over Hermite, navigation improvement, obstacle generalization, a final reconciliation method and real-world benefit are not demonstrated.

## Explicit answers to the sixteen questions

1. **Does A3 converge where A2 failed?** No. Both are nonconverged in all four sources at 80 iterations.
2. **Does V2 converge where A2 failed?** Yes, 4/4; iterations 3/3/3/4.
3. **Does V3 converge where A2 failed?** Yes, 4/4; iterations 6/6/6/7.
4. **Which variants are stable in all four?** V2 and V3. A2 and A3 are stable in 0/4.
5. **Does increasing M alone prevent boundary collapse?** No. A3 avoids first-edge collapse but collapses its outgoing edge in all four.
6. **Does the vector boundary alone prevent collapse?** In these four M2 comparisons, yes: V2 retains all edges above the declared numerical threshold and converges. This is not a universal guarantee.
7. **Is V3 required for uniform stability?** No; V2 already satisfies all eight gates in 4/4.
8. **What happens to the first edge?** A2: 1.854–2.544 μm. A3: 0.037071–0.090387 m. V2: 0.050803–0.089502 m. V3: 0.031249–0.055783 m. Exact per-source values are in the edge table.
9. **What happens to the outgoing edge?** A2: 0.106650–0.184684 m; A3 collapses to 1.862–2.445 μm. V2: 0.056004–0.095392 m; V3: 0.035683–0.060664 m.
10. **Are B and E* preserved exactly?** Yes, bit-for-bit in all initial, accepted and saved final states.
11. **Is downstream original FRESH preserved exactly?** Yes, bit-for-bit from the authenticated Native installation; raw FRESH is unchanged.
12. **Are all returned references safe?** All eight converged V2/V3 full references pass. Failed A2/A3 states also pass the geometric check but are not returned stable references. No execution safety was tested.
13. **Are there self-intersections?** No bridge or full-reference self-intersection in any of the sixteen saved final states.
14. **Do any variants approach FD epsilon?** A2 first edges and A3 outgoing edges remain roughly 1.85–2.54 times epsilon. V2/V3 minimum edges are at least 31,249 times epsilon.
15. **What is the frozen classification?** MULTIPLE_STABLE_FORMULATIONS, using the predeclared precedence.
16. **Which formulations justify a later execution study technically?** V2 and V3. Neither was executed here; no execution-performance superiority is inferred.
