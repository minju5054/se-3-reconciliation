# EXP-02 — Oracle Correspondence + Minimal SE(2) Graph Reconciliation

> **Historical status clarification (2026-09-05):** this experiment is retained unchanged as
> a graph-machinery pilot/sanity experiment. Its collaborator-interface assumption—an SE(2)
> relative-transform correspondence measurement named `Z_ij`—is obsolete. The current
> official interface and experiment are EXP-02A: OLD + FRESH +
> `SpatialEntryContext(entry_index=k, evidence)`. Old annotations and generated results are
> not reinterpreted or modified.

## Research question

Given actual OLD/NEW trajectories and verified oracle correspondences, can a minimal SE(2)
graph keep the committed execution boundary fixed, modify every editable NEW pose, reduce the
OLD→NEW inconsistency, and preserve NEW's original relative motion?

EXP-02 is the first mechanism test. It is offline and uses no Isaac Sim, LightNav inference,
correspondence detector, GTSAM, robust kernel, or online optimized execution.

## Why oracle first

Oracle annotations isolate graph formulation, transform direction, Lie operations, numerical
optimization, and graph connectivity from future colleague-detector errors. Synthetic data is
used first so the correct answer is known. One real LightNav development pair is considered
only after that known-answer acceptance passes.

## Graph variables and fixed data

Every NEW world pose is editable:

```text
X_j in SE(2), j = 0 ... N-1
```

`N` is arbitrary; neither graph construction nor solver assumes LightNav's current horizon 10.
Fixed data comprises all OLD poses `O_i`, raw NEW poses `N_j`, and the measured EXP-01B
NEW-ready pose `B`. `B` is not a variable and never changes. Graph input is the existing
observation-anchored world path; raw LightNav local output is neither recomposed nor re-anchored.

## Factor definitions

The boundary factor uses the minimal identity measurement:

```text
r_B = Log(B^-1 X_0)
```

This is an EXP-02 formulation choice, not a LightNav waypoint-time claim. A future boundary
model may need to represent transition motion as well as pose continuity.

Oracle measurements follow exactly:

```text
Z_ij = O_i^-1 X_j
r_C_ij = Log[Z_ij^-1 (O_i^-1 X_j)]
```

Original NEW relative motions and their residuals are:

```text
D_j = N_j^-1 N_(j+1)
r_NEW_j = Log[D_j^-1 (X_j^-1 X_(j+1))], j = 0 ... N-2
```

The complete `j=0...N-2` chain connects every downstream NEW pose. The minimized cost is the
sum of squared normalized residuals:

```text
E = lambda_B ||r_B / s||^2
  + lambda_C sum ||r_C / s||^2
  + lambda_N sum ||r_NEW / s||^2
```

Translation and yaw scales are separately configured as `0.10 m` and `0.10 rad`. The balanced
development weights were fixed before the run at boundary/correspondence/NEW-motion
`4 / 1 / 1`.

## SE(2) convention

Poses are `[x, y, yaw]`, meters and radians, with +y left and +yaw counter-clockwise. `Exp`
maps body tangent `[v_x, v_y, omega]` to SE(2), including stable small-angle series. `Log` is
its inverse with wrapped yaw. Solver updates are right-local manifold retractions:

```text
X_j <- X_j Exp(delta_j)
```

Unit tests cover identity, translation, rotation, small yaw, ±pi, Exp/Log round trips,
compose/inverse, and correspondence direction.

## Solver

The implementation is a NumPy damped Gauss–Newton/Levenberg–Marquardt solver with central
finite-difference Jacobians. It accepts only finite states and residuals, solves a damped normal
system, accepts only cost-decreasing updates, and records cost/damping history, iterations,
convergence, and termination reason. Singular and non-finite solves fail explicitly. Default
maximum iterations are 80; the finite-difference step is `1e-6` and initial damping `1e-3`.
No SciPy or pose-graph dependency was added.

## Phase A synthetic result

The deterministic ground truth contains nine poses with repeated local step
`[0.22 m, 0, 0.045 rad]`, producing a slight curve. OLD is GT indices 0–4 and the seven-pose
NEW ground truth begins at GT index 2. Raw NEW is the entire NEW path left-composed by the
known global perturbation `[0.20 m, 0.15 m, 0.10 rad]`. Identity correspondences are
`(OLD 2, NEW 0)`, `(OLD 3, NEW 1)`, and `(OLD 4, NEW 2)`; `B` equals the correct NEW pose 0.

- initial/final cost: `62.8351106 -> 1.8382e-26`
- convergence: 3 iterations, `cost_tolerance`
- pose recovery translation RMS/max: `1.6192e-14 / 2.9072e-14 m`
- pose recovery yaw RMS/max: `1.1200e-14 / 1.7319e-14 rad`
- correspondence translation RMS: `0.290306 m -> 2.3650e-15 m`
- correspondence yaw RMS: `0.100000 rad -> 2.7733e-15 rad`
- NEW-motion distortion translation/yaw RMS: `3.6932e-15 m / 3.1921e-15 rad`

Every non-corresponded downstream pose recovered to numerical precision and the fixed boundary
input remained byte-for-byte unchanged. This validates implementation and mechanism only; it
is not research evidence.

## Phase B real LightNav development pair

The immutable source is EXP-01B `exp01b-20260903T155402Z/trial_001`. It is timing-valid,
non-stop, finite, and has `(10, 3)` OLD/NEW arrays. It was selected because its recorded
translation-motion jump, `0.100788914 m`, was the largest among valid non-stop candidates
(`trial_003`: `0.098741912 m`).

Raw hashes:

- OLD: `8edd60bf794d067dba1820f557e39388a8c3af352db7443774108f550219cf06`
- NEW: `072c29e991bea91fad89d00ad458c00697960dc684bbdfc68cf105e3ace362fe`

The source raw files were not copied or modified. The EXP-02 output references their path and
hash, while the already-derived world NEW is saved as the graph's explicit input.

## Oracle annotation rationale

The versioned annotation contains three strictly monotonic identity correspondences:

| OLD index | NEW index | `Z_ij` | Raw world translation difference | Rationale |
|---:|---:|---|---:|---|
| 1 | 0 | `[0, 0, 0]` | 0.019257 m | first common corridor pose |
| 2 | 1 | `[0, 0, 0]` | 0.008998 m | second ordered common pose |
| 3 | 2 | `[0, 0, 0]` | 0.012177 m | third ordered common pose |

Both paths progress monotonically down the same obstacle-free corridor and the neighboring
matches agree. Nearest distance was used to generate candidates, then the full ordered sequence
and numerical pose differences were manually inspected. The identity transform asserts the
same desired physical poses; it is deliberately not `O_i^-1 N_j`, which would make the raw
residual zero by construction.

## Before/after metrics — full graph

| Metric | Raw NEW | Optimized NEW |
|---|---:|---:|
| boundary translation gap | 0.00385449 m | 0.00560869 m |
| boundary yaw gap | 0.00416388 rad | 0.00086345 rad |
| translation-motion jump | 0.10078891 m | 0.09585413 m |
| yaw-motion jump | 0.00678489 rad | 0.00312881 rad |
| correspondence translation RMS | 0.01414317 m | 0.01031208 m |
| correspondence yaw RMS | 0.00957319 rad | 0.00150561 rad |
| correspondence normalized RMS | 0.09860275 | 0.06016807 |
| NEW-motion translation distortion RMS/max | approximately 0 | 0.00186831 / 0.00493834 m |
| NEW-motion yaw distortion RMS/max | 0 | 0.00130968 / 0.00365608 rad |

Optimization converged in 3 iterations by cost tolerance. Cost decreased
`0.100380521 -> 0.050148214`. Correspondence yaw, yaw boundary, yaw-motion jump, and
translation-motion jump improved. Correspondence translation improved by about 27%. However,
the already-small translation boundary gap worsened by 1.75 mm. This is a real factor trade-off,
not a clean Case B success.

Per-correspondence translation residuals changed from
`[0.019257, 0.008998, 0.012177] m` to `[0.017503, 0.002373, 0.002651] m`; absolute yaw residuals
changed from `[0.004851, 0.010851, 0.011561] rad` to
`[0.000177, 0.002168, 0.001439] rad`.

## Downstream correction profile

| NEW index | translation correction [m] | yaw correction [rad] |
|---:|---:|---:|
| 0 | 0.001756 | 0.005027 |
| 1 | 0.006704 | 0.008683 |
| 2 | 0.009535 | 0.010122 |
| 3 | 0.010035 | 0.010122 |
| 4 | 0.010347 | 0.010122 |
| 5 | 0.010411 | 0.010122 |
| 6 | 0.010447 | 0.010122 |
| 7 | 0.010459 | 0.010122 |
| 8 | 0.010460 | 0.010122 |
| 9 | 0.010457 | 0.010122 |

Correction reached every non-corresponded pose and approached an almost rigid downstream
offset. It did not disappear at the last oracle. The full graph showed no abrupt downstream
kink; distortion was concentrated in the first two edges where boundary and correspondence
constraints compete.

## Factor ablation

| Variant | boundary gap [m/rad] | correspondence RMS [m/rad] | NEW-motion distortion RMS [m/rad] | motion jump [m/rad] |
|---|---|---|---|---|
| full `B+C+N` | 0.005609 / 0.000863 | 0.010312 / 0.001506 | 0.001868 / 0.001310 | 0.095854 / 0.003129 |
| no correspondence `B+N` | ~0 / ~0 | 0.017802 / 0.005771 | ~0 / ~0 | 0.100789 / 0.006785 |
| no NEW motion `B+C` | 0.004622 / 0.000137 | 0.010675 / 0.000317 | 0.005153 / 0.004436 | 0.092560 / 0.000235 |

Without correspondence, the graph rigidly aligns pose 0 to `B` and perfectly preserves NEW,
but correspondence alignment worsens and both motion jumps remain essentially raw. Without
NEW-motion factors, oracle-connected poses 0–2 move while poses 3–9 remain raw; correction
drops from 0.012177 m/0.011561 rad at pose 2 to numerical zero at pose 3. The resulting edge
kink reaches 0.012697 m and 0.011561 rad. Thus correspondence creates alignment and the full
NEW chain is what propagates it while suppressing the cutoff.

## Weight sensitivity

Only the predeclared correspondence/NEW-motion ratios `[0.25, 1.0, 4.0]` were run; boundary
weight remained 4 and NEW-motion weight remained 1.

| Ratio | boundary gap [m/rad] | correspondence RMS [m/rad] | distortion RMS [m/rad] | motion jump [m/rad] |
|---:|---|---|---|---|
| 0.25 | 0.002230 / 0.000518 | 0.013504 / 0.003278 | 0.001399 / 0.000749 | 0.097087 / 0.004757 |
| 1.0 | 0.005609 / 0.000863 | 0.010312 / 0.001506 | 0.001868 / 0.001310 | 0.095854 / 0.003129 |
| 4.0 | 0.011722 / 0.000942 | 0.006587 / 0.000558 | 0.000899 / 0.001677 | 0.099448 / 0.001917 |

All settings converged in three iterations. Increasing correspondence influence improves its
residual and yaw-motion jump, but worsens translation boundary continuity. No universal weight
is selected from this one development pair.

## Positive evidence

The synthetic recovery establishes correct graph direction, manifold updates, solver behavior,
and full-chain downstream propagation. On the real pair, the full graph decreased total cost,
substantially reduced correspondence yaw error, reduced correspondence translation RMS,
improved yaw boundary/motion mismatch, and kept relative-motion distortion in the millimeter/
milliradian range. The NEW-motion chain successfully carried correction through pose 9.

## Negative evidence and limitation

The dominant raw defect in this chosen pair was first-segment translation-motion mismatch, not
pose gap. The minimal identity boundary factor constrains only `X_0`; preserving NEW's long
first relative segment directly conflicts with reducing that motion jump. Consequently the full
graph improved translation-motion jump by only about 4.9% and made translation pose gap slightly
worse. The ablation confirms that larger motion-jump reduction is available only by accepting
more deformation and an unacceptable downstream cutoff when the chain is removed.

This is one manually annotated straight-corridor development pair. Oracle identity, residual
scales, and weights are not validated on unseen pairs. There is no detector, wrong-match
robustness, online graph execution, navigation-quality result, obstacle test, NavDP result, or
real-robot result.

## What we can claim

With verified oracle correspondences, the minimal SE(2) graph is mathematically valid and can
trade boundary, correspondence, and NEW-relative-motion objectives on one real LightNav pair.
It propagates overlap correction through the full future without a kink and improves several
inconsistency components, but it does not cleanly reduce every raw-switch metric.

## What we cannot claim

We cannot claim that an actual front-end works, the formulation generalizes, correspondence
weights are optimal, navigation improves, or the current boundary factor solves translational
motion discontinuity.

## Next uncertainty

The next step should be formulation refinement before freezing weights for multiple-pair
evaluation. Specifically, a future scoped experiment should test a boundary-transition motion
factor or another explicit treatment of the committed-to-first-NEW segment. Only after that
choice is fixed should development/evaluation pairs be separated and unseen LightNav pairs be
used. Oracle sensitivity remains the other major uncertainty.

## Reproduction

```bash
cd ~/Workspace/se-3-reconciliation
.venv/bin/python scripts/run_exp02_oracle_graph.py
```

Validate an immutable run:

```bash
.venv/bin/python scripts/summarize_exp02.py data/exp02/<run_id>
```

The validated run documented above is `exp02-20260904T053813Z`.
