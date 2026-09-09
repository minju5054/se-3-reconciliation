# EXP-02C — Current-M4 Factor Isolation and Failure Attribution

## 1. Research question

EXP-02C asks why the unchanged EXP-02A/EXP-02B M4 objective over-corrects some transitions,
especially frozen benign Case C at `k=0`. It is a diagnostic, not an algorithm improvement.
It adds no residual to the production objective, performs no weight search, implements no
gate or selector, and does not modify LightNav, the controller, Isaac physics, or any frozen
EXP-02A/B/B-R artifact.

The complete run reported here is
`data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z/`. Generated data are
ignored and immutable. The frozen real source is
`data/exp02b/exp02b-controller-aware-20260906T150400Z/`.

## 2. Why EXP-02B-R motivates EXP-02C

EXP-02B-R preserved the controller-independent first desired-command result in benign Case C,
`k=0`: raw `|delta v_des| = 0.00442238 m/s`, but M4 graph
`|delta v_des| = 0.24096764 m/s`. Calibration changed physical tracking, not this candidate
property. EXP-02C therefore holds the inputs, weights, scales, follower, and solver fixed and
attributes the candidate change to the current factors.

## 3. Exact current M4 factors

For selected raw suffix poses `F_j`, optimized poses `X_j`, saved boundary `B`, previous
executed pose `P`, and `d_k = ||F_k.xy - B.xy||`, M4 is exactly:

```text
E = E_entry + E_direction + E_yaw + E_fresh_motion
r_entry = Log(F_k^-1 X_k)
r_direction = (X_k.xy - B.xy) / d_k - (B.xy - P.xy) / ||B.xy - P.xy||
r_yaw = wrap(yaw(B^-1 X_k) - yaw(P^-1 B))
r_fresh,j = Log((F_j^-1 F_{j+1})^-1 (X_j^-1 X_{j+1}))
```

Every historical weight is `1.0`; scales remain `0.10 m`, `0.10 rad`, and `0.10` direction
units. The numerical solver remains the frozen LM solver. Central finite differences use
epsilon `1e-6` with right-local `T <- T * Exp(delta)` retraction. Pose anchor is not an M4
factor and is absent from this experiment.

## 4. Factor-isolation protocol

The production `TransitionGraphProblem` was left unchanged. A separate diagnostic problem in
`src/reconciliation/exp02c_factor_isolation.py` calls the same physical residual helpers and
uses the same residual block order and solver algorithm. Each run records exact accepted and
rejected LM steps. Removed factors leave the other weights unchanged.

All 5 synthetic conditions and the frozen 3 cases × `k={0,3,6}` were run across V0–V8.
Before creating an output directory, V4 was compared with all nine historical graph
candidate arrays. Maximum residual-vector, translation, and wrapped-yaw differences were all
exactly `0.0`, passing the `1e-9` limit.

## 5. Additive variants

- V0 `RAW`: `X=F`, no optimization.
- V1 `ENTRY+FRESH`: raw no-op regression.
- V2 `ENTRY+DIRECTION+FRESH`: adds direction only.
- V3 `ENTRY+YAW+FRESH`: adds yaw only.
- V4 `FULL`: exact current historical M4.
- V5 `NO_ENTRY`: diagnostic direction+yaw+fresh objective.

## 6. Leave-one-out variants

V6 `NO_DIRECTION` is numerically the same factor set as V3, and V7 `NO_YAW` is the same as
V2; their outputs are stored separately so additive and leave-one-out readings stay explicit.
V8 `DIAGNOSTIC_NO_PROPAGATION` optimizes only `X_k` under entry+direction+yaw and reconstructs
all downstream nodes from exact raw FRESH. It is not a deployable baseline. It exposes the
edge kink that the fresh-motion chain otherwise prevents.

## 7. Synthetic mechanism cases

Synthetic fixtures are tests and visual explanations, never experimental evidence.

- S0 perfectly benign: every raw factor was zero; every variant returned raw exactly.
- S1 direction-only: only direction was nonzero. V2/FULL moved the entry `0.271964 m`; V3
  was exact no-op. FULL edge RMS was `6.05e-14 m` and rigid-fit RMS `5.53e-14 m`.
- S2 yaw-only: only yaw was nonzero. V3 changed entry yaw by `0.175000 rad`; V2 was exact
  no-op. The common rotation displaced the endpoint `0.175000 m`.
- S3 magnitude stress: every current raw residual was zero although boundary-to-entry
  distance intentionally differed from the OLD step. No variant moved. This is a
  `MISSING_MECHANISM_CANDIDATE`, not a claim that OLD step length is ground truth.
- S4 local/downstream conflict: FULL moved entry and endpoint by `0.271964 m` with
  `5.53e-14 m` rigid-fit RMS. V8 moved the entry by the same amount, kept endpoint at exact
  raw, but produced `0.121626 m` fresh-edge translation RMS and `0.0880515 m` rigid-fit RMS.
  The separate desired target is visualization-only and never enters an objective.

## 8. Real frozen cases

Inputs are the exact frozen `case_high_delta_v`, `case_high_delta_omega`, and
`case_benign_delayed` OLD/FRESH/P/B values at `k=0,3,6`. Source files and top-level EXP-02B
artifacts are SHA-256 recorded. Raw data are loaded read-only through the existing transition
input boundary, copied before solving, and verified unchanged afterward.

## 9. Per-factor residual and cost

At every raw initialization, entry cost was exactly zero and fresh-motion cost was at most
floating-point noise (`7.03e-32`). Thus the initial M4 force came only from direction/yaw:

| case | k | direction cost | yaw cost |
|---|---:|---:|---:|
| high delta-v | 0 | 54.3376 | 0.0199705 |
| high delta-v | 3 | 40.6545 | 13.3733 |
| high delta-v | 6 | 64.6683 | 50.6580 |
| high delta-omega | 0 | 399.994 | 5.15240 |
| high delta-omega | 3 | 5.71096 | 5.27551 |
| high delta-omega | 6 | 5.54856 | 3.54860 |
| benign delayed | 0 | 400.000 | 1.31683e-5 |
| benign delayed | 3 | 4.11169e-5 | 3.29454e-4 |
| benign delayed | 6 | 1.70233e-4 | 0.0145320 |

Each factor artifact includes its physical residual vector, physical and normalized norm,
historical-weight diagnostic cost, and active-objective cost at initial and final state.

## 10. Per-factor gradient

For benign `k=0`, initial weighted `||J_f^T r_f||` was direction `1007.6283`, yaw
`0.0362881`, entry `0`, and fresh-motion `3.80e-15`. Across all nine real conditions,
direction gradients ranged `0.185–9831.45`; yaw ranged `0.0363–71.1745`. Full physical and
weighted gradient arrays, first-node norm, and every downstream-node norm are saved for both
initial and final states.

## 11. Factor conflict

At raw initialization the entry and fresh residuals are zero, so their gradient cosines are
correctly `null`, not zero. At the benign FULL solution, entry and direction weighted
gradients have cosine `-0.999999886`: they oppose almost exactly. Entry versus yaw is
`-0.000478`, and direction versus yaw is `0.0`. Fresh-motion's final gradient is below the
predeclared `1e-9` near-zero threshold, so its cosines remain null instead of amplifying
numerical noise.

This is a factor conflict at the solution. It does not rescue the direction target: removing
entry moves the entry farther, from `0.381925 m` to `0.396972 m`.

## 12. Benign Case C, k=0 attribution

| variant | `|delta v_des|` m/s | `|delta omega_des|` rad/s | entry m | endpoint m | rigid RMS m |
|---|---:|---:|---:|---:|---:|
| RAW / V1 | 0.00442238 | 0.00197206 | 0 | 0 | 1.40e-16 |
| V2 direction | 0.24096762 | 0.00347956 | 0.381925 | 0.381925 | 2.19e-12 |
| V3 yaw | 0.00442236 | 0.00296041 | 3.15e-12 | 0.000245751 | 7.33e-13 |
| V4 FULL | 0.24096764 | 0.00359425 | 0.381925 | 0.381931 | 7.15e-15 |
| V5 no entry | 0.24096765 | 0.00373657 | 0.396972 | 0.396984 | 2.28e-13 |
| V8 no propagation | 0.00442238 | 0.00197206 | 0.381925 | 0 | 0.114577 |

The raw B→F direction is almost antiparallel to the measured incoming direction, giving a
physical direction residual norm near `2` and normalized cost near `400`. Since direction
uses raw radius `d_k=0.198486 m`, its target is approximately the opposite point at that same
radius; raw-to-target displacement is about `2d_k=0.396972 m`. Entry preservation restrains
that target to `0.381925 m`, but does not reject it. The direction-only variant reproduces
essentially all of FULL's `delta-v` over-correction. The objective has no controller term and
therefore has no representation of the fact that raw's desired-command switch was benign.

The V8 desired command happens to match raw because the follower's lookahead sees the fixed
raw downstream after a severe first-edge kink. This is a spatial counterfactual, not evidence
that V8 is a usable reconciliation method.

## 13. Translation-magnitude limitation

No current residual compares `||P→B||` with `||B→X_k||`. Direction explicitly normalizes OLD
motion and preserves raw radius `d_k`; entry preserves raw F_k; yaw is angular; fresh-motion
preserves relative FRESH edges. Frozen real absolute magnitude differences include
`0.923521 m` (high-delta-v, `k=6`) and `0.871879 m` (high-delta-omega, `k=6`). These values do
not by themselves define an error because the LightNav rows have no intrinsic timestamp and
OLD control-interval displacement is not a waypoint interval. They do prove that changing
weights cannot create a missing direct magnitude residual.

## 14. Fresh-motion propagation and rigidity

The fresh-motion factor constrains only `X_j^-1 X_{j+1}` to equal
`F_j^-1 F_{j+1}`. A common left transform of the entire suffix leaves every such residual
unchanged. Once an entry factor moves `X_k`, the zero-deformation way to satisfy the chain is
therefore to apply nearly the same left transform to every downstream pose. Across all nine
real FULL solutions, the worst best-fit-single-transform translation RMS was only
`1.26703e-11 m`.

There is no absolute downstream anchor and no tapered return term. The current graph has many
nodes but, under these factors, uses them almost as a rigid suffix. V8 shows the alternative
without propagation: raw endpoint is recovered only by accepting a large local edge kink.

## 15. What factor is responsible for what behavior

- **Condition-specific bad target:** incoming direction drives benign `k=0` over-correction.
- **Factor conflict:** entry and direction oppose at the FULL solution; entry limits but does
  not eliminate the correction.
- **Structural limitation:** relative fresh-motion preservation propagates entry correction
  as a common left transform because no downstream absolute recovery is present.
- **Redundancy:** no current factor is established as globally redundant. V6=V3 and V7=V2
  are duplicate experimental labels by design, not evidence that yaw or direction is useless.
- **Missing mechanism:** no direct transition-magnitude residual and no local-correction/
  downstream-return residual exists.

## 16. Saved schema and controller probe

Every condition/variant directory contains `raw_fresh.npy`, `optimized.npy`, initial/final
factor residual and cost JSON, initial/final gradient NPY+JSON, initial/final cosine JSON,
optimization history, geometry, desired controller metrics, rigid-fit metrics, and metadata.
Six aggregate summaries and 13 plots live under `summary/` and `plots/`.

The controller metric uses the unchanged `TrajectoryFollower`, exact saved B, and exact final
OLD desired command. It probes desired commands spatially at B then candidate nodes X0/X1 and
reports immediate and first-three consecutive changes. It does not execute Isaac physics and
does not invent waypoint timestamps; it is explicitly not a measured response or time-aligned
trajectory metric.

## 17. GUI and commands

```bash
.venv/bin/python scripts/run_exp02c_factor_isolation.py \
  --run-id exp02c-factor-isolation-20260908T133000Z
.venv/bin/python scripts/summarize_exp02c_factor_isolation.py \
  data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z

RUN=data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z
./scripts/isaac/run_exp02c_factor_isolation_gui.sh "$RUN" \
  --view real_benign_k0
./scripts/isaac/run_exp02c_factor_isolation_gui.sh "$RUN" \
  --view synthetic_s4
```

The real legend is blue OLD, grey raw FRESH/F_k, cyan V1, orange V2, purple V3, red FULL,
green NO_ENTRY, white B, and colored X_k markers. The S4 legend is grey raw, yellow desired
diagnostic target, red FULL, magenta no-propagation, and white B. Both views also load the
official `Clearpath/Jackal/jackal.usd` used by Stage 0 and place it at the real view's saved B
or S4's declared synthetic B. This is a static pose reference: the articulation is not
initialized, the timeline is not played, and no robot motion or physics evidence is produced
(`physics_executed=false`). Metadata records the world-frame SE(2) pose and units. The terminal
explicitly prints this meaning and the robot pose, then prints factor costs/gradients, desired
deltas, entry/endpoint displacement, and rigid-fit RMS. GUI images do not replace numeric
evidence.

Interactive execution holds at `EXP02C_GUI_PHASE=READY_AND_HOLDING` until the Isaac window is
closed or `Ctrl-C` is pressed. `--no-hold` is an automation-only option that intentionally
closes the window after its viewport capture; it should not be used for visual inspection.

## 18. Claims, limitations, and one recommendation

The frozen data support a precise attribution: incoming direction caused the benign-case
candidate over-correction; entry fought it; fresh-motion propagated it almost rigidly; current
factors cannot express downstream recovery or direct transition magnitude. Solver convergence
and cost reduction do not establish that the target is desirable.

This experiment cannot claim a new objective works, a factor causes physical Jackal behavior,
OLD displacement is the correct LightNav waypoint spacing, synthetic behavior generalizes, or
any controller/terrain/contact cause. No new Isaac execution experiment was performed.

Exactly one recommended EXP-02D change is: **redesign the incoming-direction transition
residual semantics before adding any other factor**. In the primary falsifiable failure it
alone changes `|delta v_des|` by `0.236545 m/s`, whereas yaw-only changes it by
`1.71e-8 m/s`. A future downstream-intent factor remains plausible, but testing it at the
same time would confound the dominant bad-target diagnosis.
