# EXP-02D — Lookahead-Aware Transition Direction

## A. Research question

EXP-02D tests one semantic change to the existing EXP-02A/B current-M4 objective: does an
incoming-direction factor become more faithful to controller execution when it points from the
saved switch boundary `B` to the raw FRESH waypoint actually selected by the frozen follower's
lookahead, rather than to the raw FRESH entry selected as nearest progress?

This is a development-only formulation experiment. It is not a final held-out evaluation, a new
correspondence detector, a navigation benchmark, or physical execution of optimized candidates.
The immutable combined DATA-02 decision remains
`DATA02_COMBINED_DIVERSITY_INSUFFICIENT`; the failed duplicate-isolated split is not changed.

## B. Historical EXP-02C failure

EXP-02C showed that current M4 can react strongly when `F_k` lies behind or to the side of `B`,
even when the unchanged `TrajectoryFollower` considers the raw switch benign. In the frozen
benign `k=0` case, the direction factor accounts for essentially all of the false correction.
EXP-02D isolates that diagnosed semantic problem; it does not add downstream recovery, a command
factor, correspondence, feasibility, latency, obstacle, or smoothing terms.

## C. Exact historical M1 equation

Let `P` be the saved actual control pose immediately before `B`, and let the editable raw suffix
be `F_k, ..., F_(N-1)` with candidate nodes `X_0, ..., X_(N-k-1)`. Historical M4 is retained
unchanged as `M1_HISTORICAL_M4`:

```text
u_in        = (B_xy - P_xy) / ||B_xy - P_xy||
d_k         = ||F_k.xy - B.xy||
r_entry     = Log(F_k^-1 X_0)
r_dir_old   = (X_0.xy - B.xy) / d_k - u_in
r_yaw       = wrap(yaw(B^-1 X_0) - yaw(P^-1 B))
r_fresh,j   = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
```

The production `TransitionGraphProblem` and `incoming_motion_residual` remain immutable.

## D. Exact new M3 equation

The frozen follower is instantiated on the full raw observation-anchored FRESH trajectory and
evaluated exactly once at `B`. Its `nearest_index` is `k`; its `target_index` is `q`; and
`q_rel = q-k` indexes the corresponding node in the selected suffix. `M3_LOOKAHEAD` changes only
the two-component direction residual:

```text
d_q       = ||F_q.xy - B.xy||
r_look    = (X_q_rel.xy - B.xy) / d_q - u_in
```

The denominator is the fixed raw radius `d_q`, never the candidate distance. Entry, incoming-yaw,
fresh-motion factors, normalization, weights, retraction, finite differencing, and solver settings
are identical to historical M4.

If `||B-P|| <= 1e-6 m`, the transition is explicitly
`INCOMING_DIRECTION_UNDEFINED`. If `d_q <= 1e-6 m`, it is explicitly
`LOOKAHEAD_DIRECTION_UNDEFINED`. No epsilon direction or silent factor deletion is allowed.

## E. Why q is execution-relevant

The saved DATA-02 follower configuration uses a `0.25 m` path-distance lookahead. At `B`, the
follower first chooses monotonic nearest/progress index `k`, then selects the first path index at
least `0.25 m` farther along the raw path (bounded by its endpoint) as `q`. That raw `F_q` drives
the immediate heading-to-target term in the actual controller calculation. Using this exact frozen
selector is an experimental oracle for this backend study. It is not the final collaborator or
front-end answer to where two chunks should connect.

## F. Development-only DATA-02 limitation

The formulation corpus is every reconstructable `ELIGIBLE_MOVING` transition from immutable v1
and v2. The corpus is deliberately called the **EXP02D DEVELOPMENT CORPUS**. One exact ordered raw
pair dominates the combined data and links most episodes into one component, so the prior isolated
split is infeasible. Pair-balanced statistics give every unique exact ordered raw pair equal total
weight, and deterministic bootstrap resampling occurs over pair groups—not over duplicated
transitions. Transition-weighted values are secondary. These choices do not turn this corpus into
an independent test set.

LightNav rows remain arbitrary `N x 3 [x, y, yaw]` spatial waypoints without intrinsic timestamps.
No waypoint row is assigned model time in EXP-02D.

## G. Four methods

- `M0_RAW`: unchanged raw suffix `FRESH[k:]`; no optimization.
- `M1_HISTORICAL_M4`: entry + historical `B -> F_k` direction + yaw + fresh motion.
- `M2_NO_DIRECTION`: entry + yaw + fresh motion. This distinguishes useful lookahead information
  from merely removing a bad historical residual.
- `M3_LOOKAHEAD`: entry + new `B -> F_q` direction + yaw + fresh motion.

All historical weights are `1.0`. Translation, yaw, and direction residual scales are respectively
`0.10 m`, `0.10 rad`, and `0.10`. The minimum translation is `1e-6 m`. The unchanged solver uses
80 iterations, central finite differences at `1e-6`, right-local `T <- T * Exp(delta)` updates,
and the frozen LM damping/tolerance values in
[`configs/exp02d_lookahead_direction.yaml`](../configs/exp02d_lookahead_direction.yaml).

## H. Pair-balanced evaluation

For every method, the frozen follower is evaluated at exact saved `B`. With saved final OLD desired
command `(v_old, omega_old)`:

```text
delta_v     = v_method - v_old
delta_omega = omega_method - omega_old
J_cmd       = sqrt((|delta_v| / 0.15 m/s)^2 +
                   (|delta_omega| / 0.30 rad/s)^2)
```

`J_cmd` is evaluation-only and never enters an objective. Primary means first average contexts
within each exact ordered raw-pair group and then average the group means. The frozen cluster
bootstrap uses seed `20260910`, 10,000 repetitions, and a 95% percentile interval for paired
method differences.

## I. Frozen success/failure definitions

The raw BENIGN threshold is `|delta v| <= 0.05 m/s` and
`|delta omega| <= 0.10 rad/s`. CHALLENGING is `|delta v| >= 0.15 m/s` or
`|delta omega| >= 0.30 rad/s`; all other cases are INTERMEDIATE.

`BENIGN_PRESERVED`, `BENIGN_BROKEN`, `CHALLENGING_RESCUED`,
`CHALLENGING_IMPROVED` (`J_M3 <= 0.80 J_RAW`), `CHALLENGING_WORSE`
(`J_M3 >= 1.20 J_RAW`), `CHALLENGING_MIXED`, `GEOMETRY_UNDEFINED`, and
`SOLVER_FAILURE` are analysis labels, not deployment gates. The separate
`M4_FALSE_CORRECTION_RESCUED` flag means raw BENIGN, M1 non-BENIGN, and M3 BENIGN.

Representative rules are fixed before the primary run: S1 is the largest M1-to-M3 benign rescue;
S2 is the largest raw-to-M3 challenging improvement that also beats M2; F1 is the worst >=20%
M3 degradation on a turning FRESH; and F2 is the largest positional gap with lookahead alignment
and <=10% command-score change. A missing category remains `NOT_AVAILABLE`; criteria are never
relaxed to manufacture a GUI example.

The descriptive regime probes are also frozen before the primary run:

- R1 `FALSE_ENTRY_DIRECTION_MISMATCH`: `alpha_entry >= 60 deg` and
  `alpha_look <= 15 deg`.
- R2 `INTENT_CONFLICT_CANDIDATE`: `alpha_look >= 45 deg` and the raw FRESH geometry is positive
  or negative turning. Geometry alone does not prove instruction intent.
- R3 `POSITION_GAP_DIRECTION_CONSISTENT`: `alpha_look <= 15 deg` and
  `B -> F_k >= 0.20 m`.
- R4 `DEGENERATE_LOOKAHEAD`: `q == k`, `d_q <= 0.05 m`, or raw arc length
  `F_k -> F_q <= 0.05 m`.
- R5 `LARGE_CORRECTION_LITTLE_COMMAND_BENEFIT`: M3 translation deformation RMS is at least
  `0.20 m` while relative `J_cmd` reduction is at most 10% (including a worsening).
- R6 `RIGID_PROPAGATION`: entry displacement is at least `0.05 m`, endpoint and entry
  displacement differ by at most `1e-6 m`, and the best-fit single-rigid-transform translation
  and yaw RMS are each at most `1e-6` in their respective metre/radian units.

These bins are overlapping descriptive probes, not fitted gates or added objective terms.

## J–L. Aggregate results, regimes, and representatives

Pending the one-time primary development-corpus execution from the clean protocol commit. This
section is populated only in the result commit, after the frozen analysis has completed.

## M. GUI explanation and corrected actual-path semantics

The EXP-02D GUI has two explicit phases. Phase A replays only saved actual Jackal motion while the
displayed current OLD chunk was active. For transition `i>0`, its activation anchor is the
previous transition's exact saved `B`; for transition `0`, it is the earliest saved telemetry
state. Every saved telemetry row in the green buffer has
`active_chunk_id == old_chunk_id`. It ends exactly at selected `B`; no previous-chunk or
post-switch sample is shown.

At `B`, the Jackal stops. Phase B reveals offline candidates without executing them:

- blue: current OLD reference;
- green: saved actual motion only during that OLD's active interval;
- gray: selected raw FRESH suffix;
- orange: M1 historical M4 candidate;
- magenta: M3 lookahead candidate;
- optional cyan: M2 no-direction candidate;
- yellow/orange/red points: observation, `P`, and `B`;
- small labeled `F_k` and `F_q` points;
- at most two arrows: `P -> B` incoming actual motion and `B -> F_q` raw follower lookahead.

The replay says `PRESENTATION SPEED != SCIENTIFIC TIME`. Candidate paths were not physically
executed. Default inspection holds at Phase B until Isaac Sim is closed; `--no-hold` is reserved
for automated capture.

Run the quantitative protocol and an available representative GUI with:

```bash
.venv/bin/python scripts/run_exp02d_lookahead_direction.py \
  --phase primary --run-id exp02d-lookahead-primary-YYYYMMDDTHHMMSSZ

./scripts/isaac/run_exp02d_success_failure_gui.sh \
  --run data/exp02d_lookahead_direction/exp02d-lookahead-primary-YYYYMMDDTHHMMSSZ \
  --case S1
```

## N–Q. Evidence boundary and next uncertainty

EXP-02D can measure candidate geometry, immediate desired-command continuity, deformation,
near-rigid propagation, and their association with saved transition geometry. It cannot establish
physical Jackal improvement, trajectory feasibility, obstacle avoidance, navigation or instruction
success, final generalization, or final correspondence quality. A geometric
`INTENT_CONFLICT_CANDIDATE` is not proof that instruction intent was violated. An optimized
candidate is not a physical execution.

The report will explicitly retain negative evidence, including M3≈M2, challenging cases not
improved, turning cases worsened, heavy deformation for little command benefit, degenerate
lookahead, and near-rigid correction propagation. Correlations are described as associations, not
causes.

If lookahead semantics help but candidates remain almost rigid, the next uncertainty is whether a
local splice correction can recover toward downstream raw FRESH intent. That is a possible EXP-03
question only; downstream intent recovery is not implemented here.
