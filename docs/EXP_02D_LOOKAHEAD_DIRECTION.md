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

## J. Primary aggregate results

The valid primary run is
`data/exp02d_lookahead_direction/exp02d-lookahead-primary-20260910T171139Z`, generated from the
clean protocol commit `f25fda2877bcfda8a599b739c252ab174d62f5b9`. The strict read-only
validator reports:

- 959 `ELIGIBLE_MOVING`, 959 reconstruction-valid, and 959 solver-valid transitions;
- zero `INPUT_RECONSTRUCTION_FAILED`, `GEOMETRY_UNDEFINED`, and `SOLVER_FAILURE` cases;
- 191 unique ordered raw-pair groups;
- 12,489 hashed canonical result artifacts and all 10 predeclared plots;
- exact full-raw and selected-suffix first-command reconstruction error `0.0`, within the
  predeclared `1e-12` tolerance; and
- every M0 selected-suffix absolute nearest/target index equal to reconstructed raw `k/q`.

The oracle distributions were `k={0:823, 1:133, 2:1, 5:2}`,
`q={2:818, 3:133, 4:1, 9:7}`, and `q-k={2:951, 4:3, 7:1, 9:4}`.

The primary all-valid command results are below. `PB` is the predeclared pair-balanced mean;
`TW` is the secondary transition-weighted mean. Absolute-command columns headed `PB` are also
pair-balanced. Medians and p90 values are transition-level descriptive summaries.

| method | PB mean `J_cmd` | TW mean `J_cmd` | PB mean `|delta v|` (m/s) | PB mean `|delta omega|` (rad/s) | median `|delta v|` (m/s) | median `|delta omega|` (rad/s) | median `J_cmd` | p90 `J_cmd` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M0 RAW | 1.393617454 | 0.536701738 | 0.102279955 | 0.321018759 | 0.027599595 | 0.004045198 | 0.185056500 | 1.021568575 |
| M1 historical M4 | 0.917927768 | 0.633044601 | 0.079963248 | 0.162085936 | 0.089820394 | 0.004120718 | 0.599810174 | 0.875486480 |
| M2 no direction | 1.208490073 | 0.483677523 | 0.095813512 | 0.263771967 | 0.027599595 | 0.003292300 | 0.184943870 | 0.979240796 |
| M3 lookahead | **0.872420716** | **0.381861584** | **0.062969737** | 0.180882173 | **0.027586230** | 0.005384988 | 0.185072576 | 0.949984166 |

The deformation summaries distinguish changing a suffix from approximating that change by one
left-multiplied rigid transform:

| method | PB translation deformation RMS (m) | PB yaw deformation RMS (rad) | PB best-fit rigid translation RMS (m) | PB best-fit rigid yaw RMS (rad) | TW translation deformation RMS (m) | TW yaw deformation RMS (rad) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| M0 RAW | 2.53e-15 | 3.52e-17 | 8.37e-15 | 9.59e-17 | 4.34e-15 | 6.78e-17 |
| M1 historical M4 | 0.069129978 | 0.040448808 | 5.20e-12 | 2.43e-11 | 0.057798025 | 0.011791506 |
| M2 no direction | 0.022626533 | 0.040484430 | 2.68e-12 | 5.33e-11 | 0.007204264 | 0.011799366 |
| M3 lookahead | 0.043934685 | 0.041542127 | 0.006114109 | 0.055647766 | 0.013242147 | 0.012137787 |

M1 and M2 are numerically almost exact single-rigid-transform corrections under this diagnostic.
M3 is not: its nonzero rigid-fit residual means the lookahead factor produced some non-rigid
shape change. This alone does not establish useful downstream-intent recovery or feasibility.
No method changed the candidate follower target away from raw `q` in any of 959 cases. Candidate
nearest index changed from raw `k` in three M1, one M2, and three M3 cases; this post-hoc discrete
diagnostic was never fed back into optimization.

The pair-balanced command score by frozen raw difficulty was:

| raw partition (count; pair groups) | M0 RAW | M1 historical | M2 no direction | M3 lookahead |
| --- | ---: | ---: | ---: | ---: |
| BENIGN (685; 62) | 0.180025224 | 0.544424569 | **0.162880457** | 0.176888298 |
| INTERMEDIATE (178; 73) | 0.677529752 | 0.647822428 | **0.637739022** | 0.905071823 |
| CHALLENGING (96; 74) | 2.977955733 | 1.418957805 | 2.531539811 | **1.382661375** |

Pair groups can occur in more than one difficulty partition, so partition group counts do not sum
to 191. The paired cluster-bootstrap differences use `M3 - comparator`; negative favors M3:

| partition | comparison | estimate | 95% CI |
| --- | --- | ---: | ---: |
| all valid | M3 - RAW | -0.521196739 | [-0.761689581, -0.279911498] |
| all valid | M3 - M1 | -0.045507053 | [-0.150310910, 0.066939864] |
| all valid | M3 - M2 | -0.336069357 | [-0.554956379, -0.109996964] |
| BENIGN | M3 - RAW | -0.003136926 | [-0.018430030, 0.015050360] |
| BENIGN | M3 - M1 | -0.367536272 | [-0.416284235, -0.319390587] |
| BENIGN | M3 - M2 | 0.014007840 | [-0.001076388, 0.033419609] |
| INTERMEDIATE | M3 - RAW | 0.227542071 | [0.007198205, 0.539197075] |
| INTERMEDIATE | M3 - M1 | 0.257249395 | [0.113186173, 0.415290540] |
| INTERMEDIATE | M3 - M2 | 0.267332801 | [0.041005310, 0.591166217] |
| CHALLENGING | M3 - RAW | -1.595294358 | [-2.064299496, -1.131593567] |
| CHALLENGING | M3 - M1 | -0.036296430 | [-0.253722220, 0.190713752] |
| CHALLENGING | M3 - M2 | -1.148878436 | [-1.592368272, -0.712844033] |

Accordingly, M3 improved on RAW and M2 in the all-valid and CHALLENGING pair-balanced means. It
was not distinguishable from M1 overall or within CHALLENGING. In BENIGN, M3 was not
distinguishable from RAW or M2, while it was clearly lower than M1. In INTERMEDIATE, M3 was
clearly worse than all three comparators under the frozen analysis.

Of 685 raw-BENIGN cases, M1 broke 662, M2 broke zero, and M3 broke one. M3 therefore preserved
684/685 (99.8540%) transition-weighted and 98.3871% pair-balanced. The direct
`M4_FALSE_CORRECTION_RESCUED` condition occurred 661 times: 68.9260% of all valid transitions,
96.4964% of raw-BENIGN transitions, 25.9609% pair-balanced over all valid pairs, and 86.7606%
pair-balanced within raw-BENIGN pairs. Those rescues were tightly associated with opposing entry
and aligned lookahead geometry: mean/median `alpha_entry` were 3.138718/3.139311 rad, while
mean/median `alpha_look` were 0.001325/0.000832 rad. Their median `B -> F_k` and `B -> F_q`
distances were respectively 0.026040 m and 0.275935 m. The rescue count is visibly affected by
duplication: its median ordered-pair frequency was 593, which is why pair-balanced fractions are
reported beside transition counts.

Of 96 raw-CHALLENGING cases, M3 produced 42 `CHALLENGING_RESCUED`, 17
`CHALLENGING_IMPROVED`, 31 `CHALLENGING_MIXED`, and 6 `CHALLENGING_WORSE`. Their
transition-weighted fractions were 43.7500%, 17.7083%, 32.2917%, and 6.2500%; corresponding
pair-balanced fractions were 38.1596%, 21.6216%, 35.1030%, and 5.1158%.

The predeclared geometry hypothesis had pair-balanced Pearson correlation `r=0.222586234`
between `alpha_entry-alpha_look` and `J_M1-J_M3`, with frozen-seed pair-cluster bootstrap 95% CI
`[0.092550510, 0.342933471]`. The transition-weighted correlation was `0.290805536`. This is a
modest positive association consistent with the hypothesis, not a causal relationship and not a
guarantee for an individual transition.

## K. Success and failure regime analysis

Regime membership overlaps; counts below must not be summed.

### R1 — false entry-direction mismatch

R1 contained 807 transitions from 121 ordered-pair groups: 669 raw BENIGN, 114 INTERMEDIATE,
and 24 CHALLENGING. Outcomes were 668 benign preserved, one benign broken, 12 challenging
rescued, three improved, eight mixed, one worse, and 114 unlabeled raw-intermediate cases. Its
pair-balanced `J_cmd` means were RAW `0.621312493`, M1 `0.616480099`, M2 `0.541316487`, and M3
`0.511847737`. The M4 false-correction rescue occurred 661 times; its pair-balanced indicator
within R1 was `0.430188539`.

This descriptively supports the proposed mechanism: in R1, M3 was lower than M1, RAW, and M2 on
the pair-balanced mean, and the large direct-rescue count is concentrated here. Negative evidence
remains: R1 still contains one M3 benign break and one challenging worsening, and no R1-specific
bootstrap was predeclared. R1 also covers most of this duplicate-heavy development corpus, so its
transition count is not independent replication.

### R2 — intent-conflict candidate

Only three unique-pair transitions met R2. Their pair-balanced/transition-weighted means coincide:
RAW `3.633086424`, M1 `4.316777790`, M2 `3.481685752`, and M3 `4.130291028`. The cases were:

- `v1:episode_000044_transition_04`, raw INTERMEDIATE: M3 worsened `J_cmd` from `0.912903`
  to `9.886601` while M2 was `0.458701`;
- `v2:episode_000145_transition_04`, raw CHALLENGING: M3 improved `4.804955` to `2.051041`;
- `v2:episode_000148_transition_05`, raw CHALLENGING: M3 rescued `5.181401` to `0.453231`.

Thus R2 contains both a severe failure and two benefits. Its mean M3 deformation was
`0.153713 m / 0.520967 rad`, with best-fit rigid residual `0.081413 m / 1.865992 rad`. Three cases
are insufficient to infer a population effect. Turning geometry and large `alpha_look` make these
cases intent-conflict *candidates* only; the data contain no instruction-intent measurement.

### R3 — direction-consistent positional gap

R3 had zero cases. Among the 915 transitions with `alpha_look <= 15 deg`, the largest
`B -> F_k` gap was only `0.093473 m`, below the frozen `0.20 m` threshold. This corpus therefore
does not test the proposed direction-consistent large-position-gap failure.

### R4 — degenerate lookahead

R4 had two unique-pair cases, both also in R2. Neither had `q==k` or small `d_q`; both entered R4
because raw `F_k -> F_q` arc length was only `0.009503 m` or `0.009949 m`. One was challenging
improved and one challenging rescued. Mean `J_cmd` was RAW/M2 `4.993178281`, M1 `1.548016919`,
and M3 `1.252136105`.

This is negative evidence against treating every short-arc lookahead as an automatic command-score
failure. It is not evidence that the geometry is safe: there are only two cases, and mean M3
deformation was `0.197306 m / 0.663672 rad` with rigid-fit residual
`0.105258 m / 2.421125 rad`.

### R5 — large correction with little command benefit

R5 had zero cases. Twenty transitions exceeded the `0.20 m` deformation threshold, but their
relative command-score reductions ranged from `0.694314` to `0.932241`, above the frozen 10%
maximum for R5. The absence of R5 does not establish feasibility or justify large deformation;
it says only that this exact conjunction was not observed.

### R6 — rigid propagation

R6 had zero cases. Fifteen M3 candidates moved the entry by at least `0.05 m`, but none also met
both `1e-6` best-fit rigid tolerances, and none met the `1e-6 m` endpoint-entry displacement
agreement. The strict M3 rigid-propagation regime therefore was not observed. Separately, the
near-zero all-valid rigid-fit residuals for M1 and M2 show that those formulations remain almost
exactly rigid-like; that observation must not be misreported as an R6 count for M3.

## L. Deterministic representatives

Selection used the complete primary result and the frozen rules without relaxation:

| case | transition | raw class / geometry | `k/q` | `alpha_entry / alpha_look` (rad) | `B->F_k / B->F_q` (m) | `J` RAW / M1 / M2 / M3 |
| --- | --- | --- | --- | ---: | ---: | ---: |
| S1 | `v1:episode_000016_transition_00` | BENIGN / POSITIVE_TURNING | 0/2 | 3.139738 / 0.002466 | 0.052238 / 0.249178 | 0.098274 / 0.879276 / 0.077402 / 0.079256 |
| S2 | `v1:episode_000027_transition_01` | CHALLENGING / NEGATIVE_TURNING | 1/3 | 1.979661 / 0.641880 | 0.064021 / 0.307976 | 5.645091 / 1.701474 / 5.502338 / 0.382505 |
| F1 | `v1:episode_000044_transition_04` | INTERMEDIATE / NEGATIVE_TURNING | 0/9 | 1.451432 / 1.043228 | 0.087714 / 0.112969 | 0.912903 / 9.854300 / 0.458701 / 9.886601 |
| F2 | `F2_NOT_AVAILABLE` | no strict eligible case | — | — | — | — |

S1 is the largest frozen benign rescue. Historical M1 changed primarily linear command
(`|delta v|=0.131570 m/s`) and became non-BENIGN, whereas M3 stayed BENIGN
(`|delta v|=0.006541 m/s`, `|delta omega|=0.019855 rad/s`). Its near-antiparallel entry and aligned
lookahead directly illustrate the diagnosed historical semantic error. M2 was slightly lower than
M3 (`0.077402` versus `0.079256`), so S1 does not show added lookahead benefit over factor removal.

S2 is the strongest challenging improvement that also beats M2. M3 reduced
`|delta v|/|delta omega|` from RAW `0.400016 m/s / 1.492643 rad/s` to
`0.032052 m/s / 0.095177 rad/s`. However, it used `0.247120 m / 0.177793 rad` deformation and had
`0.021357 m / 0.056048 rad` rigid-fit residual. It is a strong desired-command candidate result,
not physical or feasibility evidence.

F1 is the predeclared worst turning degradation and belongs to R2. M3 increased
`|delta omega|` from `0.273871` to `2.965980 rad/s`; M2 instead reduced it to `0.137610 rad/s`.
This is a genuine counterexample to universal improvement. It is consistent with possible
continuity-versus-new-turn conflict, but instruction intent was not measured.

F2 remained `F2_NOT_AVAILABLE`; the positional-gap criterion was not weakened to manufacture an
example. A separate non-representative benign failure also matters:
`v1:episode_000050_transition_05` moved from RAW `J=0.224496` and M2 `J=0.158930` to M3
`J=0.622609`, because M3 `|delta omega|` reached `0.180688 rad/s`, despite R1 geometry. Therefore
M3 has no empirical benign-safety guarantee.

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

Validate the completed primary result and open an available representative GUI with:

```bash
.venv/bin/python scripts/validate_exp02d_lookahead_direction.py \
  data/exp02d_lookahead_direction/exp02d-lookahead-primary-20260910T171139Z

./scripts/isaac/run_exp02d_success_failure_gui.sh \
  --run data/exp02d_lookahead_direction/exp02d-lookahead-primary-20260910T171139Z \
  --case S1
```

The corrected collection-level high-motion viewer was also rerun against immutable v1
`episode_000007/transition_03` (current OLD `chunk_03`). Its exact active interval is
`[20.133334383368492, 21.733334466814995] s`, with observation at
`20.783334417268634 s` and `P` at `21.63333446159959 s`. The unscaled active-OLD path is
`0.5807040935130896 m`; all 96 telemetry rows carry the current OLD ID, and the exact preceding
switch-boundary pose is prepended separately as the activation event. The inspected output is
`data/data02_high_motion_demo/20260910T161500.312194Z/`. It shows neither previous-chunk nor
post-switch actual motion.

The final EXP-02D `--no-hold` validation outputs (the interactive command above holds by default)
are:

| case | current-OLD active interval (saved s) | saved path (m) | telemetry/display poses | capture directory |
| --- | --- | ---: | ---: | --- |
| S1 | `[17.050000889226794, 18.133334279060364]` | 0.388668168 | 66 / 66 | `gui/S1_20260910T185116.949864Z/` |
| S2 | `[18.133334279060364, 19.63333435729146]` | 0.560807006 | 90 / 91 | `gui/S2_20260910T184930.187443Z/` |
| F1 | `[22.13333448767662, 23.53333456069231]` | 0.227760859 | 84 / 85 | `gui/F1_20260910T185024.163005Z/` |

S2 and F1 have one extra display pose because the preceding exact saved `B` is prepended as the
current OLD activation boundary; it is not a previous-chunk trajectory sample. All telemetry rows
in all three manifests passed the current-OLD ID check, all displays end at the selected saved
`B`, and the manifests explicitly record `previous_chunk_actual_displayed=false`,
`post_switch_actual_displayed=false`, and `candidate_execution=false`. Each directory contains
`01_old_active.png`, `02_at_observation.png`, `03_at_B_raw.png`,
`04_method_comparison.png`, and a hash-bearing `capture_manifest.json`. F2 has no GUI because its
frozen representative status is `F2_NOT_AVAILABLE`.

Direct inspection confirmed that the official Jackal mesh changes pose in each Phase A, the
green trail grows only over the stated active interval, the small elevated yellow observation
marker is present at exact observation XY, and Phase B keeps the robot fixed at `B`. S1 and S2
make RAW/M1/M3 readily distinct. F1 does **not** satisfy the requested 65--80% candidate framing:
its RAW and M1 paths are only about `0.09 m`. Direct inspection finds the small method-colored
segments present but marginal rather than readily distinguishable; this is a real visual
limitation, not a reason to enlarge the data. Automated palette checks establish OLD, actual,
observation, and the addition of M1/M3 colors; RAW and individual point/arrow identity still rely
on direct image inspection because Hospital gray can match the RAW palette.

The viewport uses a dedicated USD pinhole camera and deterministic coarse/fine azimuth search.
Because pure projection cannot detect Hospital racks and walls, the separately hashed GUI-only
config constrains each frozen representative to an inspected open view sector in explicit world
XY and adds a target-centered presentation fill light. These alter neither saved poses nor
quantitative evidence. Every final manifest reads back
`54.999999319 deg` horizontal and `36.045051678 deg` vertical FOV and asserts agreement with the
projection plan. The earlier frozen GUI minimum height of `3 m` placed the camera above/inside the
Hospital ceiling, so the final GUI-only camera is capped at indoor eye height without changing
quantitative results. The projected candidate fractions are `0.6793` (S1), `0.3509` (S2), and
`0.0903` (F1). S1 satisfies the requested `0.65--0.80` range; S2 trades occupancy for the open
south-side view that avoids a storage rack, and F1 is limited primarily by its intrinsically short
candidate geometry. Each manifest records the achieved fraction, view sector, containment, and
limiting constraint instead of spatially scaling geometry. Lines are repeated at documented
visual-only Z levels to avoid the opaque Jackal and overlapping methods, but every trajectory
retains exact SE(2) XY. These raised lines and the fill light must not be interpreted as ground
clearance, physical vertical separation, or physical illumination evidence. Capture warm-up
renders, saved pose/time metadata, and content checks reduce the risk that a PNG reflects the
preceding animation frame rather than its requested saved pose/phase. The manifest also hashes the exact post-primary GUI runner, pure
helper, launcher, GUI render config, and primary quantitative config snapshot; this makes the
rendered derivation reproducible without conflating camera-only corrections with the frozen
quantitative protocol.

## N. What succeeded

Within this development corpus, three findings support the revised semantics:

1. M3 lowered the all-valid pair-balanced `J_cmd` relative to both RAW and M2, and both 95%
   cluster-bootstrap intervals excluded zero. Within this corpus and immediate-command metric,
   that is evidence consistent with useful direction information beyond merely deleting the
   historical direction factor.
2. The evidence was strongest in raw-CHALLENGING cases: M3 was lower than RAW and M2 by
   `1.595294` and `1.148878` pair-balanced score units, respectively, with both confidence
   intervals wholly below zero. It rescued or materially improved 59/96 challenging transitions.
3. M3 preserved 684/685 raw-BENIGN transitions while directly rescuing 661 cases in which M1
   created a non-benign correction. The direct rescues had the predicted geometry: `alpha_entry`
   near pi and `alpha_look` near zero. The modest positive geometry-benefit correlation was also
   consistent with, though it did not prove, the predeclared hypothesis.

The unchanged post-hoc follower target in all 959 cases shows that these command differences did
not come from iteratively changing the frozen raw `q` selector inside optimization.

## O. What failed or remained negative

M3 was not universally better. In the development-corpus raw-INTERMEDIATE partition, every
frozen pair-cluster bootstrap interval for `M3 - comparator` lay above zero. Six
raw-CHALLENGING cases were worse by the frozen 20% definition, and one raw-BENIGN case was
broken. F1 is a severe turning counterexample: M3 produced
`J_cmd=9.886601` from raw `0.912903`, while M2 was `0.458701`.

M3 was not distinguishable from historical M1 in the all-valid or CHALLENGING bootstrap. In
BENIGN, M3 was not distinguishable from M2, and M2 had zero benign breaks. Therefore the result is
not evidence that lookahead direction dominates factor removal everywhere; rather, the evidence
for added directional information is concentrated in challenging cases.

Large desired-command improvements sometimes required large candidate deformation. S2 changed by
`0.247120 m / 0.177793 rad`. R4's two apparent command improvements also had large deformation and
large non-rigid residuals. R3, R5, R6, and F2 were not observed under their strict frozen criteria,
so this corpus provides no affirmative evidence about those regimes. Absence is retained rather
than relabeling cases post hoc.

## P. What cannot be claimed

EXP-02D measures offline candidate geometry, one frozen follower's immediate desired-command
continuity at exact saved `B`, deformation, near-rigid propagation, and associations with saved
transition geometry. The optimized candidates were **not physically executed**. Consequently,
this experiment cannot establish:

- physical Jackal improvement or skid-steer behavior;
- dynamic or kinematic trajectory feasibility;
- obstacle avoidance, navigation success, or instruction success;
- preservation of downstream FRESH intent;
- a final correspondence/front-end detector;
- superiority on an independent held-out benchmark; or
- generalization beyond the duplicate-heavy DATA-02 development corpus.

`J_cmd` is a desired-command discontinuity probe, not an objective term or navigation metric. A
geometric `INTENT_CONFLICT_CANDIDATE` is not proof that instruction intent was violated. Likewise,
a lower candidate command jump is not evidence of better physical execution. The result validator
rechecks provenance, hashes, schemas, follower commands, classifications, aggregate summaries, and
plots, but does not independently rerun every numerical optimization; exact optimizer behavior is
also protected by the frozen formulation and historical regression tests.

## Q. Next uncertainty

The revised lookahead factor was associated with lower immediate command discontinuity in
challenging cases and avoided most historical benign false corrections in this development
corpus, but its correction can still be large and it does not explicitly recover downstream raw
intent. The next uncertainty is therefore:

> Can a transition correction remain local near the splice while downstream candidate poses
> recover toward the raw FRESH trajectory?

That question could motivate a later EXP-03 downstream-intent-recovery study. No such factor,
objective, selector, or experiment is implemented in EXP-02D.
