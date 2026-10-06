# CANONICAL_GRAPH_COMMON_B_EXECUTION_01

## Frozen protocol

**Timing-controlled offline common-B execution diagnostic.** These E1–E4 handoffs
have already been studied. This is a development mechanism comparison, not a
held-out evaluation or an online asynchronous latency benchmark.

Starting HEAD: `0ed56432b968908f83b9341d6661c11b1b1f0445` (also fetched origin/main).
Canonical planning authority: freeze `6344fddd848867fc9eb3480f19321fbc2ad418d9`,
result `0ed56432b968908f83b9341d6661c11b1b1f0445`.
Historical execution authority: DIRECT_TRANSITION_HARD_EVAL_02, freeze
`8a42e3973587eea1fd6e63f2ee240609479787af`, result
`2b8cdf89d47eed656a83768800245d820dd82abd`.

| Label | Exact saved source |
|---|---|
| E1 | episode_009_repeat_00/handoff_001 |
| E2 | episode_017_repeat_01/handoff_008 |
| E3 | episode_007_repeat_01/handoff_001 |
| E4 | episode_016_repeat_00/handoff_008 |

OSA03 stays planning-only context; the execution scope is frozen at four primary
sources before outcomes. No new source search, acquisition, LightNav, RGB or Isaac.

### References and identities

- `B_FULL_RAW = [B, F_0, ..., F_(N-1)]`: exact original world FRESH and local rows.
- `B_ENTRY_STAGE = [B, E*, original suffix]`: exact historical reference **and rollout**.
- `B_CANONICAL = [B, X_0, ..., X_(N-1)]`: exact saved canonical solution, with B
  prepended only as an interface row. **No optimizer call.**

A and B are fixed. The historical observation-to-application state-shift /
transport factor encourages editable early nodes toward `B A^-1 F_j`, equivalently
`B^-1 X_j ≈ A^-1 F_j`. Its objective, normalization, relative and anchor factors,
LM solver and solution are unchanged. This experiment adds no graph factor.

B has no original identity. X_j retains original F_j identity j and **original**
arc progress, despite its changed world pose. E* retains the frozen fractional
identity. World uses metres, +Z up and CCW yaw radians; observation local uses
+x forward, +y left. All local rows use original observation pose A: `A^-1 B`
and the exact saved original-A local raw/canonical arrays. Raw FRESH is never
anchored at B. Observation/request/receipt/application times remain distinct in
copied source audit/common-state artifacts. LightNav rows remain untimed.

Safety checks inspect every explicit edge of the planning and installed world
polylines, including B→F0 / B→X0 / B→E*. Radius .20 m, required footprint-edge
clearance .05 m and historical workspace/numerical reserve are unchanged. The
reused checker's legacy `query` text says “no B connector”: it adds no *implicit*
connector; our explicit B row is part of the checked input. Per-segment results
and `explicit_B_connector_checked` remove ambiguity. Unsafe methods are saved as
`SKIPPED_REFERENCE_INVALID`, without repair or execution.

Historical C3/Hermite/V2 are authenticated descriptive context only. There is no
compatible saved no-explicit-B Native rollout for these four sources; a Native
installation preflight is not a rollout. Consequently the isolated B insertion
effect is **N/A**. V2_SINGLE_NODE != CANONICAL_FULL_FRESH.

### Controller and logical schedule

Reuse unchanged `run_relative_factor_multisource01.run_method`,
`osa03_common_b_mpc_worker`, the official H=5 nearest+1 MPC, exact Q/R/limits,
endpoint repetition, previous_control update, generation/stale semantics and
abort-only safety guard. Source-specific state and schedules are copied byte for
byte from the historical execution run. Official source SHA256 remains
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.

Restore B pose/tick/simulation time, already-applied u_B_plus, physical u_minus
provenance and stored u_mem_B separately. Each solve uses its current submit state;
wall-clock completion cannot advance simulation or choose application time.
Validate attempted/accepted submits, releases/applications, integration count,
initial memory/generation and first legal submit state against historical B_ENTRY
through .9 s and the 180-step cap. Safety aborts preserve the exact valid prefix;
an incomplete window is censored, and an unexplained mismatch is technical failure.
A released guard-blocked command remains unapplied.

Official installation/restoration preflight replaces the numerical solve method
with a failing stub. Preflight therefore uses zero numerical MPC solves.

### Evaluation and decisions

Primary target is **full original FRESH**, with unchanged forward-only continuous
projection and shortest-angle yaw evaluation. Attachment is <=.10 m / 15 degrees
for the entire following .30 s sampled dwell. Endpoint uses the same tube/dwell
about the original FRESH final pose and actual execution time. Null remains null.
It is **original-FRESH endpoint dwell**, not navigation-task completion.
Own-installed-reference tracking is secondary and stored separately.

Full command TV uses the historical evaluator, including the physical u_minus
→ already-applied u_B_plus jump. New .3 s diagnostics use held intervals whose
start lies in `[0,.3)`, including u_B_plus, and TV between those held commands.
They exclude the pre-B jump; incomplete .3 s windows are null with observed-prefix
TV separately retained. First new command means first successful new solve result
actually applied, not the restored B command. Compare it separately to u_minus
and u_mem_B; do not impute it when withheld or guard-blocked.

Freeze meaningful early position AUC reduction at **>=5% AND >=.001 m·s**.
A timing gain requires attachment >=one integration tick earlier, except when
both attach at zero. Every positive source must preserve endpoint dwell within
one integration tick (+1e-9 numerical tolerance). Recovery from an unobserved
baseline endpoint instead requires an observed canonical endpoint.

Regressions are unsafe canonical reference with safe B_ENTRY; canonical guard
abort with completed B_ENTRY; lost attachment/endpoint; or endpoint delayed more
than one tick without compensating recovery, one-tick attachment gain or meaningful
AUC gain when both attach at zero. Compensation never waives the positive endpoint
gate. Small AUC changes alone are not catastrophic regressions.

A repeated tradeoff requires >=2 sources with meaningful transition gain and
endpoint/attachment/safety loss or command TV increase >=10% AND >=.1 (m/s linear,
rad/s angular). Report every signed metric; no scalar score.

Frozen classification precedence:

1. TECHNICAL_BLOCKED: authentication/executor/common-state/schedule/validator failure.
2. CANONICAL_INTERFACE_NOT_EXECUTABLE: >=2 unsafe installed canonical references.
3. CANONICAL_EXECUTION_GAIN_SUPPORTED: >=2 source positives, endpoint gates, no
   canonical safety regression.
4. CANONICAL_EXECUTION_TRADEOFF: >=2 transition improvements with the adverse effects above.
5. CANONICAL_EXECUTION_REGRESSION: >=2 canonical-specific regressions.
6. STAGING_REMAINS_SUFFICIENT: all evaluable B_ENTRY have both dwells; <2 positives.
7. MIXED_EXECUTION_EVIDENCE: remaining outcomes.

**Pre-science clarification of the suggested precedence:** repeated tradeoff and
regression precede the broad staging condition, so that condition cannot hide
adverse outcomes. This is frozen before any new rollout.

### Execution budget and provenance

Prepared complete references and official installed references pass safety 8/8.
Expected full-cap counts derived from authenticated schedules: 8 new rollouts,
240 MPC solves, 236 releases/applications, 4 withheld cap results, 1440 intervals.
Four B_ENTRY and twelve C3/Hermite/V2 rollouts are reused. New canonical/V2 optimizer
calls = 0. Retries = 0. New source/LightNav/RGB/Isaac calls = 0.
Order: E1–E4, B_FULL_RAW then B_CANONICAL per source. Abort prefixes are retained;
no outcome-driven rerun or code/reference change is allowed.

Run namespace: `data/canonical_graph_common_b_execution_01/primary_20261006`.
Large raw arrays/rollouts stay ignored. Config, prepared hashes, protocol/code/tests
are committed and normally pushed before science. The exact pushed freeze SHA
is saved at execution start. Saved-only validation recomputes state integration,
safety, evaluation, controller memory/selector and command diagnostics; it never
calls an optimizer or MPC. Three final PNGs and numeric sidecars are required.

## Repository-confirmed facts

Scientific freeze **91579bdaf56f82fa0aabfbbdf16602bef99f1ed9** was normally pushed
before execution. The frozen runner completed exactly eight new rollouts:
B_FULL_RAW ×4 and B_CANONICAL ×4. All reached OBSERVATION_CAP (180 integration
intervals; actual cap 3.0000001564621925 s). Four B_ENTRY and twelve C3/Hermite/V2
rollouts were reused with authenticated bytes. No historical method was rerun.

**Frozen classification: STAGING_REMAINS_SUFFICIENT.** One graph-specific positive
(E3), one canonical-specific regression (E2), one source-level tradeoff (E3).
The >=2-positive criterion was not met. All 12 central method/source pairs have
both observed dwells; no attachment or endpoint result is null in this run.

New MPC solves 240; logical releases 236; physical applications 236; withheld at
cap 4; guard-blocked applications 0; executed intervals 1440; controller numerical
failures 0. Canonical/V2 optimizer calls, retries, historical scientific reruns,
LightNav, RGB, Isaac and new acquisitions are all **0**.

Both primary .9 s and full-cap common-B parity pass **8/8 new-versus-B_ENTRY
pairs**, including initial physical state, memory, generation, first legal submit
state and attempted/accepted/release/application schedules. Preflight numerical
MPC calls are 0. No frozen source/config/implementation changed after science.

Numbers below are rounded to six decimals; JSON/CSV retain saved precision.

### Original-FRESH transition

| Source | Method | Pos AUC .3 (m·s) | Pos AUC .9 (m·s) | Yaw AUC .3 (rad·s) | Yaw AUC .9 (rad·s) | Max XY first .5 (m) |
|---|---|---|---|---|---|---|
| E1 | B + raw | 0.023996 | 0.083744 | 0.068413 | 0.106198 | 0.105011 |
| E1 | B + entry | 0.023949 | 0.082097 | 0.066946 | 0.102576 | 0.102714 |
| E1 | B + canonical | 0.025343 | 0.093918 | 0.067394 | 0.106680 | 0.119818 |
| E2 | B + raw | 0.019169 | 0.068720 | 0.045709 | 0.077714 | 0.088069 |
| E2 | B + entry | 0.018633 | 0.058066 | 0.034563 | 0.060168 | 0.072667 |
| E2 | B + canonical | 0.019318 | 0.070080 | 0.040236 | 0.065517 | 0.087124 |
| E3 | B + raw | 0.020508 | 0.056327 | 0.014311 | 0.043807 | 0.070822 |
| E3 | B + entry | 0.020474 | 0.055381 | 0.013522 | 0.043917 | 0.070402 |
| E3 | B + canonical | 0.020145 | 0.044449 | 0.016298 | 0.076276 | 0.069390 |
| E4 | B + raw | 0.012304 | 0.038557 | 0.024051 | 0.043173 | 0.048958 |
| E4 | B + entry | 0.012149 | 0.036553 | 0.021321 | 0.040032 | 0.046385 |
| E4 | B + canonical | 0.012378 | 0.038396 | 0.021788 | 0.041203 | 0.048565 |

### Attachment location and original-FRESH endpoint dwell

| Source | Method | T_attach (s) | Original row | Original arc fraction | Remaining arc (m) | T_endpoint (s) | T_post_attach (s) |
|---|---|---|---|---|---|---|---|
| E1 | B + raw | 0.700000 | 4.019229 | 0.444941 | 0.753272 | 1.550000 | 0.850000 |
| E1 | B + entry | 0.616667 | 3.580120 | 0.396478 | 0.819043 | 1.550000 | 0.933333 |
| E1 | B + canonical | 0.933333 | 5.771963 | 0.638938 | 0.489999 | 1.466667 | 0.533333 |
| E2 | B + raw | 0.000000 | 2.608704 | 0.287040 | 0.968173 | 1.183333 | 1.183333 |
| E2 | B + entry | 0.000000 | 2.608704 | 0.287040 | 0.968173 | 1.166667 | 1.166667 |
| E2 | B + canonical | 0.000000 | 2.608704 | 0.287040 | 0.968173 | 1.200000 | 1.200000 |
| E3 | B + raw | 0.000000 | 1.082061 | 0.120442 | 1.191092 | 1.416667 | 1.416667 |
| E3 | B + entry | 0.000000 | 1.082061 | 0.120442 | 1.191092 | 1.400000 | 1.400000 |
| E3 | B + canonical | 0.000000 | 1.082061 | 0.120442 | 1.191092 | 1.383333 | 1.383333 |
| E4 | B + raw | 0.000000 | 1.995137 | 0.225367 | 1.031363 | 1.200000 | 1.200000 |
| E4 | B + entry | 0.000000 | 1.995137 | 0.225367 | 1.031363 | 1.200000 | 1.200000 |
| E4 | B + canonical | 0.000000 | 1.995137 | 0.225367 | 1.031363 | 1.183333 | 1.183333 |

E1 B_ENTRY attaches 0.316667 s earlier than canonical but reaches original-FRESH
endpoint dwell 0.083333 s later. Canonical attaches with 0.490000 m remaining,
versus B_ENTRY's 0.819043 m. Attachment and endpoint dwell measure different events.
In E2–E4 all methods already satisfy sustained attachment at B (T_attach = 0).

### Safety and commands

Every complete planning/installed reference passes including the explicit B
connector. Every executed prefix is safe; no abort occurred. The smallest new
installed-reference clearance is 0.3462769945130573 m. The smallest new swept
execution lower bound is 0.36672586896571213 m, both for E3 canonical. Required
footprint-edge clearance is 0.05 m. World/local roundoff <=3.552713678800501e-15;
planned B, raw rows and saved canonical X rows are byte-exact.

| Source | Method | Installed ref clearance (m) | Swept lower bound (m) | Linear TV (m/s) | Angular TV (rad/s) | Max abs v (m/s) | Max abs ω (rad/s) |
|---|---|---|---|---|---|---|---|
| E1 | B + raw | 2.381644 | 2.537995 | 1.600000 | 3.228923 | 0.800000 | 1.180190 |
| E1 | B + entry | 2.514410 | 2.537995 | 1.600000 | 3.214470 | 0.800000 | 1.206341 |
| E1 | B + canonical | 2.538020 | 2.537988 | 0.800000 | 2.506510 | 0.800000 | 1.170081 |
| E2 | B + raw | 0.385652 | 0.566260 | 1.420029 | 3.256614 | 0.800000 | 1.103820 |
| E2 | B + entry | 0.520011 | 0.566264 | 1.420029 | 1.989962 | 0.800000 | 0.967275 |
| E2 | B + canonical | 0.566290 | 0.566271 | 1.303735 | 1.492290 | 0.800000 | 0.677697 |
| E3 | B + raw | 0.359384 | 0.396183 | 1.200000 | 1.427522 | 0.800000 | 0.776922 |
| E3 | B + entry | 0.359384 | 0.395481 | 0.983307 | 1.432638 | 0.800000 | 0.776922 |
| E3 | B + canonical | 0.346277 | 0.366726 | 0.800000 | 1.791148 | 0.800000 | 0.845105 |
| E4 | B + raw | 0.349396 | 0.386740 | 1.200000 | 1.863728 | 0.800000 | 0.577385 |
| E4 | B + entry | 0.368347 | 0.384740 | 1.200000 | 1.304986 | 0.800000 | 0.520465 |
| E4 | B + canonical | 0.368121 | 0.386555 | 0.800000 | 1.096484 | 0.800000 | 0.520465 |

First new applied commands and the two distinct jump baselines:

| Source | Method | Apply tick | v (m/s) | ω (rad/s) | Δv physical | Δω physical | Δv memory | Δω memory |
|---|---|---|---|---|---|---|---|---|
| E1 | B + raw | 193.000000 | 0.600000 | 1.062318 | -0.200000 | 0.945285 | -0.200000 | 0.445285 |
| E1 | B + entry | 193.000000 | 0.600000 | 1.117033 | -0.200000 | 1.000000 | -0.200000 | 0.500000 |
| E1 | B + canonical | 193.000000 | 0.800000 | 1.117033 | 0.000000 | 1.000000 | 0.000000 | 0.500000 |
| E2 | B + raw | 679.000000 | 0.579971 | -0.159685 | -0.000000 | -0.031917 | -0.200000 | 0.468083 |
| E2 | B + entry | 679.000000 | 0.579971 | -0.802231 | -0.000000 | -0.674464 | -0.200000 | -0.174464 |
| E2 | B + canonical | 679.000000 | 0.800000 | -0.623372 | 0.220029 | -0.495605 | 0.020029 | 0.004395 |
| E3 | B + raw | 193.000000 | 0.600000 | 0.461985 | -0.200000 | 0.174380 | -0.200000 | -0.314936 |
| E3 | B + entry | 193.000000 | 0.708346 | 0.585482 | -0.091654 | 0.297876 | -0.091654 | -0.191440 |
| E3 | B + canonical | 193.000000 | 0.800000 | 0.845105 | 0.000000 | 0.557499 | 0.000000 | 0.068183 |
| E4 | B + raw | 655.000000 | 0.600000 | -0.195674 | -0.200000 | -0.175209 | -0.200000 | 0.324791 |
| E4 | B + entry | 655.000000 | 0.600000 | -0.400857 | -0.200000 | -0.380391 | -0.200000 | 0.119609 |
| E4 | B + canonical | 655.000000 | 0.800000 | -0.429484 | 0.000000 | -0.409019 | 0.000000 | 0.090981 |

First .3 s held-command diagnostics (complete for every central rollout):

| Source | Method | Max abs v | Max abs ω | Linear TV | Angular TV |
|---|---|---|---|---|---|
| E1 | B + raw | 0.800000 | 1.180190 | 0.600000 | 1.327413 |
| E1 | B + entry | 0.800000 | 1.206341 | 0.600000 | 1.295660 |
| E1 | B + canonical | 0.800000 | 1.170081 | 0.000000 | 0.936655 |
| E2 | B + raw | 0.779971 | 0.659685 | 0.400000 | 0.968083 |
| E2 | B + entry | 0.779971 | 0.967275 | 0.400000 | 0.339507 |
| E2 | B + canonical | 0.800000 | 0.667974 | 0.020029 | 0.048998 |
| E3 | B + raw | 0.800000 | 0.776922 | 0.400000 | 0.518819 |
| E3 | B + entry | 0.800000 | 0.776922 | 0.183307 | 0.554251 |
| E3 | B + canonical | 0.800000 | 0.845105 | 0.000000 | 0.564683 |
| E4 | B + raw | 0.800000 | 0.577385 | 0.400000 | 0.706502 |
| E4 | B + entry | 0.800000 | 0.520465 | 0.400000 | 0.226935 |
| E4 | B + canonical | 0.800000 | 0.520465 | 0.000000 | 0.116313 |

Canonical linear TV is lower in all four sources. Angular TV is lower in E1/E2/E4
and higher in E3 by 0.358510 rad/s (25.024426%). The first canonical command avoids
the approximately .2 m/s braking seen in B_FULL_RAW in E1/E3/E4. E2 illustrates why
physical and memory jumps differ: canonical Δv is +.220029 m/s from physical
u_minus, but only +.020029 m/s from memory; Δω is −.495605 versus +.004395 rad/s.
Reference yaw deformation does not directly prescribe robot angular velocity.

### Independent pairwise contrasts

Each row is method minus baseline; negative AUC/time means lower/earlier.
Boundary insertion alone remains **N/A**, because a compatible no-B Native
execution is absent; no historical preflight is treated as execution evidence.

| Source | Contrast | ΔPos AUC .9 | ΔT_attach | ΔT_endpoint | ΔLinear TV | ΔAngular TV |
|---|---|---|---|---|---|---|
| E1 | B + entry − B + raw | -0.001648 | -0.083333 | 0.000000 | -0.000000 | -0.014453 |
| E1 | B + canonical − B + raw | 0.010174 | 0.233333 | -0.083333 | -0.800000 | -0.722413 |
| E1 | B + canonical − B + entry | 0.011821 | 0.316667 | -0.083333 | -0.800000 | -0.707960 |
| E2 | B + entry − B + raw | -0.010654 | 0.000000 | -0.016667 | -0.000000 | -1.266652 |
| E2 | B + canonical − B + raw | 0.001360 | 0.000000 | 0.016667 | -0.116294 | -1.764324 |
| E2 | B + canonical − B + entry | 0.012014 | 0.000000 | 0.033333 | -0.116294 | -0.497672 |
| E3 | B + entry − B + raw | -0.000946 | 0.000000 | -0.016667 | -0.216693 | 0.005117 |
| E3 | B + canonical − B + raw | -0.011878 | 0.000000 | -0.033333 | -0.400000 | 0.363626 |
| E3 | B + canonical − B + entry | -0.010932 | 0.000000 | -0.016667 | -0.183307 | 0.358510 |
| E4 | B + entry − B + raw | -0.002004 | 0.000000 | 0.000000 | -0.000000 | -0.558742 |
| E4 | B + canonical − B + raw | -0.000161 | 0.000000 | -0.016667 | -0.400000 | -0.767243 |
| E4 | B + canonical − B + entry | 0.001843 | 0.000000 | -0.016667 | -0.400000 | -0.208501 |

B_ENTRY has lower position AUC than B_FULL_RAW in all four sources; E1 attachment
is 5 ticks earlier, E2/E3 endpoint dwell is 1 tick earlier, and E1/E4 endpoint
dwell is unchanged. Canonical versus B_FULL_RAW lowers position AUC in E3/E4,
raises it in E1/E2, and lowers linear TV in all four sources.

Against B_ENTRY, canonical position AUC changes are +14.399428%, +20.691121%,
−19.739808%, +5.041491% (E1–E4). E3 satisfies the frozen early-tracking positive
and preserves endpoint dwell (1 tick earlier), while yaw AUC increases 73.680898%
and angular TV increases 25.024426%. E2 endpoint dwell is 2 ticks later with no
compensating primary gain, hence one regression. E1 attachment is 19 ticks later,
but endpoint dwell is 5 ticks earlier; E4 endpoint dwell is 1 tick earlier.

### Selector and original identities

All 12 central references have nearest identity B at the first legal submit.
The unchanged nearest+1 selector never includes B in H5 in these rollouts.
B_FULL_RAW first selects F0–F4; canonical first selects X0–X4, representing the
same original identities 0–4 and original arc starting at zero. B_ENTRY first
selects E* and its frozen downstream rows:

| Source | First submit tick | B_ENTRY H5 | Entry original row | Entry arc (m) | Entry arc fraction |
|---|---|---|---|---|---|
| E1 | 192.000000 | E*, F_1, F_2, F_3, F_4 | 0.997037 | 0.149594 | 0.110231 |
| E2 | 678.000000 | E*, F_3, F_4, F_5, F_6 | 2.608704 | 0.389789 | 0.287040 |
| E3 | 192.000000 | E*, F_2, F_3, F_4, F_5 | 1.082061 | 0.163102 | 0.120442 |
| E4 | 654.000000 | E*, F_2, F_3, F_4, F_5 | 1.995137 | 0.300059 | 0.225367 |

All three pairwise command comparisons first differ at submit/application ticks
192/193 (E1), 678/679 (E2), 192/193 (E3), 654/655 (E4). Canonical versus B_ENTRY
maximum matched execution XY separations are 0.080032, 0.028422, 0.030225 and
0.020142 m. This verifies changed controller behavior, without isolating a
selector-only causal mechanism. Changing geometry also changes the MPC problem.

Raw and entry downstream curves overlap because the saved original suffix is
unchanged; different markers/styles distinguish them in the geometry PNG.
The full-identity statement in that figure refers to B_FULL_RAW and B_CANONICAL;
B_ENTRY retains its trimmed suffix identities.

### Secondary own-installed-reference diagnostics

These are not the primary target. In particular E1 canonical own-reference AUC .9
is 0.018272 m·s, while its original-FRESH AUC is 0.093918 m·s. Good tracking of its
modified reference therefore does not imply earlier original-FRESH attachment.

| Source | Method | Own pos AUC .3 | Own pos AUC .9 | Own yaw AUC .3 | Own yaw AUC .9 | Own max XY .5 |
|---|---|---|---|---|---|---|
| E1 | B + raw | 0.021543 | 0.081291 | 0.046157 | 0.083942 | 0.105011 |
| E1 | B + entry | 0.021495 | 0.079643 | 0.044690 | 0.080321 | 0.102714 |
| E1 | B + canonical | 0.002874 | 0.018272 | 0.006217 | 0.026943 | 0.016772 |
| E2 | B + raw | 0.017432 | 0.066983 | 0.035116 | 0.067121 | 0.088069 |
| E2 | B + entry | 0.016896 | 0.056329 | 0.023970 | 0.049575 | 0.072667 |
| E2 | B + canonical | 0.000688 | 0.013001 | 0.003678 | 0.022750 | 0.003169 |
| E3 | B + raw | 0.017849 | 0.053667 | 0.008674 | 0.038170 | 0.070822 |
| E3 | B + entry | 0.017842 | 0.052749 | 0.007954 | 0.038349 | 0.070402 |
| E3 | B + canonical | 0.006689 | 0.032394 | 0.019283 | 0.029240 | 0.043395 |
| E4 | B + raw | 0.011601 | 0.037854 | 0.019676 | 0.038797 | 0.048958 |
| E4 | B + entry | 0.011447 | 0.035851 | 0.016946 | 0.035657 | 0.046385 |
| E4 | B + canonical | 0.001067 | 0.009780 | 0.000769 | 0.009686 | 0.006513 |

Historical C3/Hermite/V2 metrics are separately exported in
[`historical_context.csv`](../results/canonical_graph_common_b_execution_01/historical_context.csv).
They are authenticated context, not fresh executions or proxies for canonical.

## Research interpretation

The full-FRESH graph changes execution and has a bounded E3 position-tracking
benefit, alongside worse yaw tracking and angular command variation. It does not
meet the frozen requirement for additional benefit over staging in at least two
sources. B_ENTRY retains both dwells in 4/4 and lower early position AUC in 3/4
against canonical. The result supports retaining simple staging as the sufficient
baseline for these handoffs, while reporting canonical's command and endpoint
tradeoffs rather than collapsing them into a score.

Canonical remains a distinct, structurally valid planning formulation; this
experiment does **not establish consistent additional execution benefit beyond
simple B-aware staging**. Its single positive and single regression are both
visible. No factor/weight/correspondence/source change or follow-up experiment was
performed after observing this result.

## Direct answers to the requested questions

1. **Is B_CANONICAL safe through B→X0?** Yes, 4/4 complete installed references and
   rollouts pass; minimum reference/swept clearance .346277/.366726 m.
2. **Is B_FULL_RAW safely executable?** Yes, 4/4 references pass and all rollouts
   reach the cap safely with both dwells.
3. **How much does B insertion alone change raw FRESH behavior?** Not identifiable:
   compatible no-B raw execution is absent. No extra Native execution was added.
4. **What does B_ENTRY improve over B_FULL_RAW?** Lower early position AUC 4/4;
   E1 attachment 5 ticks earlier; E2/E3 endpoint dwell 1 tick earlier; no lost dwell.
5. **What does canonical improve over B_FULL_RAW?** Lower linear TV 4/4, angular TV
   3/4, position AUC 2/4, and earlier endpoint dwell 3/4. E1/E2 position AUC worsens;
   E3 angular/yaw metrics worsen; E2 endpoint is 1 tick later than raw.
6. **Additional benefit over B_ENTRY?** One frozen positive (E3), below the required
   two. E2 is a regression. Overall STAGING_REMAINS_SUFFICIENT.
7. **Does canonical reduce early original-FRESH AUC?** Versus B_ENTRY only E3:
   −.010932 m·s (−19.739808%). E1/E2/E4 increase.
8. **What remains when attachment is already at B?** E2–E4 all attach at zero;
   early tracking, yaw/command variation and endpoint dwell still differ. A zero
   attachment time is not evidence of identical downstream behavior.
9. **Is endpoint completion preserved?** All dwells remain observed. The one-tick
   preservation gate passes E1/E3/E4; E2 is 2 ticks later than B_ENTRY and fails it.
   These are original-FRESH endpoint dwells, not navigation goals.
10. **Do transition/TV improve?** Linear TV decreases 4/4 and angular TV decreases
    3/4 versus B_ENTRY. First-command jumps depend on physical versus memory
    baseline; E3 angular TV increases. No uniform command-smoothness claim.
11. **How are B/F0/X0/E* consumed?** B is first nearest, then nearest+1 selects
    F0 or X0 versus the fractional E*. B itself never enters H5. Original progress
    starts at zero for full-row references and at positive E* progress for staging.
12. **Did distinguishing V2 from canonical matter in execution?** Yes, canonical
    produces a distinct command/trajectory family and one new positive criterion;
    V2 is not a proxy. That distinction did not establish broad added benefit.
13. **Can graph optimization remain the central demonstrated contribution?** This
    evidence alone does not justify that claim over simple staging. The formulation
    can remain a research hypothesis, with its mixed execution outcomes disclosed;
    no new study is implemented here.

## Limitations / not demonstrated

Four previously studied saved development sources; no held-out/population or
unseen-source generalization. Offline logical timing is not asynchronous deployment
latency. Original-FRESH attachment and endpoint dwell are different from navigation
task success. No real-world or online VLA improvement, semantic intent preservation,
automatic correspondence performance or closed-loop deployment safety is shown.
No comparison isolates boundary insertion alone, and reference geometry changes do
not establish selector-only causality or directly prescribe omega(t).

## Validation, artifacts and protocol accounting

- 919 distinct tests pass: main suite 579, disjoint suite 338, plus two new focused
  cases after collection; final focused suite 60 passes. One historical
  EXP-01B/EXP-02B corpus-dependent test is skipped. Synthetic fixture solves are
  test activity, not scientific optimizer/MPC calls.
- Historical saved-only canonical and direct-transition validators pass; regression
  suite covers Local-SE2, bridge/staging/correspondence/transport/multisource/R00/R01
  and official controller/scheduler paths. Source/hash authority remains intact.
- New saved-only validation and check-only pass; CSV/JSON/figure numeric sidecars
  and PNG hashes agree. Compileall and git diff checks pass.
- Exactly three final PNGs were visually inspected. No additional final PNG or HTML.
- No post-freeze scientific code/config/reference changes, retries, scientific
  budget deviations or historical reruns. Pre-science precedence clarification
  (tradeoff/regression before staging) was explicit, tested and pushed before science.
- Unrelated user files remain outside both commits. The result commit is the commit
  adding this completed report; its full SHA is returned in the completion response.

Artifacts:

- [Primary metrics](../results/canonical_graph_common_b_execution_01/primary.csv)
- [Pairwise contrasts](../results/canonical_graph_common_b_execution_01/pairwise.csv)
- [Command metrics](../results/canonical_graph_common_b_execution_01/command_metrics.csv)
- [Common-B parity](../results/canonical_graph_common_b_execution_01/common_B_parity.csv)
- [Summary](../results/canonical_graph_common_b_execution_01/result_summary.json)
- [Exact commands](../results/canonical_graph_common_b_execution_01/commands.txt)
- [Reference geometry PNG](../results/canonical_graph_common_b_execution_01/figures/reference_geometry.png)
- [Execution comparison PNG](../results/canonical_graph_common_b_execution_01/figures/execution_comparison.png)
- [Transition/control PNG](../results/canonical_graph_common_b_execution_01/figures/transition_control_behavior.png)
