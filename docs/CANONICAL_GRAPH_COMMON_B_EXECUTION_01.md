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

Execution results will be appended after the pushed freeze and saved-only validation.

## Research interpretation

No new execution conclusion before science. Planning plausibility, B insertion,
progress trimming, graph deformation and controller smoothness are distinct claims.

## Limitations

Four previously studied saved sources; offline logical timing, not asynchronous
deployment. No held-out/population generalization, real-world or online VLA
improvement, semantic intent preservation, automatic correspondence performance,
navigation task success or closed-loop deployment safety is demonstrated.
