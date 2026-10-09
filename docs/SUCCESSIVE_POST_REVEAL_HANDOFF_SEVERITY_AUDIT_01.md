# SUCCESSIVE_POST_REVEAL_HANDOFF_SEVERITY_AUDIT_01

## Repository-confirmed source facts

Starting HEAD and freshly fetched origin/main:
`ec369a9733001d07c079960fba42acc08edb3e13`.
Source: `data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01`.
Source scientific freeze `ee330244d867fad3749c60b6a5f1630b1471ec2a`;
source result `e4f61465c13996cc8c5c70504a5735dabfa6e944`.

Authenticate the tracked integrity record against its exact source-result commit,
then the complete 401-file saved seal, 297 episode inputs, bundle manifest,
11 ready-record hashes and the exact tracked/bundled handoff index. The ten
post-reveal handoffs are C1_to_C2, C2_to_C3, C3_to_C4, C4_to_C5, C5_to_C6,
C6_to_C7, C7_to_C8, C8_to_C9, C9_to_C10 and C10_to_C11. There are eight EVOLVING
pairs and two STABLE controls: **C8_to_C9 and C10_to_C11**. The controls' raw local
bytes match within each pair; their observation anchors and world arrays differ.
Authentication, including exact B/P/observation states and original world=A*raw
installations, occurs before any new ten-handoff severity/ranking computation.

The prior C0→C1 actual Isaac comparison established exact RAW parity, identical
RAW/B_ENTRY execution, E*=F0 and B→E* gap .03723860800693844 m. Its incoming-to-FRESH
tangent mismatch was .000379661218582239 rad and method executions differed by
less than .000674 m. It is an unranked regression sanity check here, never one of
the ten screening rows or a candidate.

## Frozen screening protocol

This is saved-only screening of one DEVELOPMENT SOURCE. No method is constructed
for execution, optimized or run. Original FRESH and all raw outputs remain immutable.
World XY uses metres, yaw radians CCW, +Z up. Original FRESH world=A_observation*raw;
local +x forward/+y left. B is the actual first application state, P the immediately
preceding saved execution state. No B re-anchor and no waypoint timestamps/dt.

### Entry and geometry descriptors

Use the unchanged `SpatialCurve.correspondences(B)['C3_FORWARD_SE2']` and existing
`suffix_reference` vertex handling. Scales .10 m/15 degrees, forward XY lower
bound, earliest-arc tie tolerance1e-12 and wrapped yaw are preserved. Zero XY
segments fail closed; original raw rows are not removed to repair them. Store
full-precision entry arc/fraction, segment/alpha, E* world pose, nearest-row
diagnostic and remaining arc. Suffix rows after an interpolated E* are exact copies.

- Removed prefix arc/fraction are exactly C3 entry arc and normalized progress.
- B-to-entry gap is XY Euclidean distance; yaw gap is absolute shortest-angle error.
- OLD incoming direction uses actual P→B XY displacement. Displacement <=1e-12 m
  or nonfinite values produce a null descriptor and reason; no B-heading substitute.
- FRESH entry tangent is the original containing segment's start→end direction,
  including the earlier-segment convention at an exact vertex. Signed turn demand
  is wrap(fresh tangent−incoming direction); absolute demand is its magnitude.
- Suffix future turn uses E*^-1 times the retained original suffix: signed final
  lateral coordinate, maximum absolute lateral coordinate, signed/absolute wrapped
  E*→endpoint pose yaw change. Maximum tangent change means the maximum absolute
  wrapped difference between each retained noncollapsed XY segment direction and
  the original entry-segment tangent. It is not angular velocity or curvature.
- A→B translation/yaw and actual saved-state XY arc from exact observation state
  through B are descriptive. Host observation/request/receipt/ready/install/switch
  clocks remain distinct from simulation time and are retained in provenance.

### Existing revision fields and their naming caveat

Authenticate `evolution.json` byte-for-byte against the source-result commit;
reuse its original `evolution` and `extra_evolution` implementations/scales for
parity. Local separation is the symmetric maximum vertex-to-other-continuous-XY-
polyline distance, with each chunk in its own observation-local frame. Endpoint
lateral delta and wrapped net yaw delta retain their historical definitions.

The existing field **max_absolute_local_yaw_difference_deg** actually equals
`max(abs(wrapped fresh local yaw)) − max(abs(wrapped old local yaw))`, in degrees.
It can be negative. Preserve it under an `existing_` name, without silently taking
an absolute value or redefining it. Separately report the maximum absolute wrapped
**same-row** local yaw difference when row counts match; otherwise null. This added
pairwise descriptor is not used in ranking or candidate selection. It introduces
no new correspondence between differing row counts.

### Safety diagnostics

Reuse the exact source Hospital/cart/workspace checker and uncertainty/numerical
reserve: radius .20 m, source physical footprint convention, legacy extra .05 m
check separately. Check complete original FRESH, C3 suffix and hypothetical
straight B→E* connector separately. Store clearance, overlap and legacy pass for
each. The connector is a diagnostic, never an executed path, constructed method
or silent eligibility filter. Safety does not remove a row from the ranking.

### Five independent rankings and Pareto set

Only eight authenticated EVOLVING handoffs are ranked, descending full precision:

1. removed prefix fraction;
2. B→E* gap;
3. absolute OLD→FRESH tangent mismatch;
4. absolute wrapped E*→original endpoint yaw change;
5. historical local polyline separation.

For every ranking, ties use larger gap, larger absolute suffix net yaw, then earlier
handoff index. No rounding or approximate tie merging before selection. Report each
complete order and top3. Pareto dominance means >= on every one of these five axes
and > on at least one, with exact float comparisons. Equal vectors do not dominate
each other. No normalization, summation or weighted scalar difficulty score.

Complementary roles, from all EVOLVING handoffs, not restricted to the Pareto set:
TURN=max absolute incoming/fresh tangent mismatch; PROGRESS=max removed fraction
among remaining handoffs; REVISION=max local separation among remaining handoffs.
Apply the same tie breaks. Include each handoff once. Stable controls stay separate.
Undefined required ranking geometry fails closed with TECHNICAL_BLOCKED; do not
move E*, repair data or change a descriptor to obtain a candidate.

The selection function creates immutable records containing only handoff/index and
the five geometry values. Native metrics, methods and controller outcomes cannot
enter the selection interface. Selection occurs before native metrics are computed;
adding descriptive native values must reproduce the exact same selection.

## Pre-registered future metrics and descriptive native scope

Future primary metrics remain separate original-FRESH position/yaw AUC at .30 s.
Reuse the existing forward-only continuous projection and shortest-angle yaw, and
existing trapezoidal `auc` with interpolation of errors at exactly .30 s. Also
report the complete pre-next-install window where available. No combined score.

For heading response, delta=wrap(psi_F−theta_B), where psi_F is the unchanged C3
entry-segment tangent. Let q(t)=sign(delta)*wrap(theta(t)−theta_B)/abs(delta).
T_turn50 is the first elapsed saved-state time with q>=.5 whose immediately next
saved state also satisfies q>=.5, both inside the evaluation window. No interpolated
crossing time or dwell beyond this two-state requirement. Wrap uses C3's
atan2(sin,cos) form to avoid cancellation near zero. If abs(delta)<=**1e-12 rad**,
return null with N/A_NEAR_ZERO_TURN. If not achieved/confirmed before the horizon,
return null with CENSORED_NOT_REACHED, never the cap. Large delta omega is not
intrinsically bad; this metric measures actual heading response to geometry.

Native descriptors use only already-saved actual poses. Their window ends at the
state immediately before processing the next installation event; for the last
chunk it ends at the last saved episode state. No future controller/reference is
included. If available duration is less than .30 s, all native AUC/T_turn50 fields
are null with INCOMPLETE_0.30_NO_EXTRAPOLATION. Otherwise report primary .30 s and
full-window AUC/T_turn50 separately. T_turn50 confirmation uses actual saved times
<= the declared horizon; there is no waypoint time or fractional sample invented.
These native descriptors are context only and never selection inputs.

## Freeze, validation and outputs

Freeze config, source/index/code/selector/geometry hashes, schema, metric rules,
rankings/Pareto/roles and zero-call budget before generating the ten-row result.
Pre-freeze tests use synthetic descriptor/ranking/latency fixtures, source
hash/stable authentication and C0 entry parity only. They do not compute or rank
the ten real severity rows. Synthetic results are not scientific evidence.
Commit **Freeze post-reveal handoff severity audit**, normal push, record SHA,
then compute the full saved-only result. No post-outcome metric/ranking change.

Tracked namespace `results/successive_post_reveal_handoff_severity_audit_01/`:
freeze manifest, result_summary.json, severity.csv, rankings.json, call_accounting.json,
validation_summary.json, figure manifest and at most two final PNGs:
`figures/handoff_severity_overview.png`, `figures/hard_candidate_geometry.png`.
JSON/CSV retain full precision; display tables alone round. The frozen schema lists
all CSV fields. Larger per-handoff geometry/native projection details stay under
ignored `data/successive_post_reveal_handoff_severity_audit_01/primary_20261009`.

New scientific calls: Isaac0, MPC0, Graph0, Hermite0, B_ENTRY execution0, LightNav0,
RGB/model0, acquisition0, retries0. A runtime profile guard fails before known
solver/controller/integration/bridge/model/launch calls. Authentication may run
read-only Git commands. Only HANDOFF_SEVERITY_AUDIT_COMPLETE or TECHNICAL_BLOCKED
is allowed. This audit cannot identify a winning reconciliation method.

## Commands

```bash
git fetch origin main
git status --short
git branch --show-current
git rev-parse HEAD origin/main
RUN=data/successive_post_reveal_handoff_severity_audit_01/primary_20261009
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/severity-mpl
.venv/bin/python scripts/audit_successive_handoff_severity01.py --run "$RUN" --mode prepare
.venv/bin/python -m pytest -q tests/test_successive_handoff_severity01.py
.venv/bin/python -m pytest -q tests/test_successive*.py tests/test_long_continuous_obstacle_reveal_source01.py tests/test_continuous_obstacle*.py tests/test_spatial*.py tests/test_se2*.py tests/test_robotless*.py tests/test_online_mpc_adapter.py tests/test_osa03_common_b.py tests/test_osa03_relative_factor_ablation01.py tests/test_join_online02*.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
.venv/bin/python scripts/audit_successive_handoff_severity01.py --run "$RUN" --mode freeze
# Review exact new files + append-only work log, commit then normal push.
git commit -m 'Freeze post-reveal handoff severity audit'
git push origin main
.venv/bin/python scripts/audit_successive_handoff_severity01.py --run "$RUN" --mode run
.venv/bin/python scripts/audit_successive_handoff_severity01.py --run "$RUN" --mode validate --write
.venv/bin/python scripts/report_successive_handoff_severity01.py --run "$RUN"
.venv/bin/python scripts/audit_successive_handoff_severity01.py --run "$RUN" --mode validate
.venv/bin/python scripts/report_successive_handoff_severity01.py --run "$RUN" --check
```

## Limitations and future relative-factor hypothesis

One DEVELOPMENT SOURCE and ten dependent handoffs from the same episode.
Geometry severity is not demonstrated reconciliation need or method-performance
evidence. Stable raw local outputs can still produce different world geometry.
No population, real-world, navigation, VLA, Graph-superiority or general Graph-failure
claim is supported. The audit does not prove that the relative factor is harmful.

Future question only: **On a preselected hard-turn handoff, does removing or
weakening the FRESH relative-motion factor improve turn response / tracking AUC,
and what FRESH deformation does that introduce?**

This audit does not change that factor, lambda values or Graph formulation, and
performs no Graph-No-R implementation, solve, ablation or future four-method run.
