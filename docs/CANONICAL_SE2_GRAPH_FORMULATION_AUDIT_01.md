# CANONICAL_SE2_GRAPH_FORMULATION_AUDIT_01

## Pre-implementation formulation audit

Starting HEAD / fetched origin/main: `2b8cdf89d47eed656a83768800245d820dd82abd`. Planning only.

complete original FRESH (suffix starting at original row 0), to preserve historical Local-SE2 parity; C3 truncation only belongs to historical bridge baselines.

src/reconciliation/se2_lie.py does not exist; Exp/Log/right retraction are in se2.py and tested by test_se2_lie.py.

The following table was written before new implementation or scientific solves.

### RAW_FRESH

| Attribute | Repository-confirmed formulation |
|---|---|
| correspondence handling | none |
| OLD information | fixed A/B observation and application poses |
| FRESH information | original observation-anchored world poses |
| safety acceptance | existing full-polyline geometry gate |
| suffix frozen | True |
| editable variables | none |
| fixed variables | all original FRESH |
| boundary handling | none |
| relative motion preservation | exact |
| downstream intent preservation | exact original geometry; not semantic proof |
| editable poses | 0 |
| initialization | saved raw world |
| factor definitions | none |
| expected deformation class | identity |

### B_ENTRY

| Attribute | Repository-confirmed formulation |
|---|---|
| correspondence handling | none |
| OLD information | fixed B |
| FRESH information | frozen C3 entry and original suffix |
| safety acceptance | existing full-polyline geometry gate |
| suffix frozen | True |
| editable variables | none |
| fixed variables | B,E*,original suffix |
| boundary handling | derived B row |
| relative motion preservation | no factor; suffix exact |
| downstream intent preservation | exact raw suffix |
| editable poses | 0 |
| initialization | saved stack [B,E*,suffix] |
| factor definitions | none |
| expected deformation class | fixed staging |

### HERMITE

| Attribute | Repository-confirmed formulation |
|---|---|
| correspondence handling | none |
| OLD information | actual P->B direction |
| FRESH information | E* outgoing tangent and median suffix spacing |
| safety acceptance | existing full-polyline geometry gate |
| suffix frozen | True |
| editable variables | none (deterministic bridge interior) |
| fixed variables | B,E*,original suffix |
| boundary handling | fixed B/E* |
| relative motion preservation | no original FRESH factor |
| downstream intent preservation | exact raw suffix |
| editable poses | deterministic |
| initialization | chord-scaled cubic Hermite; equal spatial arc; shortest yaw |
| factor definitions | none |
| expected deformation class | derived bridge |

### V2_SINGLE_NODE_GRAPH

| Attribute | Repository-confirmed formulation |
|---|---|
| correspondence handling | none |
| OLD information | P->B incoming direction |
| FRESH information | E* outgoing direction, Hermite edge lengths, suffix spacing |
| safety acceptance | LM improving-proposal full-reference gate; saved convergence/collapse/self-intersection validity gates |
| suffix frozen | True |
| editable variables | X1 only in frozen M=2 variant |
| fixed variables | B,E*,original suffix |
| boundary handling | fixed B/E*; normalized vector incoming/outgoing residuals |
| relative motion preservation | bridge edge smoothness only; no full FRESH relative factor |
| downstream intent preservation | exact suffix |
| editable poses | 1 |
| initialization | saved Hermite |
| factor definitions | E_in(vector)+E_out(vector)+E_smooth+E_space; all lambda=1 |
| expected deformation class | single-node transition fitting |

### LOCAL_SE2

| Attribute | Repository-confirmed formulation |
|---|---|
| correspondence handling | none |
| OLD information | fixed A/B observation and application poses |
| FRESH information | original observation-anchored world poses |
| safety acceptance | initial and improving LM candidate complete X polyline; no B connector |
| suffix frozen | False |
| editable variables | all N FRESH X_j in SE(2) |
| fixed variables | A,B and original measurements F |
| boundary handling | distributed state-shift / transport target BA^-1 F |
| relative motion preservation | Log((F_j^-1 F_j+1)^-1 (X_j^-1 X_j+1)) |
| downstream intent preservation | soft original anchor, s^2 |
| editable poses | N |
| initialization | X=F |
| factor definitions | weighted mean L + edge mean R + weighted mean A; scales .10m,.10m,10deg; lambdas=1 |
| expected deformation class | full spatial non-rigid deformation allowed |

### SE2_GRAPH_EXISTING

| Attribute | Repository-confirmed formulation |
|---|---|
| correspondence handling | manual validated monotonic oracle pairs; Log(Z_ij^-1 (O_i^-1 X_j)) |
| OLD information | fixed boundary plus selected fixed old poses |
| FRESH information | original observation-anchored world poses |
| safety acceptance | solve_graph has no feasibility callback; caller-dependent checks are separate |
| suffix frozen | False |
| editable variables | all new X_j |
| fixed variables | old O_i, boundary B, raw new relative measurements, oracle correspondence measurements |
| boundary handling | Log(B^-1 X0) only |
| relative motion preservation | same relative edge expression, unaveraged weighted sum |
| downstream intent preservation | no original downstream anchor |
| editable poses | N |
| initialization | raw_new |
| factor definitions | boundary + correspondence + new_motion; global weights/scales; no arc-dependent L/A |
| expected deformation class | full graph with first-node boundary and oracle correspondences |

### CANONICAL_GRAPH_PROPOSED

| Attribute | Repository-confirmed formulation |
|---|---|
| correspondence handling | none |
| OLD information | fixed A/B observation and application poses |
| FRESH information | original observation-anchored world poses |
| safety acceptance | initial and improving LM candidate complete X polyline; no B connector |
| suffix frozen | False |
| editable variables | all N FRESH X_j in SE(2) |
| fixed variables | A,B and original measurements F |
| boundary handling | distributed state-shift / transport target BA^-1 F |
| relative motion preservation | Log((F_j^-1 F_j+1)^-1 (X_j^-1 X_j+1)) |
| downstream intent preservation | soft original anchor, s^2 |
| editable poses | N |
| initialization | X=F |
| factor definitions | weighted mean L + edge mean R + weighted mean A; scales .10m,.10m,10deg; lambdas=1 |
| expected deformation class | full spatial non-rigid deformation allowed |

## Degrees of freedom

| Method | Editable SE(2) nodes | Fixed suffix | Relative FRESH factor | Downstream anchor | OLD/FRESH correspondence |
|---|---:|---|---|---|---|
| B_ENTRY | 0 | yes | no | exact raw suffix | no |
| Hermite | deterministic | yes | no | exact raw suffix | no |
| V2 | 1 | yes | local bridge smoothness only | exact suffix | no |
| Canonical | N | no | yes | soft | inactive |

## Frozen formulation

A and B are fixed. The observation-to-application state-shift / transport factor moves editable early FRESH nodes X_j toward Ftilde_j = B A^-1 F_j, equivalently encouraging B^-1 X_j ≈ A^-1 F_j. Raw FRESH remains F_j=A F_local,j.

`r_L=Log(Ftilde^-1 X)`, `r_R=Log((F_j^-1 F_j+1)^-1 (X_j^-1 X_j+1))`, `r_A=Log(F^-1 X)`. Original XY arc defines s; w_L=(1-s)^2, w_A=s^2. Scales (.10 m,.10 m,10 deg). E_L and E_A are weighted means; E_R is the edge mean. E=E_L+E_R+E_A. All N nodes start at raw FRESH. No intrinsic waypoint time.

Existing Local-SE2 implements exactly this algebra and residual order L,R,A. The new core will delegate that algebra without altering its defaults. Existing se2_graph boundary Log(B^-1 X0), oracle correspondence and unaveraged motion terms define a different objective. This audit does not presume either is better.

OLD/FRESH correspondence: **IMPLEMENTED_EXISTING_BUT_NOT_ACTIVE_IN_THIS_AUDIT**. Excluded to isolate full editable-FRESH deformation. V2 is a single-node bridge fit, with a fixed original suffix; canonical permits the entire FRESH future to deform. Relative pose preservation is a geometric proxy, not proof of semantic intent.

## Claim boundary

No execution, controller benefit, online VLA benefit, automatic correspondence, navigation task success, real-world result, semantic-intent proof or general graph superiority is demonstrated. Spatial yaw changes are not actual robot angular velocity. Optimized-polyline safety is separate from actual B-to-reference transition safety; no such execution occurs.

## Frozen source and execution protocol

Run: `data/canonical_se2_graph_formulation_audit_01/primary_20261006`.
This run identifier is separate from actual processing UTC recorded in start logs.
Sources are the sealed OSA03_R00 and the four already selected DIRECT_TRANSITION
sources: E1=episode_009_repeat_00/handoff_001,
E2=episode_017_repeat_01/handoff_008, E3=episode_007_repeat_01/handoff_001,
E4=episode_016_repeat_00/handoff_008. This is source reuse for formulation diagnosis,
not a new held-out evaluation. No new source search is performed. Saved-only
historical validators may reconstruct recorded execution for authentication;
they produce no new controller commands, optimization or scientific rollout.

All baseline reference files and installed-world records are authenticated against
tracked result ledgers and source manifests. Downstream original rows remain
bit-identical to historical Native installations. Historical Hermite/V2 are read,
not regenerated or reoptimized. Saved A, B, observation, host readiness,
installation and application times are retained distinctly in per-source context.
No waypoint timestamp or dt is introduced. Exact frozen C3 parity is checked only
to authenticate baseline identities; C3 does not enter the canonical objective.
The original `se2.py`, `se2_graph.py`, Local-SE2, solver and historical methods
remain unchanged.

Prepare authenticates the full boundary-row/bridge/correspondence/transport/
multisource/R00/R01 chain, direct-transition selected records, and OSA03 Local-SE2
saved solution. It checks canonical/Local-SE2 algebra before science. No official
controller preflight is needed or invoked. Scientific execute checks the pushed
HEAD, frozen code/config/input hashes and exclusive start markers; then exactly
one raw-initialized solve per source, OSA03 then E1–E4. Retry=0. Guarded Python
entrypoints reject controller execution, model inference, RGB/Isaac acquisition,
source collection/search and subprocess launch. The guard counts the five solver
calls. Synthetic fixtures are tests, never experimental evidence.

Solver defaults are unchanged: max80 iterations, central FD1e-6, damping .001,
reject x10, accept x.3, maximum1e12, gradient/step1e-9 and cost1e-12. Right-local
update is `X <- X Exp(delta)`. Initial FRESH must be feasible; only improving and
feasible complete X polylines are accepted. Unsafe candidates increase damping.
On numerical termination preserve the last accepted feasible state and record the
failure; never retry. Geometry uses the unchanged direct Hospital/cart/workspace
checker, radius .20m, edge clearance .05m and existing numerical reserve (1e-7m).
No implicit B-to-X0 connector is included. Actual transition safety is unobserved.

## Frozen comparisons and interpretation

Same-identity raw/target comparisons use world XY Euclidean distances and shortest
yaw differences. Relative-edge translation diagnostics use the translation part
of the SE(2) log; these are labeled separately. Variable-row baseline comparisons
use both directed sets of reference vertices projected onto continuous XY line
segments, with exact squared-distance ties resolved by earliest segment/fraction.
Yaw is shortest-angle interpolation at each XY projection. The symmetric maximum
is over these vertex samples; it is not claimed as a continuous Hausdorff or
Frechet distance, and no artificial row correspondence enters the graph.
B_ENTRY has no meaningful N-row canonical-state embedding, so its L/R/A costs
are explicitly N/A. The raw, full-transport, canonical and OSA historical states
have all three costs. V2/Hermite identity comparisons include saved planning and
actual installed references; no selector or new controller query is used.

XY/yaw structural distinction tolerances are **1e-4m / 1e-4rad**. Algebra parity
uses 1e-12 absolute tolerance; frozen OSA solution reproduction uses 1e-10 absolute
tolerance, rtol0. Exact baseline/raw bytes are checked separately. Rigid fitting
reuses closed-form XY Procrustes, assessing yaw residual independently.

A source is structurally plausible only if converged and safe; distinguishable
from each B_ENTRY/Hermite/V2 baseline in symmetric vertex XY or yaw maximum;
non-rigid in XY or yaw rigid-fit RMS; first-node XY correction and reduction in
first-node distance to transported target both exceed1e-4m; endpoint correction
is <=.5 times first correction; relative translation RMS<=.10m/max<=.20m and
relative yaw RMS<=10deg/max<=20deg. Bounds are fixed diagnostic scale conventions,
not calibrated execution-quality thresholds. No monotonic correction constraint.
Historical Local-SE2 is algebraically the same formulation and serves parity,
not an independent competing family in this classification.

Precedence (evaluated across the five frozen informative sources):

1. **TECHNICAL_BLOCKED** for authentication, algebra/solution parity or infrastructure failure.
2. **CANONICAL_FORMULATION_INVALID** if at least2 sources are nonconverged or unsafe.
3. **CANONICAL_COLLAPSES_TO_RIGID_OR_EXISTING** if every source is either within rigid-fit tolerance or not distinct from all bridge baselines.
4. **CANONICAL_DISTINCT_AND_STRUCTURALLY_PLAUSIBLE** if at least2 sources meet every predicate above.
5. **CANONICAL_DISTINCT_BUT_NO_CLEAR_STRUCTURAL_GAIN** if all sources are valid, distinct and non-rigid but fewer than2 meet all structural predicates.
6. **MIXED_FORMULATION_EVIDENCE** otherwise.

Direction A applies only to structurally plausible: recommend a separate later
common-B execution comparison. Direction C applies to technical/invalid outcomes:
formulation/debugging first. Direction B applies otherwise: defer execution
because the structural gain remains unclear. None authorizes execution here.

## Frozen artifact and validation plan

Exactly four final PNGs: formulation_geometry.png, canonical_correction_profile.png,
relative_motion_nonrigidity.png and v2_hermite_geometry.png. Equal-axis geometry,
distinct line/marker styles and explicit overlap note; no HTML dependency.
Per-source raw/target/optimized/local arrays, solver and safety traces, costs,
correction profiles, relative edges, rigid fit and reference geometry stay under
ignored data. Compact tables/JSON, figure numeric sidecar and hashes are tracked.

Saved-only validation independently reconstructs the three cost means, retractions,
strict residual-dot cost ordering, acceptance/rejection and damping updates,
feasibility calls, final feasible state, original-A conversion, all diagnostics,
CSV and figure numeric/hash parity. It never invokes the optimizer. Focused tests
include synthetic end-to-end execution of the planning runner and saved validator
only, with synthetic labels; these are not scientific source solves.

## Pre-science verification notes

The focused suite passes45 tests, including raw/transport/relative residual zeros,
nonuniform N>=2 progress, immutable observation anchoring, Local-SE2 algebra parity,
right-local updates, unsafe-candidate rejection and last-feasible state retention,
fixed historical hashes/C3/baselines, guard/budget/retry controls, category
precedence, and a synthetic five-source saved-only reporting pipeline.

Pytest plugin autoload is disabled because the ambient ROS launch-testing plugin
requires unavailable `lark`. No package or environment is changed. Matplotlib
cache is placed under `/tmp`. An early synthetic guard test exposed matplotlib's
color accessor `get_rgb`; only that plotting-module accessor is exempted. Camera
RGB acquisition remains forbidden. These fixes occurred before scientific freeze.

The main regression batch passes519 tests in394.21s; the disjoint geometry/timing
batch passes338 with1 skip in7.48s (missing ignored EXP-01B/EXP-02B corpus).
The final focused45 include2 cases added after main-batch collection, giving
**859 distinct passing tests and1 skip**. `compileall` passes. Diff checks include
LF-normalized CSV output. Historical saved validators pass without a new solve.
