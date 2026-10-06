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

## Repository-confirmed results

Scientific freeze: `6344fddd848867fc9eb3480f19321fbc2ad418d9`, verified on origin/main before any solve.
Five canonical scientific calls; 5/5 converge at cost_tolerance. Retries: 0. MPC,
controller rollouts, LightNav, RGB, Isaac, GP and historical optimization calls
are all 0. All sources retain N=10 editable poses (30 scalar variables), versus
V2's one editable SE(2) pose (3 scalar variables). No raw or historical file changes.

Classification: **CANONICAL_DISTINCT_AND_STRUCTURALLY_PLAUSIBLE**. All 5 sources
meet the predeclared predicates. Follow-up **Direction A**. This is a planning
classification; it does not establish execution improvement.

OSA03 canonical and authenticated historical Local-SE2 arrays have maximum absolute
difference **0.0** across all coordinates. The saved factor costs are also identical.
This is an exact deterministic reconstruction within the required 1e-10 tolerance,
not a silently substituted historical solution. Canonical reuses existing Local-SE2
algebra; it is not a newly discovered objective.

Display tables round to 9 decimal places; CSV/JSON retain full float64 values.

### Corrections, rigidity, safety and cost

| Source | First XY m | Endpoint XY m | Max XY m | XY RMS m | Rigid-fit XY RMS m | Clearance m | Raw cost | Canonical cost |
|---|---|---|---|---|---|---|---|---|
| OSA03_R00 | 0.211144018 | 0.002186831 | 0.211144018 | 0.132792515 | 0.066834803 | 0.227809208 | 4.551116373 | 0.765263626 |
| E1 | 0.302568459 | 0.003009895 | 0.302568459 | 0.188229772 | 0.100078281 | 2.600741887 | 9.169501376 | 1.480294125 |
| E2 | 0.540300144 | 0.005585820 | 0.540300144 | 0.337874307 | 0.191101191 | 0.609146969 | 30.114623028 | 4.927380180 |
| E3 | 0.312082880 | 0.003400195 | 0.312082880 | 0.194103995 | 0.116411760 | 0.346276995 | 10.331807089 | 1.736736431 |
| E4 | 0.448048239 | 0.004401276 | 0.448048239 | 0.279947206 | 0.165886441 | 0.368121044 | 20.531982150 | 3.379145157 |

All optimized returned polylines pass unchanged .20m footprint / .05m edge-clearance
checks. Minimum across sources is 0.2278092081855581m. There are 25 accepted steps,
0 unsafe rejected proposals and 30 checked feasibility states. OSA03 has one
non-improving rejected proposal; it remains in the saved trace. Safety acceptance
was therefore active but not binding for these five solutions. Actual B-to-X0
transition safety is **NOT_EVALUATED_NO_EXECUTION**. No connector is fabricated.

### Relative-edge and yaw deformation

| Source | Relative XY-log RMS m | Relative XY-log max m | Relative yaw RMS rad | Relative yaw max rad | Raw yaw RMS rad | Raw yaw max rad | Rigid-fit yaw RMS rad |
|---|---|---|---|---|---|---|---|
| OSA03_R00 | 0.026158510 | 0.040260838 | 0.001274846 | 0.001732871 | 0.003609320 | 0.005958452 | 0.103853184 |
| E1 | 0.036135308 | 0.055486709 | 0.006300631 | 0.009886450 | 0.032724273 | 0.048279063 | 0.113931827 |
| E2 | 0.065373200 | 0.100010479 | 0.019768406 | 0.030544070 | 0.101170850 | 0.157466663 | 0.143869627 |
| E3 | 0.038830028 | 0.059526876 | 0.016520710 | 0.025737318 | 0.081374032 | 0.130399891 | 0.057818615 |
| E4 | 0.055676678 | 0.085937879 | 0.009590204 | 0.015088941 | 0.048098613 | 0.075459121 | 0.057723972 |

### Transport response and downstream recovery

| Source | First-to-target gap m | First-to-target gap reduction m | Endpoint / first | Target XY RMS / max m | Target yaw RMS / max rad |
|---|---|---|---|---|---|
| OSA03_R00 | 0.002189970 | 0.211143376 | 0.010357061 | 0.132907174 / 0.211147784 | 0.003609968 / 0.005959226 |
| E1 | 0.003124118 | 0.302563193 | 0.009947815 | 0.178934315 / 0.280652450 | 0.027960509 / 0.047330089 |
| E2 | 0.005652549 | 0.540279254 | 0.010338364 | 0.322285177 / 0.508481449 | 0.096642984 / 0.156742988 |
| E3 | 0.003239855 | 0.312060572 | 0.010895168 | 0.200116942 / 0.327140368 | 0.083044476 / 0.130402531 |
| E4 | 0.004679732 | 0.448042167 | 0.009823219 | 0.279144096 / 0.440817007 | 0.046953480 / 0.075147663 |

First-node corrections range 0.21114401806645536–0.5403001444287547m. Endpoints
remain 0.0021868314907293233–0.005585819723189501m from original FRESH. Endpoint
correction is 0.9823–1.0896% of first-node correction. Correction XY decreases
toward the endpoint in these saved solutions; monotonicity was not constrained.

### Factor decomposition

| Source | State | E_L | E_R | E_A | Total |
|---|---|---|---|---|---|
| OSA03_R00 | RAW | 4.551116373 | 0.000000000 | 0.000000000 | 4.551116373 |
| OSA03_R00 | FULL_TRANSPORT | 0.000000000 | 0.000000000 | 4.551129297 | 4.551129297 |
| OSA03_R00 | CANONICAL | 0.348589943 | 0.068480118 | 0.348193565 | 0.765263626 |
| E1 | RAW | 9.169501376 | 0.000000000 | 0.000000000 | 9.169501376 |
| E1 | FULL_TRANSPORT | 0.000000000 | 0.000000000 | 8.387672357 | 8.387672357 |
| E1 | CANONICAL | 0.661801499 | 0.131879257 | 0.686613369 | 1.480294125 |
| E2 | RAW | 30.114623028 | 0.000000000 | 0.000000000 | 30.114623028 |
| E2 | FULL_TRANSPORT | 0.000000000 | 0.000000000 | 27.974904753 | 27.974904753 |
| E2 | CANONICAL | 2.207933277 | 0.440194403 | 2.279252499 | 4.927380180 |
| E3 | RAW | 10.331807089 | 0.000000000 | 0.000000000 | 10.331807089 |
| E3 | FULL_TRANSPORT | 0.000000000 | 0.000000000 | 10.900585200 | 10.900585200 |
| E3 | CANONICAL | 0.798131950 | 0.159736994 | 0.778867486 | 1.736736431 |
| E4 | RAW | 20.531982150 | 0.000000000 | 0.000000000 | 20.531982150 |
| E4 | FULL_TRANSPORT | 0.000000000 | 0.000000000 | 20.110825014 | 20.110825014 |
| E4 | CANONICAL | 1.522557525 | 0.313008515 | 1.543579117 | 3.379145157 |

Raw E_R/E_A and full-transport E_L/E_R are numerical zero (the full precision
JSON retains roundoff). OSA03 historical cost equals canonical exactly. B_ENTRY
canonical costs remain N/A: its derived boundary/entry rows and truncated suffix
do not define an N-row state of this objective. No artificial embedding is used.

### Difference from fixed-suffix baselines

| Source | Canonical → V2 XY RMS / max m | V2 → canonical XY RMS / max m | Symmetric vertex yaw max rad |
|---|---|---|---|
| OSA03_R00 | 0.057423211 / 0.102803236 | 0.063250157 / 0.105543103 | 0.317785491 |
| E1 | 0.065581612 / 0.104521350 | 0.102850447 / 0.158577511 | 0.197498378 |
| E2 | 0.054442240 / 0.086672778 | 0.097302998 / 0.156494540 | 0.136164162 |
| E3 | 0.018786511 / 0.041110786 | 0.079340305 / 0.151928666 | 0.260637422 |
| E4 | 0.026122958 / 0.045522460 | 0.091914627 / 0.150247444 | 0.091920722 |

Symmetric vertex XY maxima versus B_ENTRY, Hermite and V2 happen to be identical
within each source:0.10554310253172167,0.15857751134111892,0.1564945404947123,
0.15192866628664847,0.1502474442610702m respectively. Directed RMS values differ;
all are retained in comparison.csv. These maxima include start-region coverage
differences: the canonical first node is editable and is not fixed at B. This
comparison is not purely a lateral-deformation metric. The raw-to-canonical rigid
fit independently confirms non-rigidity. No row-index matching was forced.

### Saved V2 versus Hermite

| Source | X1 XY delta m | X1 yaw delta rad | Bridge arc delta m | First / second direction delta rad | First / second length delta m | Installed reference max XY delta m |
|---|---|---|---|---|---|---|
| OSA03_R00 | 0.006136132 | -0.000146260 | -0.000032084 | -0.118654721 / 0.109428481 | -0.000861740 / 0.000829655 | 0.006136132 |
| E1 | 0.001411666 | -0.000013983 | 0.000031566 | -0.055334682 / 0.053405726 | -0.000121696 / 0.000153262 | 0.001411666 |
| E2 | 0.000855217 | 0.000007999 | 0.000008369 | 0.036055443 / -0.035012653 | -0.000101457 / 0.000109826 | 0.000855217 |
| E3 | 0.000825712 | -0.000009689 | 0.000005927 | -0.027186994 / 0.026551994 | -0.000104313 / 0.000110241 | 0.000825712 |
| E4 | 0.000378975 | 0.000002247 | 0.000003661 | 0.024625257 / -0.024163877 | -0.000040433 / 0.000044095 | 0.000378975 |

In E1–E4, only X1 changes geometrically; the downstream suffix remains exact.
X1 shifts are 1.411665672456437,0.8552167675767318,0.8257115782797477 and
0.37897499797001345mm. This establishes how similar the references were. It is
consistent with the earlier similar execution, but is not a causal explanation
of controller behavior: this audit makes no new selection/command query.

## Formulation interpretation

V2 and canonical solve different problems. V2 fits a single spatial transition
pose with fixed B/E*/downstream geometry, using incoming/outgoing vectors plus
bridge smoothness and spacing. Canonical edits every original FRESH pose using
distributed transport targets, measured original relative edges and soft original
anchors. Canonical does not use P→B tangent or enforce a B boundary row.

In all five sources, the canonical reference is distinguishable from the bridge
family and cannot be represented by a single fitted left rigid transform within
the frozen tolerance. Early nodes approach transported targets; late nodes return
close to original FRESH. This is geometric recovery under the chosen objective.
It does not prove that relative distortion is optimal or that raw FRESH semantics
are preserved. Existing Local-SE2 already supplies this solution family.

The current V2 results therefore do not settle the research question for a full
editable-FRESH graph. Conversely, a lower canonical objective cannot overturn
the historical staging execution results: those evaluate a different question.
The bounds used for structural plausibility are predeclared engineering scales,
not empirically validated thresholds for controller performance.

## Not demonstrated

- No execution benefit or controller benefit; no new MPC solve or rollout.
- No online VLA benefit, automatic correspondence or new acquisition.
- No navigation task success or real-world result.
- No semantic-intent preservation proof or general graph superiority.
- No necessity or optimality of E_R, and no superiority to staging.
- No actual B-to-optimized-reference transition safety.
- No held-out or population result; all five sources are reused development records.
- No conversion of spatial yaw deformation into actual robot omega(t).

## Direct answers to the twelve research questions

1. **Is V2 the same graph as the canonical research question?** No. Its editable set and factors differ; V2 has 1 editable pose, canonical has 10 here (generic N).
2. **Is V2 closer to single-node transition fitting?** Yes: fixed B/E*/suffix, editable X1 and local bridge geometry factors.
3. **How does existing se2_graph differ?** It uses first-node Log(B^-1 X0), optional validated oracle correspondences and unaveraged new-motion residual sums. It has neither distributed BA^-1 transport targets nor the soft downstream original anchor.
4. **Is existing Local-SE2 algebraically the same?** Yes. Transport/progress/raw residuals/normalized blocks/cost/order/frame conversion match; OSA03 optimized arrays match exactly (0.0 maximum difference).
5. **Does canonical produce a different reference from V2/Hermite?** Yes on 5/5 sources; symmetric vertex XY differences span0.105543103–0.158577511m. The near-identical bridge suffixes remain visible with separate markers/line styles.
6. **Rigid transport or non-rigid deformation?** Non-rigid on 5/5; best-fit XY RMS residual spans 0.066834803–0.191101191m, above 1e-4m.
7. **How much do early nodes reflect observation-to-switch transport?** First XY corrections are 0.211144018–0.540300144m; first-node gaps to their transported targets are in the table above. A/B are fixed.
8. **How far does the endpoint recover?** Endpoint corrections are2.186831491–5.585819723mm, roughly1% of first-node correction; the endpoint is softly anchored, not fixed.
9. **How much relative-motion distortion occurs?** Translation-log RMS is 0.026158510–0.065373200m; yaw RMS is 0.001274846–0.019768406rad. Maxima and individual edge logs are preserved. Distortion is nonzero but within the predeclared bounds.
10. **Is safety maintained?** Every complete optimized polyline passes; minimum footprint-edge clearance is 0.2278092081855581m. Actual transition/execution safety is untested.
11. **Is there a scientific reason for a later canonical MPC comparison?** Yes under the frozen rule: all 5 meet structural plausibility, so Direction A recommends a separate bounded common-B execution comparison. It would test a family V2 does not represent; planning does not predict a win.
12. **Are staging results alone enough to dismiss canonical execution?** No for this formulation question. They remain evidence for fixed-suffix staging, while canonical permits a different deformation. This is not evidence that canonical is necessary; execution value remains unknown.

## Follow-up decision

**Direction A — execute canonical graph later.** 다음 별도 experiment에서 common-B
MPC execution comparison을 권장한다. This recommendation follows the frozen
classification only. No execution experiment, extra factor, correspondence,
controller integration or next-stage code is implemented in this task.

## Reproduction, validation and visual review

- Starting HEAD: `2b8cdf89d47eed656a83768800245d820dd82abd`. Scientific freeze: `6344fddd848867fc9eb3480f19321fbc2ad418d9`.
- Authenticated 780 code/config/artifact files, 57 prepared input files, 339 selected source/history file hashes and the saved historical validation chain.
- New canonical calls 5, convergence 5/5, retries 0; all other scientific call counts 0. Synthetic test solves are fixtures only.
- 859 distinct tests pass, 1 historical-corpus skip; focused 45 pass. Compileall and diff/staged-diff checks pass.
- Saved-only validation passes all cost/retraction/acceptance/damping/safety/frame/source checks and exact OSA result parity. CSV and figure numeric/hash parity pass.
- Four PNGs visually inspected: axes/units/legends readable, equal XY geometry, distinct markers for overlapping curves, correction and diagnostic panels match numeric sidecar. No plot patch or result-driven code change.
- Exact commands: [commands.txt](../results/canonical_se2_graph_formulation_audit_01/commands.txt).
- No scientific protocol deviation. The full original future is the explicitly frozen suffix starting at row 0; C3 suffix is only the bridge baseline. Missing requested se2_lie.py was resolved to the actual se2.py implementation.
- Unrelated Stage0 edits and GPU-memory script remain excluded; external code, environments and historical results remain unchanged.

### PNG artifacts

- [formulation_geometry.png](../results/canonical_se2_graph_formulation_audit_01/figures/formulation_geometry.png)
- [canonical_correction_profile.png](../results/canonical_se2_graph_formulation_audit_01/figures/canonical_correction_profile.png)
- [relative_motion_nonrigidity.png](../results/canonical_se2_graph_formulation_audit_01/figures/relative_motion_nonrigidity.png)
- [v2_hermite_geometry.png](../results/canonical_se2_graph_formulation_audit_01/figures/v2_hermite_geometry.png)
