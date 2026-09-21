# GP-SE2-ATTACH-01: fixed correspondence and externally imposed duration

Question: given a fixed OLD/FRESH correspondence, can the existing constrained
SE(2) GP construct an executable transition from actual B into FRESH, and how
does an externally imposed duration affect feasibility and sustained attachment?
This is one recorded obstacle-sensitive mechanism experiment. Correspondence and
duration selection are not research contributions or optimization variables.

Starting and freshly fetched origin/main: `44e204499cb18855de2236780a4244903cd35d8a`.
JOIN-01's qualification failure, acquisition, code and outputs remain unchanged.
No new reveal, inference, scene export, selector or controller development occurs.
The two pre-existing Stage-0 config edits remain untouched and uncommitted.

## Source-only freeze

Run: `data/robotless_gp_se2_attach_01/primary_20260921T103000Z/`.
Config: `configs/gp_se2_attach_01.yaml`; authoritative numerical config is the
GP-SE2-01 primary `config_snapshot.yaml`. Hospital geometry is the original
primary export, authenticated against preserved REF-03 provenance and original
authoritative validations. The complete 881-event ledger is saved. Historical
method outcomes and their selected-case rankings are not selection inputs.

Eligibility uses original GP01 source integrity and exact client-inflight timing,
the original `VALID_HANDOFF_MOVING` label, recorded B/physical command/controller
memory verification, no required finite gate and a known route, valid B, and
independently swept-valid original suffix **and** prepared common reference.
Required edge clearance remains .05m with original geometry uncertainty 1e-7m;
obstacle sensitivity means original suffix minimum clearance <=.20m. No B-to-row
connector is inserted in this suffix metric. Mismatch requires existing
e_perp>=.10m OR reliable window direction>=20deg OR finite projected pose-yaw
disagreement>=20deg on nondegenerate simple geometry. Direction reliability
retains original .05m incoming/.02m window chord checks.

Before scanning, future sufficiency was operationalized as >=2 original suffix
rows, positive XY arc >1e-6m, and the original 30-row .1..3s preparation; the
whole original XY polyline must be simple with each segment >1e-6m to avoid
ambiguous projection. This does not assert physical 3s timestamps in LightNav.
No criterion is relaxed after scanning. Integrity errors block the experiment
instead of silently excluding corrupted cases.

730 original eligible →710 no-gate/known-route →49 obstacle-sensitive →10 mismatch
→9 unambiguous eligible cases. Lexical SHA256(`ATTACH01-v1:`+full case ID), then
case ID, selects **episode_021_repeat_01/handoff_003**. All nine eligible IDs and
all 881 reasons are preserved, including one geometry rejection. No historical
GP pilot exclusions are added; this is not held-out evaluation.

Selected original suffix clearance .0984265005393m; prepared clearance
.0984252061139m; existing e_perp .141201449561m. Native has10 rows, nearest j=0,
existing suffix starts1 and retains9 rows; common has30. Route requires no gate.
The fixed common float64 value hash is
`c67a128205a70383fbc9b89fdb21bd3d0d377287e571e52204f855500e6f3a7f`.

Important source limitation: this record is moving under the collector's
xy/yaw displacement >1e-6 criterion, but physical speed immediately before B
is ~1.80e-9m/s (omega~-5.79e-5rad/s). OLD/FRESH local raw arrays have the same
hash; their observation poses differ. This is not obstacle-reveal response or
evidence of substantially moving B at switch. The predetermined source rule
is retained without replacing the event. e_perp is the existing projection
metric, not a newly asserted signed lateral error.

## Fixed formulation and protocol

One `prepare_reference` construction per source event during scanning. The
selected cached common array is saved directly, not rebuilt per duration. Raw
FRESH remains observation-anchored. A separate validator reconstructs preparation
only to verify byte identity. B is support0; **common row k-1 corresponds to
support k**, time k*.1s. There are no intrinsic LightNav waypoint timestamps.

External T={.4,.6,.8,1.,1.2,1.4}s, all six conditions once. Original chart has
150 variables/31 supports/3s horizon. M4 cost is unchanged GP prior plus original
lambda times mean squared whitened FRESH Log residual over supports k>=kT.
The target is the SAME common[k-1], never a retimed/compressed/new suffix.
There is no FRESH preservation term before T. Original midpoint lateral
equalities, motion at u={0,.5,1}, obstacle/workspace/goal constraints remain.
No DIAG06 witness, DIAG08 reserve, extra lateral row, scaling or tolerance change.

At T,T+.1,T+.2,T+.3 append `.10²-||p-reference||²>=0` and two signed 15deg yaw
bounds. These 12 inequality rows give30 equalities/915 inequalities; original
M3 has30/903. Original physical full checker and separate **nominal** tube check
are both required; solver numeric allowance is not a tube acceptance relaxation.
Continuous-time safety is not claimed from samples.

Methods: unchanged official Native, unchanged common Adapter, generic current
M3 (original formulation with verified supplied derivatives), six fixed-T M4.
Rigid is omitted under the permitted optional scope. Each GP condition uses
the same saved original FRESH and fixed same-curvature deceleration seeds,
two starts per condition; M3 two plus M4 twelve =14 scheduled starts.
No historical latest iterate or handoff-specific seed. Each start once,
CPU float64 JAX/analytic geometry derivatives, original SLSQP200 iterations,
ftol1e-7,30s prepared budget, single-thread BLAS. All original Jacobian prefixes
remain literal. M4 reuses JOIN01's masked-cost/tube **AD algebra only**; it never
calls JoinView, join_candidates or post_join_reference.

All14 seed/formulation gates passed original primal and three-direction,
three-step FD validation before primary; both fine steps must meet existing
DIAG02 tolerances. Runtime wrap-cut guards remain. Provider compilation may be
reused across seeds/shapes; warmup/construction/solve/check costs are separate.
Preflight compilation is additional offline cost, not hidden in prepared budget.
`source.json.official_mpc` retains the historical pin/settings provenance record
from REF-03 (which contains REF-02 instrumented-wrapper metadata). ATTACH-01
does not activate that wrapper; its live official import provenance is saved
separately in `rollouts/provenance.json`. Both source hash and loaded constants
are checked. No source-progress selector is used by this experiment.

Retention: inspect initial/latest/actual solver-grid-feasible callbacks through
the unchanged DIAG04 instrumented harness. M3 preserves its historical
dense-first selected record and requires independent full validity. M4 selects
only among retained full-valid plus nominal-tube-valid records, minimum J then
initialization label/source label, separately **within each T**. There is no
cross-T method selection or deployment choice, and no RAW fallback.

Primary order: M3 I0/I1, then T=.4,.6,.8,1,1.2,1.4 each I0/I1; after planning,
Native→Adapter→M3→all available M4 in ascending external T. Independent official
MPC trackers, unchanged nearest/+1 selector and H5/.1s, original B/u_minus and
separate controller memory,30 solves/180 exact held-command steps/181 states,
3s horizon. World reference is inverted through original capture pose for path
installation, never reanchored to B. GP velocities are not feed-forward.
Invalid GP does not execute. No safety stop/pose snap is introduced.

OFFLINE SINGLE-HANDOFF COUNTERFACTUAL: optimizer/MPC wall time never advances
simulation time. Native/Adapter keep original spatial-reference diagnostic
rollout policy; their rollout success is separate from GP continuous plan checks.

## Metrics and static output

Reuse JOIN01's forward-only continuous XY polyline projection with shortest yaw
interpolation, against ORIGINAL FRESH. Closest remaining projection cannot move
backward. Earliest full following .30s saved sample window within .10m and15deg
is sustained attachment; a crossing is insufficient, failure is null. Execution
queries include original <=5ms and60Hz boundaries; GP queries use the original
<=1ms/.371 offset grid. These are sampled metrics, not continuous certificates.

Per condition report plan validity, solver termination and failures, planned
and executed attachment, relation to external T, original safety/motion/goal/dwell,
post-attachment error, pre-attachment AUC (N/A without join), full-window AUC,
command TV/accelerations and compute. Rejected latest-iterate attachment is an
explicit diagnostic, never a returned valid-plan attachment. Shortest **tested**
full-valid duration is descriptive within this exact set, not optimal T.

Figures: selected source; common-axis method small multiples; execution-to-original
FRESH distance/yaw; duration validity/attachment/clearance/solve cost; command TV.
No unavailable execution curve. Every PNG carries numeric/source/config/common
hash sidecar. Index and small review ZIP are static; GUI runtime zero.

## Exact commands

```bash
.venv/bin/python scripts/run_gp_se2_attach01.py prepare --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/run_gp_se2_attach01.py preflight --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
# Unit/relevant tests, diff review, then normal pre-primary commit AND push.
.venv/bin/python scripts/run_gp_se2_attach01.py freeze --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/run_gp_se2_attach01.py optimize --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python scripts/lightnav/gp_se2_attach01_rollout.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/run_gp_se2_attach01.py evaluate --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/plot_gp_se2_attach01.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/validate_gp_se2_attach01.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/plot_gp_se2_attach01.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z --package
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

No primary optimization or MPC comparison had run when this protocol was written.
Source scan and derivative gates are preparation, not performance outcomes.

## Actual result — 2026-09-21

Execution commit, pushed **before** any primary GP/MPC comparison:
`c540533bdf72b0a7ea31b19fd1c0819697a5596e`.
All 14 scheduled GP starts ran once; four independent rollouts made 120 primary
official-MPC solves. No new LightNav inference, reveal, GP retry, controller
modification or GUI runtime. Original physical acceptance and all frozen
numerical/input/environment/JOIN-01 hashes verify unchanged.

Operational: **GP_SE2_ATTACH_01_COMPLETED_WITH_LIMITATIONS**.
Interpretation: **positive mechanism evidence for one valid transition**, with
an **execution trade-off and no attachment-speed benefit**. T=1.4s yielded a
full-valid retained plan and a safe sustained actual attachment. This is the
shortest **tested** full-valid condition among {.4,.6,.8,1,1.2,1.4}; it is not an
optimal duration or a physically established lower feasible bound.

### Recorded timing and frames

World B=[16.3745194924,26.9007102736,3.1409555848] m,m,rad.
FRESH observation pose=[16.3829544519,26.9007051078,3.1409835201].
Physical u_minus=[1.7990134615e-9,-5.7911654836e-5] m/s,rad/s;
controller previous_control has the same values in this event, saved separately.
Raw OLD/FRESH hash is
`5231d31b148e90f1ec7efb3f5fda23a26d76b396e60d3b84fe767409eac3eec1`.

| Recorded event | Original clock / value |
|---|---|
| OLD observation | simulation 3.783333531s |
| FRESH observation | simulation 4.783333583s |
| FRESH request | host monotonic 424061.305154499s |
| FRESH receipt | host monotonic 424061.585152010s |
| Request→receipt | host duration .279997511s |
| Ready seen / path install | simulation 5.050000263s |
| Command application / B | simulation 5.183333604s |

B advanced .008434961m from FRESH observation during the recorded .400000021s
simulation interval. Original FRESH is projected from that observation, never B.
Unlike the earlier hard example, here B is **behind** FRESH's first row along the
forward direction: first-row separation .141201450m. The first infinite-line
perpendicular distance is only1.1177e-5m (projection alpha=-.942532 lies before
the segment). Thus the qualifying existing e_perp is a finite-polyline endpoint
gap, not a 14cm lateral detachment. The whole prepared suffix arc is .361077m.
This source cannot substantiate a difficult sideways spline or obstacle-reveal
reaction, even though it satisfies the declared obstacle-sensitive eligibility.

### Every new GP start

Status0=converged,8=positive directional derivative,9=iteration limit. No timeout
or retry. Full-valid in this table refers to the **latest** iterate; a retained
earlier callback can differ. Tube checks use the unchanged direct nominal .10m.

| Condition | Seed | SLSQP / iterations | Latest solver grid | Latest original full | Latest tube excess m | Prepared solve s |
|---|---|---|---|---|---:|---:|
| M3 generic | I0 | 0 /114 | pass | fail: speed | N/A | 2.479 |
| M3 generic | I1 | 0 /128 | pass | fail: speed | N/A | 2.962 |
| M4 T=.4 | I0 | 8 /159 | fail | fail | .152191 | 10.929 |
| M4 T=.4 | I1 | 9 /200 | fail | fail | .154598 | 10.557 |
| M4 T=.6 | I0 | 9 /200 | fail | fail | .075454 | 13.849 |
| M4 T=.6 | I1 | 9 /200 | fail | fail | .076883 | 10.856 |
| M4 T=.8 | I0 | 9 /200 | fail | fail | .00040478 | 15.380 |
| M4 T=.8 | I1 | 9 /200 | fail | fail | .00057224 | 17.394 |
| M4 T=1.0 | I0 | 0 /115 | pass | **pass** | 1.69338e-11 | 2.712 |
| M4 T=1.0 | I1 | 0 /140 | pass | **pass** | 5.55100e-12 | 3.951 |
| M4 T=1.2 | I0 | 0 /121 | pass | fail: speed | 5.85816e-12 | 2.697 |
| M4 T=1.2 | I1 | 0 /131 | pass | fail: speed | 6.79570e-11 | 3.385 |
| M4 T=1.4 | I0 | 0 /123 | pass | fail: speed | -.00678562 | 2.519 |
| M4 T=1.4 | I1 | 0 /143 | pass | fail: speed | -.00677870 | 3.732 |

The "grid" column is each condition's solver grid, including its tube for M4;
it is not a reinterpretation as the generic grid alone. At T=.4/.6/.8 there are
solver-grid, lateral, speed/acceleration and tube failures; exact per-start
families/magnitudes are in `review_v2/condition_details.csv`. Status8/9 is not an
infeasibility proof. At T=1.0 the only extra rejection is **nominal tube roundoff**:
maximum distance .10000000001693382/.10000000000555101m. No threshold was widened,
but this is not a physically significant failure or proof that T=1.0 is infeasible.

Generic latest min v_x=-.00022337/-.00022265m/s. T=1.2 latest min
v_x=-.00015987/-.00015746; T=1.4 latest=-.00008135/-.00008195, against the unchanged
lower-bound tolerance1e-5m/s. These original between-point rejections persist.

Returned M3 is **I1/callback_0117**, with full-grid max |v_y|9.83863e-6m/s and
min v_x=-2.48816e-6m/s, within original tolerances. Returned M4 T=1.4 is
**I0/callback_0114**, max |v_y|4.29541e-6, min v_x=-1.76849e-8, max |a_x|1.16406.
Both were actual saved solver iterates, not seeds or fabricated fallback paths.
The later invalid iterates remain saved. T=.4/.6/.8/1.0/1.2 have no accepted plan
under the frozen complete predicate and were not executed.

### Fixed duration and actual execution

| Method / external T | Full-valid GP candidate | Planned sustained attachment s | Executed sustained attachment s | Minimum executed edge clearance m | Motion / workspace / route / goal+dwell |
|---|---|---:|---:|---:|---|
| Native | spatial reference valid; GP check N/A | N/A | .155 | .09827082 | all pass |
| Adapter | spatial reference valid; GP check N/A | N/A | .155 | .09829660 | all pass |
| Generic M3 | yes | .203 | .165 | .09816152 | all pass |
| M4 .4 | no | N/A | N/A | N/A | not executed |
| M4 .6 | no | N/A | N/A | N/A | not executed |
| M4 .8 | no | N/A | N/A | N/A | not executed |
| M4 1.0 | no: nominal tube boundary | N/A | N/A | N/A | not executed |
| M4 1.2 | no | N/A | N/A | N/A | not executed |
| M4 1.4 | yes | .278 | .170 | .09838374 | all pass |

Rejected latest-iterate geometric attachment metrics are saved separately in each
start's `latest_planned_attachment.json`; they do not confer plan validity.
M4 execution attaches before its externally supplied T; .170s and1.4s measure
different things. The former projects onto any forward part of original FRESH;
the latter constrains assigned common rows at fixed planning support times.
No original FRESH row is assigned an intrinsic upstream timestamp.

| Executed method | Goal-entry s | Final position m | Final yaw rad | Linear TV m/s | Angular TV rad/s | Post-attachment mean distance m |
|---|---:|---:|---:|---:|---:|---:|
| Native | .770 | .00063379 | .000001896 | 1.60000 | .00469411 | .00346830 |
| Adapter | 1.000 | .00013423 | .000004100 | 1.66511 | .00346391 | .00348285 |
| M3 | .935 | .01451047 | .000614985 | 1.38473 | .00402656 | .00944408 |
| M4 1.4 | 1.170 | .01001069 | .001106849 | 1.02895 | .00479783 | .00874334 |

M4 attaches15ms later than Native/Adapter and5ms later than M3 on this saved
evaluation grid; it reaches goal .400/.170/.235s later. Linear TV decreases,
angular TV increases slightly, and post-attachment mean error is worse than
Native/Adapter. Clearance differences are sub-millimetre, not meaningful safety
superiority. All original motion checks pass, including first command from the
physical u_minus; maximum command-grid linear accelerations are about2m/s²
within the unchanged numerical allowance. Full metric/AUC/acceleration values
are retained in `outcomes.json` and `evaluation/*/`.

This supports constructibility and execution for one fixed correspondence, not
faster attachment or general navigation performance. No cross-method objective
numbers are treated as comparable performance scores; masking changes the cost.

### Cost, validation and review

All14 prepared solves total103.403s. By condition (both starts): M3=5.442,
T=.4=21.486,.6=24.705,.8=32.773,1.0=6.663,1.2=6.083,1.4=6.250s.
The .8 total exceeds30s because it sums two individually budgeted starts.
Construction .15456s shared provider plus .02134s views; warmup/JIT2.47883s;
post-solve original/full checks15.7020s; attachment diagnostics4.60061s.
Entire optimization phase including serialization/selection143.872s. These are
partly nested costs and must not be summed again with the phase total.
Separate source scan11.5986s, derivative preflight3.2236s, independent saved
validator29.0302s. Primary MPC solve sum .433468s (120 solves); simulation still
covers3s per independent method. No online latency benefit is inferred.

Independent validator passed, recomputing the entire881 source ledger,73 unique
retained vectors through original full checks, nominal tubes, deterministic
selection, all four exact command integrations and original execution metrics.
Scientific failure and artifact integrity are kept separate. Frozen source,
core, seed/common bytes, original geometry, official MPC and JOIN-01 remain intact.

Primary `index.html` includes all nine method/condition entries and six PNGs.
`review_v2/index.html` adds an executed-method comparison with complete marker,
footprint and clearance legends; `review_bundle_v2.zip` is the compact review.
Original primary plots and first reporting version remain preserved. V2 only
fixes reporting subplot spacing after visual review; it changes no numerical
record or outcome, and no experiment was rerun. Packaging used this separate
saved-record reporter instead of the optional original package subcommand:

```bash
.venv/bin/python scripts/report_gp_se2_attach01.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z --output review_v2 --bundle review_bundle_v2.zip
```

The largest remaining uncertainty is whether this fixed-correspondence method
is full-valid and useful with **appreciable lateral mismatch and nonzero boundary
motion** near obstacles. This selected event's along-path endpoint gap and nearly
stopped B do not answer that question. No new event or next experiment is run here.

Final verification: **2441 passed, 19 skipped** in168.22s, with local IPC socket
access. Skips concern removed historical corpora and one declared representation
fixture. Compileall and `git diff --check` pass. Repository-wide tests performed
one historical official-MPC audit in pre-primary testing and one in final testing;
these two non-primary solves are preserved separately from the120 primary solves.
No new source acquisition occurred. Final artifact validators report zero new
optimizer/MPC/VLA calls. The review ZIP contains only small saved tables, static
figures and provenance; generated data and ZIPs are excluded from Git.
