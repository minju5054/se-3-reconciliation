# OSA03_RELATIVE_FACTOR_REPLICATION_01

## Frozen protocol

**Timing-controlled offline replication** on sealed OSA03 **REPEAT_01 only**.
This is a **paired same-scenario replication**, also described as a technical
replication across OSA03 repetitions. It is not an online latency benchmark.
Starting fetched HEAD/origin/main: `0cc11dfb4d2048c758fc735509c2e8297124283b`.
The prior reports, implementation, tracked results and repository instructions were
inspected before edits. The two unrelated Stage 0 config edits remain untouched.

Run: `data/osa03_relative_factor_replication_01/primary_20260930T010000Z/`.
This fixed directory identifier is not an execution timestamp; records retain
actual UTC, host monotonic, and logical simulation clocks separately. All large
arrays, rollouts and figures stay under ignored data. Raw source files stay sealed.
Small provenance ledgers and numeric results are tracked under the same namespace.

### Source and scope

Only `data/obstacle_source_acquisition_03/primary_20260923T085200Z/` / REPEAT_01
supplies the scientific state and trajectory. The tracked acquisition summary pins
its saved validation digest; that validation pins every R01 episode file. The
sealed R01 bundle ledger, episode/raw/world parity, complete source qualification,
Hospital/cart geometry and official MPC source are authenticated before execution.
R00 artifacts supply the fixed schedule and previously measured contrasts only.
No R00 optimization or rollout is repeated. Its existing saved-only validator must
pass without modification, and all its result bytes must remain unchanged.

Independent acquisition sessions have different triggering RGB, observation poses,
world FRESH and controller states. **R00 and R01 have identical raw local FRESH
arrays**, and identical raw local OLD arrays. Therefore this experiment cannot
establish trajectory diversity, independent policy-output replication, population
replication or generalization across Navigation VLA trajectories.

Frames: world XY metres, +Z up, yaw radians CCW; observation-local x forward/y left.
`F_j = A F_local,j` is the original observation-anchored world FRESH. Raw arrays are
immutable and never re-anchored at B. Derived local references use `A^-1 X_j`.

### Formulation and solver

A and B are fixed. The **observation-to-application state-shift / transport factor**
moves editable early FRESH nodes X_j toward the transported target
`Ftilde_j = B A^-1 F_j`, equivalently encouraging `B^-1 X_j ≈ A^-1 F_j`.
Historical L keys are retained and mean S in this report.

```
r_S,j = Log(Ftilde_j^-1 X_j)
r_R,j = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A,j = Log(F_j^-1 X_j)
s_j = cumulative original-FRESH XY arc / total original-FRESH arc
w_S = (1-s)^2; w_A = s^2
normalization = diag(.10 m, .10 m, 10 degrees)
```

Weighted means/edge means remain exactly as implemented. Fixed rollout order:

| Condition | Reference | New R01 planning calls |
|---|---|---:|
| M0_NATIVE | Original FRESH | 0 |
| M1_TAPER | Existing `Exp((1-s) Log(B A^-1)) F` | 0 |
| FULL_LOCAL_SE2 | E_S + E_R + E_A | 1 |
| NO_RELATIVE | E_S + E_A | 1 |

Both planning conditions initialize exactly at original R01 FRESH, with the same
LocalSE2Problem, scales, weights, right-local SE(2) LM and safety callback. No R00
optimized reference enters either initialization. Default Full behavior remains
unchanged. No-relative removes R from the optimized vector and cost; its final old
R residual is **diagnostic relative-edge distortion, not optimized cost**.

Solver: maximum 80 iterations; central finite difference 1e-6; initial damping
1e-3; rejection ×10 / acceptance ×.3; maximum damping 1e12; gradient and step
tolerances 1e-9; cost tolerance 1e-12. Update `X_j <- X_j Exp(delta_j)`.
One call per condition, with exclusive markers; no restart, selected retry,
alternate initialization, weight/scale/lambda search or result-driven changes.
Nonconvergence/error is recorded and the condition is skipped, without repair.

Complete candidate polylines use the existing direct Hospital + cart checker:
circular radius .20 m, required footprint-edge clearance .05 m, unchanged workspace
and numerical/uncertainty conventions. Initial state must be feasible; a candidate
is accepted only when objective decreases and the complete candidate is feasible.
No B-to-first-waypoint connector, clipping, projection, cropping or obstacle cost.
An independently checked unsafe complete reference is recorded and not executed.

### Schedule and controller semantics

The exact full 180-step R00 logical schedule is authenticated through its tracked
result digest and original common-B M0 events; no schedule is synthesized from
rounded prose or nominal frequency. R01 compatibility checks B tick/time, dt,
next legal submit, original grid, version semantics, generation validity, memory,
settings and release-before-next-submit ordering. Any incompatibility blocks
scientific execution as `TECHNICAL_BLOCKED`.

Authenticated preparation facts:

```
A = [19.203260368199164, 24.346668368545373, -1.5689754090345875]
B = [19.20364874270168, 24.133335375688848, -1.5689760469373495]
u_minus = [0.8, -1.2790285101476235e-06]
u_B_plus = u_mem_B = [0.8, 0.4999987283797167]
B tick = 92; B simulation time = 1.5666667483747005 s
dt = 0.01666666753590107 s (original float32 1/60)
R01 generation = 6; fresh reference version = 1
first saved FRESH result = REPEAT_01_solve_000005
```

R00's generation is 3. Generation counters need not numerically match across
independent acquisitions; R01's exact generation 6 is restored for all four methods,
and each result must match it. The official zero-numerical-solve installation and
restoration preflight passes with generation 6. No R00 B/memory is substituted.
The initial development compatibility check unnecessarily required generation
counter equality across repetitions; this was corrected before freeze and before
any scientific call. The source state and controller semantics were unchanged.

Primary submits `[96,102,108,114,120,126,132,138,144]` and releases
`[99,103,109,115,121,127,133,139,145]` are loaded from authenticated files. The
application at tick92 is the shared saved R01 command; there is no new solve at B
and no replay of the future saved R01 result at tick96. Full schedule is in the
freeze ledger. All methods use identical common state, command, memory and clock.

The unchanged R00 execution function is reused directly. It installs each reference
through the official observation-A transform before the logical clock, restores
R01 generation/command/memory/diagnostic fields, and uses the unchanged official
worker, H=5 nearest/+1 selector, MPC objective, limits and command computation.
Its inherited per-rollout `label` names the causal-reference-comparison wrapper;
this experiment's protocol, summary and plots label the R01 study as an offline
replication. No historical implementation file is edited.

At each submit tick the simulation waits for the official solve/poll while state
and simulation clock remain fixed. The result is withheld until its predetermined
logical tick. No future state enters a solve. The previous command remains held.
Official poll updates memory before the next solve; every scheduled release
precedes the next submit, preserving the existing ordering. Stale generation,
controller failure, missed release or timeout fails closed. Wall time does not
choose the application tick; this is not real asynchronous deployment timing.

The primary causal gate requires identical attempted/accepted submits, successful
applications, all 54 integration intervals (~.9 s), initial B/held command and
controller-memory provenance. Report full 180-step parity as an additional check.
A schedule-control failure prevents reference-causal attribution and is
`TECHNICAL_BLOCKED`. Missing/censored conditions remain explicit, with no invented
attachment or continuation.

Execution keeps the existing abort-only guard: new command .1 s preview and held
command next-dt check. An unsafe proposal is never applied; retain the valid prefix.
No steering, repair or interpolated commands. Cap 180 intervals, no later FRESH.

### Evaluation and diagnostics

Primary evaluation targets **ORIGINAL R01 FRESH**, with the unchanged continuous
forward-only projection and shortest-angle yaw. Attachment requires <=.10 m and
<=15 degrees through the complete following .30 s sampled dwell. Null attachment
stays null. Report initial errors, max/growth first .5 s, position/yaw AUC .3/.9 s,
attachment time/fractional row/fractional original arc/remaining arc, swept clearance,
endpoint error, linear/angular command TV, max abs omega and termination.
Own-reference AUC .3/.9 and max .5 s error are separate secondary diagnostics.
No optimized-objective total is treated as an execution-performance score.

Planning diagnostics retain node/endpoint/per-node shifts, arc, segment lengths,
self-intersection, original-node-frame lateral correction, B-frame lateral extent,
yaw, relative-edge translation/yaw RMS/max, rigid fit, complete clearance, B-to-row0
and continuous projection distance/yaw, first >=.02 m chord versus B heading.
Both solves report costs, iterations, accepted/rejected/unsafe steps and termination.
Spatial yaw deformation is distinct from actual omega(t), chosen by official MPC.

Full and No-relative selector diagnostics record every solve's input pose, nearest
row, selected H=5 indices, produced command and logical application tick. They are
read from saved events after execution and cannot influence the controller. Report
the first differing selected-row tick, or explicitly report identical sequences.

Paired table uses signed **No-relative minus Full** for 13 metrics: max position
.5; position AUC .3/.9; yaw AUC .3/.9; attachment; remaining original arc; swept
clearance; endpoint; linear/angular TV; relative-edge translation RMS/max. Report
both repetitions and sign consistency; nulls remain null. No scalar score.

Predeclared categories, without outcome-driven thresholds:

- `REPLICATED_WITHIN_PAIRED_OSA03`: earlier attachment, nonpositive deltas for all
  three early position metrics, all references/executions safe through their caps,
  and higher relative-edge translation RMS/max without R.
- `PARTIAL_REPLICATION`: earlier attachment or consistent early position benefit,
  with the other required directions mixed or unavailable.
- `NOT_REPLICATED`: the main benefit reverses/disappears; also record scientifically
  observed planning/reference failure without relabelling it a technical failure.
- `TECHNICAL_BLOCKED`: source, schedule, controller or validation failure only;
  no reference-causal interpretation.

All exact signed metrics and safety outcomes remain visible regardless of category.

### Tests, freeze and outputs

Synthetic tests are mechanism checks, never experimental evidence. Required source,
frame, factor/default, solver/init/safety, immutable raw, schedule/release/stall,
memory/generation, abort, primary/secondary and selector checks precede execution.
The old R00 validator and Local-SE2/common-B regression tests must pass unchanged.
Compileall and diff review precede commit and normal push. The scientific runner
verifies remote HEAD and committed code bytes, then creates exclusive markers.
Exactly two planning calls and at most four rollouts are authorized after push.

Ten saved-only figures: R01 references/world execution (equal axes; cart + .25 m
center exclusion boundary, A/B, attachment/minimum-clearance markers), original
position/yaw/progress, clearance, commands, logical timeline, deformation and
paired signed-effect table. Distinct styles/markers and numerical maximum
separations describe overlapping curves. Each has a numeric and hash sidecar.
The local HTML review links all figures and is derived solely from saved records.

No new LightNav, RGB, Isaac episode, source search, external source/environment
change, MPC modification, new factor, time parameterization, GP, correspondence,
new margin, tuning, repeated online chunks or broader research stage.

### Exact commands

```bash
git fetch origin main
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode prepare --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode preflight --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLCONFIGDIR=/tmp/osa03-replication-mpl .venv/bin/python -m pytest -q tests/test_osa03_relative_factor_replication01.py
# Full regression command and counts are recorded in WORK_LOG.md before freeze.
.venv/bin/python -m compileall -q src scripts tests
git diff --check
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode freeze --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
# Review staged diff, focused commit and normal push BEFORE either solve or rollout.
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode execute --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
.venv/bin/python scripts/validate_osa03_relative_factor_replication01.py --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
MPLCONFIGDIR=/tmp/osa03-replication-mpl .venv/bin/python scripts/report_osa03_relative_factor_replication01.py --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
MPLCONFIGDIR=/tmp/osa03-replication-mpl .venv/bin/python scripts/report_osa03_relative_factor_replication01.py --validate-only --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
```

Results and the nine explicit research answers will be appended after the single
frozen execution; none are inferred from synthetic fixtures or preparation.

## Repository-confirmed facts

Scientific freeze **`f334a44c878163fb89ae673c678432f9f84c4e81`** was normally pushed
before either R01 planning call or any rollout. Classification:
**`REPLICATED_WITHIN_PAIRED_OSA03`**. Saved-only validation passed, including the
unchanged historical R00 validator. All ten figure/hash/CSV sidecars passed.
Two optimizations (one Full, one No-relative), four rollouts and 120 official MPC
solves (30 per method) completed. New LightNav, RGB, Isaac and source calls: **0**.
R00 reruns: **0**. Retries: **0**. Scientific protocol deviations: **none**.

All four complete references passed safety. All four rollouts ended at
`OBSERVATION_CAP`, with 180 intervals / 3.0000001564621925 s exposure. No planning
failure, reference skip, controller error, stale result, busy result, timeout,
safety abort or censored attachment occurred. No method attached in the first .9 s.

### Source, schedule and preserved controller state

R01 original world FRESH SHA256:
`d6e7339538f9ac8f60b25867494e5b5e15a37cc5712026469c9d90720d6828ab`.
Raw local FRESH SHA256 (identical to R00):
`8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`.
Raw local OLD SHA256 (identical to R00):
`6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae`.
R01 state/timing SHA256:
`3a97116b75e6e940fb99f95f4ae7dda40a0101a7dc6df4edd987d534fa7c1434`.
Official MPC SHA256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Pinned clean external source commit:
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`.

Every method restored the R01 values above, including generation6. Both primary
54-step and complete 180-step schedule gates passed for all four. Every host wait
retained exactly equal before/after simulation time and pose. All 30 attempted
submits were accepted. Actual submit ticks were 96 through 270 in steps of six;
successful applications, including the already-applied R01 command at92, were:

`[92,99,103,109,115,121,127,133,139,145,151,157,163,169,175,181,187,193,199,205,211,217,223,229,235,241,247,253,259,265,271]`.

These equal the frozen R00 schedule exactly. Input poses, official memory updates,
selected rows, commands, held-command integration and safety guards were recomputed
from saved records without new scientific solves.

### Primary execution against ORIGINAL R01 FRESH

Position AUC units are m·s; yaw AUC units are rad·s. Initial yaw is in radians.
Initial growth is the first-.5 s maximum distance minus initial distance. All
values below use the original FRESH evaluator, independent of the method reference.

| Method | Initial position m | Initial yaw rad | Max position .5 s m | Initial growth m |
| --- | --- | --- | --- | --- |
| Native | 0.106648553188 | 0.52360835168 | 0.21526230177 | 0.108613748582 |
| Taper | 0.106648553188 | 0.52360835168 | 0.210303865991 | 0.103655312803 |
| Full | 0.106648553188 | 0.52360835168 | 0.216171366139 | 0.109522812951 |
| No-relative | 0.106648553188 | 0.52360835168 | 0.210021055974 | 0.103372502786 |

| Method | Position AUC .3 s | Position AUC .9 s | Yaw AUC .3 s | Yaw AUC .9 s |
| --- | --- | --- | --- | --- |
| Native | 0.0479203521653 | 0.169065577542 | 0.122763571304 | 0.193367970282 |
| Taper | 0.0474739038201 | 0.16685337744 | 0.122779290506 | 0.18791121531 |
| Full | 0.0479204229036 | 0.169963894781 | 0.122774473194 | 0.193334757509 |
| No-relative | 0.0474738930022 | 0.166120972217 | 0.122777353258 | 0.191946493187 |

Attachment values require the complete following .30 s dwell. Fractional row is an index (last row 9); fractional arc is normalized original XY arc. Every attachment was observed; no null is replaced by an end time.

| Method | Attachment s | Original fractional row | Original arc fraction | Remaining original arc m |
| --- | --- | --- | --- | --- |
| Native | 1.46666674316 | 8.71784659168 | 0.968605186346 | 0.0424848909328 |
| Taper | 1.50000007823 | 8.73896508043 | 0.970955010942 | 0.0393050013241 |
| Full | 1.4833334107 | 8.78129705628 | 0.975665230469 | 0.0329309178506 |
| No-relative | 1.43333340809 | 8.47464298826 | 0.941544262805 | 0.0791049644841 |

TV includes the shared physical u_minus→u_B_plus switch. Swept clearance is a lower bound on footprint-edge clearance, with the unchanged .05 m requirement.

| Method | Swept clearance m | Endpoint error m | Linear TV m/s | Angular TV rad/s | Max abs omega rad/s |
| --- | --- | --- | --- | --- | --- |
| Native | 0.125603458688 | 0.0964389889384 | 0.799999999766 | 3.62955397677 | 1.5483330022 |
| Taper | 0.126225260214 | 0.0959816500603 | 1.19999997171 | 3.51360002933 | 1.49999872344 |
| Full | 0.125745515978 | 0.0963354155656 | 0.799999998572 | 3.53910896544 | 1.49999872585 |
| No-relative | 0.130820918566 | 0.0912985751326 | 1.19999840656 | 3.50776944702 | 1.49999871786 |

Every termination is `OBSERVATION_CAP`. All final original-FRESH projections reach
row 9 / arc 1.353245520115902 m / fraction 1, remaining arc 0, with no observed backward
unrestricted projection. This is a path-progress proxy, not proof of semantic intent.
All end speeds are near zero; all fail the inherited near-zero-both-commands diagnostic
because terminal |omega| remains .0035–.0037 rad/s. Nominal 10 Hz command-grid checks
pass. The inherited compressed application spacing99→103 remains an actual-interval
command-jump diagnostic, not continuous physical acceleration or real deployment timing.

### Secondary tracking against each method's own reference

These metrics are secondary and are never substituted into the primary table.

| Method | Own pos AUC .3 | Own pos AUC .9 | Own yaw AUC .3 | Own yaw AUC .9 | Own max pos .5 m |
| --- | --- | --- | --- | --- | --- |
| Native | 0.0479203521653 | 0.169065577542 | 0.122763571304 | 0.193367970282 | 0.21526230177 |
| Taper | 0.0182246612719 | 0.0973261938424 | 0.102281559648 | 0.167395771321 | 0.135741536994 |
| Full | 0.0169872131699 | 0.0953012265902 | 0.101992562143 | 0.17084827009 | 0.13009598666 |
| No-relative | 0.0158859288431 | 0.0878737834755 | 0.100640273011 | 0.169780454172 | 0.118106338493 |

### Planning geometry

Lateral correction is measured in the corresponding original node's body frame;
B-lateral extent uses fixed B heading. Relative-edge distortion uses the existing
SE(2) Log residual, not world-vector subtraction. The rigid fit minimizes XY squared
error for one left transform, and evaluates yaw under that same transform. Chord
threshold is the existing .02 m. Safety checks the complete reference with no connector.

| Method | First shift m | Endpoint shift m | XY arc m | Min segment m | Max segment m | Self-intersection |
| --- | --- | --- | --- | --- | --- | --- |
| Taper | 0.213333346379 | 0 | 1.17331163692 | 0.129839728113 | 0.130724153012 | false |
| Full | 0.211144018245 | 0.00218683068366 | 1.17880028618 | 0.116958801835 | 0.145860165981 | false |
| No-relative | 0.2133333463 | 0 | 1.1761688981 | 0.112274794072 | 0.147912381037 | false |

| Method | Max lateral correction m | Max B-lateral extent m | Max yaw correction deg | Reference clearance m |
| --- | --- | --- | --- | --- |
| Taper | 0.0947856906703 | 0.676244958291 | 3.65491359865e-05 | 0.221686300993 |
| Full | 0.102800269097 | 0.676301782194 | 0.341398471283 | 0.220029510626 |
| No-relative | 0.105021805624 | 0.676244958291 | 3.65491358593e-05 | 0.221686300993 |

| Method | Edge XY RMS m | Edge XY max m | Edge yaw RMS deg | Edge yaw max deg | Rigid XY RMS m | Rigid yaw RMS deg |
| --- | --- | --- | --- | --- | --- | --- |
| Taper | 0.0237037989289 | 0.0237682649824 | 4.06102661309e-06 | 4.07207209734e-06 | 0.0574113659313 | 5.21343259423 |
| Full | 0.0261585042598 | 0.040260828786 | 0.0730433579978 | 0.0992870793064 | 0.066834806652 | 5.95034390001 |
| No-relative | 0.0283785074563 | 0.0468352586946 | 4.85327950958e-06 | 7.98005243234e-06 | 0.0706015152752 | 6.46913659849 |

| Node | Taper displacement m | Full displacement m | No-relative displacement m |
| --- | --- | --- | --- |
| 0 | 0.213333346379 | 0.211144018245 | 0.2133333463 |
| 1 | 0.189567289125 | 0.205470702546 | 0.210039064188 |
| 2 | 0.165842387439 | 0.190337261993 | 0.197196901204 |
| 3 | 0.142099411695 | 0.163595889499 | 0.170562254249 |
| 4 | 0.118331178731 | 0.126830647622 | 0.129827847697 |
| 5 | 0.0946243219906 | 0.0861256959111 | 0.0829926181395 |
| 6 | 0.0709898506812 | 0.0495176810065 | 0.0425644643229 |
| 7 | 0.0473453934459 | 0.0229057837678 | 0.0160821168519 |
| 8 | 0.0237373446382 | 0.00784165745988 | 0.00329926290406 |
| 9 | 0 | 0.00218683068366 | 0 |

| Method | B→row0 m | B→row0 yaw deg | Projection distance m | Projection yaw deg | First chord vs B deg |
| --- | --- | --- | --- | --- | --- |
| Native | 0.213324006341 | 18.000810569 | 0.106648553188 | 30.0005486691 | 29.9924545919 |
| Taper | 1.09647668717e-05 | 18.0007740198 | 1.09647668669e-05 | 18.0007740198 | 35.2071230791 |
| Full | 0.00218050122362 | 18.0514357296 | 0.00108578845311 | 18.2077673429 | 31.0998319889 |
| No-relative | 1.09646995464e-05 | 18.0007740198 | 1.09646995385e-05 | 18.0007740198 | 30.6303359223 |

No-relative versus Full maximum reference XY separation is
**.006967252621980297 m**; maximum sampled execution separation is
**.019981176819074672 m**. Full versus Taper is .02449507811340204 m in reference
and .01998125498002837 m in execution. Curves overlap strongly, but are not identical;
figure styles and markers distinguish them. Native minimum reference clearance is
.22168630099308034 m, identical to Taper and No-relative here.

### Factor costs and solver

Historical L means S (state-shift/transport). The two totals contain different
terms and are not execution-performance scores.

| Method | E_S | E_R optimized | E_A | Optimized total | R diagnostic only |
| --- | --- | --- | --- | --- | --- |
| Full | 0.348589754586 | 0.0684800878325 | 0.348193451505 | 0.765263293923 | N/A |
| No-relative | 0.345922919378 | N/A | 0.345422836725 | 0.691345756103 | 0.0805339685448 |

| Method | Iterations | Accepted | Rejected | Unsafe rejected | Termination |
| --- | --- | --- | --- | --- | --- |
| Full | 6 | 4 | 2 | 0 | cost_tolerance |
| No-relative | 3 | 2 | 0 | 0 | step_tolerance |

Both start at original FRESH; initial vector cost is 4.55111554653196 (the sum of
reported factor reductions can differ in the last bit from the concatenated-vector
dot product). Full wall solve time is .08724177900012364 s and No-relative
.04436026699977447 s. These are recorded host costs, not online latency comparisons.
Full's two rejected proposals were non-improving; no unsafe step was accepted.
No-relative's R=.08053396854476672 is **diagnostic relative-edge distortion,
not optimized cost**. Neither solve used R00 initialization, a restart or a sweep.

### Official nearest-row selector diagnostics

The first selected H=5 row difference occurs at submit tick 102, logical application 103.
The sequences differ at ticks102 and132 in both R00 and R01. The per-solve diagnostic
contains all 30 records per method with poses, nearest indices, selected indices,
commands and application ticks. At tick 102:

| Method | Input pose [x,y,yaw] | Nearest row | Selected rows | Command [v,omega] | Applied tick |
| --- | --- | --- | --- | --- | --- |
| Full | [19.20994164673397, 24.000209152545146, -1.460642919770917] | 1 | [2, 3, 4, 5, 6] | [0.8, 1.4999987258522176] | 103 |
| No-relative | [19.209941646733906, 24.000209152545136, -1.4606429197740878] | 0 | [1, 2, 3, 4, 5] | [0.6000007959965704, 1.4999987178629397] | 103 |

These are observations of the unchanged official selector. The state inputs have
already accumulated tiny numerical differences, so this is not a separate isolated
selector intervention. The reference optimizer edits spatial poses; it does not
command faster rotation. Max |omega| is essentially equal, while the produced
linear command at application 103 differs by about .20 m/s.

## R00 vs R01 replication comparison

Signed contrasts below are **No-relative minus Full**. Negative AUC/attachment
means lower error/earlier attachment; positive clearance means more margin. A larger
remaining arc means attachment occurs with more original path still ahead.
All 13 requested signs match. Exact machine-readable values accompany this table.

| Metric | R00 delta | R01 delta | Same sign |
| --- | --- | --- | --- |
| max_position_error_05_m | -0.00615031384655 | -0.00615031016535 | true |
| position_auc_03_m_s | -0.000446530149801 | -0.000446529901372 | true |
| position_auc_09_m_s | -0.00384292479645 | -0.00384292256418 | true |
| yaw_auc_03_rad_s | 2.88006587183e-06 | 2.88006436124e-06 | true |
| yaw_auc_09_rad_s | -0.00138826605104 | -0.00138826432231 | true |
| sustained_attachment_s | -0.0500000026077 | -0.0500000026077 | true |
| remaining_arc_at_attachment_m | 0.0461740388182 | 0.0461740466335 | true |
| execution_clearance_lower_bound_m | 0.005075483378 | 0.005075402588 | true |
| endpoint_error_m | -0.00503683612994 | -0.00503684043305 | true |
| linear_command_TV | 0.399998408097 | 0.399998407988 | true |
| angular_command_TV | -0.0313395681909 | -0.0313395184203 | true |
| relative_translation_RMS_m | 0.00222000415737 | 0.00222000319647 | true |
| relative_translation_max_m | 0.0065744322552 | 0.0065744299086 | true |

The attachment delta is exactly -3 original integration ticks in both repeats:
-.05000000260770321 s. Full/No-relative R01 attachment times equal their respective
R00 times. The .9 s position-AUC deltas differ between repetitions by only
2.2322647064143553e-09 m·s. Absolute safety clearance is lower in R01 because the
world anchor differs: Full .12574551597848094 m versus R00 .13370942329410734 m;
No-relative .1308209185664851 m versus R00 .13878490667211216 m. Both remain above
.05 m. Full R01 required 6 iterations versus the historical R00 Full 5, with the same
solver settings; no attempt was made to force the iteration count to match.

## Research interpretation

The predeclared category is **REPLICATED_WITHIN_PAIRED_OSA03**. Within this paired
source, removing E_R again produced a millimetric reference change, modestly lower
early position error, three ticks earlier sustained attachment, and preserved
reference/execution safety and downstream progress. Relative-edge translation
distortion and linear command TV increased. Yaw AUC increased slightly at .3 s and
decreased at .9 s; angular command TV decreased. The result is a trade-off.

The paired evidence makes the necessity/strength of current E_R scientifically
questionable in this local geometry. E_R has a measured shape-regularizing effect,
but its presence did not improve early attachment here. This supports a
**regularization-versus-attachment trade-off**, not a final instruction to remove E_R.
No replacement term or new final method is proposed.

Full remains numerically distinguishable from Taper, but the executions overlap
strongly. Taper has lower .9 s position AUC than Full; Full attaches one tick earlier
and has lower linear command TV. These paired data do not establish graph superiority.

The close R00/R01 numbers are consistent with deterministic formulation/controller
behavior under nearly identical local geometry and slightly different world/controller
state. They do not constitute evidence from diverse VLA outputs. Original FRESH still
has substantial intrinsic B-start tracking difficulty: all methods initially separate
to more than .21 m and attach only near its endpoint.

## Limitations

- One scenario, two independently acquired repetitions with **identical raw local
  FRESH** and identical raw OLD; no independent policy-output diversity.
- Offline timing-controlled scheduler with paused host waits, not real asynchronous
  deployment timing, real-time throughput or a latency benchmark.
- No controller-aware factor or new selector; small reference differences interact
  with the existing nearest-row selection and MPC commands.
- Shared already-applied R01 first FRESH command means the initial switch command
  itself cannot improve in this comparison.
- Primary equal exposure is .9 s; sustained attachment occurs later, within the
  common 3 s cap. The continuation has no repeated FRESH updates.
- Safety uses the saved direct geometry, circular footprint and existing conventions;
  there is no real-robot test or complete obstacle bypass.
- Endpoint/progress/remaining-arc measures are intent proxies. They do not prove
  semantic intent preservation or instruction-side compliance.
- No population claim or broad statement of E_R necessity or superiority.

## Not demonstrated

Trajectory-diverse generalization; general E_R necessity; online asynchronous
benefit; real robot benefit; complete obstacle bypass; final-method superiority;
online repeated-chunk improvement or complete semantic intent preservation.

## Validation and artifacts

Pre-execution focused 25 / full 409 passed, one absent-historical-corpus skip.
Compileall and working/staged diff checks passed before the pushed freeze.
Saved-only validation verifies source/code hashes, both solver traces, costs,
retractions and safety, controller memory/selection, exact integration, guards,
primary/secondary metric parity and common schedules. R00 remains byte-identical.
Post-execution full regression: **409 passed, 1 skipped (39.93 s)**, with the same
absent historical corpus skip. Compileall and diff checks passed. Ten scientific
plots were visually inspected, and numeric/hash/CSV parity passed.
No scientific protocol deviation; no rerun or result-driven implementation change.

- [Local GUI review](../data/osa03_relative_factor_replication_01/primary_20260930T010000Z/index.html).
- [Execution plot](../data/osa03_relative_factor_replication_01/primary_20260930T010000Z/review/world_execution.png).
- [Paired effects](../data/osa03_relative_factor_replication_01/primary_20260930T010000Z/review/paired_effects.png).
- [Review ZIP](../data/osa03_relative_factor_replication_01/primary_20260930T010000Z/review_bundle.zip).
- [Tracked result summary](../results/osa03_relative_factor_replication_01/result_summary.json).
- [Primary CSV](../results/osa03_relative_factor_replication_01/primary.csv).
- [Signed R00/R01 CSV](../results/osa03_relative_factor_replication_01/replication_effects.csv).
- [Saved selector diagnostics](../data/osa03_relative_factor_replication_01/primary_20260930T010000Z/selector_diagnostics.json).

Exact scientific/validation/report commands are above; the exact full regression
command is in the pre-execution WORK_LOG entry. The scientific execution marker and
both planning/method markers record the same pushed freeze SHA. Results and docs are
committed separately after validation; final commit identities are reported to the user.

## Explicit answers

1. **Did the 3-tick / ~0.05 s earlier attachment from R00 replicate in R01?**
   Yes. No-relative 1.433333408087492 s versus Full 1.4833334106951952 s;
   delta -.05000000260770321 s, exactly three original integration ticks, in both repeats.
2. **Did the sign of the 0.9 s position-AUC change replicate?**
   Yes. R00 -.003842924796446434 and R01 -.0038429225641817277 m·s.
3. **Did safety remain preserved?**
   Yes. All complete references and full executions passed. R01 No-relative versus
   Full swept lower bound .1308209185664851 versus .12574551597848094 m; no abort.
4. **Did removing E_R again increase relative-edge distortion?**
   Yes for translation: RMS delta +.0022200031964668375 m, maximum
   +.006574429908603255 m. Relative yaw distortion decreases; it is not an
   across-the-board increase in every edge component.
5. **Did command-variation trade-offs replicate?**
   Yes. Linear TV increases +.3999984079884431 m/s, angular TV decreases
   -.03133951842028626 rad/s, matching R00 directions.
6. **Were Full and No-relative reference differences again only millimetric?**
   Yes. Maximum node XY separation 6.967252621980297 mm versus R00
   6.967254552074201 mm. Execution maximum separation is 19.981176819074672 mm.
7. **Did small reference differences again cause different MPC nearest-row selections?**
   Different selections are observed again: first at tick 102, Full nearest 1/H5[2–6]
   versus No-relative nearest 0/H5[1–5], both applied 103. The distinct reference
   conditions also develop tiny input-state differences; no isolated selector
   intervention was performed. Differences recur at 132 in both repetitions.
8. **Does the paired evidence support keeping, removing, or questioning E_R?**
   It supports questioning its necessity/strength and a regularization-versus-attachment
   trade-off in this source. It does not support a final-method decision to remove it.
9. **What remains unproven because R00/R01 share the same raw local FRESH?**
   Stability across diverse policy outputs, scenarios and handoff geometry; general
   E_R necessity/superiority; online repeated-update benefit, real-robot performance
   and semantic intent preservation. This is paired same-scenario replication only.
