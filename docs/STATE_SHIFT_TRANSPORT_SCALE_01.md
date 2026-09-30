# STATE_SHIFT_TRANSPORT_SCALE_01

## Frozen protocol

Starting fetched HEAD and origin/main: `0db03d81ee33605ac96fc2d0886364b8499874e9`.
**Timing-controlled offline causal reference comparison.** Diagnostic experiment,
with exactly four fixed sources from RELATIVE_FACTOR_MULTISOURCE_01:
S1 OSA03_R00; S2 episode_001_repeat_01/handoff_013;
S3 episode_008_repeat_01/handoff_023; S4 episode_013_repeat_00/handoff_020.
No acquisition, source search, source replacement, alpha tuning or repetitions.

Run: `data/state_shift_transport_scale_01/primary_20260930T080000Z`.
Directory name is a fixed identifier; actual UTC/host and simulation clocks are
recorded separately. Arrays, solver traces and rollouts stay under ignored data.
Small numeric summaries and exactly four final PNGs are tracked.

### Fixed spatial problem

A and B are fixed. The **observation-to-application state-shift / transport factor**
moves editable FRESH nodes toward a transported target:

```
G = B A^-1; G_alpha = Exp(alpha Log(G)); Ftilde(alpha) = G_alpha F
r_S = Log(Ftilde(alpha)^-1 X)
r_R = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A = Log(F_j^-1 X_j)
E(alpha) = E_S(alpha) + E_R + E_A
s = cumulative original-FRESH XY arc / total original arc
w_S = (1-s)^2; w_A = s^2
normalization = diag(.10 m, .10 m, 10 degrees)
```

Use existing weighted node means / relative-edge mean and unit lambdas.
Historical L keys retain their code spelling and mean S above. A-local raw
`F=A F_local` is immutable; derived local arrays use `A^-1 X`. Never re-anchor
raw rows at B. World XY metres, yaw radians CCW, +Z up; local x forward/y left.
At alpha=1, `B^-1 Ftilde = A^-1 F`. Alpha=.5 intentionally relaxes this identity
using complete SE(2) Log/Exp, without independently scaling pose x/y/yaw.

Compare only Native (alpha=0), Half Local-SE2 (.5), Full Local-SE2 (1).
At alpha=0, X=F gives zero objective up to floating-point roundoff. Authenticate
and reuse the four Native references/rollouts/metrics and four Full
references/rollouts/metrics; no new Native or Full solve. Reuse the four Full
planning solutions. If reuse fails authentication, stop instead of rerunning.
The new subclass overrides the transported target only. All historical files,
LocalSE2 default arithmetic, solver, controller and validators remain unchanged.

Half starts from original FRESH once/source. Same right-local `X Exp(delta)` LM,
central FD 1e-6, max80, damping .001, reject x10/accept x.3, maximum1e12,
gradient/step1e-9, cost1e-12. No warm starts, retries or compensating factors.
No waypoint time/dt, GP, twist, velocity, controller, obstacle, correspondence,
projection-join, adaptive alpha or alternate selector. Spatial reference yaw
changes do not directly specify the robot's angular command.

### Authentication, schedule and safety

Authenticate tracked historical result summary, complete saved result hash ledger,
original source/code hashes, saved B/state/geometry, and unchanged old multisource,
R00 and R01 validators. Copy common state, schedules, scenario, metric protocol,
source audit/manifest and recorded OLD-to-B byte-for-byte; reference/rollout/metric
reuse points to exact old files. The named original-world file needed by the
existing runner is copied byte-for-byte, never reconstructed.

Restore B pose/tick/simulation time, physical u_minus provenance, already-applied
u_B_plus, official previous_control, generation/version/chunk and source dt exactly.
Use source-specific frozen submit/release pairs from historical schedule.json,
without generating a new schedule. Reuse the existing official worker and runtime
function directly. Simulation waits at the submit state; wall-clock completion
cannot advance simulation or choose application ticks. Official memory poll order,
nearest-row H5 selection and command computation remain unchanged. No future state.
Require exact attempted/accepted/application sequences, initial provenance, 54
primary and 180 full integration intervals. The source's final result may remain
withheld at the excluded cap; count it without inventing an application.

Same Hospital/cart full-polyline checker, circular radius .20 m, required edge
clearance .05 m, workspace and numerical reserve. LM accepts only objective decrease
and complete-reference feasibility. No B connector, crop, repair, smoothing or new
margin. Unsafe final references are recorded and skipped. Execution uses the exact
abort-only guard; never apply the rejected command or fabricate continuation.
A failed schedule gate bars reference-causal interpretation and is reported with
its actual cause, including scientific safety censoring.

### Evaluation and predeclared interpretation

All 18 primary metrics use ORIGINAL FRESH: initial position/yaw, max .5 s position,
initial growth, position/yaw AUC .3/.9, sustained attachment, fractional row, arc
fraction, remaining arc, swept clearance, endpoint, linear/angular TV, max |omega|,
termination. Same forward-only continuous projection and shortest yaw. Attachment
requires <=.10 m and <=15 degrees for the complete following .30 s sampled dwell.
Null remains null. Half own-reference position/yaw .3/.9 AUC and .5 s max error are
separate secondary metrics. Planning includes all node/edge/rigid-fit/clearance/
chord/segment/intersection diagnostics and separate factor costs.

Report `Log(A^-1 B)` and its translation norm as the body-motion measure in the
cross-source table; separately report world `Log(B A^-1)` and full/half transform
translations, which depend on world origin when yaw is nonzero. Also report full
and half body-motion translation norms and yaw. B projection onto ORIGINAL FRESH
is separately identified from each method's B projection. No continuous curve or
optimal alpha inferred from these three discrete conditions.

Selector CSV/JSON reads finished records only: every pose, nearest row, H5 indices,
command, application tick, withheld result. Compare Half-Native and Half-Full first
differing tick; report attachment co-occurrence without attributing causality.

Hypotheses overlap, so freeze this conservative classification precedence (numeric
sign tolerance1e-9 in each unit; no fitted thresholds and no combined winner score):

1. TECHNICAL_BLOCKED for source/state/schedule/controller/validator/history failure.
2. SOURCE_DEPENDENT_TRANSPORT (H3) if Half-Full .9 s position AUC improves in at
   least one source and worsens in another across S1-S4.
3. TRANSPORT_MAGNITUDE_INSUFFICIENT (H2) otherwise if any S2/S3/S4 fails to lower
   .9 s AUC, Native is lower than both in any S2/S3/S4, or safety is not preserved.
4. FULL_TRANSPORT_OVERCOMPENSATES (H1) otherwise: safety preserved, all problematic
   S2/S3/S4 AUCs improve, and Native is not strictly better than both there.

Also show every H1-compatible safe AUC improvement, all attachment availability,
changes and trade-offs regardless of category. This prevents an H2/H3 label from
hiding partial overcompensation evidence. Attachment is not imputed when null.
The .9 s AUC defines reproducible direction counts only; all requested metrics and
nine interpretation answers remain necessary. No optimum/final method claim.

### Freeze and output discipline

Prepare/authenticate with zero new science; run synthetic tests plus old regression
suite, compileall, diff/staged review; commit and normal push before four new Half
planning solves and four eligible Half rollouts. No Native/Full/Taper rerun.
Saved-only validation and reporting follow, with PNG inspection and numeric
sidecars. Update this report/log, review, commit and normal push the small results.

Exactly four final PNGs: world_execution_overview.png,
transport_scale_primary_metrics.png, transport_scale_deltas.png,
reference_transport_overview.png. Equal world axes, whole cart + safety region,
recorded OLD-to-B, original rows, distinct execution styles/markers, attachment,
minimum clearance, endpoints and numerical overlap annotations. No selector PNG.

## Repository-confirmed facts

**TRANSPORT_MAGNITUDE_INSUFFICIENT (H2)** under the predeclared precedence.

Scientific freeze: `92a3bdaf1d6ce14e5efbfb2c8f0a90489f73233d`, committed and pushed before execution. Exactly **4 new Half planning solves, 4 new rollouts and 120 new official MPC solves**. All four planning calls converged; all four new rollouts completed 180 intervals (3.0000001564621925 s). No unsafe reference, execution abort, controller failure or retry. LightNav=0, RGB=0, Isaac=0, GP=0; source replacements=0.

Reused byte-for-byte: **8 references, 8 rollouts, 8 metric records and 4 Full planning solutions**. Native/Full new optimizer calls=0 and new MPC calls=0. The reused rollouts contain 240 historical MPC calculations; these are not new calls. Of the 120 new successful results, 119 are applied and S2's final result is held beyond the excluded cap. Each rollout also starts from its one restored common first FRESH command.

All 12 compared references and executions pass the unchanged .20 m footprint / .05 m edge clearance checks. All four sources pass exact common-state/schedule parity at both 54 and 180 intervals. Saved-only validation and unchanged multisource/R00/R01 validators pass. No host timing determines logical command application.

Half lowers original-FRESH .9 s position AUC relative to Full in 4/4 sources, worsens it in 0/4. Native has lower .9 s position AUC than both transported variants in S1 and S4. Half attaches one tick earlier than Full in S1 and ten ticks earlier in S2. Half and Full have null attachment in S3 and S4; Native attaches in S4.

Tables below use up to 12 significant digits for readability. JSON/CSV and plot sidecars retain the complete saved floating-point values. N/A denotes a missing complete dwell, never zero.

## Source/state-shift table

| Source | ID | Raw FRESH SHA256 | B tick | B simulation s | generation |
|---|---|---|---|---|---|
| S1 | OSA03_R00 | `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521` | 92 | 1.56666674837 | 3 |
| S2 | episode_001_repeat_01/handoff_013 | `82bb298736fd7a1e6289a529e0a973d4def6b3ed6b00b03506ac3694833f3503` | 997 | 16.6500008684 | 88 |
| S3 | episode_008_repeat_01/handoff_023 | `ba52161d8e7ba1f0619e11cd0b93da29d201e70074d76160d374fe1f6c87517f` | 1804 | 30.1000015698 | 326 |
| S4 | episode_013_repeat_00/handoff_020 | `f5585ef9f7ff2eeeae9992640a68cf65b6b8510baacf8ff9fd2886aa40df7552` | 1524 | 25.4333346598 | 466 |

| Source | Log(A^-1 B): vx, vy, yaw [m,m,rad] | translation norm [m] | body full / half translation [m] | full / half yaw [rad] |
|---|---|---|---|---|
| S1 | 0.213333346378, -2.38118427148e-08, -7.73821326572e-07 | 0.213333346378 | 0.213333346378 / 0.106666673189 | -7.73821326572e-07 / -3.86910663286e-07 |
| S2 | 0.492125082144, 0.000232486275148, 0.000198744114022 | 0.492125137059 | 0.492125136249 / 0.246062568428 | 0.000198744114022 / 9.93720570108e-05 |
| S3 | 0.552905537377, -0.00948700610886, 0.244350309637 | 0.552986922582 | 0.55161223101 / 0.276321528629 | 0.244350309637 / 0.122175154819 |
| S4 | 0.424662608998, -0.00157653688558, 0.0118243929094 | 0.424665535392 | 0.424663061432 / 0.212332458451 | 0.0118243929094 / 0.0059121964547 |

The body-relative Log translation norm is the cross-source state-shift measure. For completeness, the world left-transform components are reported separately below. In S3, its 9.43 m translation component is coupled with rotation about the world origin; it is **not** 9.43 m of robot travel or FRESH-node displacement.

| Source | Log(B A^-1): vx, vy, yaw | world transform full / half translation norm [m] | B to ORIGINAL FRESH projection distance / abs yaw [m,rad] |
|---|---|---|---|
| S1 | 0.000369624105073, -0.213318132803, -7.73821326572e-07 | 0.213318453033 / 0.106659226517 | 0.106648569289 / 0.523608487599 |
| S2 | 0.0630284572013, -0.49207281591, 0.000198744114022 | 0.496092976936 / 0.248046488774 | 0.127644308504 / 0.261793531143 |
| S3 | 8.18852747262, -4.7239989405, 0.244350309637 | 9.42997205591 / 4.72379716572 | 0.184683189977 / 0.385685031403 |
| S4 | 0.193818167775, -0.64812430507, 0.0118243929094 | 0.676480055131 / 0.338241505429 | 0.16088422589 / 0.505674822974 |

Source audit files retain exact observation/request/readiness/application clocks, A/B poses, physical incoming u_minus, already-applied u_B_plus, controller memory and original chunk/version. The tracked freeze ledger contains the exact states, source hashes and copied schedule hashes.

## Native vs Half vs Full results

| Source | body Log translation m | Native AUC .9 | Half AUC .9 | Full AUC .9 | Native attach s | Half attach s | Full attach s | Half−Full AUC | Half−Full attach s | safety |
|---|---|---|---|---|---|---|---|---|---|---|
| S1 | 0.213333346378 | 0.169065632516 | 0.169216433013 | 0.169963954056 | 1.46666674316 | 1.46666674316 | 1.4833334107 | -0.000747521042958 | -0.0166666675359 | PASS |
| S2 | 0.492125137059 | 0.13183311076 | 0.131791529018 | 0.13668227007 | 1.05000005476 | 1.03333338723 | 1.20000006258 | -0.00489074105217 | -0.166666675359 | PASS |
| S3 | 0.552986922582 | 0.189624121947 | 0.189328285588 | 0.199963659347 | N/A | N/A | N/A | -0.0106353737583 | N/A | PASS |
| S4 | 0.424665535392 | 0.198189215821 | 0.204465335704 | 0.217335047215 | 1.28333340026 | N/A | N/A | -0.0128697115117 | N/A | PASS |

### S1: OSA03_R00

| Primary ORIGINAL FRESH metric | Native | Half | Full |
|---|---|---|---|
| Initial position [m] | 0.106648569289 | 0.106648569289 | 0.106648569289 |
| Initial abs yaw [rad] | 0.523608487599 | 0.523608487599 | 0.523608487599 |
| Max position first .5 s [m] | 0.215262376676 | 0.215516942597 | 0.216171443661 |
| Initial separation growth [m] | 0.108613807388 | 0.108868373309 | 0.109522874372 |
| Position AUC .3 s [m s] | 0.0479203622918 | 0.0479203820683 | 0.0479204330303 |
| Position AUC .9 s [m s] | 0.169065632516 | 0.169216433013 | 0.169963954056 |
| Yaw AUC .3 s [rad s] | 0.122763624527 | 0.122766673494 | 0.122774526447 |
| Yaw AUC .9 s [rad s] | 0.193368049414 | 0.193957782739 | 0.193334827703 |
| Sustained attachment [s] | 1.46666674316 | 1.46666674316 | 1.4833334107 |
| Attachment fractional row [0..9] | 8.71784637003 | 8.71642947345 | 8.7812968397 |
| Attachment original arc fraction | 0.968605161683 | 0.96844750559 | 0.975665206371 |
| Remaining original arc at attachment [m] | 0.0424849243072 | 0.0426982717094 | 0.032930950462 |
| Execution swept clearance lower bound [m] | 0.133561094445 | 0.134292821442 | 0.133709423294 |
| Endpoint error [m] | 0.0964390213698 | 0.0957029932029 | 0.096335461697 |
| Linear command TV [m/s] | 0.799999999766 | 0.800000008518 | 0.799999998572 |
| Angular command TV [rad/s] | 3.62955577057 | 3.58522541824 | 3.53910892726 |
| Max abs omega [rad/s] | 1.548333658 | 1.52638182555 | 1.49999844641 |
| Termination | OBSERVATION_CAP | OBSERVATION_CAP | OBSERVATION_CAP |

### S2: episode_001_repeat_01/handoff_013

| Primary ORIGINAL FRESH metric | Native | Half | Full |
|---|---|---|---|
| Initial position [m] | 0.127644308504 | 0.127644308504 | 0.127644308504 |
| Initial abs yaw [rad] | 0.261793531143 | 0.261793531143 | 0.261793531143 |
| Max position first .5 s [m] | 0.163894949098 | 0.164048025152 | 0.165601785893 |
| Initial separation growth [m] | 0.0362506405941 | 0.0364037166475 | 0.0379574773894 |
| Position AUC .3 s [m s] | 0.0453024911898 | 0.0453063933689 | 0.0453377459063 |
| Position AUC .9 s [m s] | 0.13183311076 | 0.131791529018 | 0.13668227007 |
| Yaw AUC .3 s [rad s] | 0.0456241196881 | 0.0457687342965 | 0.0469310639624 |
| Yaw AUC .9 s [rad s] | 0.107103977941 | 0.108824403834 | 0.0989513627252 |
| Sustained attachment [s] | 1.05000005476 | 1.03333338723 | 1.20000006258 |
| Attachment fractional row [0..9] | 7.69045316001 | 7.60103694465 | 8.10274757945 |
| Attachment original arc fraction | 0.854497877738 | 0.844560127763 | 0.900316174521 |
| Remaining original arc at attachment [m] | 0.197121472188 | 0.210584807807 | 0.135048356177 |
| Execution swept clearance lower bound [m] | 0.757224041072 | 0.757224844485 | 0.757225430422 |
| Endpoint error [m] | 0.0793823670714 | 0.0784311726707 | 0.086092224348 |
| Linear command TV [m/s] | 0.80604106892 | 0.806041065873 | 1.38978624204 |
| Angular command TV [rad/s] | 2.546626244 | 2.49092892824 | 2.56885015891 |
| Max abs omega [rad/s] | 1.05284528875 | 1.02392242459 | 1.00282867923 |
| Termination | OBSERVATION_CAP | OBSERVATION_CAP | OBSERVATION_CAP |

### S3: episode_008_repeat_01/handoff_023

| Primary ORIGINAL FRESH metric | Native | Half | Full |
|---|---|---|---|
| Initial position [m] | 0.184683189977 | 0.184683189977 | 0.184683189977 |
| Initial abs yaw [rad] | 0.385685031403 | 0.385685031403 | 0.385685031403 |
| Max position first .5 s [m] | 0.239119858172 | 0.235708579569 | 0.242432469823 |
| Initial separation growth [m] | 0.054436668195 | 0.0510253895916 | 0.0577492798459 |
| Position AUC .3 s [m s] | 0.0643635365783 | 0.0636342740764 | 0.0644023058604 |
| Position AUC .9 s [m s] | 0.189624121947 | 0.189328285588 | 0.199963659347 |
| Yaw AUC .3 s [rad s] | 0.0761905902303 | 0.0750881381002 | 0.0774026210484 |
| Yaw AUC .9 s [rad s] | 0.175616043138 | 0.163929174169 | 0.147269351082 |
| Sustained attachment [s] | N/A | N/A | N/A |
| Attachment fractional row [0..9] | N/A | N/A | N/A |
| Attachment original arc fraction | N/A | N/A | N/A |
| Remaining original arc at attachment [m] | N/A | N/A | N/A |
| Execution swept clearance lower bound [m] | 0.380788847753 | 0.380794534645 | 0.38079617426 |
| Endpoint error [m] | 0.11523801852 | 0.120314783299 | 0.139406215096 |
| Linear command TV [m/s] | 1.30641155651 | 1.30641155769 | 2.10641152217 |
| Angular command TV [rad/s] | 4.09143330004 | 3.69925251764 | 3.57735057172 |
| Max abs omega [rad/s] | 1.97074270782 | 1.78281507165 | 1.70699028745 |
| Termination | OBSERVATION_CAP | OBSERVATION_CAP | OBSERVATION_CAP |

### S4: episode_013_repeat_00/handoff_020

| Primary ORIGINAL FRESH metric | Native | Half | Full |
|---|---|---|---|
| Initial position [m] | 0.16088422589 | 0.16088422589 | 0.16088422589 |
| Initial abs yaw [rad] | 0.505674822974 | 0.505674822974 | 0.505674822974 |
| Max position first .5 s [m] | 0.252791514218 | 0.256123137231 | 0.270607541138 |
| Initial separation growth [m] | 0.0919072883284 | 0.0952389113408 | 0.109723315248 |
| Position AUC .3 s [m s] | 0.0611986169982 | 0.0617552243068 | 0.0629959242336 |
| Position AUC .9 s [m s] | 0.198189215821 | 0.204465335704 | 0.217335047215 |
| Yaw AUC .3 s [rad s] | 0.120660784385 | 0.121297577311 | 0.122789043859 |
| Yaw AUC .9 s [rad s] | 0.230599467879 | 0.215195905424 | 0.2009656047 |
| Sustained attachment [s] | 1.28333340026 | N/A | N/A |
| Attachment fractional row [0..9] | 7.78827222868 | N/A | N/A |
| Attachment original arc fraction | 0.912753530112 | N/A | N/A |
| Remaining original arc at attachment [m] | 0.11196220915 | N/A | N/A |
| Execution swept clearance lower bound [m] | 0.870803817623 | 0.870581596798 | 0.870583703714 |
| Endpoint error [m] | 0.08453931375 | 0.101967088518 | 0.132316561214 |
| Linear command TV [m/s] | 1.45352966553 | 1.57383375318 | 1.46221189043 |
| Angular command TV [rad/s] | 4.82596744441 | 4.41387418088 | 4.39497273254 |
| Max abs omega [rad/s] | 2.27134699449 | 2.03892393664 | 1.9630604491 |
| Termination | OBSERVATION_CAP | OBSERVATION_CAP | OBSERVATION_CAP |

### Secondary own-reference tracking

These values evaluate each method against its own reference; they do not replace the preceding original-FRESH primary metrics. Full has lower own-reference .9 s position AUC than Half in all four sources, while Half has lower original-FRESH .9 s position AUC in all four.

| Source | Method | Position AUC .3 | Position AUC .9 | Yaw AUC .3 | Yaw AUC .9 | Max position .5 |
|---|---|---|---|---|---|---|
| S1 | Native | 0.0479203622918 | 0.169065632516 | 0.122763624527 | 0.193368049414 | 0.215262376676 |
| S1 | Half | 0.032942226333 | 0.134770560611 | 0.119993308195 | 0.190377697987 | 0.176435065626 |
| S1 | Full | 0.0169872144204 | 0.0953012440115 | 0.101992575979 | 0.170848313524 | 0.130096010339 |
| S2 | Native | 0.0453024911898 | 0.13183311076 | 0.0456241196881 | 0.107103977941 | 0.163894949098 |
| S2 | Half | 0.027559270796 | 0.0948233777766 | 0.046284794958 | 0.10746928279 | 0.114624441696 |
| S2 | Full | 0.0189550084472 | 0.0540505707015 | 0.0473330586203 | 0.0963629009124 | 0.145846480806 |
| S3 | Native | 0.0643635365783 | 0.189624121947 | 0.0761905902303 | 0.175616043138 | 0.239119858172 |
| S3 | Half | 0.0362457897804 | 0.131605240257 | 0.0564817495279 | 0.153697122788 | 0.162477779115 |
| S3 | Full | 0.00769811916708 | 0.0548568535947 | 0.050461205174 | 0.148315489558 | 0.0550317460036 |
| S4 | Native | 0.0611986169982 | 0.198189215821 | 0.120660784385 | 0.230599467879 | 0.252791514218 |
| S4 | Half | 0.0335530413247 | 0.128060956154 | 0.0845006426037 | 0.201253514744 | 0.163915066932 |
| S4 | Full | 0.00917231382075 | 0.0523258722643 | 0.0535085104492 | 0.190065469428 | 0.0589708885222 |

### Planning diagnostics

#### S1 geometry

| Diagnostic | Native | Half | Full |
|---|---|---|---|
| First-node correction [m] | 0 | 0.105571965679 | 0.211144018066 |
| Endpoint correction [m] | 0 | 0.00109345990842 | 0.00218683149073 |
| XY arc [m] | 1.35324552012 | 1.26422928654 | 1.17880029964 |
| Min segment [m] | 0.149753630832 | 0.133163986004 | 0.116958805401 |
| Max segment [m] | 0.150770002343 | 0.148301337795 | 0.145860165845 |
| Max lateral node correction [m] | 1.7763568394e-15 | 0.0514000209873 | 0.102800303573 |
| Max yaw correction [rad] | 0 | 0.00296973520066 | 0.00595845237739 |
| Relative-edge translation RMS [m] | 5.10221045183e-18 | 0.0130799041796 | 0.0261585100865 |
| Relative-edge translation max [m] | 9.55989120025e-18 | 0.0201314815172 | 0.0402608376743 |
| Relative-edge yaw RMS [rad] | 0 | 0.000635265537964 | 0.00127484606748 |
| Relative-edge yaw max [rad] | 0 | 0.000863733875809 | 0.00173287142318 |
| B-to-row0 position [m] | 0.213324006344 | 0.107752045828 | 0.00218050139768 |
| B-to-row0 abs yaw [rad] | 0.314173548375 | 0.314614605369 | 0.315056989961 |
| First chord vs B heading [rad] | 0.523467219302 | 0.532971683148 | 0.542794471393 |
| Self-intersection | False | False | False |
| Minimum reference clearance [m] | 0.22948689016 | 0.228647413633 | 0.227809208186 |
| Rigid-fit XY RMS [m] | 0 | 0.0339717407082 | 0.0668348028522 |
| Rigid-fit yaw RMS [rad] | 0 | 0.0475428190289 | 0.103853183761 |
| B-to-own-reference projection [m] | 0.106648569289 | 0.0547305476179 | 0.00108578872624 |
| B-to-own-reference projection abs yaw [rad] | 0.523608487599 | 0.446016668112 | 0.317785491459 |

| Original row | Half displacement from F [m] | Full displacement from F [m] |
|---|---|---|
| 0 | 0.105571965679 | 0.211144018066 |
| 1 | 0.102735264402 | 0.205470712019 |
| 2 | 0.0951684542147 | 0.19033727955 |
| 3 | 0.0817977231088 | 0.163595912011 |
| 4 | 0.063415216624 | 0.126830670666 |
| 5 | 0.0430629567575 | 0.0861257153885 |
| 6 | 0.0247590702807 | 0.0495176943489 |
| 7 | 0.0114530856094 | 0.0229057908652 |
| 8 | 0.00392092952175 | 0.00784166016473 |
| 9 | 0.00109345990842 | 0.00218683149073 |

#### S2 geometry

| Diagnostic | Native | Half | Full |
|---|---|---|---|
| First-node correction [m] | 0 | 0.243514358379 | 0.487028825706 |
| Endpoint correction [m] | 0 | 0.00254874391935 | 0.00509739567852 |
| XY arc [m] | 1.35476698981 | 1.12449182988 | 0.904391439655 |
| Min segment [m] | 0.150167768336 | 0.105870719806 | 0.0646789036111 |
| Max segment [m] | 0.150789196217 | 0.144448449545 | 0.138149307357 |
| Max lateral node correction [m] | 1.7763568394e-15 | 0.063196883487 | 0.126417400355 |
| Max yaw correction [rad] | 0 | 0.00351114129093 | 0.007049958227 |
| Relative-edge translation RMS [m] | 6.97662552286e-18 | 0.0303925004912 | 0.0607835215654 |
| Relative-edge translation max [m] | 1.34043551201e-17 | 0.0467435791878 | 0.0934847527502 |
| Relative-edge yaw RMS [rad] | 0 | 0.000763538649876 | 0.00153308543241 |
| Relative-edge yaw max [rad] | 0 | 0.00103675703522 | 0.00207973166056 |
| B-to-row0 position [m] | 0.348704611677 | 0.110157832136 | 0.145846480806 |
| B-to-row0 abs yaw [rad] | 0.261999450501 | 0.262430778632 | 0.262865334197 |
| First chord vs B heading [rad] | -0.261954610839 | -0.273530924419 | -0.286167924813 |
| Self-intersection | False | False | False |
| Minimum reference clearance [m] | 0.580552285655 | 0.715053840064 | 0.888094511037 |
| Rigid-fit XY RMS [m] | 0 | 0.0875111965206 | 0.17246955817 |
| Rigid-fit yaw RMS [rad] | 0 | 0.0655161678073 | 0.174668771237 |
| B-to-own-reference projection [m] | 0.127644308504 | 0.0654930923845 | 0.145846480806 |
| B-to-own-reference projection abs yaw [rad] | 0.261793531143 | 0.262782221712 | 0.262865334197 |

| Original row | Half displacement from F [m] | Full displacement from F [m] |
|---|---|---|
| 0 | 0.243514358379 | 0.487028825706 |
| 1 | 0.236967092112 | 0.47393441387 |
| 2 | 0.21950421446 | 0.439008882064 |
| 3 | 0.1887295067 | 0.377459556953 |
| 4 | 0.146489944941 | 0.292980139641 |
| 5 | 0.099610874564 | 0.199221488579 |
| 6 | 0.0573204097552 | 0.114640300927 |
| 7 | 0.0265324716339 | 0.0530645231116 |
| 8 | 0.00908666822338 | 0.0181731225971 |
| 9 | 0.00254874391935 | 0.00509739567852 |

#### S3 geometry

| Diagnostic | Native | Half | Full |
|---|---|---|---|
| First-node correction [m] | 0 | 0.272744965256 | 0.544455991623 |
| Endpoint correction [m] | 0 | 0.00114030551511 | 0.00229966564874 |
| XY arc [m] | 1.26133847899 | 1.05718502696 | 0.880600135999 |
| Min segment [m] | 0.0476164233176 | 0.045448501849 | 0.0431236822195 |
| Max segment [m] | 0.171384611282 | 0.159293235195 | 0.152467653228 |
| Max lateral node correction [m] | 3.5527136788e-15 | 0.0759543441861 | 0.122413917546 |
| Max yaw correction [rad] | 0 | 0.121664781823 | 0.243485752677 |
| Relative-edge translation RMS [m] | 6.73224808375e-18 | 0.0307649397965 | 0.0616579690758 |
| Relative-edge translation max [m] | 1.28229540198e-17 | 0.0495335126063 | 0.0992247284757 |
| Relative-edge yaw RMS [rad] | 0 | 0.0161086795218 | 0.0322236820979 |
| Relative-edge yaw max [rad] | 0 | 0.0262769081199 | 0.0524884602186 |
| B-to-row0 position [m] | 0.514561810948 | 0.242267423551 | 0.0320129218666 |
| B-to-row0 abs yaw [rad] | 0.0591813072888 | 0.180846089112 | 0.302667059966 |
| First chord vs B heading [rad] | 0.13213043457 | 0.267742814016 | 0.410285147083 |
| Self-intersection | False | False | False |
| Minimum reference clearance [m] | 0.131349284934 | 0.229182120649 | 0.369372221688 |
| Rigid-fit XY RMS [m] | 0 | 0.0780936658461 | 0.149354453962 |
| Rigid-fit yaw RMS [rad] | 0 | 0.132320368027 | 0.328306967905 |
| B-to-own-reference projection [m] | 0.184683189977 | 0.0940841895915 | 0.0320129218666 |
| B-to-own-reference projection abs yaw [rad] | 0.385685031403 | 0.327419417369 | 0.302667059966 |

| Original row | Half displacement from F [m] | Full displacement from F [m] |
|---|---|---|
| 0 | 0.272744965256 | 0.544455991623 |
| 1 | 0.258446276571 | 0.515860979606 |
| 2 | 0.233062295287 | 0.465097868178 |
| 3 | 0.192773191768 | 0.384614235515 |
| 4 | 0.139658680578 | 0.278652117241 |
| 5 | 0.084440195807 | 0.168584473893 |
| 6 | 0.0391499701765 | 0.078301217461 |
| 7 | 0.0132228255352 | 0.026535157873 |
| 8 | 0.00368908080702 | 0.00742515143071 |
| 9 | 0.00114030551511 | 0.00229966564874 |

#### S4 geometry

| Diagnostic | Native | Half | Full |
|---|---|---|---|
| First-node correction [m] | 0 | 0.210298207056 | 0.420594506982 |
| Endpoint correction [m] | 0 | 0.00149046938397 | 0.0029808544854 |
| XY arc [m] | 1.28328641025 | 1.14914729153 | 1.0496956602 |
| Min segment [m] | 0.0879136065011 | 0.0868889300738 | 0.0860169879751 |
| Max segment [m] | 0.158703673876 | 0.145519079262 | 0.14012399807 |
| Max lateral node correction [m] | 4.21884749358e-15 | 0.091331259144 | 0.18344836356 |
| Max yaw correction [rad] | 0 | 0.00692783040561 | 0.0138616293222 |
| Relative-edge translation RMS [m] | 7.94456339661e-18 | 0.02688986582 | 0.0537747539918 |
| Relative-edge translation max [m] | 1.17093834628e-17 | 0.0431093397027 | 0.0862091282896 |
| Relative-edge yaw RMS [rad] | 0 | 0.00233353962945 | 0.00467226847196 |
| Relative-edge yaw max [rad] | 0 | 0.00385211969993 | 0.00772482622027 |
| B-to-row0 position [m] | 0.394679042939 | 0.18453731616 | 0.0276158450726 |
| B-to-row0 abs yaw [rad] | 0.328639948005 | 0.323507705482 | 0.318373923404 |
| First chord vs B heading [rad] | -0.342947277512 | -0.349693146059 | -0.357183597821 |
| Self-intersection | False | False | False |
| Minimum reference clearance [m] | 0.918436077895 | 0.92603342673 | 0.883112147123 |
| Rigid-fit XY RMS [m] | 0 | 0.0554276814042 | 0.100071432295 |
| Rigid-fit yaw RMS [rad] | 0 | 0.150055416151 | 0.346186555433 |
| B-to-own-reference projection [m] | 0.16088422589 | 0.0733439275593 | 0.0276158450726 |
| B-to-own-reference projection abs yaw [rad] | 0.505674822974 | 0.380917361681 | 0.318373923404 |

| Original row | Half displacement from F [m] | Full displacement from F [m] |
|---|---|---|
| 0 | 0.210298207056 | 0.420594506982 |
| 1 | 0.204862789003 | 0.409724180983 |
| 2 | 0.189411793497 | 0.378823866367 |
| 3 | 0.161278080984 | 0.322558968593 |
| 4 | 0.121804017122 | 0.243611207386 |
| 5 | 0.0780153400557 | 0.156030763919 |
| 6 | 0.0404653809117 | 0.0809290069853 |
| 7 | 0.016053895317 | 0.0321064787136 |
| 8 | 0.00500920871036 | 0.0100180448472 |
| 9 | 0.00149046938397 | 0.0029808544854 |

Half has smaller first-node, endpoint, lateral and maximum-yaw correction, and smaller relative-edge translation RMS/max than Full in all four sources. Self-intersection is false for all twelve references. Reliable chords use the existing >=.02 m rule. Full per-node Log corrections and relative-edge vectors are retained in result_summary.json.

### Factor costs and solver

| Source | Condition | E_S (historical L) | E_R | E_A | Total |
|---|---|---|---|---|---|
| S1 | Half initial | 1.13777909328 | 2.60325514947e-33 | 2.54492466144e-28 | 1.13777909328 |
| S1 | Half final | 0.0871468901769 | 0.0171216375037 | 0.0870479171336 | 0.191316444814 |
| S1 | Full reused final | 0.348589942563 | 0.0684801182302 | 0.348193565422 | 0.765263626215 |
| S2 | Half initial | 6.0552322639 | 4.86733036862e-33 | 1.17122408325e-28 | 6.0552322639 |
| S2 | Half final | 0.463377016486 | 0.092389547085 | 0.463520356987 | 1.01928692056 |
| S2 | Full reused final | 1.85351556963 | 0.369540806859 | 1.8540873583 | 4.07714373479 |
| S3 | Half initial | 7.46533275622 | 4.53231642611e-33 | 2.71720285514e-28 | 7.46533275622 |
| S3 | Half final | 0.373170430971 | 0.103166691688 | 0.481245842868 | 0.957582965527 |
| S3 | Full reused final | 1.4903538263 | 0.414258049795 | 1.92117871423 | 3.82579059033 |
| S4 | Half initial | 4.54114156142 | 6.31160875628e-33 | 1.85441948672e-28 | 4.54114156142 |
| S4 | Half final | 0.312840294313 | 0.0724852505552 | 0.324201603758 | 0.709527148626 |
| S4 | Full reused final | 1.25137799441 | 0.289889056356 | 1.29682506554 | 2.8380921163 |

| Source | Half iterations | Accepted | Rejected | Unsafe rejected | Termination |
|---|---|---|---|---|---|
| S1 | 3 | 3 | 0 | 0 | cost_tolerance |
| S2 | 4 | 4 | 0 | 0 | cost_tolerance |
| S3 | 5 | 5 | 0 | 0 | cost_tolerance |
| S4 | 4 | 4 | 0 | 0 | cost_tolerance |

E_R is optimized in both Half and Full. Their state-shift targets differ, so a lower numerical objective across alpha values is not itself a better-execution result. All Half solves start at F and use the frozen solver configuration. Native needs no optimizer.

### Downstream and trajectory overlap

| Source | Max Half−Full reference XY gap [m] | Max execution XY gap [m] | Half endpoint error [m] | Half final original row | Half remaining arc [m] |
|---|---|---|---|---|---|
| S1 | 0.105572052388 | 0.0020146763465 | 0.0957029932029 | 9 | 0 |
| S2 | 0.243514467939 | 0.05999089732 | 0.0784311726707 | 9 | 0 |
| S3 | 0.272727738791 | 0.0417294982956 | 0.120314783299 | 9 | 0 |
| S4 | 0.210298137978 | 0.0342112874639 | 0.101967088518 | 9 | 0 |

All compared executions project to final original row 9 with zero remaining arc. This does not imply attachment: S3/S4 Half endpoint error remains .12031478329944348 / .10196708851798625 m. S4 Half minimum distance to the endpoint is .10096100345978497 m, still above .10 m. No attachment timestamp is assigned. The finite paths and final small angular commands do not demonstrate complete navigation success. S1 executions overlap strongly despite a .10557205238787344 m Half/Full reference gap: max execution gap is .002014676346499496 m. Markers and line styles distinguish these curves.

## Selector diagnostics

| Source | Comparison | First differing submit tick | Different H5 selections | Attachment differs | Both observed |
|---|---|---|---|---|---|
| S1 | Half vs Native | 96 | 7 | False | False |
| S1 | Half vs Full | 96 | 6 | True | True |
| S2 | Half vs Native | 1002 | 8 | True | True |
| S2 | Half vs Full | 1002 | 11 | True | True |
| S3 | Half vs Native | 1806 | 8 | False | False |
| S3 | Half vs Full | 1806 | 11 | False | False |
| S4 | Half vs Native | 1524 | 8 | True | True |
| S4 | Half vs Full | 1524 | 9 | False | False |

Every Half solve input pose, nearest original-row index, selected H5 rows, command, logical application tick and withheld flag is saved in selector_diagnostics.csv and JSON. The first difference is at the source's first new submit for both comparisons. S1/S2 Half-Full attachment changes coexist with selector changes; S3/S4 retain null attachment despite selector changes. This is interface sensitivity, not an identified selector cause.

## Cross-source analysis

| Quantity | Value |
|---|---|
| Half lower .9 s position AUC than Full | 4 |
| Half higher than Full | 0 |
| Native lower than both (S1, S4) | 2 |
| Safety preserved sources | 4 |
| Median Half−Full AUC [m s] | -0.00776305740523 |
| Median Half−Full endpoint [m] | -0.0133762417369 |
| Observed attachment Native / Half / Full | 3 / 2 / 2 |
| Null attachment Native / Half / Full | 1 / 2 / 2 |

| Source | Half−Full AUC .9 [m s] | Attachment delta [s] | Clearance delta [m] | Endpoint delta [m] |
|---|---|---|---|---|
| S1 | -0.000747521042958 | -0.0166666675359 | 0.000583398147955 | -0.000632468494103 |
| S2 | -0.00489074105217 | -0.166666675359 | -5.85937432129e-07 | -0.00766105167731 |
| S3 | -0.0106353737583 | N/A | -1.63961508981e-06 | -0.0190914317966 |
| S4 | -0.0128697115117 | N/A | -2.10691528457e-06 | -0.0303494726961 |

Half reduces .5 s maximum separation and .3 s position AUC relative to Full in all sources. However, Half increases .9 s yaw AUC relative to Full in all four. Its linear command TV falls in S2/S3, rises in S4, and differs only by about 1e-8 m/s in S1. These trade-offs remain separate; no scalar winner score is used.

## Research interpretation

The full transport target contributes to the observed original-FRESH position tracking loss in this benchmark: reducing its magnitude lowers early position AUC and endpoint error in every source without violating safety. This is an H1-compatible result in all four sources. It supports partial overcompensation of the spatial target, with different effect sizes.

The predeclared final label is nevertheless **H2: TRANSPORT_MAGNITUDE_INSUFFICIENT**, because Native remains better than both transported variants in S4, and only Native attaches there. S3 retains no attachment for any alpha. H3's predeclared AUC sign-reversal condition is absent: all Half−Full AUC deltas are negative. These facts distinguish evidence of overly strong transport from evidence that halving it solves attachment.

Full tracks its own modified reference with less position error than Half, yet has more error against original FRESH. Thus easier tracking of a transported reference need not mean better attachment to the original target. The geometric target and the controller interface both matter; unchanged official MPC computes actual v/omega commands.

### Explicit answers

1. **Does half transport improve the problematic S2/S3/S4 sources relative to Full?** Yes for original .9 s position AUC (−.0048907410521694394, −.010635373758282968, −.012869711511727 m s) and endpoint error. S2 attaches ten ticks earlier. S3/S4 still have no sustained attachment. Yaw AUC .9 increases.

2. **Does half transport preserve the OSA03 S1 benefit or degrade it?** Half improves over Full: lower .9 s AUC, one tick earlier attachment and .0005833981479546613 m more execution clearance. Half equals Native's attachment time but has slightly higher position AUC than Native (+0.0001508004970030008 m s). The reused Full itself had no earlier-attachment benefit over Native in S1; that premise is not assumed.

3. **Is execution performance monotonic from alpha=0 -> .5 -> 1?** There is no single monotonic performance relation. Position AUC rises with alpha in S1/S4, but Half has the lowest AUC in S2/S3. Endpoint error is lowest at Half in S1/S2 and at Native in S3/S4. S3 attachment is null throughout; S4 attaches only at alpha0. No continuous response curve or optimal alpha is inferred.

4. **Does Native remain better than both transported variants in any source?** Yes. Native has lower .9 s position AUC in S1 and S4. In S4 it also has lower endpoint error and is the only method with sustained attachment. This is metric-specific, not overall dominance: transported variants have lower S4 yaw AUC.

5. **Is safety preserved?** Yes for all 12 complete references and executions under the fixed checker/guard. No abort. Half−Full clearance is positive in S1 and lower by 0.59–2.11 micrometres in S2–S4, with all bounds well above .05 m.

6. **Does partial transport reduce reference deformation?** Yes. Half approximately halves first-node displacement and relative-edge translation RMS in each source. Endpoint/lateral/yaw corrections also fall. All references remain simple polylines with no self-intersection.

7. **Do selector row changes accompany execution differences?** Yes; Half differs from Native and Full at each source's first new submit tick (96,1002,1806,1524). Both attachment differences and unchanged/null attachment occur with selector changes. No selector causality is established.

8. **Is full BA^-1 transport supported, over-aggressive, or source-dependent?** The uniformly lower Half position AUC and endpoint error support full transport being overly aggressive for those metrics in this benchmark. The magnitude of the effect and Native comparisons depend on geometry. Higher Half yaw AUC prevents a blanket performance claim. A fixed full target is not supported as universally preferable.

9. **Is a scalar alpha sufficient, or should the next study move to correspondence-based attachment?** This test does not establish a sufficient scalar fix. S4 remains worse than Native and unattached at .5; S3 remains unattached for all three values. A subsequent spatial correspondence/attachment study is justified. It is not implemented here, and these three discrete values do not rule out every possible scalar formulation.

## Limitations

Four fixed development sources, no population inference or random sampling. Offline timing-controlled scheduler, not real asynchronous deployment latency. Original FRESH has intrinsic tracking difficulty, including S3 Native's null attachment. Only one frozen chunk is continued for 3 s; this can exceed its original online lifetime. No repeated FRESH updates, controller-aware factor or real robot. Relative-edge/progress diagnostics do not prove semantic intent. Only three fixed alpha values; tiny Native/Half differences and threshold-adjacent results should not be generalized. Cross-source counts summarize this benchmark only.

## Not demonstrated

Optimal alpha=.5, a final new method, general graph superiority, online latency improvement, real-world benefit, complete obstacle bypass/navigation, population effects, a correspondence solution, or elimination of original-FRESH attachment difficulty.

## PNG paths

- `/home/gpuadmin/Workspace/se-3-reconciliation/results/state_shift_transport_scale_01/figures/world_execution_overview.png`
- `/home/gpuadmin/Workspace/se-3-reconciliation/results/state_shift_transport_scale_01/figures/transport_scale_primary_metrics.png`
- `/home/gpuadmin/Workspace/se-3-reconciliation/results/state_shift_transport_scale_01/figures/transport_scale_deltas.png`
- `/home/gpuadmin/Workspace/se-3-reconciliation/results/state_shift_transport_scale_01/figures/reference_transport_overview.png`

Exactly four final PNGs. All four were visually inspected; axes, labels, nulls, cart bounds and overlapping-curve markers are readable. No extra selector or diagnostic PNG, and no post-result plot-code modification or scientific rerun.

## Completion audit

Protocol deviations: **none**. No result-driven changes, extra repetitions, source substitution, Native/Full/Taper rerun, external code change or environment modification. The conservative overlapping-hypothesis precedence was explicit in the pushed protocol. Initial status and user Stage0/GPU-script changes were preserved. Final result/docs commit is the Git commit containing this completed report; its exact SHA is provided in the completion message.

New scientific optimizer call count is supported by four exclusive optimization_start records (each calls=1, alpha=.5) and four execution_start records. All carry the pushed freeze SHA. New MPC counts are independently recomputed from saved solve events; validators and reporters make zero scientific calls. Source/worker code hashes and historical ledgers remain unchanged.

## Reproduction commands

Commands run from `/home/gpuadmin/Workspace/se-3-reconciliation` using the existing
repository environment. Official MPC uses its existing external environment without
modification. All new scientific work occurs only in the single `execute` command
following the pushed freeze.

```bash
git fetch origin main
git status --short --branch
git remote -v
git log -1 --format=fuller
git rev-parse origin/main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/transport-scale-mpl .venv/bin/python -m pytest -q tests/test_state_shift_transport_scale01.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/transport-scale-mpl .venv/bin/python -m pytest -q tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z --mode freeze
git diff --cached --check
git commit -m "Freeze four-source half-transport diagnostic with authenticated historical reuse"
git push origin main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/transport-scale-mpl .venv/bin/python scripts/report_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z
```

Preparation also ran direct saved-only historical ledger/validator authentication
before implementation. No numerical solver or controller was called by that audit.

### Pre-execution verification

New focused tests: **25 passed (29.30 s)**. Relevant regression including historical
Local-SE2/common-B/multisource/R00/R01, frame, safety, controller and timing tests:
**492 passed, 1 skipped (91.36 s)**. The skip is absent ignored historical
EXP-01B/EXP-02B corpus. Compileall and diff checks pass. Preparation and historical
saved validation made zero new scientific calls. No scientific outcome exists yet.

### Final saved-artifact verification

Final saved-only revalidation passes, including original multisource/R00/R01
validators. The independent artifact audit authenticated all 118 sealed result
files, 12 primary CSV rows, four comparison rows, 360 selector rows and exact
PNG numeric sidecars. All four PNG format/dimension/hash checks pass; dimensions
are 2700×2160, 2700×1620, 2520×1620 and 2700×2160 in the listed order.
Audit: `results/state_shift_transport_scale_01/final_artifact_validation.json`.
Final compileall and diff check pass. Frozen scientific code is unchanged, so the
pre-execution 492-pass regression remains the tested implementation.

The final documentation/numeric artifact audit used saved data only:
`OPENBLAS_NUM_THREADS=1 .venv/bin/python /tmp/audit_transport_outputs.py`.
The transient audit checks the same saved validators plus CSV/PNG sidecar and
exclusive-start counts; its complete findings are retained in the tracked JSON.

Final publication commands:

```bash
git add README.md docs/STATE_SHIFT_TRANSPORT_SCALE_01.md docs/WORK_LOG.md results/state_shift_transport_scale_01
git diff --cached --check
git diff --cached --stat
git commit -m "Report four-source transport-scale diagnostic and four validated PNGs"
git push origin main
git rev-parse HEAD
git status --short --branch
```
