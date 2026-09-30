# RELATIVE_FACTOR_MULTISOURCE_01

## 1. Repository-confirmed facts

Starting fetched HEAD and origin/main: `6f31477598514738d72006d47fe04e112b3d3a28`.
This is a **timing-controlled offline causal reference comparison** of four
trajectory-distinct saved sources. The source inventory contains 13 qualified
handoffs and 10 distinct raw FRESH arrays. Four sources passed exact official
controller installation/restoration preflight, with **zero numerical MPC calls**.
No new scientific planning or rollout has occurred at this protocol freeze.

Run: `data/relative_factor_multisource_01/primary_20260930T033000Z/`.
Directory identifiers are fixed labels, not claimed execution timestamps; actual
UTC/host and logical simulation times are saved separately. Large raw/derived
arrays and rollout logs remain under ignored data. The compact PNGs and numeric
summaries are tracked. The unrelated Stage0 config edits and GPU plot script are
preserved. Historical R00/R01 scientific artifacts and implementation bytes remain
unchanged and their saved-only validators must still pass.

### Completed frozen experiment

**MIXED_EVIDENCE.** Scientific freeze `21f029fabfb0faaa00bbb6d5930460671f63e267` was committed
and pushed before execution. Exactly 8 planning calls (4 Full + 4 No-relative),
16 rollouts and 480 new official MPC solves (30 per rollout). LightNav=0, RGB=0,
Isaac=0, GP=0. Retries=0, source replacements=0, reference-unsafe=0, execution
aborts=0, controller failures=0, planning failures=0. All 16 runs completed 180
intervals. All 4 primary and full-window common-state/schedule gates passed.
Minimum complete-reference clearance across all conditions: 0.1313492849344136 m.
Minimum execution swept clearance lower bound: 0.1335610944454762 m.

No-relative versus Full: .9 s position AUC decreased in 1/4 sources and increased
in 3/4. Attachment was 0.05000000260770321 s earlier in S1 and
0.10000000521540642 s later in S2. Both methods had null attachment in S3 and S4.
Relative-edge translation RMS and maximum increased in 4/4; linear command TV
increased in 3/4. Full/No-relative safety passed in 4/4. Exact values are retained
in the CSV/JSON outputs; displayed tables round only for readability.

## 2. Selected sources and distinct raw local FRESH proof

Selection uses the existing qualified inventory and original local path geometry,
not new planning/execution outcomes. OSA03 R00 is the sole obstacle anchor; R01 is
excluded because its raw FRESH equals R00. The other three were chosen to contrast
an almost straight rightward path with progressive left and right turns. Their
original chord-direction ranges are approximately 0.656, 42.090, and 52.390 degrees.
This is a purposive small benchmark, not a random or held-out population sample.
All original rows are retained. No source is selected by a new method score.

| Source | Role | Raw .npy SHA256 |
|---|---|---|
| OSA03_R00 | obstacle anchor | `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521` |
| episode_001_repeat_01/handoff_013 | mild right / nearly straight | `82bb298736fd7a1e6289a529e0a973d4def6b3ed6b00b03506ac3694833f3503` |
| episode_008_repeat_01/handoff_023 | progressive left turn | `ba52161d8e7ba1f0619e11cd0b93da29d201e70074d76160d374fe1f6c87517f` |
| episode_013_repeat_00/handoff_020 | progressive right turn | `f5585ef9f7ff2eeeae9992640a68cf65b6b8510baacf8ff9fd2886aa40df7552` |

All four file hashes differ. Canonical float64 array-value hashes also differ,
excluding duplicate values hidden by different file serialization:

- OSA03_R00: `c93deb81f79ac852c0ec98105326dca345871f966df1e4b9190082c19818b010`
- episode_001_repeat_01/handoff_013: `11fa7529bd5b7c1a998953106f33aaa85d74860ed74ee5b60d73535ab2556fa0`
- episode_008_repeat_01/handoff_023: `aca940cd63383e1c1f272cef370635a98ab9b60470fa7dcc55007a29f230bbf6`
- episode_013_repeat_00/handoff_020: `1f10989b3af59227172500c3989076ff71bbef95e3266881d55b23acd2692cf9`

The inventory ledger and its source manifest, validation, protocol and summary
are pinned in the config. Selected context/raw/world/execution/commands/events
are authenticated against the saved inventory manifest. Episode metadata,
Hospital export files, official MPC and every preparation input are frozen by
hash. OSA03 source, cart and original M0 schedule use the existing authenticated
R00 chain. Full paths and hashes live in the tracked freeze ledger.

All four selected sources are compatible before freeze, so no replacement is
needed or scheduled. A later protocol failure is recorded explicitly; it cannot
trigger a search based on results or a silent method/source substitution. If fewer
than three causally usable distinct sources remain, the benchmark is technically
blocked; no population comparison is inferred from the reduced set.

## 3. Protocol / common-state validity

### Spatial formulation

A and B are fixed. The **observation-to-application state-shift / transport factor**
moves editable early FRESH nodes X_j toward `Ftilde_j = B A^-1 F_j`, equivalently
encouraging `B^-1 X_j ≈ A^-1 F_j`. It does not move A to B.

`F_j = A F_local,j`. World XY is metres, +Z up, yaw radians CCW; local +x is forward
and +y left. Raw outputs remain immutable and observation-anchored. Derived local
references use `A^-1 X`; no raw B re-anchoring. Historical L keys mean S here.

```
r_S,j = Log(Ftilde_j^-1 X_j)
r_R,j = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A,j = Log(F_j^-1 X_j)
s_j = cumulative original XY arc / total original XY arc
w_S = (1-s)^2; w_A = s^2
normalization = diag(0.10 m, 0.10 m, 10 degrees)
```

Reuse the current weighted node means and relative-edge mean exactly. Native=F;
Taper=`Exp((1-s) Log(B A^-1)) F`; Full minimizes E_S+E_R+E_A; No-relative minimizes
E_S+E_A. R is absent from No-relative's optimized vector and cost, but evaluated
post hoc as **diagnostic relative-edge distortion, not optimized cost**.
There are no waypoint clocks, GP, twist, controller/velocity/omega/correspondence,
obstacle or replacement smoothness factors. Spatial yaw deformation changes the
reference; the unchanged official MPC decides actual omega(t).

Both optimized methods start at original FRESH; exactly one call each per source.
Right-local SE(2) LM: 80 max iterations, central FD 1e-6, damping .001, rejection
x10 / acceptance x.3, max damping 1e12, gradient/step tolerances 1e-9, cost tolerance
1e-12. No tuning, retries, alternative initialization, repair or result selection.
Nonconverged/failed planning conditions are recorded and skipped.

### Safety

Use complete returned polylines, radius .20 m and required edge clearance .05 m,
with the existing workspace and geometry numerical reserve. No B connector,
resampling, crop, clipping or obstacle objective. Initial and improving candidate
states pass the unchanged Hospital/cart checker; an accepted step must lower the
optimized objective and be feasible. Final whole references get an independent
GEOS union query. In the cart-free Hospital export, union-versus-nearest-part
rounding can differ by nanometres; their difference is recorded and bounded by
the **existing** 1e-7 m geometry reserve, and both must agree on acceptance. This
does not change either threshold or the optimizer's check. OSA03 retains its
historical independent checker unchanged. Unsafe references are not executed.

Execution preserves the original abort-only guard: new commands preview .1 s,
held commands preview the next source dt. An unsafe command is never applied.
Keep the valid prefix and null/censored metrics; no fabricated continuation.

### Logical schedule and exact controller state

Every method restores its source's exact B pose/tick/time, physical u_minus,
already-applied u_B_plus, actual official previous_control at the B host cut,
generation, reference version/chunk, prior command application time and first
FRESH result diagnostics. Memory is authenticated separately from physical u_minus;
it already equals the first FRESH result in all selected sources. No pending
pre-B solve is discarded. A numerical-solve-forbidden official restoration check
passes for Native/Taper before the scientific freeze.

| Source | B tick | B simulation time s | Next submit | Release schedule |
|---|---:|---:|---:|---|
| OSA03_R00 | 92 | 1.5666667483747 | 96 | exact frozen common-B M0 |
| episode_001_repeat_01/handoff_013 | 997 | 16.6500008683652 | 1002 | submit + 1 tick |
| episode_008_repeat_01/handoff_023 | 1804 | 30.1000015698373 | 1806 | submit + 1 tick |
| episode_013_repeat_00/handoff_020 | 1524 | 25.433334659785 | 1524 | submit + 1 tick |

For OSA03 use the complete previously frozen M0 schedule. For genuine sources,
keep the original absolute six-tick control grid and freeze the completion lag
from the **first recorded successful submit/application at or after B**. Repeat
that authenticated lag on the grid for the bounded window. This is a declared
logical schedule, not a replay of every historical host-dependent delay. At
handoff_020, B itself is a scheduled submit tick: the new solve reads B after the
shared first command has been established, and its result is held to B+1. The
shared first command still integrates the first interval. Other B phases wait
until their own next grid tick. No absolute tick is forced across sources.

At each submit the simulation state and clock stay fixed while the unchanged
official worker finishes and polls. Results release only at predetermined ticks;
every release precedes the next submit. Thus official memory updates occur before
the next solve exactly as required. No future state enters the solve. The existing
worker/activation/generation checks and command computation are reused. No MPC
objective, H5 horizon, nearest+1 selector, Q/R, limits or external source changes.
A bounded copy of the old runtime changes only geometry/scenario lookup and the
hardcoded next-tick assertion; a test verifies precisely those differences.

All methods have a 180-interval cap (3.0000001564621925 s) with source float32
1/60 dt. Original later chunks are excluded by this frozen-reference offline
counterfactual; this window can extend beyond the recorded original FRESH lifetime.
No future command or state is replayed. This is not repeated online updating.
A solve collected on the final interval can remain withheld at the cap; count
that numerical solve and never fabricate an application at the excluded endpoint.

Require exact attempted submits, accepted submits, successful applications,
54 primary integration intervals and initial B/command/memory provenance within
each source. Also report full-window parity. Failed gates are schedule-control
failures, excluded from reference-causal claims; preserve scientific safety causes.
No wall-time/latency claim follows from this scheduler.

### Evaluation and frozen interpretation

Primary metrics always use ORIGINAL FRESH and the unchanged forward-only continuous
projection/shortest-angle evaluator. Attachment needs <=.10 m and <=15 degrees
through the entire following .30 s sampled interval. Null remains null. Report all
18 requested primary fields, endpoint/progress diagnostics, factor costs, solver
trace/acceptance/unsafe rejection counts and planning geometry. Own-reference
.3/.9 position/yaw AUC and .5 s max position error are secondary and separate.
For cart-free sources, the legacy evaluator's cart-axis descriptors use B heading
as a documented diagnostic axis; they are not interpreted as cart distances.

Report each No-relative-minus-Full signed delta, sign counts, medians and reversed
sources for all 13 requested contrasts. Numerical zero tolerance is 1e-9 in each
reported unit; a missing attachment contributes neither earlier nor equal. No
single score. Predeclare:

- CONSISTENT_MULTISOURCE_PATTERN: strict majority of all selected sources has
  observed earlier/equal attachment, nonworse values for all three early position
  metrics, preserved Full/No-relative safety, and larger relative-edge RMS/max.
- MIXED_EVIDENCE: at least one supportive early/attachment change and at least one
  opposing change across sources takes precedence; otherwise partial support.
- NO_SUPPORT_FOR_PATTERN: no observed early/attachment improvement and the
  consistent-pattern conditions are not met.
- TECHNICAL_BLOCKED: source, schedule, controller, validator or protocol failure;
  fewer than three usable distinct sources cannot support the benchmark.

Safety/shape failure itself is a scientific outcome, not repaired or relabeled as
optimizer superiority. A lower planning objective is not better execution.

### Freeze, tests and calls

Prepare/authenticate inputs and references without planning/MPC solves. Tests use
synthetic solves/workers only. Review diff, commit and normal push implementation,
config, tests and this protocol before 8 scientific planning calls and up to 16
rollouts. Within each source solve Full then No-relative once, then execute Native,
Taper, Full, No-relative once when eligible. Validators/reports are saved-only.
No LightNav/RGB/Isaac/GP/live-source/extra controller calls. No external environment
or system Python changes. All old validators must pass unchanged.

Preparation development used `primary_20260930T030000Z` once and found the strict
union/parts numerical equality assumption (4.03e-9 m discrepancy on Taper). It made
zero scientific calls. The corrected preparation uses the existing geometry
reserve and is preserved separately at `primary_20260930T033000Z`. This is a
pre-freeze checker validation correction, not a scientific retry.

## 4. Per-source results

### Primary execution against ORIGINAL FRESH

Source codes below match all PNGs:

| Code | Source |
|---|---|
| S1 | OSA03_R00 |
| S2 | episode_001_repeat_01/handoff_013 |
| S3 | episode_008_repeat_01/handoff_023 |
| S4 | episode_013_repeat_00/handoff_020 |

All 18 requested fields, including initial errors, growth, .3/.9 yaw AUC,
attachment fractional row/arc and remaining arc, max omega and termination, are
in [primary.csv](../results/relative_factor_multisource_01/primary.csv).
The [full summary](../results/relative_factor_multisource_01/result_summary.json)
contains planning, command, endpoint, selector, source and validation records.
Every method termination is `OBSERVATION_CAP`. Null attachment means no complete
following .30 s dwell was observed in the 3.0000001564621925 s window.

| Source | Method | Position AUC .9 [m s] | Attachment [s] | Swept clearance [m] | Endpoint [m] | Linear TV [m/s] | Angular TV [rad/s] |
|---|---|---|---|---|---|---|---|
| S1 | Native | 0.169065633 | 1.46666674 | 0.133561094 | 0.0964390214 | 0.8 | 3.62955577 |
| S1 | Taper | 0.166853435 | 1.50000008 | 0.134208269 | 0.0959817055 | 1.19999997 | 3.51359993 |
| S1 | Full | 0.169963954 | 1.48333341 | 0.133709423 | 0.0963354617 | 0.799999999 | 3.53910893 |
| S1 | No-relative | 0.166121029 | 1.43333341 | 0.138784907 | 0.0912986256 | 1.19999841 | 3.50776936 |
| S2 | Native | 0.131833111 | 1.05000005 | 0.757224041 | 0.0793823671 | 0.806041069 | 2.54662624 |
| S2 | Taper | 0.136818958 | 1.16666673 | 0.75722543 | 0.0869773744 | 1.09539864 | 2.48028093 |
| S2 | Full | 0.13668227 | 1.20000006 | 0.75722543 | 0.0860922243 | 1.38978624 | 2.56885016 |
| S2 | No-relative | 0.137916228 | 1.30000007 | 0.75722543 | 0.0861923882 | 1.80559753 | 2.63360384 |
| S3 | Native | 0.189624122 | null | 0.380788848 | 0.115238019 | 1.30641156 | 4.0914333 |
| S3 | Taper | 0.194229742 | null | 0.380804182 | 0.13300318 | 1.61209852 | 3.35597299 |
| S3 | Full | 0.199963659 | null | 0.380796174 | 0.139406215 | 2.10641152 | 3.57735057 |
| S3 | No-relative | 0.200154265 | null | 0.380796258 | 0.138730909 | 2.50641143 | 3.56456155 |
| S4 | Native | 0.198189216 | 1.2833334 | 0.870803818 | 0.0845393137 | 1.45352967 | 4.82596744 |
| S4 | Taper | 0.210520128 | null | 0.870583704 | 0.12756008 | 1.48063876 | 4.28730669 |
| S4 | Full | 0.217335047 | null | 0.870583704 | 0.132316561 | 1.46221189 | 4.39497273 |
| S4 | No-relative | 0.217494672 | null | 0.870583704 | 0.129789289 | 1.43994501 | 4.46421953 |

S3 has no sustained attachment for any method. S4 Native attaches at
1.2833334002643824 s, while Taper, Full and No-relative do not. These are retained
as null outcomes; they are neither ties nor zero-time attachments. Native has
the lowest .9 s position AUC in S2, S3 and S4.

### Secondary tracking against each method's OWN reference

These values do not replace any original-FRESH primary metric.

| Source | Method | Position AUC .3 | Position AUC .9 | Yaw AUC .3 | Yaw AUC .9 | Max position .5 [m] |
|---|---|---|---|---|---|---|
| S1 | Native | 0.0479203623 | 0.169065633 | 0.122763625 | 0.193368049 | 0.215262377 |
| S1 | Taper | 0.0182246627 | 0.0973262134 | 0.102281575 | 0.167395814 | 0.135741566 |
| S1 | Full | 0.0169872144 | 0.095301244 | 0.101992576 | 0.170848314 | 0.13009601 |
| S1 | No-relative | 0.0158859297 | 0.0878737967 | 0.100640286 | 0.169780498 | 0.118106356 |
| S2 | Native | 0.0453024912 | 0.131833111 | 0.0456241197 | 0.107103978 | 0.163894949 |
| S2 | Taper | 0.0200307326 | 0.0621507351 | 0.0469353525 | 0.0960825051 | 0.150788706 |
| S2 | Full | 0.0189550084 | 0.0540505707 | 0.0473330586 | 0.0963629009 | 0.145846481 |
| S2 | No-relative | 0.0194715651 | 0.0508664325 | 0.0470415422 | 0.0952943797 | 0.150788706 |
| S3 | Native | 0.0643635366 | 0.189624122 | 0.0761905902 | 0.175616043 | 0.239119858 |
| S3 | Taper | 0.0100128854 | 0.0649870071 | 0.0492886602 | 0.140237054 | 0.0843246437 |
| S3 | Full | 0.00769811917 | 0.0548568536 | 0.0504612052 | 0.14831549 | 0.055031746 |
| S3 | No-relative | 0.00729851608 | 0.0499908902 | 0.0496030556 | 0.159739841 | 0.0441771747 |
| S4 | Native | 0.061198617 | 0.198189216 | 0.120660784 | 0.230599468 | 0.252791514 |
| S4 | Taper | 0.0113343759 | 0.0620811993 | 0.0538991696 | 0.185901243 | 0.0852659028 |
| S4 | Full | 0.00917231382 | 0.0523258723 | 0.0535085104 | 0.190065469 | 0.0589708885 |
| S4 | No-relative | 0.00857747303 | 0.0481713443 | 0.0520536436 | 0.203997686 | 0.0486753646 |

### Planning geometry

The complete per-node corrections, relative-edge SE(2) log vectors, B projection,
rigid-fit transform and all raw diagnostics are in `summary.sources.*.planning`
in the full JSON. Tables use metres and radians unless stated otherwise. No
self-intersection was observed in any reference. Native has zero correction.

| Source | Method | First correction | End correction | XY arc | Segment min | Segment max | Lateral correction max | Yaw correction max [rad] | Ref clearance |
|---|---|---|---|---|---|---|---|---|---|
| S1 | Native | 0 | 0 | 1.35324552 | 0.149753631 | 0.150770002 | 1.77635684e-15 | 0 | 0.22948689 |
| S1 | Taper | 0.213333346 | 0 | 1.17331165 | 0.12983973 | 0.130724154 | 0.0947857219 | 7.73821327e-07 | 0.22948689 |
| S1 | Full | 0.211144018 | 0.00218683149 | 1.1788003 | 0.116958805 | 0.145860166 | 0.102800304 | 0.00595845238 | 0.227809208 |
| S1 | No-relative | 0.213333346 | 0 | 1.17616891 | 0.112274799 | 0.147912381 | 0.105021842 | 7.73821324e-07 | 0.22948689 |
| S2 | Native | 0 | 0 | 1.35476699 | 0.150167768 | 0.150789196 | 1.77635684e-15 | 0 | 0.580552286 |
| S2 | Taper | 0.492132911 | 0 | 0.888285176 | 0.0984920836 | 0.09891269 | 0.127673734 | 0.000198744114 | 0.892054745 |
| S2 | Full | 0.487028826 | 0.00509739568 | 0.90439144 | 0.0646789036 | 0.138149307 | 0.1264174 | 0.00704995823 | 0.888094511 |
| S2 | No-relative | 0.492132911 | 0 | 0.89982844 | 0.053961449 | 0.143447546 | 0.127673734 | 0.000198744114 | 0.892054745 |
| S3 | Native | 0 | 0 | 1.26133848 | 0.0476164233 | 0.171384611 | 3.55271368e-15 | 0 | 0.131349285 |
| S3 | Taper | 0.549726637 | 0 | 0.851430268 | 0.0352589711 | 0.121389519 | 0.115376571 | 0.24435031 | 0.383003823 |
| S3 | Full | 0.544455992 | 0.00229966565 | 0.880600136 | 0.0431236822 | 0.152467653 | 0.122413918 | 0.243485753 | 0.369372222 |
| S3 | No-relative | 0.549726637 | 0 | 0.889400352 | 0.0468630163 | 0.157896267 | 0.123649476 | 0.24435031 | 0.351300915 |
| S4 | Native | 0 | 0 | 1.28328641 | 0.0879136065 | 0.158703674 | 4.21884749e-15 | 0 | 0.918436078 |
| S4 | Taper | 0.424775099 | 0 | 1.02852363 | 0.0839540281 | 0.133938999 | 0.155440494 | 0.0118243929 | 0.882598752 |
| S4 | Full | 0.420594507 | 0.00298085449 | 1.04969566 | 0.086016988 | 0.140123998 | 0.183448364 | 0.0138616293 | 0.883112147 |
| S4 | No-relative | 0.424775099 | 0 | 1.05652051 | 0.0871464836 | 0.144950013 | 0.192123324 | 0.0118243929 | 0.882598752 |

| Source | Method | Edge XY RMS | Edge XY max | Edge yaw RMS | Edge yaw max | Rigid-fit XY RMS | Rigid-fit yaw RMS |
|---|---|---|---|---|---|---|---|
| S1 | Native | 5.10221045e-18 | 9.5598912e-18 | 0 | 0 | 0 | 0 |
| S1 | Taper | 0.0237038046 | 0.0237682695 | 8.59803911e-08 | 8.62142464e-08 | 0.0574113625 | 0.090991645 |
| S1 | Full | 0.0261585101 | 0.0402608377 | 0.00127484607 | 0.00173287142 | 0.0668348029 | 0.103853184 |
| S1 | No-relative | 0.0283785142 | 0.0468352699 | 1.02753627e-07 | 1.68951749e-07 | 0.0706015114 | 0.112907833 |
| S2 | Native | 6.97662552e-18 | 1.34043551e-17 | 0 | 0 | 0 | 0 |
| S2 | Taper | 0.0546857714 | 0.0547774419 | 2.20826942e-05 | 2.21207524e-05 | 0.148801845 | 0.143481271 |
| S2 | Full | 0.0607835216 | 0.0934847528 | 0.00153308543 | 0.00207973166 | 0.172469558 | 0.174668771 |
| S2 | No-relative | 0.0654506391 | 0.107968612 | 2.61862144e-05 | 4.23408753e-05 | 0.181853585 | 0.192813111 |
| S3 | Native | 6.73224808e-18 | 1.2822954e-17 | 0 | 0 | 0 | 0 |
| S3 | Taper | 0.0559077145 | 0.0709188183 | 0.0281519417 | 0.0332011459 | 0.132960507 | 0.28048662 |
| S3 | Full | 0.0616579691 | 0.0992247285 | 0.0322236821 | 0.0524884602 | 0.149354454 | 0.328306968 |
| S3 | No-relative | 0.0681723797 | 0.118839681 | 0.0344501618 | 0.0593422347 | 0.158307455 | 0.358772154 |
| S4 | Native | 7.9445634e-18 | 1.17093835e-17 | 0 | 0 | 0 | 0 |
| S4 | Taper | 0.0483165107 | 0.0531141671 | 0.00133120888 | 0.00146231939 | 0.0887471705 | 0.300894654 |
| S4 | Full | 0.053774754 | 0.0862091283 | 0.00467226847 | 0.00772482622 | 0.100071432 | 0.346186555 |
| S4 | No-relative | 0.0597499849 | 0.10463144 | 0.00163346907 | 0.00281927994 | 0.105716749 | 0.377513199 |

| Source | Method | B lateral extent max | B-row0 distance | B-row0 yaw | Nearest distance | Nearest yaw | First chord vs B [rad] |
|---|---|---|---|---|---|---|---|
| S1 | Native | 0.676245107 | 0.213324006 | 0.314173548 | 0.106648569 | 0.523608488 | 0.523467219 |
| S1 | Taper | 0.676245107 | 1.09647669e-05 | 0.314172775 | 1.09647669e-05 | 0.314172775 | 0.614480237 |
| S1 | Full | 0.67630193 | 0.0021805014 | 0.31505699 | 0.00108578873 | 0.317785491 | 0.542794471 |
| S1 | No-relative | 0.676245107 | 1.09646996e-05 | 0.314172775 | 1.09646996e-05 | 0.314172775 | 0.534600214 |
| S2 | Native | 0.387315118 | 0.348704612 | 0.261999451 | 0.127644309 | 0.261793531 | -0.261954611 |
| S2 | Taper | 0.387315118 | 0.150788706 | 0.261800706 | 0.150788706 | 0.261800706 | -0.405872157 |
| S2 | Full | 0.387385993 | 0.145846481 | 0.262865334 | 0.145846481 | 0.262865334 | -0.286167925 |
| S2 | No-relative | 0.387315118 | 0.150788706 | 0.261800706 | 0.150788706 | 0.261800706 | -0.275481914 |
| S3 | Native | 0.653196341 | 0.514561811 | 0.0591813073 | 0.18468319 | 0.385685031 | 0.132130435 |
| S3 | Taper | 0.653196341 | 0.0372311237 | 0.303531617 | 0.0372311237 | 0.303531617 | 0.599764897 |
| S3 | Full | 0.654163178 | 0.0320129219 | 0.30266706 | 0.0320129219 | 0.30266706 | 0.410285147 |
| S3 | No-relative | 0.653196341 | 0.0372311237 | 0.303531617 | 0.0372311237 | 0.303531617 | 0.401404094 |
| S4 | Native | 0.852655649 | 0.394679043 | 0.328639948 | 0.160884226 | 0.505674823 | -0.342947278 |
| S4 | Taper | 0.852655649 | 0.0316079107 | 0.316815555 | 0.0316079107 | 0.316815555 | -0.486980427 |
| S4 | Full | 0.852670394 | 0.0276158451 | 0.318373923 | 0.0276158451 | 0.318373923 | -0.357183598 |
| S4 | No-relative | 0.852655649 | 0.0316079107 | 0.316815555 | 0.0316079107 | 0.316815555 | -0.34511281 |

First reliable chord uses the unchanged >=.02 m span rule.

Per-node XY correction from original FRESH, S1 [m]:

| Method | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| Taper | 0.213333346 | 0.189567298 | 0.165842403 | 0.142099432 | 0.118331201 | 0.0946243447 | 0.0709898711 | 0.0473454093 | 0.0237373537 | 0 |
| Full | 0.211144018 | 0.205470712 | 0.19033728 | 0.163595912 | 0.126830671 | 0.0861257154 | 0.0495176943 | 0.0229057909 | 0.00784166016 | 0.00218683149 |
| No-relative | 0.213333346 | 0.210039074 | 0.19719692 | 0.170562279 | 0.129827873 | 0.082992638 | 0.0425644766 | 0.0160821223 | 0.00329926417 | 0 |

Per-node XY correction from original FRESH, S2 [m]:

| Method | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| Taper | 0.492132911 | 0.437371453 | 0.382600919 | 0.327951783 | 0.273403526 | 0.218757908 | 0.164079558 | 0.109383347 | 0.0546823386 | 0 |
| Full | 0.487028826 | 0.473934414 | 0.439008882 | 0.377459557 | 0.29298014 | 0.199221489 | 0.114640301 | 0.0530645231 | 0.0181731226 | 0.00509739568 |
| No-relative | 0.492132911 | 0.484546085 | 0.454874775 | 0.393542269 | 0.300114396 | 0.192150483 | 0.0985011086 | 0.0371690436 | 0.00757387728 | 0 |

Per-node XY correction from original FRESH, S3 [m]:

| Method | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| Taper | 0.549726637 | 0.465643525 | 0.392713421 | 0.321495207 | 0.249597705 | 0.180255926 | 0.110663069 | 0.0500815157 | 0.0158737628 | 0 |
| Full | 0.544455992 | 0.51586098 | 0.465097868 | 0.384614236 | 0.278652117 | 0.168584474 | 0.0783012175 | 0.0265351579 | 0.00742515143 | 0.00229966565 |
| No-relative | 0.549726637 | 0.526646527 | 0.481686556 | 0.402084439 | 0.283579042 | 0.155029978 | 0.0539828527 | 0.00939604513 | 0.000837559832 | 0 |

Per-node XY correction from original FRESH, S4 [m]:

| Method | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| Taper | 0.424775099 | 0.375328205 | 0.324670012 | 0.274341761 | 0.223368247 | 0.170945838 | 0.118765277 | 0.0679144387 | 0.0297163928 | 0 |
| Full | 0.420594507 | 0.409724181 | 0.378823866 | 0.322558969 | 0.243611207 | 0.156030764 | 0.080929007 | 0.0321064787 | 0.0100180448 | 0.00298085449 |
| No-relative | 0.424775099 | 0.419027859 | 0.393464159 | 0.338045955 | 0.249756045 | 0.145897333 | 0.0627102488 | 0.0169662167 | 0.00274353598 | 0 |

### Factor costs and solver trace

L is reported as E_S. The No-relative diagnostic R below was evaluated after
optimization and **is not part of its optimized cost**. Totals with different
factor sets are not execution-performance scores.

| Source | Method | E_S | E_R optimized | E_A | Optimized total | R diagnostic only | Iterations | Accepted | Rejected | Unsafe rejected | Termination |
|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | Full | 0.348589943 | 0.0684801182 | 0.348193565 | 0.765263626 | null | 5 | 4 | 1 | 0 | cost_tolerance |
| S1 | No-relative | 0.345923097 | null | 0.345422957 | 0.691346054 | 0.0805340071 | 3 | 2 | 0 | 0 | step_tolerance |
| S2 | Full | 1.85351557 | 0.369540807 | 1.85408736 | 4.07714373 | null | 5 | 5 | 0 | 0 | cost_tolerance |
| S2 | No-relative | 1.84000023 | null | 1.84097911 | 3.68097934 | 0.428378639 | 3 | 3 | 0 | 0 | cost_tolerance |
| S3 | Full | 1.49035383 | 0.41425805 | 1.92117871 | 3.82579059 | null | 6 | 6 | 0 | 0 | cost_tolerance |
| S3 | No-relative | 1.48526817 | null | 1.8871696 | 3.37243777 | 0.503708129 | 5 | 5 | 0 | 0 | cost_tolerance |
| S4 | Full | 1.25137799 | 0.289889056 | 1.29682507 | 2.83809212 | null | 5 | 5 | 0 | 0 | cost_tolerance |
| S4 | No-relative | 1.23217739 | null | 1.28658378 | 2.51876117 | 0.357093662 | 5 | 4 | 1 | 0 | cost_tolerance |

### Downstream and overlap diagnostics

All 16 executions reach final forward-only projected row 9 / arc fraction 1 and
remaining original arc 0. This is a projection/progress proxy, not proof of pose
attachment or semantic intent. Full and No-relative stop outside the position
tube in S3/S4 despite that progress. No method has a near-zero command throughout
the final .30 s under the existing diagnostic threshold; this is not an abort.
Full/No-relative endpoint errors improve with R removal in S1, S3, S4 and worsen
by 0.0001001638442294922 m in S2.

| Source | NoR/Full max ref gap [m] | NoR/Full max exec gap [m] | Full/Taper max ref gap [m] | Full/Taper max exec gap [m] | Full/NoR different H5 submit ticks |
|---|---|---|---|---|---|
| S1 | 0.00696725455 | 0.0199811768 | 0.0244950798 | 0.019981255 | 102, 132 |
| S2 | 0.0161396882 | 0.0789053412 | 0.0564081375 | 0.0439584147 | 1032, 1050, 1056, 1062, 1068 |
| S3 | 0.0243860819 | 0.0560743958 | 0.0731310619 | 0.0316451922 | 1854, 1860, 1866 |
| S4 | 0.0182230088 | 0.00261783808 | 0.0541578069 | 0.0161354748 | 1590 |

Curves overlap strongly in several panels; the numeric gaps above and staggered
markers/line styles expose this. The official selector is unchanged. Observed row
selection changes are diagnostics, not a new controller or an independent causal
isolation of the selector. Full versus Taper remains numerically distinguishable,
with no uniform execution advantage for Full.


## 5. Cross-source signed-delta summary

Signed delta is **No-relative minus Full**. The exact 13-contrast per-source table
is [signed_gaps.csv](../results/relative_factor_multisource_01/signed_gaps.csv);
[sign counts and medians](../results/relative_factor_multisource_01/cross_source_signs.csv)
include explicit unavailable counts. No scalar score is computed.

| Source | AUC .9 [m s] | Attachment [s] | Clearance [m] | Endpoint [m] | Edge XY RMS [m] | Linear TV [m/s] |
|---|---|---|---|---|---|---|
| S1 | -0.0038429248 | -0.0500000026 | 0.00507548338 | -0.00503683613 | 0.00222000416 | 0.399998408 |
| S2 | 0.0012339582 | 0.100000005 | 1.40043532e-12 | 0.000100163844 | 0.00466711756 | 0.415811292 |
| S3 | 0.000190605826 | null | 8.41095115e-08 | -0.000675306283 | 0.00651441061 | 0.399999909 |
| S4 | 0.000159624788 | null | -2.24353869e-12 | -0.00252727222 | 0.00597523092 | -0.0222668821 |

| Metric | Negative | Zero | Positive | Null | Median delta | Sources reversing R00/R01 direction |
|---|---|---|---|---|---|---|
| max_position_error_05_m | 1 | 0 | 3 | 0 | 8.4237293e-05 | S2, S3, S4 |
| position_auc_03_m_s | 1 | 1 | 2 | 0 | 6.50696787e-07 | S2, S3 |
| position_auc_09_m_s | 1 | 0 | 3 | 0 | 0.000175115307 | S2, S3, S4 |
| yaw_auc_03_rad_s | 0 | 1 | 3 | 0 | 1.67348257e-05 | none |
| yaw_auc_09_rad_s | 2 | 0 | 2 | 0 | -0.000415932953 | S3, S4 |
| sustained_attachment_s | 1 | 0 | 1 | 2 | 0.0250000013 | S2 |
| remaining_arc_at_attachment_m | 1 | 0 | 1 | 2 | 0.0224253228 | S2 |
| execution_clearance_lower_bound_m | 0 | 2 | 2 | 0 | 4.2055456e-08 | none |
| endpoint_error_m | 3 | 0 | 1 | 0 | -0.00160128925 | S2 |
| linear_command_TV | 1 | 0 | 3 | 0 | 0.399999159 | S4 |
| angular_command_TV | 2 | 0 | 2 | 0 | 0.0259823277 | S2, S4 |
| relative_translation_RMS_m | 0 | 0 | 4 | 0 | 0.00532117424 | none |
| relative_translation_max_m | 0 | 0 | 4 | 0 | 0.0164530855 | none |

The attachment median +0.025000001303851604 s uses **only the two observed pairs**;
S3/S4 nulls are excluded, not treated as ties. All four pairs contribute to position
AUC and distortion medians. Signs use the predeclared 1e-9 tolerance, not a fitted
practical-effect cutoff. S3's 8.41e-8 m clearance difference is below the existing
1e-7 m geometry reserve and is not meaningful evidence of improved safety. S2/S4
clearance differences are numerical zero under the sign rule.

Predeclared criterion counts: earlier/equal observed attachment 1/4; all three
early position metrics nonworse 1/4; Full/No-relative safety preserved 4/4;
relative-edge RMS and max larger 4/4. The majority benefit pattern is not met,
and the positive OSA03 result coexists with reversals: **MIXED_EVIDENCE**.


## 6. Research interpretation

Removing E_R consistently weakens relative-edge translation preservation in these
four geometries. Its effect on execution is source-dependent. The OSA03 early-error
and attachment benefit repeats, but .9 s position AUC changes in the opposite
direction in all three different non-OSA trajectories. Those error changes are
small: +0.0012339582, +0.0001906058 and +0.0001596248 m s. The straight/mild source
also attaches six ticks later. The two stronger turns provide no Full/No-relative
attachment comparison because neither meets the full dwell.

Thus the paired OSA03 execution benefit does not transfer as a consistent pattern
in this small benchmark. E_R has an observed geometric regularizing effect; the
results do not identify a universally beneficial or harmful execution role.
No-relative gives slightly smaller endpoint error in three sources and increases
linear command variation in three, so neither is a uniform winner. The unchanged
controller's own-reference errors remain separate evidence about trackability.
In S4 Native attaches while all modified references fail the dwell; in S3 even
Native does not attach. These facts caution against treating reference deformation
or forward-only progress as successful original-FRESH attachment.

Full is distinguishable from Taper, but Full has higher .9 s position AUC in three
sources and lower in one. This is not evidence of general graph superiority,
selection of a final method, online benefit, or complete obstacle bypass.


## 7. Limitations

Four purposively selected saved sources from a development corpus; sources are
not independent population samples. One OSA03 obstacle anchor. Offline controlled
schedule, not real asynchronous deployment latency; no real robot evidence.
Original FRESH has intrinsic tracking difficulty. No controller-aware factor or
online repeated FRESH. Geometry/progress proxies do not establish semantic intent.
Source-specific absolute phases/lag rules are controlled within source, so do not
pool absolute errors as if initial conditions were identical across sources.

## 8. Not demonstrated

General graph superiority, a chosen final method, real-world or online benefit,
complete obstacle bypass, online repeated-chunk improvement, population effects,
or necessity of E_R across arbitrary sources.

## 9. PNG artifact paths

Exactly four required files, plus the selector file only if different H5 selections
are observed. World plots use equal axes and distinct markers/styles for overlap.
Full numeric sidecars authenticate plotted values and PNG hashes. No extra review
PNGs are generated by the scientific report. Unit-test figures stay temporary.

- `/home/gpuadmin/Workspace/se-3-reconciliation/results/relative_factor_multisource_01/figures/world_execution_overview.png`
- `/home/gpuadmin/Workspace/se-3-reconciliation/results/relative_factor_multisource_01/figures/benchmark_primary_metrics.png`
- `/home/gpuadmin/Workspace/se-3-reconciliation/results/relative_factor_multisource_01/figures/no_relative_minus_full_deltas.png`
- `/home/gpuadmin/Workspace/se-3-reconciliation/results/relative_factor_multisource_01/figures/reference_deformation_overview.png`
- `/home/gpuadmin/Workspace/se-3-reconciliation/results/relative_factor_multisource_01/figures/selector_diagnostic_summary.png` (conditional)

## Reproduction commands

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_relative_factor_multisource01.py --mode prepare --run data/relative_factor_multisource_01/primary_20260930T033000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/multisource-mpl .venv/bin/python -m pytest -q tests/test_relative_factor_multisource01.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_relative_factor_multisource01.py --mode freeze --run data/relative_factor_multisource_01/primary_20260930T033000Z
# Review and commit implementation/config/tests/protocol/freeze ledger, then normal push.
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_relative_factor_multisource01.py --mode execute --run data/relative_factor_multisource_01/primary_20260930T033000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_relative_factor_multisource01.py --run data/relative_factor_multisource_01/primary_20260930T033000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/multisource-mpl .venv/bin/python scripts/report_relative_factor_multisource01.py --run data/relative_factor_multisource_01/primary_20260930T033000Z
```

Pre-scientific regression: **467 passed, 1 skipped** (the unavailable ignored
EXP-01B/EXP-02B corpus), 61.55 s. The focused suite has 23 tests. The regression
included the new suite, R00/R01/common-B/Local-SE2, SE(2)/trajectory/graph, native
continuation/trackability, official adapter and robotless execution, handoff loss,
Hospital/cart geometry, source inventory and delay-state reconstruction tests.
Synthetic fixture optimizer calls are tests, not scientific evidence; real MPC,
LightNav, RGB and Isaac calls in tests are zero.

Exact regression command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/multisource-mpl .venv/bin/python -m pytest -q tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
```

## Completion audit and reporting adjustments

Saved-only validation passed, including the unmodified R00 and R01 validators,
all source/code/result hashes, complete references, solver trace replay without
solves, exact integrations, guards, official selection/memory/generation ordering,
primary/own-reference evaluator recomputation and both schedule windows.
No scientific deviation: the frozen 8 planning calls and 16 rollouts ran once;
all source data, official MPC and frozen scientific/report code bytes are intact.

A separate saved-only presentation wrapper was added after visual QA. It includes
the entire cart footprint, uses square equal-axis world panels, keeps null/tiny-delta
labels in frame and reserves annotation space. Two presentation passes corrected
axis/label spacing. All numeric sidecars compare exactly to the frozen report,
all five PNGs were inspected, and only the five requested final PNG files remain.
`presentation_audit.json` records original/final image hashes, renderer hashes and
the prior presentation pass. Frozen run ledgers and initial report bytes are not
overwritten. No scientific calculation, source choice, method or metric changed.
This is the only post-freeze implementation addition; it is presentation-only.

No required PNG is omitted. The conditional selector PNG is included because all
four sources show at least one differing Full/No-relative H5 selection. No extra
final PNGs or PDF are generated. Exact final filesystem paths appear in section 9.

Additional saved-only presentation command:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/multisource-mpl .venv/bin/python scripts/finalize_relative_factor_multisource_png.py --run data/relative_factor_multisource_01/primary_20260930T033000Z
```

For a fresh reproduction, run prepare/freeze/commit/push/execute/validate/report
once in a new directory; the saved scientific run deliberately rejects overwrite
or replay. The presentation wrapper can be repeated without scientific calls.

The new single Full planning solve for S1 reproduced the historical frozen R00
Full world array **byte-for-byte**. It remains counted as one new scientific call.
Four final-interval results in S2 (one per method) were solved but withheld at the
180-interval cap. Thus 480 new solves were completed and 476 new results were
physically applied; the 16 restored first FRESH commands are historical commands,
not additional numerical solves. No state was integrated beyond the cap.

The three tracked CSVs use LF line endings for repository diff checks. Their
original CRLF-to-LF serialization change preserves every parsed CSV field exactly;
`csv_format_audit.json` records before/after hashes. No numeric value changes.
