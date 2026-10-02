# SPATIAL_ENTRY_SUFFIX_EXECUTION_01

## Frozen protocol

Starting fetched HEAD and origin/main:
`962d63e9ec775635481f00164562afa0c0f43386`.
**Timing-controlled offline causal reference comparison.**
This is a bounded entry-selection mechanism experiment, not the final graph method.

Question: does removing the stale original FRESH prefix at the previously frozen
C3 entry, while retaining every downstream original pose, improve transition
behavior without sacrificing downstream chunk completion?

| ID | Exact frozen source | C3 arc l* [m] | Segment k | beta | Suffix rows |
|---|---|---:|---:|---:|---:|
| S1 | OSA03_R00 | 0.18474273839738128 | 1 | 0.22582990794399482 | 9 |
| S2 | episode_001_repeat_01/handoff_013 | 0.3245809061559166 | 2 | 0.15302563680147266 | 8 |
| S3 | episode_008_repeat_01/handoff_023 | 0.46193137914572885 | 2 | 0.9725845948085713 | 8 |
| S4 | episode_013_repeat_00/handoff_020 | 0.3392920662039436 | 2 | 0.22054154941072646 | 8 |

Run identifier: `data/spatial_entry_suffix_execution_01/primary_20261002T000000Z`.
The identifier is not a claim about acquisition or execution wall time; actual
execution timestamps are saved separately. Sources and source order are fixed.
No new acquisition, source selection, instruction, or parameter search.

### Authentication and immutable inputs

The YAML freezes these exact historical result SHA-256 values:

- Correspondence diagnostic: `1449e0e59cb43cb22ec03004609c5e74310bf097aaad70a28dd22d8197dbef8f`.
- Transport scale: `04104aa84d36cc15792cd80cfef93ef6fafdaf6810bfb71f143a2532e2880c68`.
- Multisource: `d935cdc58661a672cb238085f0c697c5d75d36a5d01ff5709e2782de6b0c5cc7`.

The complete historical result/input/code chains are authenticated. The old
correspondence, transport, multisource, R00 and R01 saved validators must all pass.
Native and Current Full Local-SE2 references and rollouts are read byte-for-byte
from their historical folders. No copies are recomputed, reoptimized or rerun.
Historical Half is separate descriptive numeric context only.

Official LightNav checkout: `c6f40e3220edbf7011e4f17eaf2c865416737d4d`.
Official MPC SHA-256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
The official source, adapter, worker, historical execution wrapper, evaluator and
continuous correspondence implementation remain unchanged. H=5, nearest+1,
Q/R, limits, acceleration limits and memory/generation rules remain unchanged.

A and B are fixed. Original world FRESH remains `F_j = A F_local,j`.
World XY uses metres, yaw radians CCW, +Z up; observation-local x forward/y left.
No waypoint timestamp or dt is assigned. OLD-to-B is saved provenance only.
For historical Full, the observation-to-application state-shift / transport factor
moves editable early nodes toward `B A^-1 F_j`, equivalently encouraging
`B^-1 X_j ≈ A^-1 F_j`. It does not move A to B.

### Exact C3 and suffix

Call the unchanged `SpatialCurve.correspondences(B)['C3_FORWARD_SE2']` and require
exact dictionary equality to the prior diagnostic (excluding its safety record).
C1 supplies the global XY projection arc l_XY. C3 minimizes
`(d_xy/.10 m)^2 + (wrap(theta_B-theta_F(l))/15 deg)^2` for `l >= l_XY`.
Use the frozen analytic piecewise yaw-branch minimizer, shortest-angle interpolation,
1e-12 absolute cost tie tolerance and earliest arc tie handling. No retuning.

At segment k, beta, form `[E*, F_(k+1), ..., F_(N-1)]`. If E* is the exact segment
start, preserve that original vertex once. If its XY norm and wrapped yaw differ
from the next vertex by at most 1e-12, retain that original vertex once. Otherwise
prepend the continuous interpolated E*. Original downstream world **and raw
A-local rows** are preserved exactly. Only a new interpolated E* is converted by
`A^-1 E*` for the existing input adapter. This avoids even round-trip changes to
original downstream rows. Official installed downstream rows must be bit-identical
to the saved Native installation. No B re-anchor, smoothing, resampling or bridge.

E* is a reference pose. The physical robot starts at B and the unchanged official
MPC computes commands to the suffix. No instantaneous motion to E* is assumed.
The B-to-E* straight connector is not inserted into either the reference or its
safety check. Its prior hypothetical clearance is retained only in entry provenance.

All four prepared references have N>=2, positive remaining original arc and exact
C3 parity. Zero-numerical-solve official installation preflight passes for all
four; numerical MPC solve is forbidden during that check. Complete reference
clearance uses the existing direct checker with radius .20 m, footprint-edge
clearance .05 m and unchanged uncertainty/workspace/numerical reserve. Unsafe
references would be recorded without repair or rollout.

### State and schedule

Copy each source's exact `common_state.json`, `schedule.json`, source manifest,
scenario, metric protocol and OLD-to-B array from STATE_SHIFT_TRANSPORT_SCALE_01.
This preserves B pose/tick/simulation time, physical u_minus provenance, the already
applied u_B_plus, previous_control/u_mem_B, generation, version and capture pose.

| Source | B tick | First legal submit tick |
|---|---:|---:|
| S1 | 92 | 96 |
| S2 | 997 | 1002 |
| S3 | 1804 | 1806 |
| S4 | 1524 | 1524 |

Reuse `run_relative_factor_multisource01.run_method` directly, only with the new
method name/reference. Its logical release wrapper waits at a fixed simulation
state and releases each successful result at the frozen application tick. Wall
solver time does not advance simulation. No future states enter a solve. Command
holding, previous_control changes at solve completion, pending/stale result
handling, endpoint repetition and integration are unchanged. Restore failures
stop scientific execution; no substitute controller.

Require exact Native/Full/C3 B provenance, attempted/accepted submit ticks,
successful application ticks and interval counts over both 54 primary intervals
and all 180 cap intervals. Integration dt is saved float32(1/60), approximately
.01666666753590107 s. Nominal .9 and 3 s mean the historical 54 and 180 intervals
(.900000046938658 and 3.0000001564621925 s). The abort-only guard rejects a proposed
unsafe command before application and preserves the valid prefix. Censoring is
reported; no continuation or retry is fabricated.

### Evaluation

Primary target is the entire ORIGINAL FRESH, using the unchanged forward-only
continuous projection, shortest-angle yaw and original attachment evaluator.
Report initial errors, max first-.5-s position error, separation growth,
position/yaw AUC .3/.9, attachment dwell start, original fractional row, original
arc fraction and remaining original arc at attachment. Retain command TV, max
absolute v/omega, swept clearance lower bound, termination and abort tick.

**Original-FRESH endpoint dwell time** is the first actual execution sample t
whose position is within .10 m and yaw within 15 degrees of the original final
pose, with the entire following .30 s sampled interval inside both thresholds.
Use the existing inclusive searchsorted rule with 1e-12 time tolerance. There is
no interpolation or timestamp assigned to FRESH rows. A second independent
contiguous-true-run implementation validates the dwell time.

No complete dwell means null, never the cap. Record first endpoint tube entry
without dwell separately. `T_post_attach = T_endpoint - T_attach` exists only when
both are observed (no clamping). Fixed-3-s endpoint error, projected original
row/arc and remaining arc are null if the full cap was not reached. Path length
to endpoint dwell is the integral of absolute held v over executed intervals;
mean absolute v divides it by dwell start time, and is null at zero duration or
unobserved dwell. This endpoint is not claimed to be the navigation task goal.

C3 own-reference position/yaw AUC and attachment are secondary only. The original
attachment metrics must remain identical to the historical evaluator. New
endpoint metrics are computed from saved executions only after the freeze.

Map every selected suffix row to its original identity and original arc. The
interpolated E* has a fractional original identity, not a fabricated raw row.
Report raw prefix rows removed, first presented identity/arc, first H5 original
identities at the first legal submit, and every selected arc. Any selected
original identity before the suffix entry is an implementation failure. Temporal
backward selection among remaining suffix rows is separately reported.

### Predeclared interpretation

Numerical sign tolerance is 1e-9; substantially more remaining arc means >.10 m.
Evaluate these categories in the stated precedence, without selecting a winner:

1. **TECHNICAL_BLOCKED**: authentication, restoration, interface, validator,
   unexplained runtime/schedule failure, or stale-prefix identity implementation failure.
2. **ENTRY_REFERENCE_FAILURE**: unsafe reference, execution safety failure, or
   official numerical controller error after successful restoration.
3. **FAST_ATTACHMENT_SLOW_COMPLETION_TRADEOFF**: any paired source/baseline with
   both attachment times observed, C3 earlier, and C3 endpoint later/lost or
   >.10 m more original arc remaining at attachment. Report which trigger applies;
   extra remaining arc alone does not establish a later observed endpoint time.
4. **ENTRY_TRANSITION_AND_COMPLETION_SUPPORTED**: lower .9-s original position
   AUC than Full in at least 3/4 sources, at least one baseline endpoint observed,
   no systematically later/lost endpoint, safety and entry identity checks pass.
5. **ENTRY_PROGRESS_INSUFFICIENT**: remaining valid cases; joint transition and
   completion support is absent. Entry selection alone is insufficient.

Systematically later/lost means a strict majority of sources with an observed
Native or Full endpoint dwell have C3 later than at least one such baseline, or
C3 loses that observed dwell. All paired differences, nulls and recoveries remain
visible. Faster attachment alone cannot establish improvement. If observed
attachment improves but endpoint dwell worsens, state both explicitly and do not
call it a net improvement.

### Freeze and execution discipline

Prepare/authenticate entries, references, endpoint definition, schedules and code;
run focused and relevant regressions, compileall, diff review; snapshot protocol;
commit and normal push. Record that pushed SHA before exactly four new C3
rollouts in S1-S4 order. Exclusive start markers prevent repeats. Optimizer calls
are forbidden by a runtime guard. No new Native/Full/Half rollout, LightNav, RGB
or Isaac acquisition, source selection, weight sweep or result-driven edits.

After execution: saved-only validation, four PNG report, sidecar/image inspection,
results documentation and work log, small result/docs commit and normal push.
Large rollout arrays remain in ignored data. Historical artifacts are unchanged.

Exactly four final PNGs, no HTML dependency:

1. `world_execution_overview.png`: equal-axis 2x2 original geometry and executed
   paths; saved OLD-to-B, B, C3 entry, original endpoint, attachment/endpoint-dwell
   and minimum-clearance markers; cart/geometry with .25 m centre exclusion.
2. `transition_and_endpoint_metrics.png`: .9 position AUC, attachment, endpoint
   dwell and remaining original arc at attachment. N/A remains N/A.
3. `progress_preservation.png`: C3 entry, first H5 original target progress,
   actual attachment and endpoint-dwell original projections, original endpoint.
4. `attachment_vs_completion.png`: observed T_attach/T_endpoint pairs annotated
   by remaining arc; incomplete pairs in a separate N/A table.

Styles and staggered markers distinguish overlapping executions; world panels
state measured maximum XY gaps. JSON numeric sidecars and SHA-256 ledgers bind
figures and CSVs to the saved validation summary.

## Repository-confirmed facts

Pushed scientific freeze: `124361eb61a716bca038a2fd4ce15818b0986f3e`.
One new C3 rollout per source: **4 rollouts, 120 official MPC solves, 119 applied
results, 0 reconciliation optimizer, 0 LightNav, 0 RGB, 0 Isaac, 0 retries**.
S2's final successful solve is held beyond the cap by the frozen schedule, exactly
as in its historical comparisons. Native/Full reuse: 8 saved rollouts; no rerun.
All 12 compared executions end at `OBSERVATION_CAP`, with 180 intervals each.

Exact initial state/command/memory and attempted/accepted/application tick
comparability passed in all four sources, through both the .9 s window and the
full cap. All five historical saved validators passed. The new saved-only
validator passed before and after reporting; the report also recomputed saved
metrics. No frozen code, config, reference, schedule or input changed after push.

Frozen classification: **ENTRY_TRANSITION_AND_COMPLETION_SUPPORTED**.
Original-FRESH .9 position AUC is lower than Full in 4/4 sources. No source has
later or lost endpoint dwell versus an observed Native/Full endpoint. No C3
selected identity precedes its entry; no C3 selector progress moves backward
through the saved rollout. No reference or execution safety failure occurred.

Tables display rounded values; JSON/CSV retain full precision. Times are actual
execution seconds after B. N/A is null, never an imputed cap. Full is Current Full
Local-SE2; its historical result is unchanged.

### Core transition and completion comparison

Position AUC units m·s; remaining arc, endpoint error and clearance in m; linear TV
in m/s and angular TV in rad/s. Attachment progress is fractional original row /
original arc fraction. Endpoint means original-FRESH endpoint dwell time.

| Source | Method | AUC .9 | T_attach | Progress at attach | Arc left at attach | T_endpoint | T_post_attach | Endpoint error at 3 s | Swept clearance | Linear TV | Angular TV |
|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | Native | 0.169066 | 1.466667 | 8.717846 / 0.968605 | 0.042485 | 1.516667 | 0.050000 | 0.096439 | 0.133561 | 0.800000 | 3.629556 |
| S1 | Full | 0.169964 | 1.483333 | 8.781297 / 0.975665 | 0.032931 | 1.516667 | 0.033333 | 0.096335 | 0.133709 | 0.800000 | 3.539109 |
| S1 | C3 Entry-Suffix | 0.169066 | 1.466667 | 8.717846 / 0.968605 | 0.042485 | 1.516667 | 0.050000 | 0.096439 | 0.133561 | 0.800000 | 3.629556 |
| S2 | Native | 0.131833 | 1.050000 | 7.690453 / 0.854498 | 0.197121 | 1.250000 | 0.200000 | 0.079382 | 0.757224 | 0.806041 | 2.546626 |
| S2 | Full | 0.136682 | 1.200000 | 8.102748 / 0.900316 | 0.135048 | 1.333333 | 0.133333 | 0.086092 | 0.757225 | 1.389786 | 2.568850 |
| S2 | C3 Entry-Suffix | 0.131833 | 1.050000 | 7.690453 / 0.854498 | 0.197121 | 1.250000 | 0.200000 | 0.079382 | 0.757224 | 0.806041 | 2.546626 |
| S3 | Native | 0.189624 | N/A | N/A / N/A | N/A | N/A | N/A | 0.115238 | 0.380789 | 1.306412 | 4.091433 |
| S3 | Full | 0.199964 | N/A | N/A / N/A | N/A | N/A | N/A | 0.139406 | 0.380796 | 2.106412 | 3.577351 |
| S3 | C3 Entry-Suffix | 0.186554 | N/A | N/A / N/A | N/A | N/A | N/A | 0.112354 | 0.380789 | 1.306412 | 4.053282 |
| S4 | Native | 0.198189 | 1.283333 | 7.788272 / 0.912754 | 0.111962 | 1.400000 | 0.116667 | 0.084539 | 0.870804 | 1.453530 | 4.825967 |
| S4 | Full | 0.217335 | N/A | N/A / N/A | N/A | N/A | N/A | 0.132317 | 0.870584 | 1.462212 | 4.394973 |
| S4 | C3 Entry-Suffix | 0.198189 | 1.283333 | 7.788272 / 0.912754 | 0.111962 | 1.400000 | 0.116667 | 0.084539 | 0.870804 | 1.453530 | 4.825967 |

### Initial and early errors

Initial position/yaw errors are identical across the three methods within each source. Yaw units below are radians (AUC rad·s).

| Source | Method | Initial XY [m] | Initial yaw [rad] | Max XY .5 [m] | Separation growth [m] | XY AUC .3 | Yaw AUC .3 | Yaw AUC .9 |
|---|---|---|---|---|---|---|---|---|
| S1 | Native | 0.106649 | 0.523608 | 0.215262 | 0.108614 | 0.047920 | 0.122764 | 0.193368 |
| S1 | Full | 0.106649 | 0.523608 | 0.216171 | 0.109523 | 0.047920 | 0.122775 | 0.193335 |
| S1 | C3 Entry-Suffix | 0.106649 | 0.523608 | 0.215262 | 0.108614 | 0.047920 | 0.122764 | 0.193368 |
| S2 | Native | 0.127644 | 0.261794 | 0.163895 | 0.036251 | 0.045302 | 0.045624 | 0.107104 |
| S2 | Full | 0.127644 | 0.261794 | 0.165602 | 0.037957 | 0.045338 | 0.046931 | 0.098951 |
| S2 | C3 Entry-Suffix | 0.127644 | 0.261794 | 0.163895 | 0.036251 | 0.045302 | 0.045624 | 0.107104 |
| S3 | Native | 0.184683 | 0.385685 | 0.239120 | 0.054437 | 0.064364 | 0.076191 | 0.175616 |
| S3 | Full | 0.184683 | 0.385685 | 0.242432 | 0.057749 | 0.064402 | 0.077403 | 0.147269 |
| S3 | C3 Entry-Suffix | 0.184683 | 0.385685 | 0.235029 | 0.050346 | 0.063700 | 0.074971 | 0.173208 |
| S4 | Native | 0.160884 | 0.505675 | 0.252792 | 0.091907 | 0.061199 | 0.120661 | 0.230599 |
| S4 | Full | 0.160884 | 0.505675 | 0.270608 | 0.109723 | 0.062996 | 0.122789 | 0.200966 |
| S4 | C3 Entry-Suffix | 0.160884 | 0.505675 | 0.252792 | 0.091907 | 0.061199 | 0.120661 | 0.230599 |

### Downstream and command diagnostics

All 12 have final projected original row 9, arc fraction 1 and remaining projected
original arc 0 at the fixed 180-interval cap. These are projection results; four
executions still have no endpoint-tube entry (all S3 methods and S4 Full).
First endpoint-tube entry equals T_endpoint for each of the eight observed dwells.
There are no transient-only or insufficient-dwell endpoint entries in this run.
Maximum absolute v is .8 m/s for all 12. All safety abort ticks are null.

| Source | Method | First endpoint tube entry [s] | Path to endpoint dwell [m] | Mean absolute v before dwell [m/s] | Max absolute omega [rad/s] |
|---|---|---|---|---|---|
| S1 | Native | 1.516667 | 1.180000 | 0.778022 | 1.548334 |
| S1 | Full | 1.516667 | 1.180000 | 0.778022 | 1.499998 |
| S1 | C3 Entry-Suffix | 1.516667 | 1.180000 | 0.778022 | 1.548334 |
| S2 | Native | 1.250000 | 0.984487 | 0.787589 | 1.052845 |
| S2 | Full | 1.333333 | 0.990781 | 0.743086 | 1.002829 |
| S2 | C3 Entry-Suffix | 1.250000 | 0.984487 | 0.787589 | 1.052845 |
| S3 | Native | N/A | N/A | N/A | 1.970743 |
| S3 | Full | N/A | N/A | N/A | 1.706990 |
| S3 | C3 Entry-Suffix | N/A | N/A | N/A | 1.963121 |
| S4 | Native | 1.400000 | 1.067485 | 0.762489 | 2.271347 |
| S4 | Full | N/A | N/A | N/A | 1.963060 |
| S4 | C3 Entry-Suffix | 1.400000 | 1.067485 | 0.762489 | 2.271347 |

### Frozen entries and preserved identities

Original rows are zero-indexed. E* is a fractional original identity; it is not a raw row. Every following integer row remains exact.

| Source | l* [m] | l*/L | E* world [x m, y m, yaw rad] | Raw prefix rows removed | First presented original identity | Arc left at E* [m] | Suffix minimum clearance [m] |
|---|---|---|---|---|---|---|---|
| S1 | 0.18474273839738128 | 0.1365182708172272 | [19.29589449218289, 24.196826321126334, -1.0453675393771789] | 2 | 1.2258299079439947 | 1.168502781718514 | 0.229486890 |
| S2 | 0.3245809061559166 | 0.23958430386696167 | [18.747621139721804, 9.891583390188735, -1.708655151798606] | 3 | 2.153025636801473 | 1.0301860836555385 | 0.824529094 |
| S3 | 0.46193137914572885 | 0.3662231723212672 | [17.89727685765006, 31.99213199452277, -0.3242954780329643] | 3 | 2.972584594808571 | 0.7994070998418927 | 0.211489855 |
| S4 | 0.3392920662039436 | 0.264393095332806 | [19.516333811852085, 9.044315276940615, -1.8652517646249573] | 3 | 2.2205415494107266 | 0.9439943440438663 | 1.029576806 |

First H5 selections at identical first legal submit states:

| Source | Method | Submit tick | First H5 original identities | First H5 original arc [m] | First H5 before C3 entry |
|---|---|---|---|---|---|
| S1 | Native | 96 | [3, 4, 5, 6, 7] | 0.451861533 | False |
| S1 | Full | 96 | [1, 2, 3, 4, 5] | 0.150756405 | True |
| S1 | C3 Entry-Suffix | 96 | [2, 3, 4, 5, 6] | 0.301251678 | False |
| S2 | Native | 1002 | [4, 5, 6, 7, 8] | 0.602176150 | False |
| S2 | Full | 1002 | [1, 2, 3, 4, 5] | 0.150768970 | True |
| S2 | C3 Entry-Suffix | 1002 | [4, 5, 6, 7, 8] | 0.602176150 | False |
| S3 | Native | 1806 | [4, 5, 6, 7, 8] | 0.622519581 | False |
| S3 | Full | 1806 | [1, 2, 3, 4, 5] | 0.166443212 | True |
| S3 | C3 Entry-Suffix | 1806 | [3, 4, 5, 6, 7] | 0.466037433 | False |
| S4 | Native | 1524 | [3, 4, 5, 6, 7] | 0.458835111 | False |
| S4 | Full | 1524 | [1, 2, 3, 4, 5] | 0.150927954 | True |
| S4 | C3 Entry-Suffix | 1524 | [3, 4, 5, 6, 7] | 0.458835111 | False |

Full's first H5 target has an original identity before C3 entry in 4/4 sources.
C3's first target is after entry in 4/4, and all 120 saved new solve selections
remain at/after entry. Nearest+1 does not directly select E* as an H5 target here;
its first selected rows are unchanged downstream originals. The reference still
starts at E* and the physical robot still starts at B.

### Own-reference secondary metrics

| Source | Own XY AUC .3 | Own XY AUC .9 | Own yaw AUC .3 | Own yaw AUC .9 | Own attachment [s] |
|---|---|---|---|---|---|
| S1 | 0.047920362 | 0.169065632 | 0.122763625 | 0.193368049 | 1.466666743 |
| S2 | 0.045302491 | 0.131833111 | 0.045624119 | 0.107103977 | 1.050000055 |
| S3 | 0.063699869 | 0.186553953 | 0.074971489 | 0.173208231 | N/A |
| S4 | 0.061198617 | 0.198189216 | 0.120660784 | 0.230599468 | 1.283333400 |

### Overlap and paired outcomes

| Source | Max C3/Native XY gap [m] | Max C3/Full XY gap [m] | C3 minus Full XY AUC .9 | C3 minus Full T_attach | C3 minus Full T_endpoint |
|---|---|---|---|---|---|
| S1 | 5.34408183585e-10 | 0.002303515 | -0.000898322 | -0.016666668 | 0.000000000 |
| S2 | 0 | 0.059806736 | -0.004849159 | -0.150000008 | -0.083333338 |
| S3 | 0.00935243005058 | 0.048784616 | -0.013409706 | N/A | N/A |
| S4 | 0 | 0.055066980 | -0.019145831 | N/A | N/A |

S2 and S4 C3/Native executed XY samples are exactly equal; S1's maximum XY gap is
5.344081835845081e-10 m. S3's maximum gap is .009352430050584302 m. C3/Native
attachment and endpoint dwell times are equal in all three sources with observed
dwells. C3 versus Full: S1 attachment is one tick earlier with equal endpoint
dwell; S2 attachment is nine ticks earlier and endpoint dwell five ticks earlier;
S4 recovers both dwells; S3 retains nulls.

No paired source has earlier attachment and later endpoint dwell. C3 attaches
with .009553974314 m more arc than Full in S1 and .062073116011 m more in S2;
neither exceeds the frozen .10 m criterion. T_post_attach is longer for C3 in
S1/S2 because attachment occurs earlier in time and original progress, while
absolute endpoint dwell is equal/earlier. These distinct intervals are preserved.
C3 yaw AUC .9 is higher than Full in all four; command angular TV is higher in
S1/S3/S4 and lower in S2. No scalar ranking combines these metrics.

### Artifacts and inspection

- [Result summary](../results/spatial_entry_suffix_execution_01/result_summary.json)
- [All primary metrics](../results/spatial_entry_suffix_execution_01/primary.csv)
- [Entry references](../results/spatial_entry_suffix_execution_01/entry_reference.csv)
- [Attachment/endpoint comparison](../results/spatial_entry_suffix_execution_01/attachment_endpoint_comparison.csv)
- [Original selector identities](../results/spatial_entry_suffix_execution_01/selector_progress.csv)
- [Numeric figure sidecars](../results/spatial_entry_suffix_execution_01/figure_manifest.json)

Exactly four final PNGs, visually inspected with all numeric sidecars checked
against saved arrays/metrics (8 observed attachment/endpoint pairs, 4 N/A pairs):

1. [World execution overview](../results/spatial_entry_suffix_execution_01/figures/world_execution_overview.png) — 2700×2160.
2. [Transition and endpoint metrics](../results/spatial_entry_suffix_execution_01/figures/transition_and_endpoint_metrics.png) — 2700×1620.
3. [Progress preservation](../results/spatial_entry_suffix_execution_01/figures/progress_preservation.png) — 2700×1620.
4. [Attachment versus completion](../results/spatial_entry_suffix_execution_01/figures/attachment_vs_completion.png) — 2700×1620.

No additional final PNGs or HTML dependency. Large raw rollout records stay under
ignored data. Only compact derived CSV/JSON/PNG and documentation are committed.

## Research interpretation

The frozen joint criterion supports progress-preserving entry selection compared
with current Full in these four saved sources. Removing the stale prefix and
retaining the original downstream geometry removes Full's initial progress reset,
lowers early position error/AUC, preserves safety and does not delay observed
endpoint dwell. S4 recovers the behavior already demonstrated by Native.

This establishes no useful execution advantage over Native in S1/S2/S4: two
executions are exactly equal and the third agrees to sub-nanometre XY precision.
S1 and S3 also show that different first H5 identities can produce near-identical
or only modestly changed actual execution. The mechanism comparison changes the
reference presented to the same controller; it does not isolate an abstract
progress variable independently of reference geometry and row selection.

S3 remains unresolved: C3 lowers .9 position AUC relative to both baselines and
reduces the terminal position error to .112354395 m, but neither attachment nor
endpoint dwell is observed. Projected arc completion is insufficient to establish
endpoint-tube dwell. Progress preservation helps the declared four-source
comparison, but is not sufficient for reliable transition behavior in every case.

A bounded follow-up studying a fixed-B to fixed-entry transition with fixed
original downstream suffix is reasonable, especially for S3. These results do not
establish that graph optimization is necessary or superior. No bridge or graph
transition is implemented here. The yaw/command trade-offs and strong Native
agreement should remain explicit baselines in any next study.

## Limitations

Four frozen sources; offline timing control is not real asynchronous deployment
timing. Original FRESH has intrinsic tracking difficulty. There is no new
controller-aware or transition factor. No population, semantic intent preservation,
real-world, online repeated-chunk or final-method claim. Endpoint dwell concerns
this original FRESH chunk, not navigation task completion. Reference geometry
preservation alone does not guarantee command or execution equivalence.

## Not demonstrated

General graph superiority, real-world speedup, online VLA improvement, C3 optimality,
navigation task completion, complete obstacle bypass, or a final reconciliation
method. A graph-optimized B-to-entry transition is explicitly excluded here.

## Commands and validation

From the repository root, with the existing `.venv` (no environment changes):

```bash
git fetch origin main
git rev-parse HEAD
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/entry-suffix-mpl .venv/bin/python -m pytest -q tests/test_spatial_entry_suffix_execution01.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/entry-suffix-mpl .venv/bin/python -m pytest -q tests/test_spatial_entry_suffix_execution01.py tests/test_spatial_correspondence_selector_diag01.py tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_spatial_entry.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z --mode freeze
# Review and commit only the namespace files, protocol, freeze summary and work log; normal push.
git push origin main
# Only after successful push:
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/entry-suffix-mpl .venv/bin/python scripts/report_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z --check-only
```

Preparation performed no numerical controller solves. During implementation,
synthetic tests caught an incorrect test call signature and a ±pi representation
check; both were corrected before freeze. Coordinate parity compares wrapped yaw.
The threshold test follows the existing wrap convention without changing its
numerical thresholds. No scientific execution occurred during these corrections.

Pre-freeze focused tests: **34 passed** (42.01 s). Final regression suite:
**571 passed, 1 skipped** (155.35 s). The skip is the unavailable ignored
EXP-01B/EXP-02B historical corpus. Compileall and `git diff --check` passed.
The four-PNG renderer also passed its synthetic fixture with explicit nulls.


## Final scientific questions

1. **Does C3 suffix eliminate stale-prefix/progress-reset behavior?** Yes in all
   four: every selected original identity is at/after C3 entry, and there is no
   backward selection over execution. Full's first target is before C3 entry in
   all four. C3 sometimes starts at an earlier allowed target than Native.
2. **Does it reduce early original-FRESH AUC versus Full?** Yes, .9 position AUC
   falls by .000898322/.004849159/.013409706/.019145831 m·s in S1-S4. Yaw AUC .9
   increases, so the result does not mean every early error improves.
3. **Does it attach earlier versus Full?** S1 by .016666668 s; S2 by .150000008 s.
   S4 gains observed attachment at 1.283333400 s where Full is null. S3 remains null.
4. **Where on original FRESH does it attach?** C3 fractional rows S1/S2/S4:
   8.717846367/7.690453160/7.788272229; arc fractions .968605161/.854497878/.912753530.
   S3 has no attachment location.
5. **How much original arc remains when attachment occurs?** C3 S1/S2/S4:
   .042484925/.197121472/.111962209 m. S3 N/A. The whole suffix is not complete at
   attachment; endpoint dwell is a separate measure.
6. **Does earlier attachment correspond to earlier endpoint dwell?** S1's endpoint
   dwell is unchanged at 1.516666746 s; S2's is .083333338 s earlier at 1.250000065 s.
   S4 gains endpoint dwell at 1.400000073 s. Native and C3 times are equal where observed.
7. **Does any method attach earlier but finish the original FRESH suffix later?**
   No paired source has earlier attachment and later observed endpoint dwell.
   S1/S2 C3 has longer T_post_attach and more arc left at attachment, with equal/
   earlier absolute endpoint dwell. Do not confuse these two durations.
8. **Does C3 recover S4, where Native attached but Half/Full did not?** Yes;
   attachment 1.283333400 s and endpoint dwell 1.400000073 s, both matching Native.
   Half is historical attachment context only and was not rerun.
9. **Does C3 help S3, where none of Native/Half/Full attached?** It lowers position
   AUC .9 and final error, but attachment and endpoint dwell remain null. It does
   not recover sustained attachment.
10. **Is safety preserved?** All new references and executions pass. C3 swept
    clearance lower bounds S1-S4 are .133561095/.757224041/.380789059/.870803818 m,
    all above .05 m. No safety abort or unapplied unsafe continuation.
11. **Is progress preservation alone sufficient?** It meets the frozen aggregate
    joint criterion, but is insufficient to recover S3 and gives no meaningful
    advantage over Native in three sources. It is not a generally sufficient method.
12. **Is a graph-optimized B->entry transition justified as the next study?** It is
    a reasonable bounded hypothesis to investigate the residual transition problem
    with fixed B, fixed E* and fixed original suffix. Its necessity and benefit
    are unproven; nothing from that proposed study is implemented here.

Protocol deviations: **none in scientific execution**. All fixes and layout work
preceded the pushed freeze. No retries, result-driven code/config edits, baseline
reruns, solver tuning or new acquisition. Date headings use the supplied task date;
run logs retain their own actual wall-clock stamps.
