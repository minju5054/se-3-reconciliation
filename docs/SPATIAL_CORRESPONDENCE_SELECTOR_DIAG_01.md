# SPATIAL_CORRESPONDENCE_SELECTOR_DIAG_01

## Frozen protocol

Starting fetched HEAD and origin/main:
`1456e8eabd0e84b37203f54fdf567f53de06cf46`.
**Saved-only matched-state correspondence and official-selector diagnostic.**
This measures reference geometry and selector response. Saved execution outcomes
are joined afterward as observational context; no claim of selector causality.

Exactly four unchanged sources:

| ID | Saved source |
|---|---|
| S1 | OSA03_R00 |
| S2 | episode_001_repeat_01/handoff_013 |
| S3 | episode_008_repeat_01/handoff_023 |
| S4 | episode_013_repeat_00/handoff_020 |

Run identifier: `data/spatial_correspondence_selector_diag_01/primary_20260930T120000Z`.
The directory is an identifier; the actual UTC evaluation time is saved separately.
Native, Half and Full references, installed controller arrays, saved Native
submit states, geometry and results are authenticated from
RELATIVE_FACTOR_MULTISOURCE_01 and STATE_SHIFT_TRANSPORT_SCALE_01. Their complete
result ledgers and frozen input/code chains must pass, including the saved-only
transport, multisource, R00 and R01 validators. No historical code is edited.

No new optimization, MPC numerical solve, rollout, state integration, controller
memory update, command application, LightNav, RGB or Isaac acquisition. Old saved
validators may reconstruct an integration step to verify a saved rollout; this is
read-only historical verification, not a new rollout. Synthetic tests are tests
only, never evidence. Optional Genuine13 expansion is not performed.

### Continuous original-FRESH geometry

A and B stay fixed. Original raw FRESH remains observation-anchored: `F=A F_local`.
World XY metres, yaw radians CCW, +Z up; observation-local x forward/y left.
No raw output is overwritten or re-anchored at B. OLD remains fixed provenance.
This is specifically B-to-FRESH correspondence, not OLD-to-FRESH correspondence.

Original FRESH is an untimed spatial curve: linear XY and shortest-angle yaw
within each original segment. Primary progress `l` is cumulative original XY arc
in metres; secondary progress `s=l/L`. There is no waypoint timestamp or dt.

- C0_ROW0: `l=0`, a conceptual historical baseline, not the official MPC selector.
- C1_XY_PROJECTION: globally minimum squared XY distance, earliest arc on ties.
- C2_SE2_PROJECTION: globally minimize
  `J=(d_xy/.10 m)^2+(wrap(theta_B-theta_F(l))/15 deg)^2`.
- C3_FORWARD_SE2: minimize the same J subject to `l>=l_C1`.

The two scales are the existing attachment thresholds, with no extra coefficient.
The implementation partitions each segment at wrapped-yaw branch boundaries and
solves the resulting quadratic analytically. Every branch endpoint and stationary
point is considered. Absolute cost tie tolerance is 1e-12 (squared metres for C1,
dimensionless J for C2/C3); pick the earliest arc among tied candidates. Dense
synthetic oracle score tolerance is 1e-7. No numerical optimizer is called.
Zero XY-length segments fail closed without deleting raw identities; their arc
parameter would not uniquely specify changing yaw. An interior vertex belongs to
the earlier segment; the final endpoint uses the final segment. Nearest raw row
means nearest original arc, earliest identity on an exact tie, and is diagnostic
only. Continuous results are never snapped to rows.

Report arc, normalized progress, segment/alpha, target pose, diagnostic nearest
raw row, position/yaw/tangent mismatch relative to B, remaining arc and J.
Tangent is the containing original XY chord. Near-exhausted means remaining arc
<=0.10 m, using the already frozen positional attachment threshold as a descriptive
label, not a selection gate.

### Matched-state selector audit

Use the exact saved Native submit-request pose, equal to its saved integration
state, at every accepted official submit tick in the original 54-interval primary
window, both endpoints included. The nominal window is .9 s; the actual source
float32 integration dt gives 54/60 approximately .90000004694 s. No new tick grid
or simulation is constructed. Frozen ticks:

| Source | Absolute ticks |
|---|---|
| S1 | 96,102,108,114,120,126,132,138,144 |
| S2 | 1002,1008,1014,1020,1026,1032,1038,1044,1050 |
| S3 | 1806,1812,1818,1824,1830,1836,1842,1848,1854 |
| S4 | 1524,1530,1536,1542,1548,1554,1560,1566,1572,1578 |

37 states, three references/state, exactly **111 scientific official-selector
queries**. Use exact saved `restoration.installed_world` arrays, the arrays the
unchanged historical official tracker actually consumed. Authenticate the stored
world/local reference files too. Installed arrays differ only by historical
coordinate round-trip roundoff, checked within 1e-12; record those differences.
This diagnostic performs no new reference transformation. Original row identity
j is one-to-one in all three arrays.

Authenticate official MPC whole-file SHA256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Extract and compile only the unchanged function ASTs `wrap_angle` and
`build_pose_aligned_reference`; record source/AST hashes and line ranges. Do not
import the full external module, instantiate a tracker, initialize CasADi, or
submit/poll a solve. Official selection uses weighted squared XY and wrapped yaw,
Q=(10,10,1), nearest original row in the supplied reference, then the next H=5
rows with endpoint repetition and sequential yaw unwrap. This is the existing
selector, not a new selector implementation. An independent audit maps returned
poses to their original identities.

A runtime Python-call guard counts the official function and blocks repository
optimizer/integration entry points and the official controller runtime before
function bodies execute. Extracted functions have no controller, memory or
integrator in their namespace. Tests inject forbidden calls to prove failure.
Saved-only validation independently checks weighted nearest indices, physical
rows and sequential yaw unwrap without invoking the official selector again.

For every query save pose, tick/time, method/reference identity, nearest row,
physical nearest and H5 poses, original H5 identities/arcs, first/mean/min/max
identity and original arc, and endpoint repetition. Preserve exact unwrapped
selected yaws as well as stored wrapped physical row poses.

Progress-reset deltas are modified minus Native at the identical pose. Save
nearest, first-H5 and mean-H5 row and arc deltas. Primary reset is negative first
H5 original arc below -1e-12 m. Save per-source counts/fractions, maximum backward
first row/arc, mean first-arc delta, other negative counts and first differing
Half/Native, Full/Native and Half/Full H5 query ticks. A source has reset if any
matched tick has negative first arc; systematic means at least 3/4 sources;
persistent means two consecutive matched submit ticks.

For the B-state table, C0-C3 are evaluated at B. Official rows come from the first
legal matched query, whose state differs from B for S1-S3. Report that query pose,
tick and elapsed time. Report original-identity geometry relative to B separately
from the physical modified target's B gap. The world plot marks physical official
H5-start targets. It does not pretend every first query was at B.

### Descriptive safety

For C1-C3, use the existing direct Hospital/cart checker on the straight B-to-target
connector, circular radius .20 m, required footprint-edge clearance .05 m, unchanged
workspace convention and numerical reserve. Label it **hypothetical straight
connector diagnostic only**. Record minimum clearance, geometric .05 m threshold
and full checker validity. Never reject a correspondence because this hypothetical
line fails. Also check the complete remaining original suffix, including its
interpolated first point. No connector or suffix is executed or repaired.

### Interpretation frozen before evaluation

Join saved original-FRESH .9 s position AUC, complete-dwell sustained attachment
(including nulls), endpoint error and Half-minus-Full only after all matched-state
queries finish. No fitted relationship or scalar score.

1. TECHNICAL_BLOCKED: unavailable/authentication/extraction/validation failure.
2. TRANSPORT_PROGRESS_RESET_SUPPORTED: Full resets in >=3 sources and persistent
   reset coexists with worse Full execution in at least one source. Worse means
   .9 AUC > Native+1e-9 or later/missing attachment when Native attaches. This supports
   a plausible mechanism, not causality. Next study: explicit entry l* on original
   FRESH while preserving downstream progress, not another whole-prefix transport.
3. CORRESPONDENCE_WITHOUT_RESET: every C1/C2/C3 selects positive progress in all
   sources, while neither Half nor Full resets in >=3 sources. Investigate
   controller/reference compatibility or another entry objective first.
4. MIXED_CORRESPONDENCE_EFFECT: remaining valid cases; report differing source
   effects and identify geometry distinctions before choosing a global rule.

No graph factor, new optimization method, bridge, crop execution, controller-aware
factor, selector replacement, alpha tuning, timing parameterization, GP or source
search is implemented. EXP-02A's historical incoming-motion formulation and oracle
annotations are not reinterpreted; that report did not validate WHICH-k selection.

### Outputs and execution discipline

Authenticate, freeze protocol/code/input hashes and matched poses, test synthetic
numerics and historical regressions, review diff, commit and normal-push before
new correspondence evaluation or official-selector queries on these sources.
Then evaluate once, validate saved results, create exactly three final PNGs:

1. `correspondence_world_overview.png`: four equal-axis world panels, OLD, original
   rows, B, C1-C3, first physical Native/Half/Full H5 targets and nearby geometry.
2. `selector_progress_reset.png`: first H5 original arc versus matched submit ticks.
3. `correspondence_rule_summary.png`: C0-C3 progress, B position/yaw gaps, remaining
   arc and a separate saved-attachment table.

No full execution trajectory is drawn in the first figure. Nested open markers,
styles and numeric sidecars expose overlaps. No HTML or extra final PNGs.
Small summaries/CSV/PNG are tracked; detailed derived records stay in ignored data.
Raw and existing derived artifacts remain separate and immutable.

## Repository-confirmed facts

Scientific freeze: `a3e2066c91f3e764827bdece04cdf341338b03e4`, pushed before the single
saved-only evaluation. Starting SHA: `1456e8eabd0e84b37203f54fdf567f53de06cf46`.
Frozen classification: **TRANSPORT_PROGRESS_RESET_SUPPORTED**.
Full selected an earlier original H5-start identity at **37/37 matched states**
across all four sources. Half did so at **31/37 states**. Every source had persistent
Full reset and a higher saved Full original-FRESH .9 s position AUC than Native.
These are matched selector facts plus separately reused execution observations.

### Authentication and counts

Both prior complete result ledgers passed. The saved transport, multisource, R00
and R01 validators passed before and after evaluation. No reference was regenerated.
The maximum installed-versus-saved world-array roundoff was
`1.7763568394002505e-14`; exact saved installed arrays were used, within the frozen
1e-12 check. Original observation anchoring and A/B were unchanged. Official whole
source and both extracted function source/AST hashes are in `freeze_summary.json`.

111 scientific official-selector calls; optimizer=0, numerical MPC=0, rollouts=0,
state integrations=0, memory updates=0, command applications=0, LightNav=0, RGB=0,
Isaac=0. Saved validators invoke the official selector zero times; each new
validation pass independently checks the 111 saved outputs. Synthetic test calls
and historical saved-step verification are not scientific queries or rollouts.

Initial focused tests: 35 passed. The final new namespace has 36 passing cases
inside the **537 passed, 1 skipped** regression run (113.97 s). The skip requires
an absent ignored historical EXP-01B/EXP-02B corpus. Compileall, working/staged diff
checks, source authentication, all four historical validators, saved metrics,
111 independent selection checks and exactly-three-PNG checks passed.
No protocol deviation, retry, source change or post-freeze code/config change.
Optional Genuine13 was not performed.

### Continuous correspondence at exact B

All rows below are continuous original-FRESH geometry, not selected raw rows.
Yaw/tangent gaps are unsigned shortest angles. Segment indices start at zero;
alpha is interpolation within the segment. C0 is the conceptual row-0 baseline.
Full machine precision, target world poses and diagnostic nearest identities are
in `b_correspondence.csv` and `result_summary.json`.

| Source | Rule | Original arc m | s | Segment / alpha | B gap m | Yaw gap deg | Tangent gap deg | Remaining m |
|---|---|---|---|---|---|---|---|---|
| S1 | C0 | 0.000000000 | 0.000000000 | 0 / 0.000000000 | 0.213324006 | 18.000818 | 29.992462 | 1.353245520 |
| S1 | C1 | 0.184736536 | 0.136513687 | 1 / 0.225788693 | 0.106648569 | 30.000556 | 30.002488 | 1.168508984 |
| S1 | C2 | 0.094471509 | 0.069811063 | 0 / 0.626650051 | 0.139727568 | 25.520554 | 29.992462 | 1.258774011 |
| S1 | C3 | 0.184742738 | 0.136518271 | 1 / 0.225829908 | 0.106648569 | 30.000556 | 30.002488 | 1.168502782 |
| S2 | C0 | 0.000000000 | 0.000000000 | 0 / 0.000000000 | 0.348704612 | 15.011463 | 15.008894 | 1.354766990 |
| S2 | C1 | 0.324535153 | 0.239550532 | 2 / 0.152721530 | 0.127644309 | 14.999664 | 14.993260 | 1.030231837 |
| S2 | C2 | 0.324580906 | 0.239584304 | 2 / 0.153025637 | 0.127644317 | 14.999661 | 14.993260 | 1.030186084 |
| S2 | C3 | 0.324580906 | 0.239584304 | 2 / 0.153025637 | 0.127644317 | 14.999661 | 14.993260 | 1.030186084 |
| S3 | C0 | 0.000000000 | 0.000000000 | 0 / 0.000000000 | 0.514561811 | 3.390839 | 7.570516 | 1.261338479 |
| S3 | C1 | 0.461931379 | 0.366223172 | 2 / 0.972584595 | 0.184683190 | 22.098125 | 18.829142 | 0.799407100 |
| S3 | C2 | 0.415656585 | 0.329536117 | 2 / 0.663615832 | 0.190392325 | 19.645646 | 18.829142 | 0.845681894 |
| S3 | C3 | 0.461931379 | 0.366223172 | 2 / 0.972584595 | 0.184683190 | 22.098125 | 18.829142 | 0.799407100 |
| S4 | C0 | 0.000000000 | 0.000000000 | 0 / 0.000000000 | 0.394679043 | 18.829682 | 19.649432 | 1.283286410 |
| S4 | C1 | 0.339292066 | 0.264393095 | 2 / 0.220541549 | 0.160884226 | 28.973033 | 30.975649 | 0.943994344 |
| S4 | C2 | 0.305468316 | 0.238035962 | 1 / 1.000000000 | 0.164401278 | 27.416085 | 24.439063 | 0.977818095 |
| S4 | C3 | 0.339292066 | 0.264393095 | 2 / 0.220541549 | 0.160884226 | 28.973033 | 30.975649 | 0.943994344 |

Signed continuous progress differences:

| Source | C2 minus C1 m | C3 minus C1 m | C3 minus C2 m |
|---|---|---|---|
| S1 | -0.090265027030 | +0.000006202696 | +0.090271229726 |
| S2 | +0.000045752921 | +0.000045752921 | +0.000000000000 |
| S3 | -0.046274794243 | +0.000000000000 | +0.046274794243 |
| S4 | -0.033823750629 | +0.000000000000 | +0.033823750629 |

C1/C3 markers overlap strongly: their maximum arc difference is 0.000045753 m. C2/C3 coincide exactly in S2; C1/C3 coincide exactly in S3 and S4. No continuous C1-C3 target has remaining arc <=0.10 m; the minimum remaining arc is 0.799407100 m. All closest XY gaps exceed .10 m, so none of these correspondences places the unmodified robot at B inside the positional attachment tube.

### Matched-state progress reset

Negative deltas below mean earlier original progress. Maxima are backward magnitudes for first H5 row/arc. Mean deltas retain their sign. Nearest-row and mean-H5 deltas, fractions and maxima are also saved in `progress_reset_summary.csv`; all three definitions have the same negative counts in these data.

| Source | Reference | Negative ticks | Fraction | Max rows | Max backward m | Mean delta m | First different tick |
|---|---|---|---|---|---|---|---|
| S1 | Half | 7/9 | 77.778% | 1 | 0.150770002 | -0.117005149 | 96 |
| S1 | Full | 9/9 | 100.000% | 2 | 0.301379857 | -0.200580383 | 96 |
| S2 | Half | 8/9 | 88.889% | 2 | 0.300617984 | -0.167108736 | 1002 |
| S2 | Full | 9/9 | 100.000% | 3 | 0.451508328 | -0.367734851 | 1002 |
| S3 | Half | 8/9 | 88.889% | 2 | 0.315826775 | -0.178336777 | 1806 |
| S3 | Full | 9/9 | 100.000% | 3 | 0.487211386 | -0.387843458 | 1806 |
| S4 | Half | 8/10 | 80.000% | 1 | 0.158703674 | -0.124335798 | 1524 |
| S4 | Full | 10/10 | 100.000% | 2 | 0.313669356 | -0.232342366 | 1524 |

First legal matched query and original identities:

| Source | Tick | Time after B s | Nearest rows N/H/F | First H5 rows N/H/F | First Half vs Full tick |
|---|---|---|---|---|---|
| S1 | 96 | 0.066666670 | 2/1/0 | 3/2/1 | 96 |
| S2 | 1002 | 0.083333338 | 3/1/0 | 4/2/1 | 1002 |
| S3 | 1806 | 0.033333335 | 3/1/0 | 4/2/1 | 1806 |
| S4 | 1524 | 0.000000000 | 2/1/0 | 3/2/1 | 1524 |

The first query differs from B in S1-S3 and equals B only in S4. Full first nearest row is 0 in all four; Native nearest rows are 2,3,3,2. The official lookahead still advances one row: Full first H5 identity is 1, never row 0. The H5 horizon is five raw identities with endpoint repetition. The physical modified targets shown in the world PNG are distinct from original row locations. The CSV provides both the original-identity B geometry (arc, normalized progress, distance/yaw/tangent, remaining arc, segment/alpha) and actual physical-target B position/yaw gaps, separately for nearest and H5-start targets.

First H5 original-identity geometry relative to B:

| Source | Reference | Identity | Original arc m | Original B gap m | Original yaw deg | Physical B gap m | Physical yaw deg |
|---|---|---|---|---|---|---|---|
| S1 | Native | 3 | 0.451861533 | 0.287622431 | 29.994185 | 0.287622431 | 29.994185 |
| S1 | Half | 2 | 0.301251678 | 0.157954727 | 30.000014 | 0.207474769 | 30.101063 |
| S1 | Full | 1 | 0.150756405 | 0.111931080 | 30.000715 | 0.143973321 | 30.110291 |
| S2 | Native | 4 | 0.602176150 | 0.305572358 | 14.987135 | 0.305572358 | 14.987135 |
| S2 | Half | 2 | 0.301558166 | 0.129695842 | 15.001241 | 0.201792464 | 15.117236 |
| S2 | Full | 1 | 0.150768970 | 0.215585319 | 15.008857 | 0.283987134 | 15.129685 |
| S3 | Native | 4 | 0.622519581 | 0.259339869 | 29.457592 | 0.259339869 | 29.457592 |
| S3 | Half | 2 | 0.316265676 | 0.235215598 | 14.378112 | 0.120181795 | 20.780658 |
| S3 | Full | 1 | 0.166443212 | 0.357461896 | 10.396740 | 0.184054227 | 24.054092 |
| S4 | Native | 3 | 0.458835111 | 0.200435211 | 34.475743 | 0.200435211 | 34.475743 |
| S4 | Half | 2 | 0.305468316 | 0.164401278 | 27.416085 | 0.143895961 | 27.358128 |
| S4 | Full | 1 | 0.150927954 | 0.258761526 | 21.309199 | 0.167739221 | 20.873500 |

### Descriptive geometry safety

Every candidate connector and remaining original suffix passed the unchanged complete-polyline checker. These straight connectors are **hypothetical straight connector diagnostic only**; none is a current reference or execution. Footprint-edge clearance, metres:

| Source | Rule | Connector clearance m | Suffix clearance m | Connector >=.05 + reserve | Suffix >=.05 + reserve |
|---|---|---|---|---|---|
| S1 | C1 | 1.095851731 | 0.229486890 | PASS | PASS |
| S1 | C2 | 1.095851731 | 0.229486890 | PASS | PASS |
| S1 | C3 | 1.095851731 | 0.229486890 | PASS | PASS |
| S2 | C1 | 0.757253287 | 0.824491558 | PASS | PASS |
| S2 | C2 | 0.757253287 | 0.824529094 | PASS | PASS |
| S2 | C3 | 0.757253287 | 0.824529094 | PASS | PASS |
| S3 | C1 | 0.211489855 | 0.211489855 | PASS | PASS |
| S3 | C2 | 0.192205246 | 0.192205246 | PASS | PASS |
| S3 | C3 | 0.211489855 | 0.211489855 | PASS | PASS |
| S4 | C1 | 0.878001183 | 1.029576802 | PASS | PASS |
| S4 | C2 | 0.878001183 | 1.018635465 | PASS | PASS |
| S4 | C3 | 0.878001183 | 1.029576802 | PASS | PASS |

### Saved execution response: observational context only

These are unchanged STATE_SHIFT_TRANSPORT_SCALE_01 metrics against ORIGINAL FRESH. Attachment requires the frozen complete following .30 s dwell; nulls remain null. No execution was performed here.

| Source | Reference | Position AUC .9 s m s | Attachment s | Endpoint error m |
|---|---|---|---|---|
| S1 | Native | 0.169065633 | 1.466666743 | 0.096439021 |
| S1 | Half | 0.169216433 | 1.466666743 | 0.095702993 |
| S1 | Full | 0.169963954 | 1.483333411 | 0.096335462 |
| S2 | Native | 0.131833111 | 1.050000055 | 0.079382367 |
| S2 | Half | 0.131791529 | 1.033333387 | 0.078431173 |
| S2 | Full | 0.136682270 | 1.200000063 | 0.086092224 |
| S3 | Native | 0.189624122 | null (no complete dwell) | 0.115238019 |
| S3 | Half | 0.189328286 | null (no complete dwell) | 0.120314783 |
| S3 | Full | 0.199963659 | null (no complete dwell) | 0.139406215 |
| S4 | Native | 0.198189216 | 1.283333400 | 0.084539314 |
| S4 | Half | 0.204465336 | null (no complete dwell) | 0.101967089 |
| S4 | Full | 0.217335047 | null (no complete dwell) | 0.132316561 |

Half-minus-Full saved context (no combined score):

| Source | AUC delta m s | Attachment delta s (null if unavailable) | Endpoint delta m |
|---|---|---|---|
| S1 | -0.000747521 | -0.016666668 | -0.000632468 |
| S2 | -0.004890741 | -0.166666675 | -0.007661052 |
| S3 | -0.010635374 | null (no complete dwell) | -0.019091432 |
| S4 | -0.012869712 | null (no complete dwell) | -0.030349473 |

### Three PNGs and numeric sidecars

- [correspondence_world_overview.png](../results/spatial_correspondence_selector_diag_01/figures/correspondence_world_overview.png)
- [selector_progress_reset.png](../results/spatial_correspondence_selector_diag_01/figures/selector_progress_reset.png)
- [correspondence_rule_summary.png](../results/spatial_correspondence_selector_diag_01/figures/correspondence_rule_summary.png)

`figure_manifest.json` contains the plotted coordinates, original identities/progress, rule metrics, attachment table and image hashes. The images were visually inspected and checked against numeric sidecars. Dimensions: 2700x2160, 2700x1620 and 2700x1800 pixels, respectively. No extra final PNG or HTML dependency.

## Research interpretation

The four-source matched-state evidence supports a transport-induced reset of
original FRESH progress in the official selector. Full sends the selector back
by as much as 2/3/3/2 rows in S1/S2/S3/S4, at every sampled submit. Half reduces
both the maximum and average backward displacement in all four sources, but
retains reset in 7/9, 8/9, 8/9 and 8/10 queries. Transport changes the relation
between physical row location and original row identity; physical closeness of
a shifted target does not mean downstream original progress is preserved.

The same four sources have worse saved Full .9 s position AUC than Native, and
S4 attaches only with Native. This coexistence supports progress reset as a
plausible mechanism, without identifying its contribution to execution error.
The audit uses saved Native poses only. Controller commands, future states,
terminal behavior and semantic intent under an explicit entry were not tested.

A justified next study is to choose an explicit B-to-original-FRESH entry l* and
preserve downstream original progress. **C3 is a candidate to test**, because
its forward constraint prevents the yaw term from selecting behind the XY
projection. C1 should remain a simple geometry control. C2 retreats relative to
C1 by 9.03 cm in S1, 4.63 cm in S3 and 3.38 cm in S4, while reducing yaw mismatch;
that is a geometric trade-off, not proof of a bad execution target. In S2 the
three rules practically coincide. These observations do not establish one rule
as optimal, and no new factor, crop, bridge or execution was implemented.

The next uncertainty is whether preserving original entry progress can reduce
early error and recover attachment under the unchanged controller while retaining
downstream geometry and safety. Entry selection alone does not resolve how the
robot reaches that spatial target; the geometric closest gaps are already above
the attachment position threshold in every source.

## Limitations

Four fixed sources; frozen references and Native states; no counterfactual
execution; existing outcomes came from an offline timing-controlled comparison,
not real asynchronous deployment timing. Original FRESH has intrinsic tracking
difficulty. No population correlation, causal selector conclusion, semantic intent
proof, controller-aware factor, or general superiority claim.

## Not demonstrated

A new reconciliation method, safer/better execution of any candidate entry,
general graph superiority, real-world improvement, complete obstacle bypass,
online repeated-chunk improvement, or necessity of any factor across sources.

## Reproduction commands

Run from the repository root with the existing repository environment. No system
or external environment is changed. `prepare` authenticates only; `execute` is
permitted only after the frozen commit is pushed. Validation/reporting never
reinvoke the official selector or any numerical solver.

```bash
git fetch origin main
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/correspondence-selector-mpl .venv/bin/python -m pytest -q tests/test_spatial_correspondence_selector_diag01.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_spatial_correspondence_selector_diag01.py --run data/spatial_correspondence_selector_diag_01/primary_20260930T120000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/correspondence-selector-mpl .venv/bin/python -m pytest -q tests/test_spatial_correspondence_selector_diag01.py tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_spatial_entry.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_spatial_correspondence_selector_diag01.py --run data/spatial_correspondence_selector_diag_01/primary_20260930T120000Z --mode freeze
# Review and stage only this namespace plus the append-only work log.
git diff --cached --check
git commit -m "Freeze saved-only spatial correspondence and matched selector diagnostic"
git push origin main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_spatial_correspondence_selector_diag01.py --run data/spatial_correspondence_selector_diag_01/primary_20260930T120000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_spatial_correspondence_selector_diag01.py --run data/spatial_correspondence_selector_diag_01/primary_20260930T120000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/correspondence-selector-mpl .venv/bin/python scripts/report_spatial_correspondence_selector_diag01.py --run data/spatial_correspondence_selector_diag_01/primary_20260930T120000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_spatial_correspondence_selector_diag01.py --run data/spatial_correspondence_selector_diag_01/primary_20260930T120000Z --check-only
# Inspect the three PNGs and their numeric sidecars; append facts and interpretation.
git diff --check
git diff --cached --check
git commit -m "Report matched selector progress reset with three validated PNGs"
git push origin main
```

## Explicit answers to the eleven research questions

1. **Where does B project under C1/C2/C3?** Original arc metres (C1/C2/C3):
   S1 0.184736536 / 0.094471509 / 0.184742738;
   S2 0.324535153 / 0.324580906 / 0.324580906;
   S3 0.461931379 / 0.415656585 / 0.461931379;
   S4 0.339292066 / 0.305468316 / 0.339292066. Normalized progress,
   containing segments and interpolation alphas are in the continuous table.
2. **How different are the rules in original arc metres?** C2-minus-C1 is
   -0.090265027 / +0.000045753 / -0.046274794 / -0.033823751.
   C3-minus-C1 is +0.000006203 / +0.000045753 / 0 / 0. C3 prevents the
   C2 backward shifts in S1/S3/S4. The full signed difference table is above.
3. **Does Full select earlier progress at identical states?** Yes, all 37/37
   matched queries. Maximum backward first-H5 arc: 0.301379857 / 0.451508328 /
   0.487211386 / 0.313669356 m for S1-S4.
4. **Does Half reduce reset?** Yes in every source by maximum and mean first-H5
   arc displacement; its maximum backward arcs are 0.150770002 / 0.300617984 /
   0.315826775 / 0.158703674 m. Reset remains at 31/37 queries.
5. **Is the effect consistent?** The Full backward direction and persistent
   effect occur in all four. Magnitudes and Half coincidence with Native vary.
6. **Is it present at the first matched submit?** Yes for Half and Full in all
   four, at ticks 96/1002/1806/1524. Half and Full also differ at those first
   ticks. Native/Half/Full first H5 identities are 3/2/1, 4/2/1, 4/2/1, 3/2/1.
7. **Does reset coexist with worse execution?** Yes: Full .9 s position AUC is
   higher than Native in all four saved outcomes. Full attaches later in S1/S2;
   all methods lack attachment in S3; only Native attaches in S4. This is
   observational context, not a causal execution result of this diagnostic.
8. **Are correspondences near the exhausted endpoint?** No C1-C3 target is
   within the frozen 0.10 m remaining-arc label. Minimum remaining arc is
   0.799407100 m, at S3 C1/C3.
9. **Are candidate suffixes safe?** All 12 original suffixes pass the unchanged
   direct geometry check. All 12 hypothetical straight connectors also pass;
   minimum clearance across these checks is 0.192205246 m. This certifies the
   supplied geometric polylines only, not a future robot trajectory.
10. **Which rule is justified for the next study?** C3 is a defensible candidate
    for an explicit forward entry, with C1 as control. No rule has demonstrated
    execution superiority. C2's earlier yaw-favoring entries remain a trade-off.
11. **Does evidence support an explicit B-to-FRESH spatial-entry direction?**
    Yes as the next bounded research question: progress reset is directly
    observed at matched states, including Full nearest-row 0 in every first
    query. It does not yet justify claiming a successful replacement method.
