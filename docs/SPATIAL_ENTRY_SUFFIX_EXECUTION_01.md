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

Preflight: four exact historical C3 matches, four successful zero-solve controller
restorations; original downstream installed arrays match Native exactly. Source,
reference and schedule evidence is in `results/spatial_entry_suffix_execution_01/freeze_summary.json`.
Scientific execution results will be appended after the pushed freeze.

## Research interpretation

No new execution interpretation before the freeze. The experiment tests whether
progress-preserving suffix selection helps under the existing controller.

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
