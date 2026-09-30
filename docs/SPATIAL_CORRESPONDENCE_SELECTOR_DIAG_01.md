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

Pending pushed freeze and single saved-only evaluation. No new scientific result
is claimed in this protocol commit.

## Research interpretation

Pending evaluation. A proximity-based correspondence does not validate semantic
intent or establish a best entry rule.

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
