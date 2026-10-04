# B_TO_ENTRY_BOUNDARY_ROW_ABLATION_04

## Frozen protocol

Starting HEAD and fetched origin/main: `56f33a145fcd0809f4d9947cbd8813c3b83f4e24`.
**Timing-controlled offline causal reference comparison.** Four development sources.
Run identifier: `data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z`.
This is an identifier; actual host UTC execution start is recorded separately.

Question: is inserting the fixed current boundary B before frozen C3 entry E*,
without an intermediate pose X1, sufficient to recover S3 attachment AND
original-FRESH endpoint dwell? If so, does it reduce Hermite/V2's easy-source
endpoint delay? This is a reference-staging ablation. No new graph formulation,
optimization, smoothing, correspondence, controller or time parameterization.

| Source | Exact source | Development role |
|---|---|---|
| S1 | OSA03_R00 | Obstacle / straight-to-detour |
| S2 | episode_001_repeat_01/handoff_013 | Mild nearly-straight |
| S3 | episode_008_repeat_01/handoff_023 | Progressive-left-turn hard case |
| S4 | episode_013_repeat_00/handoff_020 | Progressive-right-turn |

### Authentication and historical reuse

Authenticate V2 result commit `56f33a145fcd0809f4d9947cbd8813c3b83f4e24`,
scientific freeze `7bf24d60b87774859ffef3f6d56f524a4fb7ad15`, result SHA256
`7c70358be5eb509ee3698e70bd4598b65863a53cb1ca9f6f3d96d63034fdaeed`.
Authenticate its complete result/input/code chain, DIAG_02, bridge01, C3,
correspondence, transport, multisource, R00 and R01 through existing saved-only
validators. Historical validators reproduce saved algebra and integration for
parity; these are checks, not new scientific planning solutions or rollouts.
No Hermite or V2 reference is generated for this experiment. The new builder
only stacks saved B/E* and exact Native rows. Native/C3/Hermite/V2 arrays,
metrics and execution files remain byte-for-byte unchanged and are never rerun.

Official MPC SHA256 remains
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
World XY metres, yaw radians CCW, +Z up; observation-local x forward/y left.
Original FRESH remains `F_j=A F_local,j`. A and B are fixed. No B re-anchor.
Observation, ready and switch/application timestamps retain distinct saved fields.
No waypoint timestamps or dt are assigned.

### Structural comparison and exact reference

| Method | Structure | Execution |
|---|---|---|
| C3 | `[E*, F...]` | Historical reuse |
| B_ENTRY_STAGE | `[B, E*, F...]` | Only new condition |
| Hermite | `[B, X1_H, E*, F...]` | Historical reuse |
| V2 | `[B, X1_V2, E*, F...]` | Historical reuse |

Copy exact planning B from common state and E* from authenticated C3. Use saved
C3 duplicate handling: its first entry/vertex appears once, followed only by
its downstream original identities. E* keeps its original fractional row and
arc metadata. B (and historical bridge_1) has no fabricated original identity.
The suffix after E* copies exact Native-installed world rows and original raw
A-local rows. Only derived B/E* use `A^-1 X` for the unchanged adapter.
Record local/world file hashes, installed array hash and derived roundoff;
require <=1e-12 roundoff and bit-identical installed original suffix.
The generic builder supports the frozen vertex-dedup convention and N>=2.
All four scientific references require positive original remaining arc.

Zero-numerical-MPC-solve preflight uses the existing official initializer with
its solve entry point forbidden. Check arbitrary reference length, exact
boundaries, suffix, previous_control and generation. Technical failure blocks
execution. No X1, Hermite construction, optimization, resampling, new yaw,
transport or new correspondence is used in reference construction/execution.

### Safety, state and logical timing

The actual checked full polyline is **B -> E* -> original suffix**, including
the newly staged B-to-entry edge. The legacy checker query text can contain
"no B connector"; it refers to not implicitly adding a connector. Here B is
explicitly in the supplied array and every supplied edge is checked. The
installation record explicitly names the checked polyline.

Unchanged circular radius .20 m, required footprint-edge clearance .05 m,
Hospital/cart/workspace conventions and 1e-7 numerical reserve. Check both
planning and installed reference. Unsafe reference: record, no repair, no
execution. Abort-only runtime guard: never apply an unsafe proposed command;
retain valid prefix, no retry or fabricated continuation.

Copy exact common-state and schedule bytes from V2, which authenticated the
bridge/C3 schedules. Preserve B pose/tick/time, physical u_minus provenance,
already-applied u_B_plus, previous_control/u_mem_B, generation/version,
integration dt, source geometry and absolute submit/release ticks.
Reuse `run_relative_factor_multisource01.run_method` unchanged. Official H=5,
nearest+1, Q/R, limits, acceleration limits, endpoint repetition, solver,
previous_control update order and stale/generation logic are unchanged.
Host wall waiting cannot advance logical simulation or choose application ticks.

| Source | B tick | First legal submit tick |
|---|---:|---:|
| S1 | 92 | 96 |
| S2 | 997 | 1002 |
| S3 | 1804 | 1806 |
| S4 | 1524 | 1524 |

Require attempted/accepted submit and successful application sequences,
interval counts and initial provenance to match all historical methods through
54 primary and all 180 cap intervals. Saved integration dt is
0.01666666753590107 s; nominal .9/3 s correspond to
.900000046938658/3.0000001564621925 s. Explained safety/numerical failures are
censored; unexplained restoration/schedule failures are TECHNICAL_BLOCKED.

### Evaluation and diagnostics

Primary target: full ORIGINAL FRESH. Reuse forward-only continuous projection,
shortest-angle yaw, initial errors, max XY first .5 s, separation growth,
position/yaw AUC .3/.9, sustained attachment, fractional original row/arc and
remaining arc at attachment, original-FRESH endpoint dwell and cap metrics.
Both dwell tests require <=.10 m XY and <=15 deg yaw through the complete
following .30 s sampled interval, with the historical 1e-12 time tolerance.
Endpoint dwell refers to the ORIGINAL FRESH final pose, not a navigation goal.
No complete dwell means null, never the cap. T_post_attach only exists when
both dwell times are observed. Include first endpoint tube entry, cap endpoint
error/projection/remaining arc, path length to dwell and mean abs(v) before dwell.
Report swept clearance lower bound, linear/angular TV, max abs(v/omega),
termination and abort tick. New own-reference metrics are secondary only.

Use successful official selection records generated naturally during each
rollout, including submitted results withheld beyond the cap. Every row records
tick/elapsed time, actual input pose, nearest identity, first/all H5 identities,
physical H5 poses, original raw/fractional/arc metadata, B/E*/bridge_1 presence
and first-H5 original arc where defined. No new selector experiment. Require
identical actual first legal submit states before the central selector comparison.
Do not infer first targets from reference row order or claim every first submit
is at B. Report nearest=B count, first H5=E* count, E* inclusion count, last
B/E* sample, first original H5 start and first-to-last sampled exposure span.
No continuous exposure claim or independent selector causality claim.

Compare C3->B_ENTRY, B_ENTRY->Hermite, B_ENTRY->V2 at matched successful submit
and applied interval ticks. Delta means right minus left. Report first different
H5 identities, first different submit/application command, v/omega delta and
actual pose separation at all common saved state ticks (including terminal).
Shared historical command is COMMON_B_COMMAND and excluded from post-reference
interpretation. Command difference threshold: abs(component)>1e-9.

### Classification precedence and direction

One discrete time change requires >=`integration_dt_s-1e-9` seconds.
Other scalar equality tolerance is 1e-9 native units; retain stricter historical
rules. No statistical significance or scalar winner score.
S3 recovery requires BOTH attachment and original-FRESH endpoint dwell.
For easy S1/S2/S4, delay = method endpoint time minus C3 endpoint time.

1. **TECHNICAL_BLOCKED**: authentication/construction/restoration/interface,
   unexplained schedule, validator or implementation failure.
2. **BOUNDARY_STAGE_REFERENCE_OR_EXECUTION_FAILURE**: unsafe new reference,
   official numerical controller failure or attributable guard abort; no repair.
3. **INTERMEDIATE_BRIDGE_NEEDED_SUPPORTED**: B_ENTRY lacks either S3 dwell,
   while authenticated Hermite/V2 both have both. Direction **B**: intermediate
   geometry is supported in S3 under this fixed setup; graph superiority remains
   unresolved because deterministic Hermite also succeeds.
4. **STAGING_SUFFICIENT_AND_PENALTY_REDUCED**: B_ENTRY recovers S3, all easy dwells
   remain, delay no worse than Hermite within 1e-9 in all three easy sources,
   at least one tick reduction in at least two easy sources, safety passes.
   Direction **A**: investigate simpler progress/staging before any graph claim.
5. **STAGING_SUFFICIENT_NO_PENALTY_GAIN**: S3 recovered, easy dwells remain,
   no >=one-tick worsening beyond Hermite, but reduction criterion unmet.
   Direction **C**: investigate selector/reference-topology interaction.
6. **STAGING_RECOVERY_WITH_NEW_TRADEOFF**: S3 recovered but any easy dwell lost
   or >=one tick later than Hermite. Direction **C**.
7. **MIXED_STAGING_EFFECT**: remaining scientifically valid cases.

Technical/scientific failure has no invented A/B/C direction; record unavailable
until its prerequisite is resolved. No next-direction implementation in this task.
Graph necessity/superiority is not established by failure of staging alone.

### Freeze, calls and outputs

Before science: authenticate, construct references, zero-solve preflight, freeze
identity maps/safety/schedules/metrics/classification/code, focused/regression
tests, compileall, diff review, commit and normal push. Record pushed freeze SHA.
Then exactly one B_ENTRY_STAGE rollout S1/S2/S3/S4, no retry. Unsafe source is
skipped under the stated gate. Runtime forbids reconciliation optimizer entry
points. Expected new calls: four rollouts, 120 official MPC solves from frozen
schedules; optimizer=0, historical reruns=0, V3=0, LightNav/RGB/Isaac=0.
A scientific failure is retained and reported; no result-driven method change.

After science: saved-only validation, exactly three compact PNGs and numerical
sidecars, visual/numeric inspection, facts/interpretation/limitations, 22 explicit
answers, append-only work log, conservative README, result/docs commit and push.
Large arrays stay ignored under data. No system/external environment changes.

Exactly three final PNGs (no HTML):

1. `world_execution_overview.png`: equal-axis S1-S4 actual execution, original,
   OLD-to-B, fixed B/E*, attachment/endpoint markers and nearby geometry.
2. `staging_transition_completion.png`: position AUC .9, attachment, original
   arc fraction/remaining arc at attachment, endpoint dwell; yaw AUC context.
3. `selector_staging_diagnostic.png`: four structures, first actual H5/nearest,
   sampled E* exposure count/span, easy endpoint delays and S3 both-dwell status.

Use distinct line styles and staggered markers for overlap; explicit N/A.
Machine outputs: result_summary.json, primary.csv, structural_comparison.csv,
paired_timing.csv, selector_exposure.csv, command_comparison.csv,
reference_geometry.csv, figure_manifest.json. CSV uses LF. Sidecars and SHA256
bind the plots to validated saved numbers.

## Limitations and claim boundaries

Four development sources and one execution per new method/source. Controlled
offline logical timing, intrinsic original-FRESH tracking difficulty, fixed
controller and no held-out acquisition. No graph/Hermite superiority, final
method, obstacle/population generalization, real asynchronous deployment or
real-world navigation benefit, navigation-task completion or C3/B_ENTRY
optimality is demonstrated. Spatial geometry is untimed; actual omega comes
from the unchanged official MPC.

## Prepared references (before execution)

All four references and zero-numerical-solve official installations pass.
Planning B/E* bits are exact. Installed original suffix bits match Native.

| Source | Rows | E* arc [m] | World file SHA256 | Installed clearance [m] | Derived roundoff max |
|---|---:|---:|---|---:|---:|
| S1 | 10 | 0.18474273839738128 | `fa72a4f109dc7a1d7113131c8471a55aaf194dc2cab81fbbeea55e846deddb59` | 0.22948689016047874 | 3.552713678800501e-15 |
| S2 | 9 | 0.3245809061559166 | `b7ee7e0146385624797af133f58bfc975e4c6a1f21bb9d1cfd022d1c5bf6be48` | 0.7572532867773836 | 3.552713678800501e-15 |
| S3 | 9 | 0.46193137914572885 | `2f4605a65239153e6bf21acfb893a58ae8e04f63e242ffaf59f9b4fd11b7c18a` | 0.21148985508008777 | 1.7763568394002505e-14 |
| S4 | 9 | 0.3392920662039436 | `33f79be1e89f208b317a5e692e7eb6f2cf4553c9ddff29257f1e5bc5883d5718` | 0.8780011873414322 | 5.329070518200751e-15 |

## Reproduction commands

Use the existing repository .venv. Do not rerun scientific execution once its
exclusive start records exist. The report/validator only read saved rollouts.

```bash
git fetch origin main
git rev-parse HEAD origin/main
git status --short
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_boundary_row_ablation04.py --run data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/boundary04-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_boundary_row_ablation04.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/boundary04-regression-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_boundary_row_ablation04.py tests/test_b_to_entry_vector_bridge_execution03.py tests/test_b_to_entry_graph_formulation_diag02.py tests/test_b_to_entry_bridge01.py tests/test_spatial_entry_suffix_execution01.py tests/test_spatial_correspondence_selector_diag01.py tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_spatial_entry.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_boundary_row_ablation04.py --run data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z --mode freeze
git diff --cached --check
git commit -m "Freeze boundary-row staging ablation with unchanged official execution"
git push origin main
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_boundary_row_ablation04.py --run data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_boundary_row_ablation04.py --run data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/boundary04-mpl .venv/bin/python scripts/report_b_to_entry_boundary_row_ablation04.py --run data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_boundary_row_ablation04.py --run data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z --check-only
```

### Pre-freeze verification

Focused coverage: **35 passed** (31 in 70.80 s plus four added edge/identity
checks in .44 s). Relevant regression including all 35: **696 passed, 1 skipped
in 403.31 s**. The known skip requires the absent ignored EXP-01B/EXP-02B corpus.
An initial test pass found a wrong synthetic vertex-count expectation and a
misnamed plotting metric key; both were fixed before freeze, with no scientific
calls. All three fixture PNG layouts and explicit nulls were visually inspected.
Compileall and working/staged diff checks pass. No historical scientific code,
controller or environment was modified; unrelated Stage0/GPU-script edits remain.
