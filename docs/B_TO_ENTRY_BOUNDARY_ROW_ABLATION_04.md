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

## Repository-confirmed facts

Scientific freeze: `94f1a70fa5721f15bca4287dbda2ff0b3634f659`. Actual execution start UTC: `2026-10-04T07:10:27.532401+00:00`.

Frozen classification: **STAGING_SUFFICIENT_AND_PENALTY_REDUCED**. Scientific direction: **A**.

### Scientific calls and comparability

| Counter | Value |
| --- | --- |
| optimizer_calls | 0 |
| new_B_ENTRY_rollouts | 4 |
| MPC_solves | 120 |
| MPC_applications | 119 |
| historical_rollouts_reused | 16 |
| historical_reruns | 0 |
| V3_rollouts | 0 |
| LightNav | 0 |
| RGB | 0 |
| Isaac | 0 |
| retries | 0 |

All historical references/executions are authenticated without rerun. New records are exclusively B_ENTRY_STAGE, in S1-S4 order. No optimizer, source acquisition, retry or result-driven change. S2 retains the scheduled final successful result beyond the cap; this is not a missing solve.

All twenty historical/new method state and schedule gates pass through both 54 and 180 intervals. First legal submit state is exactly identical within each source; later matched ticks can have different robot states. The actual fixed cap is 3.0000001564621925 s.

### Central structural and execution comparison

Times are actual execution seconds after B. Endpoint denotes **original-FRESH endpoint dwell**, not a navigation goal. Null remains N/A. XY AUC units m s; yaw AUC rad s; arc, endpoint error and clearance metres. Tables are rounded; machine artifacts retain full precision.

| Source | Method | Structure | XY AUC .9 | T_attach | Original row | Arc fraction | Arc remaining | T_endpoint | T_post_attach | Endpoint error at cap |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | C3 (saved) | [E*, F...] | 0.169065632382 | 1.46666674316 | 8.71784636692 | 0.968605161337 | 0.042484924776 | 1.51666674577 | 0.0500000026077 | 0.096439021209 |
| S1 | B Entry (new) | [B, E*, F...] | 0.152550131241 | 1.36666673794 | 7.19083642322 | 0.799187280619 | 0.271748912885 | 1.65000008605 | 0.28333334811 | 0.0706625092125 |
| S1 | Hermite (saved) | [B, X1_H, E*, F...] | 0.157503974584 | 1.46666674316 | 7.71413414341 | 0.857096752655 | 0.19338317928 | 1.66666675359 | 0.200000010431 | 0.077380066851 |
| S1 | V2 (saved) | [B, X1_V2, E*, F...] | 0.157505673387 | 1.46666674316 | 7.7141316191 | 0.857096473308 | 0.193383557305 | 1.66666675359 | 0.200000010431 | 0.0773826817703 |
| S2 | C3 (saved) | [E*, F...] | 0.13183311076 | 1.05000005476 | 7.69045316001 | 0.854497877738 | 0.197121472188 | 1.25000006519 | 0.200000010431 | 0.0793823670714 |
| S2 | B Entry (new) | [B, E*, F...] | 0.129118949918 | 1.03333338723 | 7.07630220224 | 0.786240921879 | 0.289593742811 | 1.33333340287 | 0.300000015646 | 0.071324359722 |
| S2 | Hermite (saved) | [B, X1_H, E*, F...] | 0.132991596869 | 1.10000005737 | 7.42772733102 | 0.825298435669 | 0.236679912424 | 1.35000007041 | 0.250000013039 | 0.0760370052558 |
| S2 | V2 (saved) | [B, X1_V2, E*, F...] | 0.132992620512 | 1.10000005737 | 7.42772686348 | 0.825298383707 | 0.23667998282 | 1.35000007041 | 0.250000013039 | 0.0760381621995 |
| S3 | C3 (saved) | [E*, F...] | 0.186553953328 | N/A | N/A | N/A | N/A | N/A | N/A | 0.11235439506 |
| S3 | B Entry (new) | [B, E*, F...] | 0.177747462471 | 1.23333339766 | 6.35342889995 | 0.801186144954 | 0.250771565525 | 1.4833334107 | 0.250000013039 | 0.0652146944587 |
| S3 | Hermite (saved) | [B, X1_H, E*, F...] | 0.182253440506 | 1.46666674316 | 6.09489906865 | 0.767945556311 | 0.292699199044 | 1.76666675881 | 0.300000015646 | 0.0604422053494 |
| S3 | V2 (saved) | [B, X1_V2, E*, F...] | 0.181902995367 | 1.4833334107 | 6.20433021787 | 0.782015715247 | 0.274951966173 | 1.76666675881 | 0.28333334811 | 0.0615202442102 |
| S4 | C3 (saved) | [E*, F...] | 0.198189215821 | 1.28333340026 | 7.78827222868 | 0.912753530112 | 0.11196220915 | 1.40000007302 | 0.116666672751 | 0.08453931375 |
| S4 | B Entry (new) | [B, E*, F...] | 0.166655717127 | 1.08333338983 | 5.97990356958 | 0.721966940773 | 0.356796046506 | 1.46666674316 | 0.383333353326 | 0.0439289166791 |
| S4 | Hermite (saved) | [B, X1_H, E*, F...] | 0.158973356936 | 1.05000005476 | 5.2885398519 | 0.637306486163 | 0.465439657392 | 1.55000008084 | 0.500000026077 | 0.0304487538869 |
| S4 | V2 (saved) | [B, X1_V2, E*, F...] | 0.159008123879 | 1.05000005476 | 5.29308608443 | 0.637863191849 | 0.46472524455 | 1.55000008084 | 0.500000026077 | 0.0306349818938 |

### Initial and early original-FRESH errors

| Source | Method | Initial XY | Initial yaw | Max XY .5 | Separation growth | XY AUC .3 | Yaw AUC .3 | Yaw AUC .9 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | C3 (saved) | 0.106648569289 | 0.523608487599 | 0.215262376486 | 0.108613807198 | 0.0479203622588 | 0.122763624543 | 0.193368049369 |
| S1 | B Entry (new) | 0.106648569289 | 0.523608487599 | 0.186462519119 | 0.0798139498304 | 0.0461666319548 | 0.122797883019 | 0.1858397597 |
| S1 | Hermite (saved) | 0.106648569289 | 0.523608487599 | 0.192250391649 | 0.0856018223601 | 0.0461978105283 | 0.124983475826 | 0.196380784635 |
| S1 | V2 (saved) | 0.106648569289 | 0.523608487599 | 0.192252120037 | 0.0856035507481 | 0.0461978215005 | 0.124984232387 | 0.196384208514 |
| S2 | C3 (saved) | 0.127644308504 | 0.261793531143 | 0.163894949098 | 0.0362506405941 | 0.0453024911898 | 0.0456241196881 | 0.107103977941 |
| S2 | B Entry (new) | 0.127644308504 | 0.261793531143 | 0.158597381486 | 0.0309530729816 | 0.0446600151784 | 0.0460382342301 | 0.10365780552 |
| S2 | Hermite (saved) | 0.127644308504 | 0.261793531143 | 0.161568396677 | 0.0339240881724 | 0.0447513464311 | 0.0497156647152 | 0.103789438819 |
| S2 | V2 (saved) | 0.127644308504 | 0.261793531143 | 0.16156949475 | 0.0339251862459 | 0.0447513906534 | 0.0497169574747 | 0.103790512186 |
| S3 | C3 (saved) | 0.184683189977 | 0.385685031403 | 0.235029318754 | 0.0503461287766 | 0.06369986862 | 0.0749714888083 | 0.173208230511 |
| S3 | B Entry (new) | 0.184683189977 | 0.385685031403 | 0.209455483839 | 0.0247722938618 | 0.0603035822117 | 0.0675951585427 | 0.15458442242 |
| S3 | Hermite (saved) | 0.184683189977 | 0.385685031403 | 0.205591408233 | 0.020908218256 | 0.0600195353242 | 0.0750877706482 | 0.123986312039 |
| S3 | V2 (saved) | 0.184683189977 | 0.385685031403 | 0.206070181364 | 0.0213869913868 | 0.0600196074788 | 0.0750934336363 | 0.130565658586 |
| S4 | C3 (saved) | 0.16088422589 | 0.505674822974 | 0.252791514218 | 0.0919072883284 | 0.0611986169982 | 0.120660784385 | 0.230599467879 |
| S4 | B Entry (new) | 0.16088422589 | 0.505674822974 | 0.207896900254 | 0.0470126743642 | 0.0558628355411 | 0.113525417039 | 0.214590525788 |
| S4 | Hermite (saved) | 0.16088422589 | 0.505674822974 | 0.191420918781 | 0.0305366928912 | 0.0543860644842 | 0.112389487606 | 0.205457144905 |
| S4 | V2 (saved) | 0.16088422589 | 0.505674822974 | 0.191469987566 | 0.0305857616764 | 0.0543877354846 | 0.112387242799 | 0.205355811925 |

### Easy-source endpoint delays versus C3

| Source | B Entry delay [s] | Hermite delay [s] | V2 delay [s] | B minus Hermite [s] | B improves >=one tick |
| --- | --- | --- | --- | --- | --- |
| S1 | 0.133333340287 | 0.150000007823 | 0.150000007823 | -0.0166666675359 | True |
| S2 | 0.0833333376795 | 0.100000005215 | 0.100000005215 | -0.0166666675359 | True |
| S4 | 0.0666666701436 | 0.150000007823 | 0.150000007823 | -0.0833333376795 | True |

### Other downstream descriptors

| Source | Method | First endpoint tube [s] | Row at cap | Arc fraction at cap | Arc left at cap [m] | Path to dwell [m] | Mean abs(v) before dwell |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | C3 (saved) | 1.51666674577 | 9 | 1 | 0 | 1.18000006242 | 0.778021978599 |
| S1 | B Entry (new) | 1.65000008605 | 9 | 1 | 0 | 1.13333338935 | 0.686868684996 |
| S1 | Hermite (saved) | 1.66666675359 | 9 | 1 | 0 | 1.1435514989 | 0.686130863553 |
| S1 | V2 (saved) | 1.66666675359 | 9 | 1 | 0 | 1.14355172864 | 0.6861310014 |
| S2 | C3 (saved) | 1.25000006519 | 9 | 1 | 0 | 0.984486552292 | 0.787589200758 |
| S2 | B Entry (new) | 1.33333340287 | 9 | 1 | 0 | 0.974359720237 | 0.730769752065 |
| S2 | Hermite (saved) | 1.35000007041 | 9 | 1 | 0 | 0.984110691019 | 0.728970844218 |
| S2 | V2 (saved) | 1.35000007041 | 9 | 1 | 0 | 0.984110836857 | 0.728970952246 |
| S3 | C3 (saved) | N/A | 9 | 1 | 0 | N/A | N/A |
| S3 | B Entry (new) | 1.4833334107 | 9 | 1 | 0 | 0.823867774837 | 0.555416448451 |
| S3 | Hermite (saved) | 1.76666675881 | 9 | 1 | 0 | 0.818252928244 | 0.463162010699 |
| S3 | V2 (saved) | 1.76666675881 | 9 | 1 | 0 | 0.822209652303 | 0.46540166571 |
| S4 | C3 (saved) | 1.40000007302 | 9 | 1 | 0 | 1.06748481022 | 0.762489110393 |
| S4 | B Entry (new) | 1.46666674316 | 9 | 1 | 0 | 0.975836578732 | 0.665343087163 |
| S4 | Hermite (saved) | 1.55000008084 | 9 | 1 | 0 | 0.948575075935 | 0.611983888041 |
| S4 | V2 (saved) | 1.55000008084 | 9 | 1 | 0 | 0.949292925697 | 0.612447016895 |

### Safety and control

| Source | Method | Swept clearance [m] | Linear TV | Angular TV | Max abs(v) | Max abs(omega) | Termination | Abort tick |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | C3 (saved) | 0.133561094659 | 0.800000015802 | 3.62955576994 | 0.8 | 1.54833365799 | OBSERVATION_CAP | N/A |
| S1 | B Entry (new) | 0.1591870121 | 2.00000002628 | 3.41328397183 | 0.8 | 1.49999842928 | OBSERVATION_CAP | N/A |
| S1 | Hermite (saved) | 0.152790686503 | 2.00000006058 | 3.80325774564 | 0.8 | 1.24264920207 | OBSERVATION_CAP | N/A |
| S1 | V2 (saved) | 0.152788103572 | 2.00000006051 | 3.80304101355 | 0.8 | 1.24256462102 | OBSERVATION_CAP | N/A |
| S2 | C3 (saved) | 0.757224041072 | 0.80604106892 | 2.546626244 | 0.8 | 1.05284528875 | OBSERVATION_CAP | N/A |
| S2 | B Entry (new) | 0.757232394509 | 1.60604109177 | 2.40679145304 | 0.8 | 1.00282879941 | OBSERVATION_CAP | N/A |
| S2 | Hermite (saved) | 0.75723460538 | 1.60604109889 | 2.86993447476 | 0.8 | 0.896706981334 | OBSERVATION_CAP | N/A |
| S2 | V2 (saved) | 0.7572346068 | 1.60604109886 | 2.87000215884 | 0.8 | 0.89663881695 | OBSERVATION_CAP | N/A |
| S3 | C3 (saved) | 0.380789059471 | 1.3064115609 | 4.05328189383 | 0.8 | 1.96312087527 | OBSERVATION_CAP | N/A |
| S3 | B Entry (new) | 0.380827155294 | 1.91926127142 | 4.2579622713 | 0.8 | 1.66764804678 | OBSERVATION_CAP | N/A |
| S3 | Hermite (saved) | 0.380828238952 | 2.43562319211 | 4.11720833512 | 0.8 | 1.23664146913 | OBSERVATION_CAP | N/A |
| S3 | V2 (saved) | 0.380829896035 | 2.52632482411 | 4.18440652526 | 0.8 | 1.23645118171 | OBSERVATION_CAP | N/A |
| S4 | C3 (saved) | 0.870803817623 | 1.45352966553 | 4.82596744441 | 0.8 | 2.27134699449 | OBSERVATION_CAP | N/A |
| S4 | B Entry (new) | 0.872818875179 | 1.61801639274 | 4.31763475302 | 0.8 | 1.96306043923 | OBSERVATION_CAP | N/A |
| S4 | Hermite (saved) | 0.872961422232 | 2.22218993294 | 4.1168103898 | 0.8 | 1.62454283014 | OBSERVATION_CAP | N/A |
| S4 | V2 (saved) | 0.872961333495 | 2.21830319863 | 4.11303515772 | 0.8 | 1.62615017842 | OBSERVATION_CAP | N/A |

### Complete reference geometry

| Source | Method | Rows | XY arc [m] | Minimum segment [m] | Maximum segment [m] | Reference clearance [m] | Self-intersection |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | C3 (saved) | 9 | 1.16850278172 | 0.116508939295 | 0.150770002343 | 0.22948689016 | False |
| S1 | B Entry (new) | 10 | 1.27515135119 | 0.106648569469 | 0.150770002343 | 0.22948689016 | False |
| S1 | Hermite (saved) | 11 | 1.27534108334 | 0.0516644082998 | 0.150770002343 | 0.22948689016 | False |
| S1 | V2 (saved) | 11 | 1.27530899885 | 0.050802668327 | 0.150770002343 | 0.22948689016 | False |
| S2 | C3 (saved) | 8 | 1.03018608366 | 0.127427475975 | 0.150569285045 | 0.824529094151 | False |
| S2 | B Entry (new) | 9 | 1.15783040036 | 0.127427475975 | 0.150569285045 | 0.757253286777 | False |
| S2 | Hermite (saved) | 10 | 1.15786428377 | 0.0626736487016 | 0.150569285045 | 0.757253286777 | False |
| S2 | V2 (saved) | 10 | 1.1578955251 | 0.0622304056792 | 0.150569285045 | 0.757253286777 | False |
| S3 | C3 (saved) | 8 | 0.799407099842 | 0.00410605338911 | 0.171384611282 | 0.21148985508 | False |
| S3 | B Entry (new) | 9 | 0.984090289819 | 0.00410605338911 | 0.184683189977 | 0.21148985508 | False |
| S3 | Hermite (saved) | 10 | 0.984182232008 | 0.00410605338911 | 0.171384611282 | 0.21148985508 | False |
| S3 | V2 (saved) | 10 | 0.984301007051 | 0.00410605338911 | 0.171384611282 | 0.21148985508 | False |
| S4 | C3 (saved) | 8 | 0.943994344044 | 0.0879136065011 | 0.158703673876 | 1.02957680557 | False |
| S4 | B Entry (new) | 9 | 1.10487856993 | 0.0879136065011 | 0.16088422589 | 0.878001187341 | False |
| S4 | Hermite (saved) | 10 | 1.10519147494 | 0.0778895530481 | 0.158703673876 | 0.878001187341 | False |
| S4 | V2 (saved) | 10 | 1.1050774293 | 0.0765102169725 | 0.158703673876 | 0.878001187341 | False |

The new reference includes the actual B-to-E* edge. E* and every downstream original pose have frozen provenance; no X1 exists. No shape repair or planning solve was performed.

### Actual first legal submit selector comparison

| Source | Method | Tick | Actual input pose [x,y,yaw] | Nearest | First H5 | Complete H5 |
| --- | --- | --- | --- | --- | --- | --- |
| S1 | C3 (saved) | 96 | [19.204616420161308,24.090013606205353,-1.5356427948050633] | E* | F_2 | ["F_2","F_3","F_4","F_5","F_6"] |
| S1 | B Entry (new) | 96 | [19.204616420161308,24.090013606205353,-1.5356427948050633] | B | E* | ["E*","F_2","F_3","F_4","F_5"] |
| S1 | Hermite (saved) | 96 | [19.204616420161308,24.090013606205353,-1.5356427948050633] | B | bridge_1 | ["bridge_1","E*","F_2","F_3","F_4"] |
| S1 | V2 (saved) | 96 | [19.204616420161308,24.090013606205353,-1.5356427948050633] | B | bridge_1 | ["bridge_1","E*","F_2","F_3","F_4"] |
| S2 | C3 (saved) | 1002 | [18.88091559905879,9.807793015222233,-1.4887640776416609] | F_3 | F_4 | ["F_4","F_5","F_6","F_7","F_8"] |
| S2 | B Entry (new) | 1002 | [18.88091559905879,9.807793015222233,-1.4887640776416609] | B | E* | ["E*","F_3","F_4","F_5","F_6"] |
| S2 | Hermite (saved) | 1002 | [18.88091559905879,9.807793015222233,-1.4887640776416609] | B | bridge_1 | ["bridge_1","E*","F_3","F_4","F_5"] |
| S2 | V2 (saved) | 1002 | [18.88091559905879,9.807793015222233,-1.4887640776416609] | B | bridge_1 | ["bridge_1","E*","F_3","F_4","F_5"] |
| S3 | C3 (saved) | 1806 | [17.84115860483165,31.810155537028734,-0.6838866723247516] | E* | F_3 | ["F_3","F_4","F_5","F_6","F_7"] |
| S3 | B Entry (new) | 1806 | [17.84115860483165,31.810155537028734,-0.6838866723247516] | B | E* | ["E*","F_3","F_4","F_5","F_6"] |
| S3 | Hermite (saved) | 1806 | [17.84115860483165,31.810155537028734,-0.6838866723247516] | B | bridge_1 | ["bridge_1","E*","F_3","F_4","F_5"] |
| S3 | V2 (saved) | 1806 | [17.84115860483165,31.810155537028734,-0.6838866723247516] | B | bridge_1 | ["bridge_1","E*","F_3","F_4","F_5"] |
| S4 | C3 (saved) | 1524 | [19.668567955248744,8.99227202834442,-1.3595769416513406] | E* | F_3 | ["F_3","F_4","F_5","F_6","F_7"] |
| S4 | B Entry (new) | 1524 | [19.668567955248744,8.99227202834442,-1.3595769416513406] | B | E* | ["E*","F_3","F_4","F_5","F_6"] |
| S4 | Hermite (saved) | 1524 | [19.668567955248744,8.99227202834442,-1.3595769416513406] | B | bridge_1 | ["bridge_1","E*","F_3","F_4","F_5"] |
| S4 | V2 (saved) | 1524 | [19.668567955248744,8.99227202834442,-1.3595769416513406] | B | bridge_1 | ["bridge_1","E*","F_3","F_4","F_5"] |

These are actual official selections at identical first legal submit states, not a counterfactual selector query or an inference from row order. Physical H5 poses and original metadata for every successful submit are retained in selector_exposure.csv.

### Sampled selector exposure

| Source | Method | Nearest B count | First H5 E* count | Any H5 E* count | bridge_1 count | Last B/E* tick | Last B/E* elapsed [s] | E* sampled span [s] | First original H5-start tick |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | C3 (saved) | 0 | 0 | 0 | 0 | N/A | N/A | N/A | 96 |
| S1 | B Entry (new) | 3 | 3 | 3 | 0 | 108 | 0.266666680574 | 0.200000010431 | 114 |
| S1 | Hermite (saved) | 3 | 0 | 3 | 3 | 108 | 0.266666680574 | 0.200000010431 | 114 |
| S1 | V2 (saved) | 3 | 0 | 3 | 3 | 108 | 0.266666680574 | 0.200000010431 | 114 |
| S2 | C3 (saved) | 0 | 0 | 0 | 0 | N/A | N/A | N/A | 1002 |
| S2 | B Entry (new) | 2 | 2 | 2 | 0 | 1008 | 0.183333342895 | 0.100000005215 | 1014 |
| S2 | Hermite (saved) | 2 | 0 | 2 | 2 | 1008 | 0.183333342895 | 0.100000005215 | 1014 |
| S2 | V2 (saved) | 2 | 0 | 2 | 2 | 1008 | 0.183333342895 | 0.100000005215 | 1014 |
| S3 | C3 (saved) | 0 | 0 | 0 | 0 | N/A | N/A | N/A | 1806 |
| S3 | B Entry (new) | 5 | 5 | 5 | 0 | 1830 | 0.433333355933 | 0.400000020862 | 1836 |
| S3 | Hermite (saved) | 5 | 3 | 8 | 5 | 1848 | 0.73333337158 | 0.700000036508 | 1854 |
| S3 | V2 (saved) | 4 | 4 | 8 | 4 | 1848 | 0.73333337158 | 0.700000036508 | 1854 |
| S4 | C3 (saved) | 0 | 0 | 0 | 0 | N/A | N/A | N/A | 1524 |
| S4 | B Entry (new) | 4 | 4 | 4 | 0 | 1542 | 0.300000015646 | 0.300000015646 | 1548 |
| S4 | Hermite (saved) | 3 | 2 | 5 | 3 | 1548 | 0.400000020862 | 0.400000020862 | 1554 |
| S4 | V2 (saved) | 3 | 2 | 5 | 3 | 1548 | 0.400000020862 | 0.400000020862 | 1554 |

Exposure spans are first-to-last observed submit samples, not continuous exposure. Successful submissions withheld beyond the cap remain recorded. B and bridge_1 never acquire fabricated original-FRESH progress IDs; E* retains its frozen fractional identity.

### First matched command/selection differences

| Source | Pair (right minus left) | First H5 tick | First command submit | First different applied tick | Left [v,omega] | Right [v,omega] | Delta [v,omega] | Max actual XY separation [m] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | C3_ENTRY_SUFFIX__B_ENTRY_STAGE | 96 | 96 | 99 | [0.7999999919821373,0.9999984535065339] | [0.5999999936181277,0.9999984509086809] | [-0.1999999983640096,-2.5978530437953395e-09] | 0.170911766791 |
| S1 | B_ENTRY_STAGE__HERMITE_BRIDGE | 96 | 96 | 99 | [0.5999999936181277,0.9999984509086809] | [0.5999999913675107,0.9999984407546877] | [-2.2506170260783165e-09,-1.0153993201811318e-08] | 0.0131464052838 |
| S1 | B_ENTRY_STAGE__V2_GRAPH_BRIDGE | 96 | 96 | 99 | [0.5999999936181277,0.9999984509086809] | [0.5999999913761624,0.9999984407508268] | [-2.2419652800920176e-09,-1.0157854113401754e-08] | 0.0131512027707 |
| S2 | C3_ENTRY_SUFFIX__B_ENTRY_STAGE | 1002 | 1002 | 1003 | [0.8,-1.0028287900554527] | [0.5999999921535283,-1.0028287994077167] | [-0.20000000784647176,-9.35226407428047e-09] | 0.0797543455327 |
| S2 | B_ENTRY_STAGE__HERMITE_BRIDGE | 1002 | 1002 | 1003 | [0.5999999921535283,-1.0028287994077167] | [0.5999999917084545,-0.8967069813342794] | [-4.450737556283002e-10,0.10612181807343735] | 0.00767930101909 |
| S2 | B_ENTRY_STAGE__V2_GRAPH_BRIDGE | 1002 | 1002 | 1003 | [0.5999999921535283,-1.0028287994077167] | [0.599999991716704,-0.8966388169504756] | [-4.36824243443823e-10,0.1061899824572411] | 0.00768122657392 |
| S3 | C3_ENTRY_SUFFIX__B_ENTRY_STAGE | 1806 | 1806 | 1807 | [0.6000000042544564,1.2828150778304457] | [0.2935884441167989,1.2828150714944249] | [-0.3064115601376575,-6.336020819119881e-09] | 0.303126686756 |
| S3 | B_ENTRY_STAGE__HERMITE_BRIDGE | 1806 | 1806 | 1807 | [0.2935884441167989,1.2828150714944249] | [0.29358844212237917,1.2366414691290075] | [-1.994419740469766e-09,-0.04617360236541734] | 0.228702100908 |
| S3 | B_ENTRY_STAGE__V2_GRAPH_BRIDGE | 1806 | 1806 | 1807 | [0.2935884441167989,1.2828150714944249] | [0.2935884421909909,1.2364511817103945] | [-1.9258080130590827e-09,-0.04636388978403039] | 0.224184262348 |
| S4 | C3_ENTRY_SUFFIX__B_ENTRY_STAGE | 1524 | 1524 | 1525 | [0.7055854373652094,-0.9630604604328946] | [0.3698124144424887,-0.9630604595453922] | [-0.3357730229227207,8.87502404900431e-10] | 0.149217200668 |
| S4 | B_ENTRY_STAGE__HERMITE_BRIDGE | 1524 | 1524 | 1525 | [0.3698124144424887,-0.9630604595453922] | [0.35764121618210637,-0.963060456227792] | [-0.012171198260382321,3.317600194563397e-09] | 0.0911376809436 |
| S4 | B_ENTRY_STAGE__V2_GRAPH_BRIDGE | 1524 | 1524 | 1525 | [0.3698124144424887,-0.9630604595453922] | [0.35764121626468254,-0.963060456226863] | [-0.012171198177806153,3.3185292291904034e-09] | 0.0903759342771 |

Every common successful submit/result and applied interval, plus all-state pose-separation traces including the terminal sample, is retained in the machine outputs. Shared pre-existing command intervals are labeled COMMON_B_COMMAND and excluded from first-difference interpretation. Threshold is >1e-9 per command component.

### Secondary own-reference metrics

| Source | Own XY AUC .3 | Own XY AUC .9 | Own yaw AUC .3 | Own yaw AUC .9 | Own attachment [s] |
| --- | --- | --- | --- | --- | --- |
| S1 | 0.0312166874019 | 0.137516601064 | 0.0342807597733 | 0.0998194407401 | 1.36666673794 |
| S2 | 0.0303791318561 | 0.114838066596 | 0.0220734421389 | 0.0796930134289 | 1.03333338723 |
| S3 | 0.0152225198568 | 0.105649251447 | 0.0546248411037 | 0.316164701553 | 1.23333339766 |
| S4 | 0.0177467737189 | 0.118903639553 | 0.0510460592508 | 0.249293813697 | 1.08333338983 |

These secondary metrics do not enter the primary original-FRESH classification.

### Validation and artifacts

Saved-only validation and the complete historical validator chain pass. All
87 frozen code/config files and 84 frozen input files are unchanged. Exactly
three final PNGs were visually inspected; independent checks matched every
plotted execution/attachment/endpoint marker, metric panel and selector/timing
sidecar to saved records. CSV row counts: primary20, structural16, paired4,
selector20, command2520 and geometry20. PNG hashes and LF CSV serialization pass.
No frozen code/configuration or report renderer changed after the pushed freeze.
Scientific execution protocol deviations: **none**. No optimizer call, retry,
new historical rollout, source acquisition or result-driven method change.

- [World execution overview](../results/b_to_entry_boundary_row_ablation_04/figures/world_execution_overview.png)
- [Transition and completion metrics](../results/b_to_entry_boundary_row_ablation_04/figures/staging_transition_completion.png)
- [Selector staging diagnostic](../results/b_to_entry_boundary_row_ablation_04/figures/selector_staging_diagnostic.png)
- [Result summary](../results/b_to_entry_boundary_row_ablation_04/result_summary.json)
- [Primary metrics](../results/b_to_entry_boundary_row_ablation_04/primary.csv)
- [Structural comparison](../results/b_to_entry_boundary_row_ablation_04/structural_comparison.csv)
- [Paired timing](../results/b_to_entry_boundary_row_ablation_04/paired_timing.csv)
- [Selector exposure](../results/b_to_entry_boundary_row_ablation_04/selector_exposure.csv)
- [Command comparison](../results/b_to_entry_boundary_row_ablation_04/command_comparison.csv)
- [Reference geometry](../results/b_to_entry_boundary_row_ablation_04/reference_geometry.csv)
- [Figure manifest](../results/b_to_entry_boundary_row_ablation_04/figure_manifest.json)

## Research interpretation

**STAGING_SUFFICIENT_AND_PENALTY_REDUCED; Direction A.** B_ENTRY_STAGE recovers
both S3 dwells without X1 and advances endpoint dwell relative to Hermite/V2 in
all easy sources: S1/S2 by one integration tick and S4 by five. This meets the
predeclared reduction criterion in 3/3 easy sources. No easy dwell is lost and
all reference/execution safety checks pass. These four sources do not establish
a need for intermediate X1 geometry or graph optimization for S3 recovery.
Stop trying to prove graph necessity on these sources; investigate a simpler
progress/staging reconciliation method before any graph claim. No next study
or held-out collection is implemented here.

The critical selector contrast is observed at the same actual first legal
submit state in each source. C3 starts H5 at F_2/F_4/F_3/F_3; B_ENTRY starts at
E* in all four, with B nearest. Hermite/V2 start at bridge_1, followed by E*.
Only S4's first legal submit is at B; S1-S3 have already moved under the shared
pre-existing command. Thus the staging effect on that first target is observed,
not assumed from row order. B_ENTRY's E* exposure spans .2/.1/.4/.3 s in saved
submit samples, compared with .2/.1/.7/.4 s for Hermite/V2. This descriptive
pattern does not prove that exposure duration alone causes endpoint timing.

S3 attachment occurs at 1.2333333976566792 s and endpoint dwell at
1.4833334106951952 s, respectively 14/17 ticks earlier than Hermite and
15/17 ticks earlier than V2. Attachment occurs at original arc fraction
.8011861449541925, with .25077156552514435 m remaining; both bridges attach at
earlier original progress with more arc remaining. Position AUC .9 is lower
than C3, Hermite and V2. S3 endpoint error at the cap is .06521469445869597 m:
lower than C3's .11235439505980861 m, but above Hermite's .0604422053494252 m
and V2's .061520244210175545 m. Angular TV is higher than both bridges while
linear TV is lower. This is not uniform improvement in every metric.

The easy-source completion penalty is reduced, not eliminated. B_ENTRY remains
8/5/4 ticks later than C3 in S1/S2/S4 (.13333334028720856/.08333333767950535/
.06666667014360428 s). In all three it attaches earlier than C3 but reaches
endpoint dwell later, with more original arc left at attachment. Faster
attachment therefore remains a transition/completion trade-off versus C3.
S4 also has higher early XY/yaw AUC and two-tick later attachment than
Hermite/V2, despite earlier endpoint dwell and lower linear TV. No scalar score
combines these distinct outcomes.

Commands first differ at the first new submit for each source under the frozen
>1e-9 component threshold. S1 B_ENTRY versus Hermite/V2 begins with only about
1e-8 rad/s angular difference; that threshold crossing is not itself evidence
of a materially large command change. Full matched command and actual-state
traces are retained. Hermite/V2 still overlap strongly in S1/S2/S4 as established
in V2's authenticated study; different line styles/staggered markers distinguish
them. B_ENTRY-versus-Hermite maximum matched XY gaps are .013146405283770125,
.007679301019094081, .22870210090841908 and .09113768094356049 m in S1-S4.

This is four-source development evidence under a fixed controller and offline
logical schedule. It does not establish a final method, graph/Hermite superiority,
obstacle or population generalization, real asynchronous or real-world benefit,
navigation-task completion, or optimality of C3/B_ENTRY. The reference remains
untimed; official MPC determines the actual v/omega commands.

## Explicit answers to the twenty-two questions

1. **Exactly B, E*, suffix with no X1?** Yes. Exact planning B/E* and Native-installed
   downstream bits pass; rows are 10/9/9/9. No interpolation or resampling was added.
2. **Optimizer calls exactly zero?** Yes; no new planning solve or Hermite construction
   supplied the new reference. Historical algebra checks only authenticate saved records.
3. **Safety preserved?** Yes. Minimum new swept execution clearance is
   .15918701209977898 m, above .05 m; every full reference is safe, no safety abort.
4. **First legal nearest for C3/B_ENTRY/Hermite/V2?** S1 E*/B/B/B;
   S2 F_3/B/B/B; S3 E*/B/B/B; S4 E*/B/B/B.
5. **First H5 at the same state?** S1 F_2/E*/bridge_1/bridge_1;
   S2 F_4/E*/bridge_1/bridge_1; S3 and S4 F_3/E*/bridge_1/bridge_1.
   Exact first-state equality is checked; later states are not assumed equal.
6. **Does inserting B actually bring E* into H5?** Yes at the first legal submit
   in 4/4. C3 has no E* inclusion at any successful submit in these saved rollouts.
7. **How long is E* exposed?** B_ENTRY inclusion counts 3/2/5/4; first-to-last
   sample spans .20000001043081284/.10000000521540642/.40000002086162567/
   .30000001564621925 s. Last ticks 108/1008/1830/1542. Nearest=B and first-H5=E*
   counts equal these inclusion counts. No continuous exposure claim.
8. **S3 sustained attachment recovered?** Yes: 1.2333333976566792 s.
9. **S3 endpoint dwell recovered?** Yes: 1.4833334106951952 s. C3's two dwells
   remain null, not converted to cap values.
10. **S3 AUC .9?** B_ENTRY .17774746247128803; C3 .18655395332828628;
    Hermite .18225344050596992; V2 .18190299536729632 m s.
11. **S3 cap endpoint error?** B_ENTRY .06521469445869597; C3 .11235439505980861;
    Hermite .0604422053494252; V2 .061520244210175545 m. Lower than C3, higher
    than both bridge baselines.
12. **S1/S2/S4 endpoint dwells preserved?** Yes, at 1.650000086054206 /
    1.3333334028720856 / 1.4666667431592941 s.
13. **B_ENTRY delay versus C3?** .13333334028720856 / .08333333767950535 /
    .06666667014360428 s for S1/S2/S4.
14. **Lower than Hermite/V2?** Yes in all three, by 1/1/5 ticks. Their delays
    were .15000000782310963/.10000000521540642/.15000000782310963 s.
15. **Faster attachment but slower endpoint dwell?** Yes, B_ENTRY versus C3 in
    S1/S2/S4. Attachment gains are .10000000521540642/.01666666753590107/
    .20000001043081284 s, while the positive endpoint delays above remain.
    No B_ENTRY-versus-Hermite/V2 pair has faster attachment with slower endpoint.
16. **First command difference from C3?** Submit ticks 96/1002/1806/1524;
    applied ticks 99/1003/1807/1525. Full commands/deltas are in the comparison table.
17. **First command difference from Hermite/V2?** The same submit/applied ticks
    under >1e-9 component tolerance. S1 starts with only numerical-scale differences.
18. **How far do executions separate?** Maximum matched XY separation from C3:
    .17091176679056982/.07975434553265141/.3031266867563835/.14921720066801528 m.
    From Hermite: .013146405283770125/.007679301019094081/.22870210090841908/
    .09113768094356049 m. From V2: .013151202770698561/.007681226573922689/
    .22418426234834418/.0903759342770648 m. These compare actual states at common
    elapsed times; they are not geometric distances between whole path sets.
19. **Is intermediate X1 supported as necessary for S3?** No. B_ENTRY recovers
    both dwells without X1 under the same authenticated controller and schedule.
20. **Is graph optimization necessary or superior?** **NO.** These results do
    not distinguish a necessary graph benefit from simpler staging or Hermite.
21. **Frozen classification?** STAGING_SUFFICIENT_AND_PENALTY_REDUCED.
22. **Scientific direction?** **A**: investigate simpler progress/staging
    reconciliation before any graph-necessity claim. Do not implement the next
    direction or collect held-out data in this experiment.
