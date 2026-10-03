# B_TO_ENTRY_VECTOR_BRIDGE_EXECUTION_03

## Frozen protocol

Starting HEAD and fetched origin/main: `54eadfc9881d12dad3343fb06932236075f3f256`.
**Timing-controlled offline causal reference comparison.** Four development sources;
first execution study of the saved stable vector-boundary V2 graph. Numerical
stability in DIAG_02 does not establish execution performance.
Run identifier: `data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z`.
The identifier is not the actual start time; host UTC is recorded separately.

Question: does V2 preserve Hermite's S3 attachment and original-FRESH endpoint
recovery while reducing Hermite's easy-source completion penalty?

### Sources and authenticated reuse

| Source | Exact saved source | Role |
|---|---|---|
| S1 | OSA03_R00 | Obstacle / straight-to-detour |
| S2 | episode_001_repeat_01/handoff_013 | Mild nearly-straight control |
| S3 | episode_008_repeat_01/handoff_023 | Progressive-left-turn hard case |
| S4 | episode_013_repeat_00/handoff_020 | Progressive-right-turn control |

Reuse Native, C3 Entry-Suffix and Hermite execution records byte-for-byte.
Authenticate their full-precision metrics via the prior bridge result ledger,
including S3 missing Native/C3 dwells and observed Hermite dwells, and easy-source
Hermite endpoint delays. No rounded metric is used as input.

Reuse DIAG_02 result commit `65efd5be9b64046eff0c30018696ae33966a20ba` and
scientific freeze `e228237fdf8e68df3db6ab27a5c83eb7da6df76c`.
Tracked result SHA256 `9f252ea1cdd243353cb1f5ca009ab1bed1604fae7aba90ff76a430e76c88f41f`.
Bridge result SHA256 `18e79982aca83fae2035d7496883fe40508dfe346d2830285697baf801efdd6e`.
Entry-suffix result SHA256 `e86083eb2f812e84c898885f2039b6520d336dd802c391dcaf29342cf17aa948`.
All source, controller, input and run ledgers are authenticated. Saved-only
DIAG_02 validation includes historical bridge, entry, correspondence, transport,
multisource, R00 and R01 validators. Reconstructing saved integration records for
validation is not a new scientific rollout or controller call.

### Fixed planning reference and installation

V2 is selected before execution for minimality: M=2, `[B, X1, E*]`, one interior
pose. Both V2 and V3 were stable in 4/4; V3 had no required stability advantage
and adds a row that can change H5 selection. No V2 superiority or V3 inferiority
is asserted. V3 is neither executed nor used as fallback.

Copy the exact saved V2 `last_accepted_bridge.npy` and `converged_reference.npy`.
No LM call, numerical reconstruction, polish, resampling, yaw adjustment or
changed edge length. The existing solution used spatial vector boundary targets
from its Hermite initial edge lengths and unchanged smoothness/spacing. There
is no new planning objective or factor in this experiment. New optimizer calls=0.
Missing or unauthenticated V2 artifacts mean TECHNICAL_BLOCKED; do not regenerate.

World XY metres, yaw radians counterclockwise, +Z up. Observation-local x forward,
y left. Original FRESH stays `F=A F_local`, never re-anchored at B. B, X1 and E*
world bits come directly from the saved solution. Downstream world poses are the
exact authenticated Native installed rows; raw original local rows are unchanged.
For the existing official adapter, only the three derived bridge rows use `A^-1 X`.
Their local/world installation roundoff must be <=1e-12; planning boundary bits
stay exact. All downstream installed world rows must match Native bit-for-bit.
Save planning world/local hashes, installed-world array/hash, labels and roundoff.

Labels: `B`, `bridge_1`, `E*`, then `F_k...`. Derived bridge rows have no fabricated
original-FRESH progress ID. A zero-numerical-solve official installation preflight
checks arbitrary N support, saved state and downstream bits. Official MPC solve
is monkeypatched to raise if called in that existing preflight. A failed technical
preflight blocks execution. Unsafe reference is recorded without repair or rollout.

### Controller, timing and safety

Reuse the unchanged official worker, adapter and `run_relative_factor_multisource01.run_method`.
MPC SHA256 `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
H=5, nearest+1 weighted XY/wrapped-yaw selection, Q/R, command/acceleration limits,
previous_control, generation/stale handling, solver and memory order are unchanged.

Copy source-specific common-state and schedule files directly from the historical
bridge run: exact B/tick/time, physical u_minus, applied u_B_plus, controller memory,
generation/version, integration dt and geometry. Use the same deterministic release
wrapper. A slow wall-clock solve pauses the logical simulation; it cannot choose
the application tick. No future state enters a solve.

Compare attempted and accepted submit sequences, result-application sequences,
integration counts and initial provenance through 54 and 180 intervals. Successful
rollouts must match both historical C3/Hermite and the frozen schedule exactly.
Known guard/numerical-controller failures retain their valid prefix and are reported
as censored; missing full parity is never fabricated. Unexplained timing/restoration
mismatch is TECHNICAL_BLOCKED.

Safety includes the complete actual polyline `B -> X1 -> E* -> suffix`, using the
unchanged Hospital/cart/workspace conventions, radius .20 m, required edge clearance
.05 m and numerical reserve 1e-7 m. Check both saved planning and actually installed
references. Keep the existing abort-only guard; an unsafe proposed command is never
applied. No repair or retry. No changes to external LightNav/MPC or environments.

### Evaluation and diagnostics

Primary target: full ORIGINAL FRESH. Reuse exact historical forward-only continuous
projection, shortest yaw, position/yaw AUC .3/.9, initial error, max XY error .5,
separation growth and sustained attachment. Attachment uses <=.10 m and <=15 deg
through the complete following .30 s sampled interval. Report fractional original
row, arc fraction and remaining original arc at attachment.

Reuse endpoint evaluator: first actual execution time satisfying the same pose tube
around the ORIGINAL FRESH final pose for complete .30 s dwell. Call it **original-FRESH
endpoint dwell time**, never navigation-goal completion. Null remains null. Report
T_endpoint-T_attach only if both observed, first tube entry, endpoint error and
original progress/remaining arc at the fixed 180-interval nominal 3 s cap, path length
to dwell and mean |v| before dwell. No waypoint time/dt is introduced.

Report swept clearance lower bound, termination/abort tick, linear/angular command
TV, max |v|/|omega|. Own-reference V2 position/yaw AUC .3/.9 and attachment are
secondary and cannot replace primary metrics.

Hermite/V2 geometry: exact nodes, chord, XY length/ratio, edge lengths, turning
max/RMS, full-reference clearance, self-intersection and maximum bridge/full-reference
XY/yaw differences. No A/V objective total comparison or scalar winner score.

Use natural successful solve-result selection records only. For C3/Hermite/V2,
report first legal H5/nearest/first identities, submit counts containing bridge_1
and E*, first H5 start at an original F row, last H5 submit containing any derived
row (B/bridge_1/E*), elapsed time of that submit and first-to-last exposure span.
Counts include a successful final submitted result withheld beyond the cap; its
application is explicitly null. Exposure is sampled and descriptive, not continuous
controller occupancy or evidence of causation.

At common logical submit ticks compare solve-result commands and H5 identities;
at every common integration/application interval compare actual held commands,
active H5 identities and actual pose deltas (wrapped yaw). The shared initial
historical command is labeled COMMON_B_COMMAND. Record first differing submit
result and first differing applied command using absolute component difference
>1e-9, and maximum matched XY separation. No separate counterfactual selector run.

### Fixed comparisons, classification and next decision

For attachment/endpoint comparisons, an observed discrete change requires at least
`integration_dt_s - 1e-9` seconds. Other scalar comparisons use 1e-9 native units;
stricter historical parity rules remain intact. No significance claim.
Primary mechanism comparison is V2 versus Hermite; easy-source non-regression is
V2 versus C3. Native is saved context. Preserve the historical fact that Hermite
recovered S3 but delayed easy-source endpoint dwell.

Fixed precedence:

1. TECHNICAL_BLOCKED: authentication, exact-artifact, restoration, interface,
   unexplained schedule or validator failure.
2. VECTOR_REFERENCE_OR_EXECUTION_FAILURE: unsafe V2 reference, official numerical
   controller failure, or safety abort attributable to V2. Keep valid prefixes.
3. HARD_CASE_RECOVERY_LOST: valid S3 V2 lacks either attachment or endpoint dwell.
4. VECTOR_GRAPH_TRADEOFF_WORSE: S3 retains both, but any easy source loses an observed
   endpoint dwell or V2 endpoint is >=one tick later than Hermite.
5. VECTOR_GRAPH_TRADEOFF_REDUCED: S3 retains both; all easy endpoint dwells remain;
   V2 minus Hermite endpoint <=1e-9 s in all three easy sources and <=negative one
   tick in at least two; safety passes.
6. VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN: S3 retains both and easy endpoint
   dwells remain; neither prior worse nor reduced condition holds.
7. MIXED_VECTOR_EXECUTION_EFFECT: remaining scientifically valid cases.

Next-step A corresponds to TRADEOFF_REDUCED: viable candidate worth freezing for
later held-out obstacle evaluation. B corresponds to RECOVERY_NO_CLEAR_TRADEOFF_GAIN:
works but further formulation work before held-out evaluation. C covers lost
recovery/regression/failure: do not freeze V2 as the candidate. Technical blocking
also means no candidate can be justified yet. Do not collect held-out data here.

### Freeze, budgets and outputs

Authenticate -> freeze exact references/schedules/metrics/classification -> focused
and relevant regression tests -> diff review -> implementation/config/tests/protocol
commit -> normal push -> record freeze SHA. Only then execute exactly one V2 rollout
for S1, S2, S3, S4 in that order. Expected 4 V2 rollouts; MPC solves only from those
schedules. Optimizer, Native/C3/Hermite/V3 reruns, LightNav/RGB/Isaac, source search,
instruction changes and retries all zero. Record failures without substitution.

Saved-only validation precedes reporting. Exactly three final PNGs, no HTML:
`world_execution_overview.png`, `transition_completion_metrics.png`,
`bridge_tradeoff_diagnostics.png`. Machine-readable result_summary, primary,
paired_comparison, bridge_geometry, selector_exposure, command_comparison and
figure_manifest live under `results/b_to_entry_vector_bridge_execution_03/`.
Large arrays stay in ignored data. Numeric sidecars and PNG hashes are validated.

Four-source development evidence only: no obstacle/population/real-world claim,
real asynchronous online benefit, task completion, universal graph superiority,
V2 optimality, V3 inferiority or final-method claim.

## Reproduction commands

Use the existing repository and official-controller environments unchanged.

```bash
git fetch origin main
git rev-parse HEAD origin/main
git status --short
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/vector03-mpl .venv/bin/python scripts/run_b_to_entry_vector_bridge_execution03.py --run data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/vector03-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_vector_bridge_execution03.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/vector03-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_vector_bridge_execution03.py tests/test_b_to_entry_graph_formulation_diag02.py tests/test_b_to_entry_bridge01.py tests/test_spatial_entry_suffix_execution01.py tests/test_spatial_correspondence_selector_diag01.py tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_spatial_entry.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_vector_bridge_execution03.py --run data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z --mode freeze
# Review and stage this namespace and WORK_LOG only; commit and normal push.
git diff --cached --check
git commit -m "Freeze saved V2 bridge execution comparison"
git push origin main
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_vector_bridge_execution03.py --run data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_vector_bridge_execution03.py --run data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/vector03-mpl .venv/bin/python scripts/report_b_to_entry_vector_bridge_execution03.py --run data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_vector_bridge_execution03.py --run data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z --check-only
```

### Prepared source authentication and installation

All four installation preflights passed with zero numerical MPC solves.

| Source | Raw FRESH SHA256 | Exact V2 bridge SHA256 | Exact V2 full reference SHA256 |
|---|---|---|---|
| S1 | `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521` | `4c05419857a15016b5f22d87b7ca1d830607d05e5d7d381fe765387ad6f05963` | `0504c0023fe091c14eb6534f8b45ea6f1a6e5ae356b6f342ba1830729b362cd0` |
| S2 | `82bb298736fd7a1e6289a529e0a973d4def6b3ed6b00b03506ac3694833f3503` | `fe85dbf82ef11e0a8e3cab9a7ab57a7ee67eeb335d3be2a3050deeff57fbb686` | `dfa42d0afbc904f14b241457ac5020f82434732ba63fbc643ee48caa9b729c27` |
| S3 | `ba52161d8e7ba1f0619e11cd0b93da29d201e70074d76160d374fe1f6c87517f` | `3d5cbb8e3e312b0f5906f73377f23c57da3e0eded826c5e1d5292cc93cefe148` | `9220c60eb9091fe053569e1f0d85b54b34f7d093392bbe63441862049366dee8` |
| S4 | `f5585ef9f7ff2eeeae9992640a68cf65b6b8510baacf8ff9fd2886aa40df7552` | `9dafaa566467cbf51a899928eecc1acaa20eedac7c57bff706460c03a4bdbece` | `a226c8a0d8237fd645ce5e2cbf5c267ab78d683d03a0f11004e67d0582f0d818` |

| Source | Rows | B tick | Derived installation roundoff max | Installed reference clearance m |
|---|---:|---:|---:|---:|
| S1 | 11 | 92 | 7.1054273576010019e-15 | 0.22948689016047874 |
| S2 | 10 | 997 | 3.5527136788005009e-15 | 0.75725328677738357 |
| S3 | 10 | 1804 | 1.7763568394002505e-14 | 0.21148985508008777 |
| S4 | 10 | 1524 | 5.3290705182007514e-15 | 0.87800118734143218 |

### Pre-freeze validation

Focused tests: **25 passed in 63.94 s**. Relevant regression: **661 passed,
1 skipped in 320.02 s**. The known skip needs the absent ignored EXP-01B/EXP-02B
corpus. Rendering-only square-axis adjustment was made before freeze and its test
passed again (**1 passed in 8.25 s**). Scientific code did not change after the
full regression. Compileall and working/staged diff checks pass.

A preparation metadata-copy error was fixed before freeze; its partial derived
folder is preserved as `prepare_debug_01`. It performed one successful zero-solve
installation preflight and no scientific calls. The completed preparation passed
all four preflights. There has been no V2 rollout or optimization before freeze.

## Repository-confirmed facts

Scientific freeze **`7bf24d60b87774859ffef3f6d56f524a4fb7ad15`** was normally pushed before execution. Actual execution start UTC: `2026-10-03T07:40:27.891713+00:00`. Classification: **VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN**. Next-step decision: **B**.

Exactly **4 V2 rollouts**, **120 new official MPC solves**, **119 logical applications** and **0 reconciliation optimizer calls**. S2's final solve result is withheld beyond the fixed cap, as in its historical schedule. Twelve historical Native/C3/Hermite rollouts were reused; their reruns, V3 rollouts, LightNav/RGB/Isaac and retries are all zero. No source, method, schedule or controller change after freeze.

All four V2 rollouts terminate at OBSERVATION_CAP with 180 intervals, no controller error and no safety abort. All sixteen compared rollouts match the source-specific common B state/command/memory provenance and frozen submit/application schedule through both .9 s and the full 180-interval cap. The cap is 3.0000001564621925 s on the saved integration grid; the nominal label is 3 s. Derived-row installation roundoff is reported above; downstream Native bits and all saved planning B/X1/E* bits are preserved.

Tables round displayed numbers to 12 significant digits. CSV/JSON preserve full saved precision. N/A represents an unobserved metric, never the cap. Attachment row is the continuous original row index; arc fraction is a separate normalized spatial quantity. All time measurements below are actual execution time after B.

### Transition, attachment location and endpoint dwell

| Source | Method | XY AUC .9 [m s] | T_attach [s] | Original row | Original arc fraction | Arc left at attach [m] | T_endpoint [s] | T_post_attach [s] | Endpoint error at cap [m] |
|---|---|---|---|---|---|---|---|---|---|
| S1 | Native (saved) | 0.169065632516 | 1.46666674316 | 8.71784637003 | 0.968605161683 | 0.0424849243072 | 1.51666674577 | 0.0500000026077 | 0.0964390213698 |
| S1 | C3 (saved) | 0.169065632382 | 1.46666674316 | 8.71784636692 | 0.968605161337 | 0.042484924776 | 1.51666674577 | 0.0500000026077 | 0.096439021209 |
| S1 | Hermite (saved) | 0.157503974584 | 1.46666674316 | 7.71413414341 | 0.857096752655 | 0.19338317928 | 1.66666675359 | 0.200000010431 | 0.077380066851 |
| S1 | V2 (new) | 0.157505673387 | 1.46666674316 | 7.7141316191 | 0.857096473308 | 0.193383557305 | 1.66666675359 | 0.200000010431 | 0.0773826817703 |
| S2 | Native (saved) | 0.13183311076 | 1.05000005476 | 7.69045316001 | 0.854497877738 | 0.197121472188 | 1.25000006519 | 0.200000010431 | 0.0793823670714 |
| S2 | C3 (saved) | 0.13183311076 | 1.05000005476 | 7.69045316001 | 0.854497877738 | 0.197121472188 | 1.25000006519 | 0.200000010431 | 0.0793823670714 |
| S2 | Hermite (saved) | 0.132991596869 | 1.10000005737 | 7.42772733102 | 0.825298435669 | 0.236679912424 | 1.35000007041 | 0.250000013039 | 0.0760370052558 |
| S2 | V2 (new) | 0.132992620512 | 1.10000005737 | 7.42772686348 | 0.825298383707 | 0.23667998282 | 1.35000007041 | 0.250000013039 | 0.0760381621995 |
| S3 | Native (saved) | 0.189624121947 | N/A | N/A | N/A | N/A | N/A | N/A | 0.11523801852 |
| S3 | C3 (saved) | 0.186553953328 | N/A | N/A | N/A | N/A | N/A | N/A | 0.11235439506 |
| S3 | Hermite (saved) | 0.182253440506 | 1.46666674316 | 6.09489906865 | 0.767945556311 | 0.292699199044 | 1.76666675881 | 0.300000015646 | 0.0604422053494 |
| S3 | V2 (new) | 0.181902995367 | 1.4833334107 | 6.20433021787 | 0.782015715247 | 0.274951966173 | 1.76666675881 | 0.28333334811 | 0.0615202442102 |
| S4 | Native (saved) | 0.198189215821 | 1.28333340026 | 7.78827222868 | 0.912753530112 | 0.11196220915 | 1.40000007302 | 0.116666672751 | 0.08453931375 |
| S4 | C3 (saved) | 0.198189215821 | 1.28333340026 | 7.78827222868 | 0.912753530112 | 0.11196220915 | 1.40000007302 | 0.116666672751 | 0.08453931375 |
| S4 | Hermite (saved) | 0.158973356936 | 1.05000005476 | 5.2885398519 | 0.637306486163 | 0.465439657392 | 1.55000008084 | 0.500000026077 | 0.0304487538869 |
| S4 | V2 (new) | 0.159008123879 | 1.05000005476 | 5.29308608443 | 0.637863191849 | 0.46472524455 | 1.55000008084 | 0.500000026077 | 0.0306349818938 |

### Early error and yaw metrics

| Source | Method | Initial XY [m] | Initial yaw [rad] | Max XY .5 [m] | Separation growth [m] | XY AUC .3 [m s] | Yaw AUC .3 [rad s] | Yaw AUC .9 [rad s] |
|---|---|---|---|---|---|---|---|---|
| S1 | Native (saved) | 0.106648569289 | 0.523608487599 | 0.215262376676 | 0.108613807388 | 0.0479203622918 | 0.122763624527 | 0.193368049414 |
| S1 | C3 (saved) | 0.106648569289 | 0.523608487599 | 0.215262376486 | 0.108613807198 | 0.0479203622588 | 0.122763624543 | 0.193368049369 |
| S1 | Hermite (saved) | 0.106648569289 | 0.523608487599 | 0.192250391649 | 0.0856018223601 | 0.0461978105283 | 0.124983475826 | 0.196380784635 |
| S1 | V2 (new) | 0.106648569289 | 0.523608487599 | 0.192252120037 | 0.0856035507481 | 0.0461978215005 | 0.124984232387 | 0.196384208514 |
| S2 | Native (saved) | 0.127644308504 | 0.261793531143 | 0.163894949098 | 0.0362506405941 | 0.0453024911898 | 0.0456241196881 | 0.107103977941 |
| S2 | C3 (saved) | 0.127644308504 | 0.261793531143 | 0.163894949098 | 0.0362506405941 | 0.0453024911898 | 0.0456241196881 | 0.107103977941 |
| S2 | Hermite (saved) | 0.127644308504 | 0.261793531143 | 0.161568396677 | 0.0339240881724 | 0.0447513464311 | 0.0497156647152 | 0.103789438819 |
| S2 | V2 (new) | 0.127644308504 | 0.261793531143 | 0.16156949475 | 0.0339251862459 | 0.0447513906534 | 0.0497169574747 | 0.103790512186 |
| S3 | Native (saved) | 0.184683189977 | 0.385685031403 | 0.239119858172 | 0.054436668195 | 0.0643635365783 | 0.0761905902303 | 0.175616043138 |
| S3 | C3 (saved) | 0.184683189977 | 0.385685031403 | 0.235029318754 | 0.0503461287766 | 0.06369986862 | 0.0749714888083 | 0.173208230511 |
| S3 | Hermite (saved) | 0.184683189977 | 0.385685031403 | 0.205591408233 | 0.020908218256 | 0.0600195353242 | 0.0750877706482 | 0.123986312039 |
| S3 | V2 (new) | 0.184683189977 | 0.385685031403 | 0.206070181364 | 0.0213869913868 | 0.0600196074788 | 0.0750934336363 | 0.130565658586 |
| S4 | Native (saved) | 0.16088422589 | 0.505674822974 | 0.252791514218 | 0.0919072883284 | 0.0611986169982 | 0.120660784385 | 0.230599467879 |
| S4 | C3 (saved) | 0.16088422589 | 0.505674822974 | 0.252791514218 | 0.0919072883284 | 0.0611986169982 | 0.120660784385 | 0.230599467879 |
| S4 | Hermite (saved) | 0.16088422589 | 0.505674822974 | 0.191420918781 | 0.0305366928912 | 0.0543860644842 | 0.112389487606 | 0.205457144905 |
| S4 | V2 (new) | 0.16088422589 | 0.505674822974 | 0.191469987566 | 0.0305857616764 | 0.0543877354846 | 0.112387242799 | 0.205355811925 |

### Easy-source endpoint penalty

| Source | Hermite minus C3 [s] | V2 minus C3 [s] | V2 minus Hermite [s] | V2 improvement >=one tick? |
|---|---|---|---|---|
| S1 | 0.150000007823 | 0.150000007823 | 0 | False |
| S2 | 0.100000005215 | 0.100000005215 | 0 | False |
| S4 | 0.150000007823 | 0.150000007823 | 0 | False |

The delays are exactly equal between V2 and Hermite in all three easy sources: 9/6/9 integration ticks in S1/S2/S4. Zero easy sources improve by a tick. S3 endpoint dwell is also exactly equal to Hermite, while attachment is one tick later. All easy endpoint dwells are retained, but the delay relative to C3 remains.

### Other downstream descriptors

| Source | Method | First endpoint tube entry [s] | Original row at cap | Arc fraction at cap | Arc left at cap [m] | Path to endpoint dwell [m] | Mean abs(v) before dwell [m/s] |
|---|---|---|---|---|---|---|---|
| S1 | Native (saved) | 1.51666674577 | 9 | 1 | 0 | 1.18000006295 | 0.778021978952 |
| S1 | C3 (saved) | 1.51666674577 | 9 | 1 | 0 | 1.18000006242 | 0.778021978599 |
| S1 | Hermite (saved) | 1.66666675359 | 9 | 1 | 0 | 1.1435514989 | 0.686130863553 |
| S1 | V2 (new) | 1.66666675359 | 9 | 1 | 0 | 1.14355172864 | 0.6861310014 |
| S2 | Native (saved) | 1.25000006519 | 9 | 1 | 0 | 0.984486552292 | 0.787589200758 |
| S2 | C3 (saved) | 1.25000006519 | 9 | 1 | 0 | 0.984486552292 | 0.787589200758 |
| S2 | Hermite (saved) | 1.35000007041 | 9 | 1 | 0 | 0.984110691019 | 0.728970844218 |
| S2 | V2 (new) | 1.35000007041 | 9 | 1 | 0 | 0.984110836857 | 0.728970952246 |
| S3 | Native (saved) | N/A | 9 | 1 | 0 | N/A | N/A |
| S3 | C3 (saved) | N/A | 9 | 1 | 0 | N/A | N/A |
| S3 | Hermite (saved) | 1.76666675881 | 9 | 1 | 0 | 0.818252928244 | 0.463162010699 |
| S3 | V2 (new) | 1.76666675881 | 9 | 1 | 0 | 0.822209652303 | 0.46540166571 |
| S4 | Native (saved) | 1.40000007302 | 9 | 1 | 0 | 1.06748481022 | 0.762489110393 |
| S4 | C3 (saved) | 1.40000007302 | 9 | 1 | 0 | 1.06748481022 | 0.762489110393 |
| S4 | Hermite (saved) | 1.55000008084 | 9 | 1 | 0 | 0.948575075935 | 0.611983888041 |
| S4 | V2 (new) | 1.55000008084 | 9 | 1 | 0 | 0.949292925697 | 0.612447016895 |

### Safety and commands

| Source | Method | Swept clearance lower bound [m] | Linear TV [m/s] | Angular TV [rad/s] | Max abs(v) [m/s] | Max abs(omega) [rad/s] |
|---|---|---|---|---|---|---|
| S1 | Native (saved) | 0.133561094445 | 0.799999999766 | 3.62955577057 | 0.8 | 1.548333658 |
| S1 | C3 (saved) | 0.133561094659 | 0.800000015802 | 3.62955576994 | 0.8 | 1.54833365799 |
| S1 | Hermite (saved) | 0.152790686503 | 2.00000006058 | 3.80325774564 | 0.8 | 1.24264920207 |
| S1 | V2 (new) | 0.152788103572 | 2.00000006051 | 3.80304101355 | 0.8 | 1.24256462102 |
| S2 | Native (saved) | 0.757224041072 | 0.80604106892 | 2.546626244 | 0.8 | 1.05284528875 |
| S2 | C3 (saved) | 0.757224041072 | 0.80604106892 | 2.546626244 | 0.8 | 1.05284528875 |
| S2 | Hermite (saved) | 0.75723460538 | 1.60604109889 | 2.86993447476 | 0.8 | 0.896706981334 |
| S2 | V2 (new) | 0.7572346068 | 1.60604109886 | 2.87000215884 | 0.8 | 0.89663881695 |
| S3 | Native (saved) | 0.380788847753 | 1.30641155651 | 4.09143330004 | 0.8 | 1.97074270782 |
| S3 | C3 (saved) | 0.380789059471 | 1.3064115609 | 4.05328189383 | 0.8 | 1.96312087527 |
| S3 | Hermite (saved) | 0.380828238952 | 2.43562319211 | 4.11720833512 | 0.8 | 1.23664146913 |
| S3 | V2 (new) | 0.380829896035 | 2.52632482411 | 4.18440652526 | 0.8 | 1.23645118171 |
| S4 | Native (saved) | 0.870803817623 | 1.45352966553 | 4.82596744441 | 0.8 | 2.27134699449 |
| S4 | C3 (saved) | 0.870803817623 | 1.45352966553 | 4.82596744441 | 0.8 | 2.27134699449 |
| S4 | Hermite (saved) | 0.872961422232 | 2.22218993294 | 4.1168103898 | 0.8 | 1.62454283014 |
| S4 | V2 (new) | 0.872961333495 | 2.21830319863 | 4.11303515772 | 0.8 | 1.62615017842 |

V2 minimum swept clearance across sources is **0.1527881035724341 m**. Every V2 termination reason is OBSERVATION_CAP and every safety_abort_tick is null. Full saved and installed V2 reference checks pass; minimum installed reference clearance is **0.21148985508008777 m**. Bridge and full-reference self-intersection flags are false for Hermite and V2 in all sources. This is safety evidence for these saved geometries and rollouts only.

### Exact saved bridge geometry

Each pose is world [x m, y m, yaw rad]. B and E* are identical in Hermite/V2; only the saved interior pose differs. Full machine-precision coordinates are also in bridge_geometry.csv.

| Source | Fixed B | Fixed E* | Hermite X1 | V2 saved X1 |
|---|---|---|---|---|
| S1 | [19.2036305531, 24.1433353602, -1.56897602647] | [19.2958944922, 24.1968263211, -1.04536753938] | [19.2466465532, 24.17195065, -1.30717178292] | [19.2489625327, 24.1662683654, -1.30731804311] |
| S2 | [18.8740626659, 9.87410162667, -1.44686167546] | [18.7476211397, 9.89158339019, -1.7086551518] | [18.81219806, 9.88413939468, -1.57775841363] | [18.8121725703, 9.88060081593, -1.57766682995] |
| S3 | [17.8285426106, 31.8207158811, -0.709980509436] | [17.8972768577, 31.9921319945, -0.324295478033] | [17.859429919, 31.9055736918, -0.517137993734] | [17.8659057858, 31.9020461242, -0.517395262048] |
| S4 | [19.6685679552, 8.99227202834, -1.35957694165] | [19.5163338119, 9.04431527694, -1.86525176462] | [19.5966420237, 9.02216254265, -1.61241435314] | [19.5949776072, 9.01320702229, -1.61208991552] |

| Source | Method | Chord [m] | Bridge length [m] | Length/chord | Edges [m] | Max turn [rad] | RMS turn [rad] | Reference clearance [m] |
|---|---|---|---|---|---|---|---|---|
| S1 | Hermite (saved) | 0.106648569469 | 0.106838301622 | 1.00177904077 | [0.0516644082998, 0.0551738933223] | 0.119275426526 | 0.119275426526 | 0.22948689016 |
| S1 | V2 (new) | 0.106648569469 | 0.106806217127 | 1.00147819759 | [0.050802668327, 0.0560035487999] | 0.108807775703 | 0.108807775703 | 0.22948689016 |
| S2 | Hermite (saved) | 0.127644316704 | 0.127678200116 | 1.00026545179 | [0.0626736487016, 0.0650045514148] | 0.0460852757382 | 0.0460852757382 | 0.757253286777 |
| S2 | V2 (new) | 0.127644316704 | 0.127709441445 | 1.00051020479 | [0.0622304056792, 0.0654790357662] | 0.0638948063141 | 0.0638948063141 | 0.757253286777 |
| S3 | Hermite (saved) | 0.184683189977 | 0.184775132166 | 1.00049783734 | [0.0903043401475, 0.0944707920182] | 0.0631115916652 | 0.0631115916652 | 0.21148985508 |
| S3 | V2 (new) | 0.184683189977 | 0.184893907209 | 1.00114096595 | [0.0895020407675, 0.0953918664418] | 0.0955422807363 | 0.0955422807363 | 0.21148985508 |
| S4 | Hermite (saved) | 0.16088422589 | 0.161197130899 | 1.00194490795 | [0.0778895530481, 0.0833075778512] | 0.124706372176 | 0.124706372176 | 0.878001187341 |
| S4 | V2 (new) | 0.16088422589 | 0.161083085259 | 1.00123604019 | [0.0765102169725, 0.0845728682861] | 0.099513702998 | 0.099513702998 | 0.878001187341 |

| Source | Max node XY difference [m] | Max node yaw difference [rad] | Max matched execution XY difference [m] |
|---|---|---|---|
| S1 | 0.00613613225001 | 0.0001462601864 | 4.80173603418e-06 |
| S2 | 0.00353867055633 | 9.15836758657e-05 | 1.92799152207e-06 |
| S3 | 0.00737431920411 | 0.000257268313975 | 0.0315836819939 |
| S4 | 0.00910887629394 | 0.000324437617349 | 0.000761902224437 |

Maximum complete-reference differences equal the bridge-node differences because the entire downstream suffix is unchanged. S1/S2 executions overlap to 4.802/1.928 micrometres at matched command ticks; S4 maximum separation is .762 mm. S3 differs by up to .031583682 m. Distinct line styles and staggered markers identify overlapping curves in the world PNG.

### Natural selector exposure

| Source | Method | First submit | Nearest identity | First H5 identity | First H5 identities |
|---|---|---|---|---|---|
| S1 | C3 (saved) | 96 | E* | F_2 | F_2, F_3, F_4, F_5, F_6 |
| S1 | Hermite (saved) | 96 | B | bridge_1 | bridge_1, E*, F_2, F_3, F_4 |
| S1 | V2 (new) | 96 | B | bridge_1 | bridge_1, E*, F_2, F_3, F_4 |
| S2 | C3 (saved) | 1002 | F_3 | F_4 | F_4, F_5, F_6, F_7, F_8 |
| S2 | Hermite (saved) | 1002 | B | bridge_1 | bridge_1, E*, F_3, F_4, F_5 |
| S2 | V2 (new) | 1002 | B | bridge_1 | bridge_1, E*, F_3, F_4, F_5 |
| S3 | C3 (saved) | 1806 | E* | F_3 | F_3, F_4, F_5, F_6, F_7 |
| S3 | Hermite (saved) | 1806 | B | bridge_1 | bridge_1, E*, F_3, F_4, F_5 |
| S3 | V2 (new) | 1806 | B | bridge_1 | bridge_1, E*, F_3, F_4, F_5 |
| S4 | C3 (saved) | 1524 | E* | F_3 | F_3, F_4, F_5, F_6, F_7 |
| S4 | Hermite (saved) | 1524 | B | bridge_1 | bridge_1, E*, F_3, F_4, F_5 |
| S4 | V2 (new) | 1524 | B | bridge_1 | bridge_1, E*, F_3, F_4, F_5 |

| Source | Method | bridge_1 count | E* count | Any derived count | First H5 starts at original: tick | Last derived H5 tick | Last derived elapsed [s] | First-to-last span [s] |
|---|---|---|---|---|---|---|---|---|
| S1 | C3 (saved) | 0 | 0 | 0 | 96 | N/A | N/A | N/A |
| S1 | Hermite (saved) | 3 | 3 | 3 | 114 | 108 | 0.266666680574 | 0.200000010431 |
| S1 | V2 (new) | 3 | 3 | 3 | 114 | 108 | 0.266666680574 | 0.200000010431 |
| S2 | C3 (saved) | 0 | 0 | 0 | 1002 | N/A | N/A | N/A |
| S2 | Hermite (saved) | 2 | 2 | 2 | 1014 | 1008 | 0.183333342895 | 0.100000005215 |
| S2 | V2 (new) | 2 | 2 | 2 | 1014 | 1008 | 0.183333342895 | 0.100000005215 |
| S3 | C3 (saved) | 0 | 0 | 0 | 1806 | N/A | N/A | N/A |
| S3 | Hermite (saved) | 5 | 8 | 8 | 1854 | 1848 | 0.73333337158 | 0.700000036508 |
| S3 | V2 (new) | 4 | 8 | 8 | 1854 | 1848 | 0.73333337158 | 0.700000036508 |
| S4 | C3 (saved) | 0 | 0 | 0 | 1524 | N/A | N/A | N/A |
| S4 | Hermite (saved) | 3 | 5 | 5 | 1554 | 1548 | 0.400000020862 | 0.400000020862 |
| S4 | V2 (new) | 3 | 5 | 5 | 1554 | 1548 | 0.400000020862 | 0.400000020862 |

C3 selects only original downstream F rows in H5 from its first legal submit. Hermite/V2 have the same last-derived-row submit and first-original-start submit in every source. Only S3 bridge_1 exposure count differs: Hermite 5 versus V2 4; both retain E* for 8 submits. No causal selector experiment was performed. The common exposure duration and unchanged easy endpoint delay are a descriptive association, not proof that exposure caused the delay.

### First command differences and matched execution

| Source | First different submit tick | First different applied tick | Hermite [v, omega] | V2 [v, omega] | Delta v | Delta omega |
|---|---|---|---|---|---|---|
| S1 | 102 | 103 | [0.399999981839, 1.1842293004] | [0.39999998185, 1.18411757876] | 1.15527587496e-11 | -0.000111721634712 |
| S2 | 1002 | 1003 | [0.599999991708, -0.896706981334] | [0.599999991717, -0.89663881695] | 8.24951218448e-12 | 6.81643838037e-05 |
| S3 | 1806 | 1807 | [0.293588442122, 1.23664146913] | [0.293588442191, 1.23645118171] | 6.86117274107e-11 | -0.000190287418613 |
| S4 | 1536 | 1537 | [0.0677256436575, -1.33293408168] | [0.0696690107374, -1.33509261411] | 0.00194336707996 | -0.00215853242779 |

The first differing command uses the same H5 identities for Hermite/V2 in every source. The threshold is >1e-9 per command component. Every common successful submit and every common applied interval is retained in command_comparison.csv, including selected identities, held commands, command deltas and matched actual pose deltas. Shared pre-existing commands are marked COMMON_B_COMMAND.

| Source | V2-Hermite XY AUC .9 [m s] | V2-Hermite attach [s] | V2-Hermite linear TV | V2-Hermite angular TV |
|---|---|---|---|---|
| S1 | 1.69880276293e-06 | 0 | -7.53010986898e-11 | -0.000216732093213 |
| S2 | 1.0236422007e-06 | 0 | -3.12134762481e-11 | 6.76840789531e-05 |
| S3 | -0.000350445138674 | 0.0166666675359 | 0.0907016319936 | 0.0671981901436 |
| S4 | 3.47669423398e-05 | 0 | -0.00388673431012 | -0.00377523207872 |

### Secondary V2 own-reference metrics

| Source | Own XY AUC .3 | Own XY AUC .9 | Own yaw AUC .3 | Own yaw AUC .9 | Own max XY .5 [m] | Own attachment [s] |
|---|---|---|---|---|---|---|
| S1 | 0.0312208299272 | 0.142414683454 | 0.0320944174193 | 0.104232321135 | 0.192252120037 | 1.46666674316 |
| S2 | 0.0304430689371 | 0.118684298795 | 0.0231100567026 | 0.0771836114142 | 0.16156949475 | 1.10000005737 |
| S3 | 0.0135744928705 | 0.0754487579219 | 0.0458114545665 | 0.34251722345 | 0.0727942370042 | 1.4833334107 |
| S4 | 0.0133756684379 | 0.0954191119366 | 0.0488656147838 | 0.286036988942 | 0.130137514457 | 1.05000005476 |

Own-reference position AUC units are m s; yaw AUC units are rad s. These secondary values do not enter the primary classification.

### Validation, PNGs and protocol accounting

Saved-only validation and all historical validators pass. Reference hashes, actual installation, unchanged official settings, memory order, logical releases, saved integration/guard records, primary/own metrics and endpoint dwell parity pass. Tests: 25 focused; 661 regression passed and 1 known corpus skip; one rendering test passed after the pre-freeze axis adjustment. Compileall and working/staged diff checks pass. Synthetic fixture/test computations are not scientific calls.

Exactly three final PNGs were visually inspected alongside numerical sidecars. No post-freeze code/configuration or plotting change, no new planner solve, no retry or historical rollout occurred. The preparation metadata bug and pre-freeze rendering adjustment above did not change scientific results. **Scientific execution protocol deviations: none.**

- [world_execution_overview.png](../results/b_to_entry_vector_bridge_execution_03/figures/world_execution_overview.png)

- [transition_completion_metrics.png](../results/b_to_entry_vector_bridge_execution_03/figures/transition_completion_metrics.png)

- [bridge_tradeoff_diagnostics.png](../results/b_to_entry_vector_bridge_execution_03/figures/bridge_tradeoff_diagnostics.png)

- [result_summary.json](../results/b_to_entry_vector_bridge_execution_03/result_summary.json)

- [primary.csv](../results/b_to_entry_vector_bridge_execution_03/primary.csv)

- [paired_comparison.csv](../results/b_to_entry_vector_bridge_execution_03/paired_comparison.csv)

- [bridge_geometry.csv](../results/b_to_entry_vector_bridge_execution_03/bridge_geometry.csv)

- [selector_exposure.csv](../results/b_to_entry_vector_bridge_execution_03/selector_exposure.csv)

- [command_comparison.csv](../results/b_to_entry_vector_bridge_execution_03/command_comparison.csv)

- [figure_manifest.json](../results/b_to_entry_vector_bridge_execution_03/figure_manifest.json)

## Research interpretation

V2 preserves the hard-case recovery achieved by Hermite: S3 has both sustained attachment and original-FRESH endpoint dwell. Its .9 s position AUC is 0.18190299536729632 m s, versus 0.18225344050596992 for Hermite and 0.18655395332828628 for C3. The reduction versus Hermite is 0.00035044513867360516 m s. Attachment occurs one tick later than Hermite, at later original progress with 0.0177472328709918 m less remaining arc; endpoint dwell is unchanged. Endpoint error at the cap rises from .0604422053494252 to .061520244210175545 m, and linear/angular TV rise by .09070163199355008/.06719819014358208. This is a small transition/control trade-off rather than a completion gain.

S1/S2/S4 keep endpoint dwell but retain exactly the Hermite delay relative to C3: .15000000782310963/.10000000521540642/.15000000782310963 s. Thus V2 does not meet the declared requirement to reduce the penalty by at least one tick in two easy sources. No easy source worsens endpoint time versus Hermite. S4 still illustrates faster attachment with slower endpoint dwell versus C3: attachment is .23333334550261497 s earlier while endpoint dwell is .15000000782310963 s later, with .352763035400411 m more original arc remaining at attachment.

The frozen classification is VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN. **Decision B: V2 works, but does not improve the Hermite trade-off enough; more formulation work is needed before held-out evaluation.** Numerical stability enabled execution, but this execution study does not establish an additional completion benefit from the graph. V2 should not yet be frozen as the candidate for held-out obstacle testing on the evidence requested here. No next formulation or source collection is implemented.

## Limitations and not demonstrated

Four development sources, one rollout per source/method, controlled offline logical timing and a fixed official MPC. These are deterministic grid comparisons, not statistical significance estimates or real asynchronous deployment timing. Original FRESH has intrinsic tracking difficulty. No controller-aware factor, semantic-intent proof, obstacle/population generalization, real-world speedup, navigation-task completion, online repeated-chunk improvement, V2 optimality, V3 inferiority, universal graph superiority or final reconciliation method is demonstrated. V3 has no execution evidence here. A later held-out study remains a separate decision.

## Explicit answers to the eighteen questions

1. **Was exact DIAG_02 V2 reused without optimization?** Yes. Exact bridge/full-reference file hashes match; new reconciliation optimizer calls=0.

2. **Did V2 execute safely in all four?** Yes. All four reach 180 intervals with no numerical controller error or abort; minimum swept clearance .1527881035724341 m.

3. **Does V2 recover S3 attachment?** Yes, at 1.4833334106951952 s; one tick later than Hermite.

4. **Does V2 recover S3 original-FRESH endpoint dwell?** Yes, at 1.7666667588055134 s, exactly the Hermite time.

5. **How does S3 AUC compare?** V2 .18190299536729632, Hermite .18225344050596992, C3 .18655395332828628 m s at .9 s.

6. **How does S3 endpoint error compare?** At the fixed cap: V2 .061520244210175545 m, Hermite .0604422053494252 m, C3 .11235439505980861 m.

7. **Does V2 preserve easy-source endpoint dwell?** Yes, all S1/S2/S4 endpoint dwells remain observed.

8. **What are easy-source delays versus C3?** Hermite and V2 are identical: S1 .15000000782310963, S2 .10000000521540642, S4 .15000000782310963 s.

9. **Does V2 reduce Hermite completion delay?** No; endpoint time is identical to Hermite in all four sources, with zero easy sources improving by a tick.

10. **Is there faster attachment but slower endpoint completion?** Yes versus C3 in S4 for both bridges. V2 attaches .23333334550261497 s earlier but endpoint dwell is .15000000782310963 s later. No V2-versus-Hermite pair shows that pattern.

11. **How different are the references?** Maximum XY difference S1/S2/S3/S4: .006136132250009791/.0035386705563328607/.007374319204112393/.009108876293935806 m. Only X1 differs; B/E*/suffix are exact.

12. **How different are actual executions?** Maximum matched XY separation S1/S2/S3/S4: 4.801736034184658e-6/1.9279915220717356e-6/.0315836819939053/.0007619022244366991 m.

13. **How long are bridge rows exposed to H5?** Last derived-row submit occurs at .2666666805744171/.18333334289491177/.7333333715796471/.40000002086162567 s after B, identical for Hermite/V2. Counts containing any derived row are 3/2/8/5. Detailed first/last spans and identities are above.

14. **What commands first differ?** First differing submit ticks 102/1002/1806/1536; first differing applied ticks 103/1003/1807/1537. Commands and deltas are in the table and CSV. First differing H5 identities are equal between methods.

15. **Does TV improve or worsen?** S3 both increase; S4 both decrease. S1/S2 linear TV differences are below 1e-9; angular TV decreases in S1 and increases in S2. Exact deltas are above; no scalar control score is used.

16. **Is safety preserved?** Yes for the full installed references and these four executions. B/E*/suffix provenance and unchanged abort-only guard validation pass.

17. **What is the frozen classification?** VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN.

18. **Should V2 be frozen before held-out obstacle testing?** Decision B. It works and preserves S3 recovery, but does not reduce the easy-source Hermite completion penalty; further formulation work is warranted before selecting the held-out candidate. No held-out acquisition is started.
