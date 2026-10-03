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
