# B_TO_ENTRY_BRIDGE_01

## Frozen protocol

Starting fetched HEAD / origin/main: `50b4fd7afdbb7183ab03b4222fe12514d293fb3f`.
**Timing-controlled offline causal reference comparison.** Four-source development
mechanism experiment, with S3 as the unresolved case and S1/S2/S4 as controls.
Historical Native and C3 Entry-Suffix execution records are authenticated and
reused without modification or rerun. No Full/Half execution is requested.

| ID | Frozen source |
|---|---|
| S1 | OSA03_R00 |
| S2 | episode_001_repeat_01/handoff_013 |
| S3 | episode_008_repeat_01/handoff_023 |
| S4 | episode_013_repeat_00/handoff_020 |

The parent result summary SHA256 is
`e86083eb2f812e84c898885f2039b6520d336dd802c391dcaf29342cf17aa948`.
Its complete result ledger, input/code chain, and saved-only entry, correspondence,
transport, multisource, R00 and R01 validators must pass. Official MPC source SHA256
remains `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Run identifier: `data/b_to_entry_bridge_01/primary_20261002T020000Z`.
Actual processing UTC timestamps are recorded separately from this identifier.

### Fixed frames, entry and downstream identities

World XY metres, yaw radians counterclockwise, +Z up; observation-local x forward,
y left. Raw FRESH stays observation-anchored, `F=A F_local`; A and B stay fixed.
C3 is reproduced by the unchanged frozen continuous correspondence implementation,
including its tie and wrapped-yaw handling, and must exactly equal the prior record.

P is the actual recorded pose at B_tick-1, authenticated through the original
execution records and the saved OLD-to-B array. Both endpoint poses B and E* are
copied bit-for-bit into every bridge. The complete reference is
`[B, interior bridge rows, E*, original downstream suffix]`. B is a derived boundary
row, not a raw FRESH row. Downstream world rows are copied from the exact historical
Native `restoration.installed_world` array. Their original observation-local raw
rows are passed unchanged to the existing official installation path, reproducing
those installed downstream bits exactly. Only derived bridge rows are represented
as `A^-1 X` for this interface. A world/local round trip can cause <=1e-12 numerical
roundoff on derived installed bridge rows; fixed planning B/E bits are never
optimized or replaced. That representation roundoff is reported explicitly.

### Hermite baseline and frozen density

Let `u_in=normalize(B.xy-P.xy)`, `u_out=normalize(F_next.xy-E*.xy)`, and
`d_BE=||E*.xy-B.xy||`. Cubic Hermite XY endpoint derivatives are exactly
`m0=d_BE u_in`, `m1=d_BE u_out`. There is no tangent-magnitude tuning.
The retained suffix for the row scale is `[E*, original downstream poses]`;
`d_F` is the median of its XY segment lengths, including its first partial segment.

Compute continuous Hermite arc length with SciPy quadrature, absolute/relative
error tolerances 1e-12, limit 200, requiring error estimate <=1e-10. Freeze
`M=max(2,ceil(L_H/d_F))` before any graph solve or rollout. Invert cumulative arc
with Brent root finding (`xtol=1e-14`, `rtol=4*float64 epsilon`), and sample M equal
arc intervals. Shortest-angle yaw B→E follows normalized arc. Boundary bits are
copied after interpolation. Curve parameter and arc fraction are spatial, untimed
quantities; there is no waypoint timestamp or waypoint dt.

### SE(2) graph

Only X_1..X_(M-1) vary. X_0=B and X_M=E* are fixed. Initialize exactly from the
frozen Hermite bridge. The downstream suffix is excluded from variables.

- Incoming: `wrap(direction(B,X_1)-direction(P,B))/15 deg`.
- Outgoing: `wrap(direction(X_(M-1),E*)-direction(E*,F_next))/15 deg`.
- Smoothness: `D_j=X_j^-1 X_(j+1)`;
  `Log(D_(j-1)^-1 D_j)` divided by `(d_F,d_F,10 deg)` for j=1..M-1.
- Spacing: `(d_(j+1)-d_j)/d_F` for consecutive bridge edge lengths.

Each factor cost is the unaveraged sum of its squared normalized residual
components. All four lambda values are one; total cost has no half factor.
Spatial smoothness/spacing do not represent acceleration/speed. P→B supplies
only direction; its timed displacement magnitude is absent from the objective.
Spatial yaw deformation does not directly command robot angular velocity.

Use unchanged right-local LM: max80, central FD1e-6, damping .001, increase10,
decrease.3, maximum1e12, gradient/step tolerances1e-9, cost tolerance1e-12.
Every improving proposal must pass the existing direct complete-polyline checker
and have bridge XY edges >1e-12 m so endpoint directions are defined. Collapsed
proposal edges have finite atan2(0,0) residual evaluation but are rejected by this
domain check; no additional penalty or smoothness term is introduced.
No repair, projection, resampling or smoothing follows optimization. Nonconvergence
or a numerical solver exception is recorded with no graph rollout and classified
TECHNICAL_BLOCKED. Unsafe initial/returned reference is recorded as a reference
failure, without repair or retuning.

### Execution, safety and metrics

Freeze exact common B state/clock, physical u_minus provenance, already-applied
u_B_plus, controller previous_control, generation/version and original dt. Reuse
the exact source-specific transport-experiment schedule bytes. The unchanged
logical scheduler waits in wall time at each submitted state, and releases results
only at the predetermined tick. Official MPC remains H=5, nearest+1, unchanged
Q/R, limits, acceleration bounds, memory and stale-result behavior. This is not
real asynchronous deployment timing or a latency benchmark.

Full reference safety now explicitly includes B→bridge→E*→suffix: circular
radius .20 m, required footprint-edge clearance .05 m, unchanged Hospital/cart,
workspace and 1e-7 m numerical reserve. The abort-only execution guard remains
unchanged; unsafe commands are unapplied and valid prefixes are retained.
Compare attempted/accepted submit ticks, application ticks, integration counts
and initial provenance through both 54 and 180 intervals. Abort cases remain
censored; any unexplained schedule mismatch is TECHNICAL_BLOCKED.

Reuse the exact original-FRESH continuous forward projection and shortest-yaw
metrics: position/yaw AUC .3/.9, attachment (.10 m/15 deg for complete following
.30 s), attachment original row/arc fractions and remaining arc. Reuse the exact
original-FRESH endpoint dwell metric and null policy from the parent experiment.
Report T_endpoint-T_attach only if both are observed, endpoint error at the
historical 180-step nominal 3 s cap, swept clearance, command TV, max |v|/|omega|,
termination and abort. Attachment and endpoint dwell are distinct execution
outcomes. Neither gives LightNav rows timestamps or establishes navigation task
completion. Own-reference metrics remain secondary.

Report bridge arc/chord/ratio, chord turning max/RMS, min/max segment length,
separate factor costs, full reference clearance, polyline self-intersection,
and first official H5 selected bridge/original identities. Self-intersection uses
Shapely LineString.is_simple and is diagnostic only. Bridge nodes have bridge
identities; do not assign them fabricated original-FRESH progress identities.

### Predeclared interpretation

No single score. Use the requested categories with this fixed precedence:

1. TECHNICAL_BLOCKED: authentication, restoration, solver, validator or unexplained
   schedule failure.
2. BRIDGE_REFERENCE_FAILURE: unsafe reference, official numerical controller failure
   after successful restoration, or guard abort attributable to the new reference.
3. BRIDGE_TRADEOFF: S3 transition/recovery improves but any easy source loses an
   observed endpoint dwell or its dwell is >.10 s later than C3. Safety failures
   take precedence as BRIDGE_REFERENCE_FAILURE.
4. GRAPH_BRIDGE_SUPPORTED: Graph recovers S3 attachment/endpoint beyond C3/Hermite,
   with no easy-source endpoint loss or clear completion regression. Additional
   recovery means a dwell when Hermite has neither, or endpoint dwell when Hermite
   lacks endpoint dwell. Faster already-observed attachment alone is insufficient.
5. SIMPLE_BRIDGE_SUFFICIENT: Hermite recovers S3 attachment or endpoint, Graph adds
   no such recovery, and Hermite has no later easy-source endpoint beyond 1e-9 s.
6. BRIDGE_PARTIAL_ONLY: S3 .9 position AUC is lower by >1e-9 m s, but neither bridge
   obtains S3 attachment/endpoint and there is no major easy-source regression.
7. BRIDGE_INSUFFICIENT: remaining cases without meaningful supported recovery.

The AUC sign threshold is numerical, not a claim of practical effect size; report
absolute and relative magnitudes, .3 AUC and yaw tradeoffs too. Compare against C3
for mechanism effects and retain Native as historical context. Any improvement
without preserved safety is never support. No post-hoc category or tuned cutoff.

### Freeze and outputs

Authenticate → construct/save Hermite and entries/M → zero-solve official preflight
→ focused/regression tests → review → commit/normal push → record freeze SHA.
Then source order S1..S4, each with one Hermite rollout, one graph solve and one
eligible graph rollout. Maximum/exact expected: 4 graph solves, 8 new rollouts;
no retry, Native/C3 rerun, LightNav, RGB, Isaac, source search or instruction change.
Synthetic tests are not scientific evidence or scientific budget calls.

Saved-only validators recompute metrics, factor residuals and safety without a
new optimizer or controller solve. Small result JSON/CSV and exactly four PNGs are
tracked; large arrays and runtime traces remain under ignored data. Final PNGs:
`world_execution_overview.png`, `bridge_geometry.png`,
`transition_and_completion_metrics.png`, `attachment_vs_completion.png`.
No HTML dependency or extra final PNGs. Plot equal-axis XY and distinguish overlaps.

## Repository-confirmed facts

Results will be appended after the pushed freeze and single bounded execution.

## Research interpretation

Reserved until saved-only validation completes.

## Limitations

Four-source development experiment; offline controlled timing; intrinsic original
FRESH tracking difficulty; fixed official controller; no controller-aware factor;
no semantic-intent proof. No obstacle/population/real-world generalization, online
VLA improvement, navigation goal completion, graph superiority beyond these sources,
or final-method claim. If a method is frozen later, evaluate it on newly collected
held-out obstacle geometries in a separate experiment.

### Prepared spatial constants (before scientific execution)

| Source | M | d_F m | Continuous Hermite XY arc m |
|---|---:|---:|---:|
| S1 | 2 | 0.1501826264886162 | 0.12379645828894754 |
| S2 | 2 | 0.15051322580364157 | 0.14702648743330163 |
| S3 | 2 | 0.15648214800275878 | 0.21321503574581863 |
| S4 | 2 | 0.15214175181339587 | 0.18683023531723233 |

All four prepared Hermite polylines passed reference safety and official
zero-numerical-solve installation/restoration. M=2 means exactly one editable
interior pose in each graph. The controller receives the sampled polyline; the
continuous Hermite endpoint derivative does not imply its coarse first/last
chord exactly matches that tangent. Both errors are measured by E_in/E_out.

### Reproduction commands

Use the existing repository `.venv` without modifying it or external environments.
The focused test contains 29 tests. The full relevant command is:

```bash
git fetch origin main
git rev-parse HEAD
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z --check-only
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/bridge-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_bridge01.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/bridge-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_bridge01.py tests/test_spatial_entry_suffix_execution01.py tests/test_spatial_correspondence_selector_diag01.py tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_spatial_entry.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --mode freeze
# Review/stage only this namespace, protocol and append-only work log; commit and normal push.
git diff --cached --check
git commit -m "Freeze fixed-boundary B-to-entry bridge comparison"
git push origin main
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/bridge-mpl .venv/bin/python scripts/report_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --check-only
```
