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

Pending the pushed scientific freeze and single bounded execution.

## 5. Cross-source signed-delta summary

Pending saved-only validation; no outcome is preselected.

## 6. Research interpretation

The benchmark tests whether the paired OSA03 pattern extends to different original
local geometries. Neither the source choice nor interpretation declares a final
method or general need for E_R.

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
