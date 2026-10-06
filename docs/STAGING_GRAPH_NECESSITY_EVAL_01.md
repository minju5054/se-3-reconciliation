# STAGING_GRAPH_NECESSITY_EVAL_01

## Frozen protocol

Starting HEAD and fetched origin/main: `4e26e547cbda3cbfee8dc842a40091388cde7543`.
Branch main. Unrelated Stage0 configuration changes and GPU memory script are
preserved. Run identifier: `data/staging_graph_necessity_eval_01/primary_20261006`.
Actual processing host UTC is recorded independently of this identifier.

Question: outside the registered development episodes, is fixed boundary staging
sufficient, is intermediate geometry needed, or does frozen V2 provide additional
execution value beyond BOTH staging and deterministic Hermite? A negative outcome
is a normal completion. No formulation tuning or new acquisition is authorized.

**Timing-controlled offline causal reference comparison.** Saved genuine corpus,
source-disjoint from registered prior development use; not an unseen independent
test set. All reference rows are untimed spatial poses. World XY metres, +Z up,
yaw radians CCW; raw observation-local x forward/y left. FRESH remains
`F=A F_local`, never re-anchored at B. Observation, request, ready, installation
and first-FRESH application clocks remain distinct. B is that actual application
boundary; physical incoming u_minus differs conceptually from controller memory.

### Development exclusion audit

The machine registry reads only identity membership from GP01 and REF03 source
selection manifests, source-only Genuine13 ledger membership (all 13 subsequently
used in execution-loss/delay-attribution development), plus documented ATTACH01
and staging sources. It registers **47 distinct handoffs, 33 entire recorded
episodes**. In particular 013/01/024, 001/01/013, 008/01/023, 013/00/020 and
021/01/003 are excluded with their entire episode. GP02, REF01/02/04 and DIAG01–08
reuse GP01/REF03 members; no new exclusion identity is lost. OSA03 R00/R01 are
separate acquisition roots and never enter this corpus. Historical EXP/DATA02
used a different, since-cleaned acquisition lineage, not these 881 event IDs.

An episode is the recorded `episode_NNN_repeat_RR` identifier. The other repeat
of the same initial scenario is not automatically excluded: source/episode
separation does not establish scenario-family independence. The initial 13 are
not relabeled held-out. The registry includes per-source evidence paths/hashes
and per-episode exclusion reasons. No historical AUC, attachment, endpoint,
controller TV, convergence or method success field enters the ranking function.

### Inventory, source gates and deterministic selection

Scan all **881** events in the authenticated original online corpus. Reuse REF03
source hashes, original completion manifests, observation transforms and the
unchanged direct Hospital geometry checker. Reconstruct recorded observation-to-B
held-command steps only to authenticate saved motion, not as a new rollout.

Require finite original OLD/FRESH Nx3, FRESH N>=2/nonzero arc, required clocks,
reproducible original observation transforms and unchanged source-only timing
checks (real-time pacing, causal overlap, saved post-switch exposure and inflight
capture). No ranking by the original acquisition's future tracking outcome.
Require whole original FRESH, B and recorded observation-to-B swept history safe;
radius .20 m, edge clearance .05 m and existing 1e-7 m numerical uncertainty.
Moving means physical v_minus>.20 m/s AND recorded accumulated travel>=.02 m.

Reuse C3's unchanged analytic continuous piecewise SE(2) minimizer, .10 m / 15 deg
scales, earliest-arc 1e-12 cost ties, shortest yaw and forward lower bound at C1 XY
projection. Require remaining original arc>=.50 m and >=5 suffix rows (E* plus at
least four downstream rows for nontrivial H5 content). No endpoint imputation or
source repair. Check complete actual B->E*->original-suffix reference safety.

P is the exact saved pose immediately before B. The two independent descriptors:
`delta_phi=abs(wrap(direction(B,E*)-direction(P,B)))` and
`rho=|E*-B|/d_F`, `spacing_severity=abs(log(rho))`. d_F is the historical median
XY edge of `[E*, original suffix]`, including its partial first segment. Any
nonfinite or <=1e-12 m incoming/chord/suffix edge fails closed using historical
bridge EPS. This is a numerical domain rule, not a new direction-reliability
threshold. Do not add the post-solve 1e-5 m collapse cutoff to source selection.

First take up to three descending delta_phi; then up to three descending spacing
severity excluding already selected episodes. Exact lexical case-ID ties. The
selector accepts only a closed geometry-only SelectionRecord, with no outcome
or file-reading interface. Target6/minimum4; no threshold relaxation or replacement
based on execution. If fewer than4, freeze INSUFFICIENT_SOURCE_DIVERSITY and make
zero scientific optimizer/MPC calls. No source acquisition follows a shortfall.

Cumulative gates:881 ->838 valid timing/source ->725 whole FRESH safe ->722 B safe
->718 history safe ->564 moving ->481 adequate future/B_ENTRY geometry/severity
->**134 eligible in 23 nonexcluded episodes**. The deterministic order is:

| ID | Source | delta_phi rad | rho | abs(log rho) | Remaining arc m | B / first submit / release lag ticks |
|---|---|---:|---:|---:|---:|---|
| E1 | episode_011_repeat_01/handoff_001 | 1.93993692808537 | 0.381268779308186 | 0.964250695071812 | 1.20410049643219 | 189 / 192 / 1 |
| E2 | episode_015_repeat_01/handoff_005 | 1.83489691172411 | 0.387662228940363 | 0.947620862470457 | 1.15061052573068 | 433 / 438 / 1 |
| E3 | episode_018_repeat_00/handoff_018 | 1.78960535793396 | 0.474174712996055 | 0.746179432382024 | 0.849086953093281 | 1373 / 1374 / 1 |
| E4 | episode_003_repeat_01/handoff_010 | 1.57081911748484 | 1.00928500564022e-05 | 11.5036833000151 | 0.985800511755208 | 819 / 822 / 1 |
| E5 | episode_026_repeat_01/handoff_008 | 1.57110591372852 | 1.09642010818934e-05 | 11.4208750394973 | 0.875382300630129 | 643 / 648 / 1 |
| E6 | episode_024_repeat_01/handoff_009 | 1.57068429325673 | 1.75492609477951e-05 | 10.9504987201758 | 0.945215068340802 | 741 / 744 / 1 |

E4–E6 have approximately1.5–2.6 micrometre B-to-entry gaps. The prescribed
symmetric abs(log rho) selects severe compression as well as long gaps. They
remain selected; there is no post-selection reliability or minimum-gap tuning.
There are five distinct raw FRESH arrays among six selected sources (E5/E6 share
one). All six references use Hermite M=2; V2 M=2 is fixed independently. These
geometric facts are known before any scientific optimization or execution.

### Methods and immutable boundaries

Order is C3_ENTRY_SUFFIX, B_ENTRY_STAGE, HERMITE_BRIDGE, V2_GRAPH_BRIDGE.
Structures are `[E*, F...]`, `[B,E*,F...]`, `[B,X1_H,E*,F...]`, `[B,X1_V2,E*,F...]`.
Reuse suffix_reference, staged_reference, hermite_bridge and DiagnosticProblem
V2 directly. Exact planning B/E*, exact Native-installed downstream world rows,
exact original observation-local raw suffix rows. Only derived poses are represented
as A^-1 X for the adapter; record installation roundoff and verify <=1e-12.
B/X1 receive no original identity; E* preserves fractional original row/arc.
No raw mutation, smoothing, transport, resampling or new yaw generation for staging.

Hermite uses chord-scaled incoming/outgoing tangents, equal spatial arc samples,
shortest yaw versus normalized arc and unchanged d_F density. V2 uses exactly one
editable interior node, initialized from the same M=2 Hermite. Fixed vector
boundary targets equal initial Hermite edge lengths times original unit tangents;
normalize by d_F. Reuse spatial SE(2) smoothness normalized by d_F/d_F/10deg and
spacing differences divided by d_F. All lambdas1. P->B distance is not a target.
No graph factor is added or changed.

Solver is unchanged right-local LM: max80, central FD1e-6, damping1e-3, rejection
x10, acceptance x.3, maximum1e12, gradient/step1e-9, cost decrease1e-12. Full
reference safety and >1e-12 bridge-edge domain acceptance remain unchanged.
Post-solve V2 stability reuses all eight DIAG02 gates, including min bridge edge
>1e-5 m and no bridge self-intersection. Nonconverged/unstable last iterates remain
diagnostics, never executable fallback references. No repair, retry or V3.

### Common B, schedule and execution

Use the original generic load_source authentication and run_method executor.
Freeze exact saved B/tick/simulation time, A, u_minus provenance, applied first
FRESH u_B_plus, previous_control/memory, generation/version and original integration
dt=float32(1/60). No pending historical solve may be discarded at B. Source-specific
schedule repeats the first saved post-B completion lag on the original absolute
six-tick grid, exactly the predeclared multisource periodic_schedule rule. This
schedule is constructed from saved timing before outcomes; no new Native rollout.

Official MPC unchanged: SHA256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`, H5 nearest+1,
Q/R, limits, acceleration limits, previous_control, endpoint repetition and stale
handling. A slow solve pauses logical simulation; release uses predetermined ticks.
Validate exact common state, attempted/accepted submits and applications through
54 and180 intervals. Safety/numerical failures retain their valid censored prefix;
unexplained schedule/restoration failures are technical blockers. An unsafe
proposed command is never applied. No controller/environment/source edits.

Zero-solve official Native installations authenticate downstream bits without a
Native rollout. Then zero-solve C3/B_ENTRY/Hermite installations check arbitrary N,
state restoration and safety. Twelve installation preflights pass with zero
numerical solves. V2 after solve uses the same unchanged official installation.

### Evaluation and frozen classification

Reuse full ORIGINAL FRESH forward-only continuous projection, shortest yaw,
position/yaw AUC .3/.9, max XY .5, separation growth and sustained attachment.
Attachment requires <=.10 m and <=15deg for the complete following .30s sampled
interval. Report original row, arc fraction and arc left at attachment. Reuse
original-FRESH endpoint dwell with the same tube/dwell around its final pose.
Null remains null; endpoint is not a navigation goal. Reuse first tube entry,
post-attach duration only when both observed, nominal3s cap error/progress/arc,
path and mean abs(v) before dwell, TV, max abs(v/omega), safety/abort/termination.
Record actual official nearest/first/all H5 identities, derived-row exposure and
first original H5 start; no selector-causality claim.

BOTH_DWELLS requires both observed AND a valid safe reference/execution.
GRAPH_SPECIFIC_POSITIVE requires valid/safe V2 BOTH_DWELLS and either both B_ENTRY
and Hermite lack BOTH_DWELLS, or all three have both with V2 attachment at least
one tick earlier than each and endpoint no later than either. One tick means
integration_dt_s-1e-9; scalar tolerance1e-9. AUC alone cannot qualify.
INTERMEDIATE_GEOMETRY_POSITIVE means B_ENTRY lacks both while Hermite and V2
have both. V2 regressions: unsafe reference, attributable guard abort, or loss of
endpoint/attachment retained by BOTH simpler methods. Nonconvergence is also
reported separately as negative evidence.

Precedence (unchanged after outcomes):

1. TECHNICAL_BLOCKED: authentication/restoration/interface/schedule/validator failure.
2. INSUFFICIENT_SOURCE_DIVERSITY: selected<4; zero scientific calls.
3. GRAPH_SPECIFIC_GAIN_SUPPORTED: >=2 graph-positive selected sources, no V2 safety
   failure and no loss of endpoint retained by both simpler methods.
4. INTERMEDIATE_GEOMETRY_NEEDED: >=2 intermediate positives, prior criterion unmet.
5. STAGING_EVALUATION_SUPPORTED: B_ENTRY BOTH_DWELLS in every technically valid
   selected source and prior graph criterion absent.
6. GRAPH_FORMULATION_NOT_ROBUST: >=2 invalid/nonconverged V2, simpler methods
   technically valid. This follows staging in the requested precedence; failure
   counts remain visible even if staging determines the overall label.
7. MIXED_EVIDENCE: remaining scientifically valid cases.

Keep separate V2-minus-B_ENTRY and V2-minus-Hermite AUC/attachment/endpoint deltas
and pre-outcome severity. No combined score, post-hoc cutoff or population inference.

### Freeze and call accounting

Prepare/authenticate/select/reference/preflight -> tests -> compileall/diff review
-> commit+normal push -> verify pushed freeze SHA -> six V2 planning calls in
E1..E6 order -> source order x C3/B_ENTRY/Hermite/V2 rollout order. Exclusive start
markers and directories prevent retries. Reconciliation calls are forbidden in
prepare/validation/rollout phases. V2 uses the unchanged historical planning
wrapper with a literal V2 variant. No V3 path is exposed in this experiment CLI.

Upper budgets from frozen schedules: optimizer6, rollouts24, MPC solves720,
applications712. Invalid references are skipped; abort prefixes may reduce counts.
No LightNav, RGB, Isaac, new source acquisition or historical scientific rerun.
No outcome-driven code/config/source changes. Large arrays stay ignored under data.

Saved-only source re-scan, trace/feasibility/damping audit, exact boundary/suffix
checks, official state/selection/command/integration/guard audit and independent
endpoint dwell parity precede reports. Exactly three compact PNGs with numeric
and hash sidecars: source_reference_geometry, execution_outcome, severity_vs_benefit.
N/A stays explicit and invalid V2 geometry is labeled as a diagnostic last iterate.

## Reproduction commands

Use existing .venv; do not rerun scientific processing after its exclusive marker.

```bash
git fetch origin main
git status --short
git branch --show-current
git rev-parse HEAD origin/main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_staging_graph_necessity_eval01.py --run data/staging_graph_necessity_eval_01/primary_20261006 --mode inventory
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_staging_graph_necessity_eval01.py --run data/staging_graph_necessity_eval_01/primary_20261006 --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/staging-eval-mpl .venv/bin/python -m pytest -q tests/test_staging_graph_necessity_eval01.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_staging_graph_necessity_eval01.py --run data/staging_graph_necessity_eval_01/primary_20261006 --mode freeze
git diff --cached --check
git commit -m "Freeze source-disjoint staging graph necessity evaluation"
git push origin main
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python scripts/run_staging_graph_necessity_eval01.py --run data/staging_graph_necessity_eval_01/primary_20261006 --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_staging_graph_necessity_eval01.py --run data/staging_graph_necessity_eval_01/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/staging-eval-mpl .venv/bin/python scripts/report_staging_graph_necessity_eval01.py --run data/staging_graph_necessity_eval_01/primary_20261006
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_staging_graph_necessity_eval01.py --run data/staging_graph_necessity_eval_01/primary_20261006 --check-only
```

The initial inventory was invoked through the same scan/exclusion functions from
an inline Python command while the CLI was being implemented. It made no numerical
optimization/MPC call. Protocol metadata was completed before freeze; selected IDs,
thresholds, references and schedules were not changed. All actual commands/logs
are retained locally and the regression command is appended below before freeze.

## Pre-science verification

The second source-only audit reproduced all 881 ledger rows, exclusion registry,
selection order and 3,987 authenticated file hashes exactly. Zero numerical MPC
preflight used 12 worker launches and 24 reference installations (Native plus
C3/B_ENTRY/Hermite for six sources); these are not scientific rollouts.

Relevant regression: **742 passed, 1 skipped in 403.39 s**. It included the then
46 focused tests plus 696 historical tests. Two additional insufficient-source
stop fixtures and the final renderer/validator guard edits were covered by the
latest focused run: **48 passed in 2.44 s**. Thus 744 distinct tests passed across
these runs; the known missing ignored EXP-01B/EXP-02B corpus test is the one skip.
Historical saved validator chains ran within the regression suite without new
scientific solves. Synthetic three-PNG output was visually inspected; it is not
experimental evidence. Compileall and unstaged/staged diff checks passed.

The full regression command was:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/staging-eval-regression-mpl .venv/bin/python -m pytest -q --basetemp=/tmp/staging-eval-regression tests/test_staging_graph_necessity_eval01.py tests/test_b_to_entry_boundary_row_ablation04.py tests/test_b_to_entry_vector_bridge_execution03.py tests/test_b_to_entry_graph_formulation_diag02.py tests/test_b_to_entry_bridge01.py tests/test_spatial_entry_suffix_execution01.py tests/test_spatial_correspondence_selector_diag01.py tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_spatial_entry.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/staging-eval-focused-mpl .venv/bin/python -m pytest -q tests/test_staging_graph_necessity_eval01.py
```

## Repository-confirmed facts

Scientific results pending pushed freeze and single execution.

## Research interpretation

No scientific method outcome has been generated at this protocol stage.

## Not demonstrated / limitations

Not unseen population evaluation, online asynchronous VLA validation, real-world
validation, navigation task success, semantic intent proof, automatic correspondence
validation, full editable-FRESH graph validation or graph superiority unless the
frozen GRAPH_SPECIFIC_GAIN_SUPPORTED criterion passes. Episode-repeat separation
is not scenario-family separation. Shared raw arrays limit diversity. Numerical
micrometre gaps and the fixed V2 collapse gate may be relevant; no threshold is
changed to favor a result. The next full-FRESH graph/correspondence stage is excluded.
