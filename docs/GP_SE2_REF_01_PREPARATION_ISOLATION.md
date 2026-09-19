# GP-SE2-REF-01: suffix selection and resampling factor isolation

This fixed diagnostic asks which FRESH preparation operation changes official
MPC targets and execution on the GP-SE2-02 large-turn regression, with one benign
control. It separates input changes, matched-state selection, closed-loop commands
and original goal/dwell outcomes. No GP/rigid optimization, new VLA input,
controller modification, horizon extension or preprocessing repair is performed.

## Frozen source and design

Starting main and fetched origin/main are
`47668b868e84173fab4516ab7d5edb65ef75b841`. Both unrelated Stage 0 configuration
edits remain untouched. Authoritative GP-SE2-02 and GP-SE2-01 validations and
published artifact hashes are verified; their raw/derived arrays, results,
images, validated environment and pinned external MPC are preserved.

Primary source is `data/robotless_gp_se2_02/primary_20260919T024000Z/`; original
reference/configuration source resolves to
`data/robotless_gp_se2_01/primary_20260918T054000Z/`. The environment is the
validated original copy identified by that provenance, not a regenerated map.
New output is `data/robotless_gp_se2_ref_01/primary_20260919T062000Z/`.

| Case/order | Event | Native rows N | Original nearest j | Fixed suffix k | Removed original rows |
|---|---|---:|---:|---:|---|
| PRIMARY_LARGE_TURN | episode_014_repeat_01/handoff_026 | 10 | 0 | 1 | 0 |
| BENIGN_CONTROL | episode_001_repeat_01/handoff_002 | 10 | 1 | 2 | 0, 1 |

Original `prepare_reference` determines k once from F_native at original B using
the original weighted XY/wrapped-yaw distance. It is never recalculated on a
resampled array or recut externally during rollout. For F=F_native and S=F[k:]:

| Variant | Suffix factor | Resample factor | Reference | Large-turn / benign rows |
|---|---|---|---|---|
| R00_NATIVE | off | off | F | 10 / 10 |
| R10_SUFFIX_ONLY | on | off | S | 9 / 8 |
| R01_RESAMPLE_ONLY | off | on | R(F) | 30 / 30 |
| R11_CURRENT_ADAPTER | on | on | R(S), identical to F_common | 30 / 30 |

R uses unchanged `interpolate_rows`: linear XY, shortest-angle yaw, source row
times linspace(0.1,3.0,L), queries arange(1,31)*0.1, original constant-reference
behavior for L=1. These are interpolation conventions; intrinsic LightNav
waypoint timing is null and the MPC receives only pose rows. All references keep
the original world frame and goal endpoint. Periodic endpoint pose equivalence
and literal bitwise equality are separately recorded.

Row provenance preserves original left/right indices, alpha, fractional row
coordinate, XY, wrapped/unwrapped yaw and original/interpolated identity. Row
progress is neither time nor distance. Geometry audits retain removed prefix
poses, duplicate/rotation rows, corner/yaw knots, path length and yaw variation.
Bidirectional point-to-polyline distances use at most 1 mm sample spacing with
at least 100 subdivisions per nonzero segment; they are sampled diagnostics,
not continuous geometric equivalence proofs. Near-zero translation is a
predeclared 1 mm diagnostic, separate from the 1e-12 m rotation-only classification.

## Official selection and execution semantics

The pinned MPC has HORIZON=5, MPC_DT_S=0.1 and CONTROL_RATE_HZ=10. It selects the
five rows following the nearest weighted pose, clamps to/repeats the final row,
and sequentially unwraps each selected yaw relative to the preceding yaw,
starting at the current pose. Gains, objective, limits, tie-breaking and this
next-row policy are unchanged. Local row numbers across variants have different
meanings, so comparisons use original fractional coordinates and world poses.

All 30 GP-SE2-02 Native control input poses per case are frozen probe states.
The official selector and independent `selection_audit` evaluate all four
references at each identical state: 240 selector calls, no state integration,
memory update, tracker construction or MPC solve. All probes precede primary
rollouts. Logs include per-row XY/yaw cost contributions, tie margins, source
mapping, actual unwrapped target yaw, physical lookahead and goal-row inclusion.

Primary order is large-turn then benign, each R00,R10,R01,R11 once. Each fresh
official tracker starts at original B with original physical u_minus and
recorded previous_control separately restored. World rows pass through the
inverse original capture transform and official path installation, with
roundtrip checks; never a B reanchor. Original counterfactual_rollout and
_synchronous_solve are reused unchanged. Each rollout has 30 solves at
0,0.1,...,2.9 s, 180 exact held-command unicycle ticks at 60 Hz and 181 states
through 3 s. Total planned primary solves are 240; historical/other solves zero.
Failure/hold/collision handling is unchanged. Wall time never advances simulation.

R00/R11 compare against the original stored Native/Adapter results. Before
primary, selected-reference tolerance is fixed at 1e-12, command tolerance at
1e-6 (original audit values), and accumulated state/endpoint diagnostic tolerance
at 1e-6 absolute metres/radians with rtol=0. Indices, original inputs/settings
and success predicates must agree exactly. Literal bitwise equality is reported
separately. These diagnostic tolerances do not change physical acceptance.

## Evaluation and interpretation

Original evaluate_rollout supplies unchanged footprint/clearance/workspace,
motion, original route, 0.15 m position, 15-degree yaw and final 0.20 s dwell
predicates. `terminal_goal_dwell_s` remains required duration; a separately named
sampled achieved dwell diagnostic is added. First goal entry/time-to-goal stays
N/A if absent. Commands, acceleration and yaw come from newly applied MPC controls,
not reference derivatives. Three-second failure does not imply permanent failure.

Each continuous metric reports all four values and five contrasts: suffix
without/with resampling, resampling without/with suffix, and
y11-y10-y01+y00 interaction. Nulls remain null. Observation flags use a frozen
1e-6 native-unit numerical floor, not a practical-benefit or significance
threshold. Success flags are shown separately, not combined into a score.
The two dependent development-corpus events do not support population inference.

Static evidence is primary. No new GUI is implemented for this narrow diagnostic.

## Observed results

Experiment code was committed as `48a8b47267b0d31bf1794e9dbba6a47ff10619f2` before freezing 11 execution/config/test files and 78 input files. All scheduled conditions executed exactly once; no technical or result-driven numerical retry occurred. Four R00/R11 reproduction checks pass with **all compared values bitwise equal**, including selected indices/poses/yaw, controller memory, commands, predictions, states, endpoint metrics and success flags. Wall timings are measured anew, not expected to reproduce.

| Event | Variant | Success | Position error (m) | Absolute yaw error (deg) | Original time to goal (s) | Dwell pass | Achieved final sampled dwell (s) |
|---|---|---|---:|---:|---:|---|---:|
| Large turn | R00_NATIVE | PASS | 0.00837072 | 0.33293881 | 1.67 | True | 1.330 |
| Large turn | R10_SUFFIX_ONLY | PASS | 0.00746595 | 0.26586555 | 1.595 | True | 1.405 |
| Large turn | R01_RESAMPLE_ONLY | FAIL | 0.02543699 | 37.30355257 | N/A | False | 0.000 |
| Large turn | R11_CURRENT_ADAPTER | FAIL | 0.02555209 | 38.82754310 | N/A | False | 0.000 |
| Benign | R00_NATIVE | PASS | 0.00006089 | 0.00001234 | 1.28 | True | 1.720 |
| Benign | R10_SUFFIX_ONLY | PASS | 0.00006089 | 0.00001227 | 1.28 | True | 1.720 |
| Benign | R01_RESAMPLE_ONLY | PASS | 0.00017018 | 0.00045756 | 2.185 | True | 0.815 |
| Benign | R11_CURRENT_ADAPTER | PASS | 0.01628985 | 0.00175422 | 2.59 | True | 0.410 |

Required terminal dwell is 0.20 s for every condition. The last column is an additional sampled duration, never substituted into the original predicate. Both failed large-turn conditions fail **GOAL_YAW_FAILURE and GOAL_DWELL_FAILURE**; position, clearance, workspace, route and actual motion checks pass. All eight have zero controller failures. Minimum clearance is 0.30212125 m for all four large-turn variants and 1.32694497 m for all four benign variants. No missing time-to-goal is filled with zero or the horizon.

### Descriptive factor contrasts

Each row below is computed from the four measured runs. The complete 42-metric/event table is `aggregate/factor_contrasts.csv`; all five contrasts and N/A values are retained. These are not population estimates.

| Event / metric | Suffix without R | Resampling without S | Suffix with R | Resampling with S | Interaction |
|---|---:|---:|---:|---:|---:|
| Large turn / terminal_position_error_m | -0.000904765129 | +0.0170662712 | +0.000115096826 | +0.0180861331 | +0.00101986195 |
| Large turn / terminal_yaw_error_deg | -0.0670732583 | +36.9706138 | +1.52399053 | +38.5616775 | +1.59106379 |
| Large turn / time_to_goal_s | -0.075 | N/A | N/A | N/A | N/A |
| Benign / terminal_position_error_m | -2.02674667e-09 | +0.000109289161 | +0.0161196644 | +0.0162289556 | +0.0161196665 |
| Benign / terminal_yaw_error_deg | -7.42349859e-08 | +0.000445217936 | +0.00129666252 | +0.00174195469 | +0.00129673676 |
| Benign / time_to_goal_s | +0 | +0.905 | +0.405 | +1.31 | +0.405 |

Large-turn success transitions: suffix alone retains PASS; resampling alone changes PASS→FAIL; adding suffix to resampling retains FAIL. Thus the combined adapter is not an interaction-only failure: resampling alone is sufficient to reproduce failure under this fixed protocol. Suffix affects continuous behavior (including −0.067073° yaw-error change without resampling and +1.523991° with resampling); its yaw-error interaction is +1.591064°. Benign remains PASS in all four conditions, with resampling adding 0.905 s to goal time without suffix and 1.310 s with suffix; interaction is +0.405 s.

## Mechanism: input → selection → command → outcome

**Measured input changes.** Large-turn suffix removes original row 0, including
a 1.694023° prefix yaw transition. R01 keeps the full 118.299734° accumulated
input yaw variation; R11 keeps the suffix's 116.605712°. Every variant retains
the original final goal pose. Resampling does not delete the final target, but
places intermediate targets at different row-order progress. It also produces
small XY corner cuts: sampled maximum bidirectional polyline deviations are
about 0.240 mm (R01) and 0.236 mm (R11). Therefore this experiment isolates the
specified resampling operator, not density independently of every geometric
effect of that operator.

**Matched-state evidence.** On exactly the same historical Native input pose at
t=1.0 s (yaw 28.117°), Native/suffix-only select original rows 5–9, ending at the
original goal yaw −31.593°. R01 selects fractional progress 4.655–5.897, ending
at +3.366°; R11 selects 4.586–5.690, ending at +6.633°. These different targets
exist without a new MPC solve or any feedback-induced state divergence.

| Event / measurement | R00 | R10 | R01 | R11 |
|---|---:|---:|---:|---:|
| Large turn: final row first selected on matched Native states (s) | 0.9 | 0.9 | 1.7 | 1.8 |
| Large turn: final row first selected in actual closed loop (s) | 0.9 | 0.9 | N/A | N/A |
| Benign: final row first selected on matched Native states (s) | 0.5 | 0.5 | 1.2 | 1.3 |
| Benign: final row first selected in actual closed loop (s) | 0.5 | 0.5 | 2.0 | 2.5 |

No backward original-progress selection, exact tie or declared near-tie occurs
in any case/variant's matched or closed-loop records. The smallest matched-state
margin is 4.81185e-7 weighted cost units in large-turn R01; that condition's
closed-loop minimum is 6.73427e-4. Nearest-row costs and first/second margins
are saved per probe/control step; argmin tie policy remains unchanged. Local array indices alone do not establish
progress. `selected_original_progress_vs_time.png` and its numeric sidecar show
actual fractional original coordinates and final-goal inclusion separately for
matched-state probes and closed-loop runs.

**Closed-loop evidence.** Selections differ already at t=0, but large-turn first
commands are nearly identical because the original acceleration limits bind.
Using the frozen 1e-6 command comparison tolerance, Native versus R01 first
diverges at 0.2 s and Native versus R11 at 0.3 s. At t=1.0 s, applied omega is
−1.448657 / −1.807168 / −0.559136 / −0.650448 rad/s in R00/R10/R01/R11 order.
The corresponding actual yaws are 28.117 / 25.035 / 72.587 / 66.896 degrees.
At t=2.9 s the two resampled variants still select only through original progress
7.1379 / 6.7931 (goal row 9), with last target yaw −14.457° / −9.664°.
Their final applied omega remains −0.561130 / −0.425055 rad/s. They are still
turning at the 3 s endpoint; the observed failure is within this horizon,
not proof of permanent inability to rotate.

On benign, the initial five-row XY horizon arc changes from 0.601721 m (Native)
to 0.186620 m (R01) and 0.144872 m (R11). R01's first applied speed is about
0.600000 m/s versus Native's 0.800000 m/s. Resampling delays progress and goal
entry here too, while all four still satisfy the fixed endpoint and dwell tests.
A smaller angular command total variation in the failed large-turn runs is
not better execution: those runs have not completed the required rotation.

**What is established.** For this fixed large-turn event and unchanged official
MPC, resampling alone is sufficient to change selected targets, commands and
success; suffix alone is not sufficient to cause failure. The logs localize the
representation effect to changed physical/yaw lookahead of the fixed five-row
selector and the ensuing closed-loop progression. The endpoint remains present
in the input yet does not enter the resampled runs' actual reference horizon.
The observed interaction is a continuous-metric effect, not a requirement for
the failure to exist. Missing yaw, a universal double-skip failure, and changed
controller memory are not supported explanations for this result.

**What remains unproven.** No experiment separately holds every interpolated
geometry change constant while varying row density; no controller counterfactual
holds the selected horizon fixed. A density/lookahead explanation has direct
log support, but exclusive mediation by density is not established. These two
events do not estimate general failure frequency. This preparation diagnosis
does not resolve GP between-point motion violations or explain every GP failure.

## Compute, provenance and verification

The existing MPC environment is unchanged: external checkout
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`, Python 3.11.16, CasADi 3.7.2,
NumPy 2.4.6. Official MPC source SHA256 is
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
All 240 primary MPC solves complete successfully; there are 240 matched-state
selector calls and **zero additional/historical MPC solves, zero GP/rigid solves,
zero VLA calls**. All eight independent tracker instances preserve original
capture roundtrip, B, physical u_minus and recorded previous_control separately.

| Measured component | Wall time (s) | Scope |
|---|---:|---|
| Input/seed-free preparation | 0.069732 | Excludes preceding preservation/hash audit |
| Official runtime load | 0.022335 | Within worker process |
| Matched-state probes | 0.125582 | No MPC optimization |
| Eight-rollout batch | 1.242219 | Includes solve, integration, enrichment and writes |
| Actual MPC solve calls | 0.917046 | Subset of rollout batch; not additive to it |
| Entire worker process | 1.492671 | Includes the worker components above |
| Original environment loading | 0.044829 | Post-solve evaluation environment |
| Outcome evaluation | 0.417395 | Eight original evaluations and reproduction/contrast output |

Large-turn MPC solve subtotal is 0.522615 s and benign subtotal 0.394431 s.
Per-variant timings remain in `mpc_output/summary.json` and `aggregate/summary.json`.
Nested times must not be added into an inflated total. Initial preservation
hashing and interactive analysis/reporting are not included in the 0.069732 s
preparation timer; no complete cold end-to-end cost claim is made. All wall times
are offline costs, never added to the 3 s simulation horizon.

Original/core preservation covers 40,564 file paths plus both unrelated user
configuration hashes. `preservation_before.json` and final
`preservation_after.json` retain exact inventories. Original config is copied
byte-for-byte and all frozen input/code hashes are recorded separately.
The independent validator rechecks source arrays, lineage, pinned selector
semantics, exact integration, memory, original acceptance, reproduction,
contrasts and actual figure numbers. Expected scientific failures remain valid
artifact outcomes.

The first non-authoritative checker preflight found six metadata-key omissions
in the checker for already-frozen XY sidecars (`reference_factorization`). The
original failed check and correction record are retained under `verification/`;
only the reporting validator changed. The second preflight passes 668,477 checks.
No numerical result, tolerance or primary execution was changed or repeated.
The first full pytest run encountered two existing local-IPC PermissionErrors
under the restricted sandbox (1,837 passed, 19 skipped); its log is preserved.
Final tests with permitted local IPC and authoritative validation are recorded
below after completion.

## Evidence and reproduction commands

Open `data/robotless_gp_se2_ref_01/primary_20260919T062000Z/index.html` for all 22
required PNGs and exact numeric/hash sidecars, or use `review_bundle.zip` in the
same directory for the compact shareable report. Suggested report sequence:
input yaw versus original progress → selected reference yaw versus time →
angular command versus time → actual yaw and original-goal error. The matching
XY overlay gives spatial context but does not alone explain the large rotation.

The original input-row overview has dense labels and an overlapping subplot
label in both cases. Those original PNGs and hashes remain preserved. A separate
rendering-only correction at
`data/robotless_gp_se2_ref_01/presentation_20260919T063000Z/` adds readable per-variant
input detail with complete row/yaw/progress tables under a new output ID/source
hash; it does not rerun the selector, MPC or numerical experiment. The other
primary figures passed visual inspection. The optional Isaac GUI was not run:
`NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY`. These are scientific plots, not simulator
screenshots or a newly executed online episode.

Commands executed from the repository (original run refuses overwrite):

```bash
run=data/robotless_gp_se2_ref_01/primary_20260919T062000Z
.venv/bin/python scripts/run_gp_se2_ref01.py prepare --run "$run"
# Implementation commit, then freeze before any primary probe/solve.
.venv/bin/python scripts/run_gp_se2_ref01.py freeze --run "$run"
.venv/bin/python scripts/run_gp_se2_ref01.py execute --run "$run"
.venv/bin/python scripts/run_gp_se2_ref01.py evaluate --run "$run"
.venv/bin/python scripts/plot_gp_se2_ref01.py --run "$run"
.venv/bin/python scripts/render_gp_se2_ref01_detail.py --source-run "$run" \
  --output data/robotless_gp_se2_ref_01/presentation_20260919T063000Z
.venv/bin/python scripts/package_gp_se2_ref01_review.py --run "$run"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
.venv/bin/python scripts/run_gp_se2_ref01.py finalize --run "$run"
.venv/bin/python scripts/validate_gp_se2_ref01.py --run "$run"
```

No shell launcher was added or changed. Generated arrays, PNGs, ZIPs, datasets,
external source and environments are excluded from Git. Starting, experiment
and final Git SHAs are reported separately; final SHA/push confirmation lives in
local `git_completion.json` to avoid a self-referential artifact hash cycle.

## Diagnosis and next single experiment

`native_adapter_regression_reproduced`, `suffix_effect_observed`,
`resampling_effect_observed`, `interaction_observed` and
`matched_state_selection_change_observed` are all true under the predeclared
descriptive definitions. `mechanism_localization_level` is
`INPUT_SELECTION_COMMAND_OUTCOME_MEASURED`. These flags describe a diagnosis,
not graph optimization or navigation improvement.

Next single proposed experiment: keep the R11 30-row array and its geometry
fixed, and change only the horizon selection stride from derived-row units to
original-row progress units. Keep nearest-row policy, five targets, MPC dt,
gains, objective and original success predicates fixed. This would test the
lookahead mechanism more directly while holding the stored input geometry
constant. A precise selection/progress rule must be frozen before that follow-up;
it is not implemented or tested here. No deployment or GP-feasibility benefit is
inferred from this proposal.

## Final validation and status

Operational status: **GP_SE2_REF_01_COMPLETED**. This means the diagnostic
schedule and evidence checks completed, not that navigation improved.

- Authoritative `validation.json`: PASS, **709,970 checks**, zero errors, no
  deferred checks, 8.005008 s; zero MPC/optimizer solves during validation.
- Final full pytest: **1,843 passed, 19 existing skips**, 94.27 s. Local IPC was
  permitted for the two existing socket tests; no test logic was relaxed.
  Compileall and `git diff --check` pass; no shell launcher changed.
- All **40,564** original/core file hashes, official checkout and both unrelated
  user config hashes match before/after. Frozen execution source remains unchanged.
- Primary review ZIP: **5,081,835 bytes**, 61 files / 22 PNGs; SHA256
  `36138db62462c4ff48c7ffd183e47b45eb43bd297697741b33b18a4008dc0085`.
- Supplemental `presentation_20260919T063000Z/input_details.zip`:
  **3,683,258 bytes**, 20 files / 8 PNGs; SHA256
  `a47d5d476a280aa32e6e15f18fb01b7ab22c542587fa8c1382fbe9040e34e060`.
  All ZIP members pass saved-source hash/size and CRC readback checks. Its index
  intentionally links the separate primary index for the remaining figures.
- No primary numerical retry, changed reference/controller/formulation, new
  inference, new online episode or new simulator-runtime claim occurred.
