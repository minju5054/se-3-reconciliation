# GP-SE2-02: hard-handoff transfer and fixed-MPC counterfactuals

This diagnostic transfer pilot asks whether the verified supplied-Jacobian GP
finds valid candidates on the three prescribed difficult handoffs, and whether
those references improve execution with the unchanged official pose-reference
MPC. It includes one benign control. Candidate convergence/availability, full
plan validity, original rollout success and objective improvement are separate.
These are dependent events from the development corpus, not a held-out benchmark
or an estimate of hard-case prevalence.

## Frozen scope

Starting main is `1462e81d9c8ebba04466a07d10898284d0fccf2e`, equal to fetched
origin/main. Both unrelated Stage 0 configuration edits remain untouched.
The original GP-SE2-01 run, both diagnostic runs, validated environment export,
existing images, source data and official MPC are retained and hashed.

| Role | Original event | Position mismatch m | Original direction diagnostic |
|---|---|---:|---|
| BENIGN_CONTROL | episode_001_repeat_01/handoff_002 | 0.00001205 | 0.00390 degrees |
| HARD_POSITION_AND_DIRECTION | episode_013_repeat_01/handoff_024 | 0.279113 | 36.5413 degrees |
| HARD_LARGE_TURN | episode_014_repeat_01/handoff_026 | 0.162573 | reliable window direction unavailable; raw yaw travel 130.191 degrees |
| HARD_POSITION_STRAIGHT | episode_000_repeat_01/handoff_019 | 0.323368 | 0.002115 degrees |

All four original routes contain no required gates. M2 keeps workspace, motion
and goal constraints; M3 additionally keeps the original obstacle inequalities.
Gate-required cases are outside this fixed scope; the provider is not extended.

The GP-SE2-01 configuration is copied byte for byte. T=3 s, support_dt=0.1 s,
31 supports, 150 variables, fixed original B and physical body twist, objective
weights, GP prior/interpolation, goal/route, tolerances, footprint 0.20 m and
clearance 0.05 m are unchanged. LightNav waypoint_dt remains null; the temporal
grid is an explicit downstream convention. There is no new terminal-stop
constraint. Plan terminal-pose acceptance differs from execution's final 0.20 s
goal dwell.

M0_NATIVE uses original F_native; M0_ADAPTER uses original F_common. M1 calls
the original solve_rigid with both original initializations. Each M2/M3 problem
uses unchanged FRESH plus the fixed same-curvature deceleration seed from its
own physical initial twist. No goal fitting, restoration or fallback is added.
SEED_ONLY independently checks this same deceleration vector without optimizing.

The planned order is the table's case order, then rigid identity/alignment,
M2 I0/I1 and M3 I0/I1: 16 GP and eight rigid starts, sequentially once. SLSQP,
CPU float64 JAX/analytic geometry, 200 iterations, 30 s prepared solve per GP
start and one BLAS thread remain fixed. Fresh providers compile per start;
construction/warmup and all validation costs are separate. Both initialization
costs contribute to each GP method's total.

## Derivative applicability and selection

The authoritative DIAG-02 root artifact validator and
`derivative_checks/authoritative_validation.json` are verified with their hashes,
distinct from preserved earlier diagnostic attempts. The provider, original
primal evaluator and full checker remain unchanged.

Before primary optimization, 32 point records are frozen: each case/method/seed
and one deterministic small perturbation. Original primal agreement and
multi-step, fixed-direction derivative tests reuse the original DIAG-02
tolerances/family rules. This is a compatibility check, not case selection.
Unsupported starts remain in the ledger; no guard is removed and no FD fallback
is supplied. Genuine mismatches block the affected start and remain evidence.

Each start retains the original minimum-objective dense-feasible candidate
among its initial/latest/actual callbacks. Across I0/I1 the same dense-first
minimum-cost rule applies, with initialization index breaking ties. The selected
candidate then needs the unchanged independent full and offset-grid acceptance.
A full-invalid selection is not replaced by an iterate with better MPC behavior.
M2 receives the same independent obstacle/known-workspace check.

## Counterfactual execution

Only full-valid GP and SEED_ONLY plans are submitted. Native/adapter/rigid use
the original GP-SE2-01 available-reference policy; any plan-invalid diagnostic
rollout is explicitly labelled and never called a validated plan.

Every candidate starts a new official MPC instance at original B, with physical
u_minus and recorded previous_control preserved separately. For
episode_000_repeat_01/handoff_019 these are respectively
[0.8,0.0001009450943702976] and [0.6192657763912551,0.00008006755974495329].
The historical solve is audited at its original input state, not forced to
match a new solve at B. World references use the inverse original capture
transform, with roundtrip validation, never a B reanchor. Planned GP velocity
is not sent as feed-forward to the pose-reference MPC.

The original schedule solves at t=0,0.1,...,2.9 s and integrates exact held
unicycle commands at 60 Hz through 3.0 s. Wall computation time does not advance
simulation. Failure/hold and collision-record-and-continue policies are unchanged.
No inference, new RGB/history, pose snap or extra stopping rule is introduced.
These offline counterfactuals do not test movement during optimizer latency or
online deployment readiness.

## Transfer compatibility before primary

All 32 frozen points pass, authorizing all 16 planned GP starts. Maximum primal
absolute difference is 4.88854e-12; maximum required scaled directional error is
0.157844 against the unchanged threshold 1. The smoke checks 170,686 required
smooth directional entries. Two obstacle entries at h=2e-5 for the straight
hard case's M3 deceleration seed cross a bilinear cell boundary and are separately
classified with branch metadata instead of treated as classical central
derivatives. No point is unsupported and no mismatch threshold changes.

Initial independent full checks show I0 infeasible for every case. I1 is
full-feasible only for the benign control; both GP methods' I1 seeds fail full
acceptance on all three hard cases. The seeds and goal are retained unchanged.
The transfer smoke takes 11.5103 s, separate from subsequent primary solve costs.

Implementation validation, frozen-run results, renderer evidence and final
reproduction commands are recorded below after their actual completion.

## Completed primary result

Operational status is **GP_SE2_02_COMPLETED**, subject to the authoritative final
artifact audit linked below. Execution interpretation is
**NO_ADDITIONAL_EXECUTION_BENEFIT**. Completion is not a research-success claim.

The first result is an adverse transition: on the large-turn event, native RAW
succeeds, M2's independently valid GP candidate executes but fails original goal
yaw/dwell, and M3 supplies no valid candidate. These are two method regression
records for one event, not two independent episodes. No RAW-failure → GP-success
or adapter-success → GP-failure transition occurs. Native → adapter itself loses
the large-turn success, so the reference-preparation effect must remain separate
from GP deformation. No hard event is successful under either GP method.

All planned 16 GP starts and eight original rigid starts ran once, sequentially.
There were no derivative guards triggered, timeout, retries, extra restarts,
case changes or scientific execution-code changes after freeze. Twelve GP starts
report SLSQP convergence, five retain a full-valid candidate, and four have a
full-valid final iterate. The fifth is a previously saved actual callback on the
large-turn M2/I1 run; its later final iterate fails dense motion validation.

### Every GP start

I0 is initially full-invalid for all cases. I1 is initially full-valid only for
benign. `J retained` is N/A when the original dense-first retention policy finds
no candidate, even if SLSQP reports convergence. Both GP methods retain all final
vectors/rejected checks in their start directories. No selected candidate is an
unchanged initial seed. `C` means SLSQP CONVERGED; other rows show its status code.

| Case | Method | Start | Termination | Iterations | Final full-valid | Retained full-valid | J initial | J retained | Selected source | Prepared s |
|---|---|---|---|---:|---|---|---:|---:|---|---:|
| Benign | M2 | I0 | C | 128 | True | True | 13.217924 | 0.194335090 | callback_0128 | 1.846 |
| Benign | M2 | I1 | C | 121 | True | True | 3.963366 | 0.194335067 | callback_0121 | 1.701 |
| Benign | M3 | I0 | C | 128 | True | True | 13.217924 | 0.194335090 | callback_0128 | 2.069 |
| Benign | M3 | I1 | C | 121 | True | True | 3.963366 | 0.194335067 | callback_0121 | 1.913 |
| Position + direction | M2 | I0 | C | 140 | False | False | 2937.344517 | N/A | N/A | 2.325 |
| Position + direction | M2 | I1 | C | 144 | False | False | 128.361602 | N/A | N/A | 2.381 |
| Position + direction | M3 | I0 | C | 140 | False | False | 2937.344517 | N/A | N/A | 2.645 |
| Position + direction | M3 | I1 | C | 142 | False | False | 128.361602 | N/A | N/A | 2.695 |
| Large turn | M2 | I0 | C | 115 | False | False | 751.438324 | N/A | N/A | 2.149 |
| Large turn | M2 | I1 | C | 119 | False | True | 164.119234 | 6.090669746 | callback_0115 | 2.227 |
| Large turn | M3 | I0 | C | 115 | False | False | 751.438324 | N/A | N/A | 2.421 |
| Large turn | M3 | I1 | C | 113 | False | False | 164.119234 | N/A | N/A | 2.418 |
| Position straight | M2 | I0 | 9: Iteration limit reached | 200 | False | False | 795.520503 | N/A | N/A | 10.940 |
| Position straight | M2 | I1 | 4: Inequality constraints incompatible | 9 | False | False | 142.789357 | N/A | N/A | 0.562 |
| Position straight | M3 | I0 | 3: More than 3*n iterations in LSQ subproblem | 85 | False | False | 795.520503 | N/A | N/A | 8.218 |
| Position straight | M3 | I1 | 4: Inequality constraints incompatible | 9 | False | False | 142.789357 | N/A | N/A | 0.621 |

Both benign I0 starts recover feasibility. Both benign I1 starts change the path
and reduce objective from 3.963365592 to 0.194335067, an absolute decrease
3.769030525 (95.0967%). This is an optimization metric, not a navigation gain.
Hard large-turn M2/I1 recovers feasibility from an invalid seed, so it is not
counted as improvement of a known-feasible seed. Its retained objective is
6.090669746 versus invalid seed objective 164.119233527. No hard final iterate
has independent full feasibility, despite eight of twelve hard starts converging.

Motion explains the converged hard rejections, not obstacle/workspace failure:

- Position/direction: final dense minimum forward speed is approximately
  -0.00301 m/s and maximum absolute lateral velocity 1.30e-4 m/s at t=0.48 s;
  linear acceleration reaches approximately -2.000078 m/s² at t=0.03 s.
  Collocation checks pass, but these between-point values fail original limits.
- Large turn: all four final iterates have negative forward speed around
  -9.57e-5 to -9.94e-5 m/s at t=1.57 s, beyond original tolerance. M2/I1's
  callback_0115 passes full/offset checks; later final objective 6.090655961
  is lower but invalid. The accepted callback has offset max |v_y|=9.451e-6 m/s.
  M3's rejection therefore does not establish that obstacle constraints make
  this case infeasible: M2's retained path independently passes the same
  environment check, while finite numerical iteration histories differ.
- Position/straight: M2/I0 reaches the fixed 200-iteration limit; M3/I0 reaches
  the internal LSQ subproblem limit; both I1 runs report incompatible inequalities.
  SLSQP status 4 is an algorithm termination, not proof of global infeasibility.

All eight rigid starts converge. The original minimum-cost retained solutions
use alignment on benign, position/direction and straight, and identity on large
turn. Rigid candidate availability is four, spatial plan validity is two; the
first hard and straight candidates have endpoint position errors 0.150002135 m
and 0.150022949 m, slightly above the independent spatial check's exact 0.15 m
limit. The original rigid squared-distance inequality and tolerance admitted
these candidates; the independent plan check rejects them without relaxing its
threshold. Its initial connector is explicitly UNDEFINED, not a validated
B-to-reference motion segment. Their three-second diagnostic rollouts are
retained and labelled plan-invalid.
GP continuous-time acceptance and baseline spatial-reference validation are
intentionally different levels of evidence.

### Candidate, plan and execution counts

Each cell is candidate available / independently plan-valid / rollout performed /
original rollout success. An unavailable rollout contributes no fabricated error
or zero-motion trace. SEED_ONLY retains an invalid seed as evidence but does not
submit it to the controller.

| Method | Benign (one event) | Hard (three events) |
|---|---|---|
| M0_NATIVE | 1 / 1 / 1 / 1 | 3 / 3 / 3 / 1 |
| M0_ADAPTER | 1 / 1 / 1 / 1 | 3 / 3 / 3 / 0 |
| M1_RIGID | 1 / 1 / 1 / 1 | 3 / 1 / 3 / 0 |
| M2_GP_NO_OBSTACLE | 1 / 1 / 1 / 1 | 1 / 1 / 1 / 0 |
| M3_GP_CONSTRAINED | 1 / 1 / 1 / 1 | 0 / 0 / 0 / 0 |
| SEED_ONLY | 1 / 1 / 1 / 1 | 3 / 0 / 0 / 0 |

All 24 method/event outcomes are present. Sixteen independent MPC rollouts ran:
12 native/adapter/rigid, three GP (benign M2/M3 and large-turn M2), and one benign
seed-only. The eight omitted rollouts are five GP candidate-unavailable outcomes
and three invalid seeds. All four historical-solve audits pass at their original
input states. Each actual rollout makes 30 fresh MPC solves: 480 total.

| Case | Native | Adapter | Rigid | M2 | M3 | Seed-only |
|---|---|---|---|---|---|---|
| Benign | success | success | success | success | success | success |
| Position + direction | position/dwell fail | position/dwell fail | invalid plan; position/dwell fail | no candidate | no candidate | invalid plan; not run |
| Large turn | success | yaw/dwell fail | yaw/dwell fail | yaw/dwell fail | no candidate | invalid plan; not run |
| Position straight | motion + position/dwell fail | motion + position/dwell fail | invalid plan; motion + position/dwell fail | no candidate | no candidate | invalid plan; not run |

No performed rollout has a physical collision, required-clearance failure,
unknown-workspace violation or controller failure. Minimum measured footprint
clearance is 0.302121 m; this does not make the failed goal/motion outcomes successes.
Original goal tolerances are 0.15 m and 15 degrees, with final 0.20 s goal dwell.
The reported `terminal_goal_dwell_s` field is the required interval, not achieved
dwell duration; its separate boolean records whether that interval passes.

For the large turn, final yaw errors are native 0.333°, adapter 38.827°, rigid
45.446°, M2 26.684°. Their final position errors are respectively 0.008371,
0.025552, 0.119324 and 0.011733 m. M2 improves yaw error relative to adapter but
still fails the 15° success condition. This failed pair is not included in the
success-conditioned secondary-metric comparison. Endpoint validity of a GP plan
has not guaranteed the official pose-reference MPC's actual yaw/dwell behavior.

For position/straight, all three executed baselines have about 0.38922 m final
position error and 3.80734 m/s² first control-grid acceleration magnitude versus
the original 2 m/s² bound. Recorded memory was 0.619266 m/s while physical speed
was 0.8 m/s. The fresh command is about 0.419266 m/s: internal memory constrains
the change to about -0.2 m/s, while the physical applied change is -0.380734 m/s.
That input-memory discrepancy is preserved, not attributed to GP mathematics.

### Benign quality and controls

All methods succeed on benign; only these pairs support the protocol's
success-conditioned quality comparisons. M2 and M3 return exactly the same
reference and produce identical states/commands under separate controller
instances; their measured compute durations differ. They do not return the
seed-only reference.

| Method | Time to goal s | Final position error m | Linear command TV m/s | Angular command TV rad/s | Max control-grid linear acceleration m/s² |
|---|---:|---:|---:|---:|---:|
| M0_NATIVE | 1.280 | 0.000061 | 0.799988 | 0.001288 | 2.000000 |
| M0_ADAPTER | 2.590 | 0.016290 | 0.539193 | 0.001359 | 1.974988 |
| M1_RIGID | 2.645 | 0.036063 | 0.644998 | 0.001137 | 2.000000 |
| M2_GP_NO_OBSTACLE | 2.595 | 0.016354 | 0.608654 | 0.000427 | 1.999999 |
| M3_GP_CONSTRAINED | 2.595 | 0.016354 | 0.608654 | 0.000427 | 1.999999 |
| SEED_ONLY | 1.865 | 0.024471 | 0.783501 | 0.000131 | 0.717603 |

Relative to adapter, GP reduces benign angular TV but increases linear TV and
is 0.005 s later to goal. Relative to seed-only, GP reduces final position error
and linear TV, but arrives 0.73 s later and has larger angular TV and acceleration.
Relative to rigid, GP improves this benign endpoint error and arrives 0.05 s
earlier. Native reaches the goal earliest. These tradeoffs do not support an
unqualified execution-quality improvement claim. GP's mean translation change
from F_common is 0.010126 m on benign; it is a secondary preservation measure.
No RAW/rigid/GP ranking is inferred from their different objective definitions.

## Measured offline computation

Costs are stored per start, per method and in `aggregate/compute_summary.json`.
All 16 supplied starts use fresh provider instances; CPU float64 compilation is
not reused. Each prepared solve retains the original 30 s budget/200 iterations.
No observed start reaches wall-time timeout; solve duration spans 0.562–10.940 s.

| Non-overlapping GP start cost components, all 16 | Seconds |
|---|---:|
| Case/seed load preparation | 0.231901 |
| Derivative graph construction | 0.624772 |
| Compilation and first-call warmup | 12.263738 |
| Harness setup | 0.017322 |
| Prepared solve, including original initial evaluation | 47.133004 |
| Original post-solve dense checking | 1.076535 |
| Independent full/offset checking | 19.365055 |
| Sum of these cold start costs | 80.712326 |

The entire optimization phase takes 87.411890 s, including rigid, persistence
and final plan checks, and is a larger timing scope. Do not add it to its GP
subcomponents. All rigid methods together take 0.780014 s including their checks.
Both starts count in each GP method's total below; no selection-only cost is used.

| Case | Rigid total s | M2 total s | M3 total s |
|---|---:|---:|---:|
| Benign | 0.162757 | 15.821620 | 16.088796 |
| Position + direction | 0.226677 | 6.841040 | 7.516331 |
| Large turn | 0.201174 | 6.901184 | 7.031413 |
| Position straight | 0.189407 | 13.465309 | 10.673899 |

Before primary, environment loading takes 0.041683 s, seed construction 0.000892 s,
initial full checks 5.159220 s, and transfer smoke 11.510307 s. Optimization and
evaluation reload the unchanged environment in 0.037798 and 0.042385 s.
SEED_ONLY has no optimization cost; its construction/checking is included in
preparation and not claimed to be computationally free.

The isolated MPC batch takes 2.239165 s; its 480 rollout MPC solves account for
1.586437 s. Outcome evaluation takes 0.880615 s. Optimization + MPC batch +
outcome evaluation is 90.531671 s of measured phase time, excluding the separate
preparation/smoke/environment costs, plots, renderer, artifact audit, tests,
interactive development and idle time. This is not an end-to-end online latency.

Across GP starts: 1,829 reported iterations, 5,048 objective calls and actual
primal cache misses, 5,064 equality calls, 5,064 inequality calls, and 1,826 calls
to each of the three supplied derivative callbacks. There are 5,048 primal GP
interpolations, 7,464 primal environment queries, 1,826 AD core evaluations and
2,677 derivative environment queries. Counts are different scopes, not additive
independent trajectories. Per-start profiling preserves the nesting of caller,
primal numerical components and derivative phases; no overlapping percentages
are added. The 1,825 recorded callbacks and 1,829 solver iterations need not
coincide on unsuccessful SLSQP exits.

## Evidence and reporting provenance

Primary numerical run:
`data/robotless_gp_se2_02/primary_20260919T024000Z/`.
Experiment implementation SHA:
`bd273b22ed9ddcb35a3f2eadcf550895adee46b9`.
The final report commit SHA and push verification are recorded in the local
`git_completion.json`, avoiding a self-referential document hash.

- `index.html`: all four cases and all six method outcomes; 48 required original
  PNGs with numeric/config/source-hash sidecars. Actual past is derived only from
  the original saved execution history; future traces are new counterfactuals.
- `aggregate/all_starts.csv`: all 24 GP/rigid initialization records.
- `aggregate/outcome_matrix.csv`: candidate/plan/rollout availability, collision,
  clearance, workspace, motion, position/yaw, dwell, all failure reasons and
  nullable errors for all 24 method/event outcomes.
- `aggregate/paired_metrics.csv`, `regressions.csv`, `timing.csv` and
  `compute_summary.json`: paired success metrics, regressions and complete costs.
- Separate GUI root `data/robotless_gp_se2_02/gui_primary_20260919T024000Z/`:
  12 genuine renderer captures (benign and first hard × six methods), four cases
  selectable, runtime validation and saved-sample readback. Label:
  `OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON`. No new inference, optimization
  or execution occurs during replay.
- `review_bundle.zip`: compact all-case overlays, tables, completion report,
  scientific narrative and actual GUI images; excludes raw RGB, full source
  dataset/environment, checkpoint, solver vectors and external source.
- `final_report.json` supersedes only the interim operational-status field in
  `aggregate/summary.json`; the evaluation-stage summary is preserved unchanged.
- `validation.json` is the authoritative independent final artifact audit;
  earlier `verification/` reports are preserved development checks, not its
  replacement. The first artifact preflight exposed validator schema issues
  (heterogeneous absent CSV fields and JSON-serialized nested CSV cells), repaired
  only in the reporting validator with corruption tests. Primary data was not
  changed or rerun to address those bookkeeping errors.

The frozen feasible-objective plots have an autoscale presentation limitation:
for sparse late feasible callbacks their x-axis omits the earlier N/A interval.
Original figures and values remain preserved. Separate reporting-only
`feasible_objective_history_full_time.png` figures expose the complete prepared
solve interval on common per-case axes; their own numeric/source sidecars record
the correction. The reporting cost key was clarified from an initially mistaken
`harness_initial_evaluation_s` label to `harness_setup_s`; original derived
reports and exact hashes remain under `reporting_revisions/initial_cost_label/`.
Neither correction modifies original measurements or scientific execution code.
The first supplemental legend obscured its short late feasible segment; that
rendering and its hashes are retained in `history_supplement_attempt01/`. Moving
the legend in the final supplemental rendering changes no plotted values.

## Limitations and next single experiment

The current fixed formulation has two exposed limitations: collocation
convergence can leave between-point GP motion invalid, and a valid GP pose
reference can fail goal yaw/dwell under the unchanged official controller.
Gate derivatives remain unsupported and untested; these four cases have no
gates. Static validated oracle geometry, ideal held-command unicycle execution,
three-second horizon, original controller memory and this development-corpus
selection limit inference. There is no physical robot, online optimization
latency test, MPC velocity feed-forward, or new LightNav update. Current results
are not a derivative-only causal comparison against the historical FD study,
whose second initialization differed.

The next single experiment should isolate **F_native versus F_common reference
preparation on the same large-turn event**, keeping the official MPC and original
goal fixed and auditing its selected reference targets and yaw evolution. That
RAW-success → adapter-failure transition exists before GP optimization, making
reference preparation the first uncertainty to localize. No such new experiment,
new optimizer, grid change, horizon change or tuning is performed here.

## Reproduction and final validation

Final full regression suite: **1,767 passed, 19 skipped in 84.78 s**; skips retain
their documented historical-corpus/fixture reasons. Compileall and diff checks
pass. The corrected independent numerical/report preflight passes **17,720
checks** in 42.55 s; the authoritative `validation.json` additionally checks the
completed package, all 52 plots, 12 renderer captures and final preservation
inventory. Its exact check count and audit cost are recorded in that report.

Use a new run directory for any future execution; these commands are the actual
phases, not instructions to overwrite this primary run. The environment pins and
software versions are archived in protocol/source snapshots. Numerical phases
set single-thread BLAS, CPU JAX and float64 before numerical imports.

```bash
.venv/bin/python scripts/run_gp_se2_02.py prepare --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/run_gp_se2_02.py smoke --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/run_gp_se2_02.py freeze --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/run_gp_se2_02.py optimize --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/run_gp_se2_02.py mpc --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/run_gp_se2_02.py evaluate --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/plot_gp_se2_02.py --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/supplement_gp_se2_02_history.py --run data/robotless_gp_se2_02/primary_20260919T024000Z
# GUI inherited DISPLAY=:1 and XAUTHORITY=/run/user/1000/gdm/Xauthority.
./scripts/launch_gp_se2_comparison_replay.sh --run data/robotless_gp_se2_02/primary_20260919T024000Z --output data/robotless_gp_se2_02/gui_primary_20260919T024000Z --verify --no-hold
.venv/bin/python scripts/report_gp_se2_02_results.py --run data/robotless_gp_se2_02/primary_20260919T024000Z --gui data/robotless_gp_se2_02/gui_primary_20260919T024000Z
.venv/bin/python scripts/complete_gp_se2_02_review.py --run data/robotless_gp_se2_02/primary_20260919T024000Z --gui data/robotless_gp_se2_02/gui_primary_20260919T024000Z --narrative docs/GP_SE2_02_HARD_HANDOFF_TRANSFER.md
.venv/bin/python scripts/run_gp_se2_02.py finalize --run data/robotless_gp_se2_02/primary_20260919T024000Z
.venv/bin/python scripts/validate_gp_se2_02.py --run data/robotless_gp_se2_02/primary_20260919T024000Z --output data/robotless_gp_se2_02/primary_20260919T024000Z/validation.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```
