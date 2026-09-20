# GP-SE2-DIAG-07: lateral-only plan rejection vs official MPC execution

Question: can an unchanged saved G3 reference that fails only the original
plan-level lateral predicate satisfy the unchanged official-MPC execution
predicate? Execution never changes plan validity or creates a deployable hard
candidate. This is one fixed hard event, not a new navigation benchmark.

## Motivation and frozen protocol

DIAG-03 localized collocation/interior motion gaps with consistent interpolation.
DIAG-04 added quarter lateral equalities and motion inequalities and encountered
numerical regression. DIAG-05 retained only quarter inequalities and recovered
convergence, with residual speed/acceleration and lateral violations. DIAG-06
added three frozen signed motion witnesses: sampled nonlateral motion checks
passed, while all four hard latest plans still failed lateral acceptance.
DIAG-07 changes no optimization or acceptance. It only authorizes diagnostic
execution of those exact rejected pose references through the existing MPC.

Starting main: `f454a3de86a18160ec182cccab3f4857f1b92bf0`, equal to fetched
`origin/main`. Source G3 execution: `ca53a0744a98a5bbce1d153b5d056273d21b4fb1`.
Run: `data/robotless_gp_se2_diag_07/primary_20260920T103000Z/`.
DIAG-06: `data/robotless_gp_se2_diag_06/primary_20260920T083200Z/`.
Original inputs resolve through its GP-SE2-02 manifest, not reconstructed data.

Five provenance references, in this order:

1. Hard `episode_013_repeat_01/handoff_024`, M2 / I0_FRESH.
2. Same hard event, M2 / I1_DECEL.
3. Same hard event, M3 / I0_FRESH.
4. Same hard event, M3 / I1_DECEL.
5. Benign `episode_001_repeat_01/handoff_002`, M3 / I1_DECEL.

Each vector is the saved `latest_iterate`; poses/twists must equal the saved
latest support arrays literally. The interface receives exactly `poses[1:]`,
30 world pose rows. No GP velocity feed-forward, new interpolation, projection
onto a feasible path, smoothing, suffix recomputation or re-anchoring occurs.
The hard historical selected vector remains null. Benign latest equals the
saved retained optimized vector and differs from the initial seed.

The original full checker (original dense, independent plan and 1 ms/.371-offset
checks) is rerun before primary. Hard eligibility requires every named criterion
except lateral to pass. Additional failure stops primary. Benign must pass all.
The original lateral tolerance stays 1e-5 m/s. Original speed/acceleration
numerical allowances, environment uncertainty and goal/route thresholds remain.

Deduplication requires exact float64 C-order world-reference bytes and identical
frozen event/state/config identity. No approximate merging is allowed. The
pre-execution gate finds five unique references (four hard), so the frozen
primary is five independent tracker instances, 150 MPC solves. Two separate
historical solves (one per event at its recorded historical input, not B) precede
primary. Both audits pass. No retry or historical RAW/adapter/rigid rollout.

All five plan rechecks reproduce the entire DIAG-06 full-check result literally.
30,937 transitive source/core/environment/result/acquisition file hashes are
checked, plus the two unrelated Stage-0 user edits. The online authoritative
validator is valid with its existing collection limitations. No corpus is rerun.

| Reference | Exact world float64 value SHA256 | Max plan abs(vy), m/s | Full plan |
|---|---|---:|---|
| hard_m2_i0 | aed35adfde636881dba890799fa4e1c9cd63560dc57fb9fbdf2d3da01b5aea8e | 0.000121841764019 | FAIL: lateral only |
| hard_m2_i1 | d9c42d59b9d011b0adea24c1ea5c62a8646dc3a8572064395102df284614a9fc | 0.000121888332794 | FAIL: lateral only |
| hard_m3_i0 | 42e87952cabacc98034265523ee7debae14fe24fd2acd941632fb085d2fc57db | 0.000121786499873 | FAIL: lateral only |
| hard_m3_i1 | 62404cd9bf90c678fd1e19c4f9970125b4153165782911e996f5c4235cec9068 | 0.000121861559880 | FAIL: lateral only |
| benign_m3_i1 | 1a9f2e323bd6bcfac6009d5dd6d63c56bc7d37c2245a1a8c8863b7e062c88086 | 0.00000002197233237 | PASS |

Exact source paths, file SHA256, vector/support/world value SHA256, shape, dtype,
flags and alias mapping are in `reference_manifest.json`. These are five
execution conditions from two handoffs, not five independent events.

## Execution semantics and scope

The new composition calls unchanged `counterfactual_rollout` and renames its
legacy candidate storage keys to reference terminology. No tracker subclass,
selector replacement or global patch is introduced. The pinned external MPC is
loaded read-only from its existing mujoco_demo virtual environment. Checkout
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`; MPC source SHA256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.

Original B, physical u_minus and pre-FRESH controller previous_control are
separate preserved fields. The original capture transform is inverted, then
unchanged set_body_path reinstalls the world reference with its existing 1e-11
roundtrip tolerance. B never replaces the capture transform. The official
nearest weighted XY/wrapped-yaw selector takes the next five rows, with its
original tie-break and sequential yaw unwrap. HORIZON=5, MPC_DT_S=.1,
CONTROL_RATE_HZ=10. New solves at t=0,.1,...,2.9 drive 180 exact held-command
1/60 s integration steps and 181 states, ending at 3 s. Compute time does not
advance simulation. Official error/zero-command/hold behavior stays unchanged;
no collision-triggered stop, pose snap or new observation is introduced.

Each hard result keeps `plan_valid=false`, `deployment_candidate=false`,
`plan_failure_reason=lateral_velocity`, `diagnostic_execution_authorized=true`
and `diagnostic_execution_only=true`. The hard label is:
**PLAN INVALID: LATERAL ONLY / DIAGNOSTIC EXECUTION / NOT A DEPLOYMENT CANDIDATE**.
Every rollout is **OFFLINE COUNTERFACTUAL / NOT ONLINE NAVIGATION**.

The actual command has only [v, omega]; there is no lateral command channel.
No fabricated executed-vy measurement is recorded. The model is structurally
unicycle; this is not a validation of real wheel/robot dynamics.

The unchanged evaluator requires original clearance, known workspace, route,
goal position <=.15 m, yaw <=15 degrees, final .20 s dwell, command-grid motion
bounds and zero controller failures. Final goal is original FRESH, not the last
GP target. Final-reference-row horizon inclusion and original-goal-region
inclusion are separately recorded. Prediction geometry means saved Euler-node
polylines; unexecuted tails are not actual execution.

## Commands, validation and interpretation

From repository root (no dependency or system Python changes):

```bash
.venv/bin/python scripts/run_gp_se2_diag07.py prepare --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
.venv/bin/python scripts/run_gp_se2_diag07.py audit --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_gp_se2_diag07.py tests/test_gp_se2_rollout.py tests/test_gp_se2_evaluation.py -q
# Commit implementation and this protocol before primary.
.venv/bin/python scripts/run_gp_se2_diag07.py freeze --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
.venv/bin/python scripts/run_gp_se2_diag07.py execute --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
.venv/bin/python scripts/run_gp_se2_diag07.py evaluate --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
MPLCONFIGDIR=/tmp/gp_diag07_mpl .venv/bin/python scripts/plot_gp_se2_diag07.py --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
.venv/bin/python scripts/validate_gp_se2_diag07.py --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

The saved-record validator independently repeats plan/execution checks,
reconstructs all integration ticks, verifies actual selector/input/memory logs,
checks source/output hashes and numeric plot/table agreement. It never calls
MPC/GP/VLA. Scientific failure is not artifact corruption. New actual-runtime
counts include primary MPC and separate historical audits; synthetic tests are
implementation evidence only.

If a hard reference succeeds, the observation is conservative plan rejection on
this fixed event/reference; it does not make its continuous GP plan valid. If
all hard executions fail, no execution-success counterexample was observed;
that does not establish a causal role for lateral violation. Benign failure
triggers a harness/control warning. Four starts are not independent handoffs,
no population conclusion follows, no online latency or physical collision
response is tested, and no lateral threshold/soft penalty/new representation is
authorized by this experiment.

At protocol commit: preparation, five original plan gates, exact deduplication,
two actual historical audits and 56 relevant implementation tests pass. Primary
counterfactuals have not yet run. Results will be appended after the frozen run.

## Completed result

Execution/freeze SHA: `666fd14ac6fa2ba14b8467e1e889cd9faec76760`.
Operational status: **GP_SE2_DIAG_07_COMPLETED_WITH_LIMITATIONS**.
Research interpretation: **LATERAL_INVALID_REFERENCES_EXECUTION_FAIL**.
Benign control: **BENIGN_EXECUTION_PRESERVED**.

All five planned unique reference conditions ran once in order, through five
independent official tracker instances. Four hard references are close but not
byte-identical and therefore were not merged. There are 150 primary MPC solves
plus two separate historical-audit solves: **152 total**. No new GP/rigid solve,
VLA inference, RGB capture, online episode or Isaac GUI runtime occurred. There
was no retry, reference edit, acceptance change or scientific code change after
freeze. All primary solves have zero controller numerical failures.

| New reference | Full plan | Full execution | Final position m | Absolute yaw deg | Goal time s | Achieved final dwell s | Original execution failure |
|---|---|---|---:|---:|---:|---:|---|
| Hard M2/I0 | FAIL: lateral only | FAIL | 0.169928611 | 2.965731 | N/A | 0 | goal position; dwell |
| Hard M2/I1 | FAIL: lateral only | FAIL | 0.169928028 | 2.965885 | N/A | 0 | goal position; dwell |
| Hard M3/I0 | FAIL: lateral only | FAIL | 0.169931910 | 2.965717 | N/A | 0 | goal position; dwell |
| Hard M3/I1 | FAIL: lateral only | FAIL | 0.169929155 | 2.966142 | N/A | 0 | goal position; dwell |
| Benign M3/I1 | PASS | PASS | 0.016353398 | 0.003229 | 2.595 | 0.405 | none |

Hard full-plan rejection remains unchanged; the original selected hard
candidate remains N/A in DIAG-06. The raw unchanged execution evaluator reports
`goal_failure`; the separate diagnostic taxonomy records both
`EXECUTION_GOAL_FAILURE` and `EXECUTION_DWELL_FAILURE`. All four hard trajectories
never enter the original simultaneous goal region in the fixed 3 s; even their
minimum sampled position error is 0.169820–0.169824 m, above 0.15 m. Terminal yaw
passes the original 15 degree bound. This is fixed-horizon failure, not evidence
that the reference could never reach a goal under a different experiment.

Every execution passes required clearance, known workspace, route and actual
command motion constraints. No physical overlap occurs. Minimum footprint-edge
clearance is 1.84301915 m for the four hard conditions and 1.32694497 m for benign,
well above the unchanged 0.05 m plus original uncertainty/curve allowance.
The original route is preserved; these two events have no required finite gate,
so route pass must not be described as a demonstrated gate traversal.

| Reference | First [v, omega] | Delta from physical u_minus | Final [v, omega] | Max abs dv/dt m/s² | Max abs domega/dt rad/s² | Linear / angular command TV |
|---|---|---|---|---:|---:|---|
| Hard M2/I0 | [0.699766429, 0.484304733] | [-0.100233571, 0.499999964] | [0.031529151, -0.194956383] | 1.999999947 | 4.999999638 | 2.307564 / 3.199355 |
| Hard M2/I1 | [0.699766008, 0.484304733] | [-0.100233992, 0.499999964] | [0.031525577, -0.194950931] | 1.999999947 | 4.999999638 | 2.307557 / 3.199336 |
| Hard M3/I0 | [0.699765890, 0.484304733] | [-0.100234110, 0.499999964] | [0.031521040, -0.194959589] | 1.999999947 | 4.999999638 | 2.307571 / 3.199351 |
| Hard M3/I1 | [0.699766265, 0.484304733] | [-0.100233735, 0.499999964] | [0.031539773, -0.194947854] | 1.999999947 | 4.999999638 | 2.307556 / 3.199332 |
| Benign M3/I1 | [0.799999995, 0.000035977] | [-0.000000005, 0.000150721] | [0.265133879, -0.000005161] | 1.999998719 | 0.001507208 | 0.608655 / 0.000428 |

Commands are m/s and rad/s; total variations are m/s and rad/s. Command-grid
acceleration includes the first step from physical u_minus, not an inferred GP
velocity. Hard applied speed ranges approximately 0.013843–0.717057 m/s and
max abs omega is about 0.989997 rad/s. Benign applied speed ranges
0.265134–0.8 m/s. The actual lateral command dimension is **nonexistent**.
No measured executed-vy channel is manufactured.

## Selection, prediction and goal-margin observations

All 150 actual selected five-row references, 150 saved six-pose MPC predictions,
900 held integration commands and 905 states are retained. The independent audit
reconstructs every integration state exactly and checks actual target indices,
XY, periodic yaw, input state and controller-memory chain. No nearest-progress
backward jump occurs in any of the five executions.

For each hard execution:

- A selected target inside the original goal region first appears at 1.9 s.
- The final GP reference row first enters the five-row horizon at 2.7 s and
  stays in the selected horizon thereafter. The final GP row is not literally
  the raw FRESH goal, so these are different diagnostics.
- The GP plan endpoint position error is 0.133143–0.133146 m, inside the original
  0.15 m region. It has only approximately 0.01685 m of radial goal margin.
- The actual final state is approximately 0.038461 m from that GP endpoint,
  leaving the actual original-goal distance near 0.16993 m. These distances are
  not collinear scalar quantities and must not simply be summed.
- Saved prediction polylines have minimum footprint-edge clearance at least
  1.84132786 m; no unsafe inside-corner prediction is observed here.
- Initial physical command and recorded controller memory happen to be equal:
  [0.8, -0.015695230803144968]. They were independently loaded, not forced equal.
  No memory-discrepancy explanation is supported for this event.

Benign first goal-region target inclusion is 2.1 s, final-reference-row inclusion
2.5 s and original goal entry 2.595 s. Plan endpoint error is about 0.000039737 m,
leaving substantially more original-goal position margin. Benign final tracking
distance to its final reference is about 0.016390 m. Its original physical speed
and controller memory are also equal; the source values were preserved.

These measurements associate the hard failure with a limited goal margin and a
remaining endpoint tracking displacement under this row selector. They do not
prove a unique cause, do not establish that a different selector would recover
it, and do not isolate a causal effect of v_y. Targets did reach the original
goal region, so “the goal was never in the MPC horizon” is not supported. Final
yaw is small enough to pass, so this is not the large-turn yaw regression from
REF-01. Saved prediction tails remain predictions and are not executed traces.

## Historical context — not new rollouts

The same hard event's GP-SE2-02 original baseline records and configuration are
hash-verified context only:

| Historical method | Original full execution | Position m | Yaw deg | Spatial plan |
|---|---|---:|---:|---|
| M0_NATIVE | FAIL: goal | 0.209819705 | 0.129967 | valid |
| M0_ADAPTER | FAIL: goal | 0.204503995 | 0.271836 | valid |
| M1_RIGID | FAIL: goal | 0.327621052 | 7.969731 | invalid diagnostic rollout |

The new hard G3 references have smaller terminal position error than these
historical records, but still fail the unchanged success predicate. This is not
a new paired RAW/GP causal comparison or navigation improvement. No baseline
was rerun; historical wall times are not paired compute comparisons.

## Computation and artifact verification

Measured preparation is 8.577 s, including source rehashing and 1.178 s of five
plan rechecks (nested, not additional). Historical-audit worker phase is 0.102 s.
The primary worker phase is 0.794 s; its five rollout durations sum to 0.529 s,
including 0.492 s of official MPC solve time. These are nested timings and are
not summed as independent costs. Evaluation stage is 0.328 s; original full
execution-check calls sum to 0.162 s. Process startup/import and report/validation
costs are separate from these measured worker phases. None advances simulation.

| Reference | Rollout wall s | Official MPC solve total s | Full execution evaluation s |
|---|---:|---:|---:|
| Hard M2/I0 | 0.139545 | 0.131007 | 0.038141 |
| Hard M2/I1 | 0.102514 | 0.095124 | 0.030302 |
| Hard M3/I0 | 0.101430 | 0.093972 | 0.030118 |
| Hard M3/I1 | 0.100127 | 0.092689 | 0.030565 |
| Benign M3/I1 | 0.085260 | 0.078913 | 0.033158 |

The frozen saved-record validator independently recomputed all scientific
plan/execution results and record checks. Its only error was
`artifact hash plot_console.log`: the plotter inventoried its redirected stdout
while empty and then wrote its final summary. The initial failed validator,
original hash inventory and complete log are preserved unchanged. This is a
report metadata error, not a failed execution or altered acceptance.

The separate `finalize_gp_se2_diag07_review.py` accepts only that exact log-prefix
case, rejects any other scientific/source error, rechecks all five full plans,
all five executions/integration/selection streams, every numeric plot sidecar
and ZIP member, then writes the authoritative result:

`verification/validation.json`: **valid=true, errors=[]**.

`verification/reporting_completion.json` records the empty-prefix/final-log
hashes and the exact supersession scope; `verification/output_hashes.json` binds
the completed logs and all outputs. The original `validation.json` is not the
final authority. Validation performs zero new MPC/GP/rollout operations.
The first validator took 10.478 s and final independent completion check 9.615 s.

Initial five-row multi-panel images had overlapping status/title text. A separate
per-reference presentation preserves all original files and uses identical
numeric data, clearer labels and goal-region circles. All 50 new readable PNGs
have independently checked numeric/hash sidecars. Representative world and goal
plots were visually inspected at original image resolution. No GUI was run.

Preferred evidence:

- `presentation/index.html`: all five conditions and all 50 readable figures.
- `review_bundle_readable.zip`: self-contained index, 50 figures, sidecars,
  result tables, source/config/reference identity and limitations (7,042,036 B).
- `aggregate/plan_vs_execution.csv`: distinct plan and execution rows.
- `aggregate/execution_outcomes.csv`, `command_metrics.csv`, `selection_metrics.csv`.
- `rollouts/ref_00` through `ref_04`: complete actual commands, states,
  selections, predictions, original evaluation and diagnostic geometry.
- `verification/validation.json`: final saved-record authority.

Reporting-only commands, after the unchanged once-only execution:

```bash
MPLCONFIGDIR=/tmp/gp_diag07_mpl .venv/bin/python scripts/present_gp_se2_diag07.py --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
MPLCONFIGDIR=/tmp/gp_diag07_mpl .venv/bin/python scripts/finalize_gp_se2_diag07_review.py --run data/robotless_gp_se2_diag_07/primary_20260920T103000Z
```

## Interpretation and next single uncertainty

**No execution-success counterexample to the strict lateral criterion was
observed.** All four unique lateral-invalid hard references also failed execution
at original goal position/dwell, while the plan-valid benign control succeeded.
This does **not** show that nonzero planned v_y caused failure or that the strict
plan criterion is necessary. It supplies no permission to relax the 1e-5 lateral
threshold. Hard plans remain invalid and unavailable for deployment.

The next single uncertainty is whether sufficient **plan endpoint margin inside
the unchanged original goal region**, with this fixed MPC/selector and execution
predicate, changes this hard goal/dwell failure. A later bounded comparison can
vary only that planning margin and preserve the original lateral acceptance;
any remaining lateral-invalid references must remain explicitly diagnostic.
This is a proposal, not an implemented constraint, new solve or horizon extension.
It would address the measured 1.685 cm plan-goal margin versus roughly 3.846 cm
final tracking displacement before claiming that lateral relaxation helps.
The hard/soft/structural lateral-design choice remains unresolved.

## Final repository checks and complete runtime accounting

Full required command: **2,350 passed, 19 existing skips in 163.35 s**.
The first sandbox full run had two existing Unix-domain IPC permission failures
(2,345 passed, 19 skipped); the required command was repeated with local IPC
permission, without changing code or experimental results. All 32 new DIAG-07
implementation/reporting tests pass. `compileall src scripts tests` and
`git diff --check` pass. No shell launcher changed. Existing skips concern absent
historical ignored corpora and one declared representation fixture.

Complete actual-MPC accounting includes a pre-existing integration test that
calls the official historical audit for `episode_008_repeat_01/handoff_013`.
It ran once in the 56-test relevant set, once in the first full suite and once
in the IPC-enabled full suite. These are **three additional test-only historical
solves**, no counterfactuals and no new experimental conditions. They are not
included in the frozen experiment's 152 solves. Therefore the task-wide actual
MPC call count is **155 = 150 primary + 2 protocol historical audits + 3 existing
regression-test historical audits**. All three test invocations passed; pytest
retention removed the first temporary audit directory, while the two full-suite
audit records are preserved separately with their test provenance. No new audit
was run to replace that temporary test artifact. The primary five reference
records/rollouts and two protocol audit records are complete.

The frozen `aggregate/summary.json` runtime total of 152 is the experimental
schedule, not this task-wide total including legacy runtime tests. Final
completion metadata preserves both counts, final/report SHA, normal push status
and test/source hashes. Code/tests/docs only are committed; all generated data,
figures and ZIPs stay ignored, with the two unrelated config edits untouched.
