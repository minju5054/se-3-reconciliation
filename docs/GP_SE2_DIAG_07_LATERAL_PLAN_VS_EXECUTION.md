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
