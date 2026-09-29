# OSA03_RELATIVE_FACTOR_ABLATION_01

## Frozen protocol

**Timing-controlled offline causal reference comparison.** One sealed OSA03
REPEAT_00 source; one new NO_RELATIVE planning solve and one rollout per eligible
reference. This is not an online latency benchmark or real asynchronous deployment
timing. No source selection, new LightNav, RGB or Isaac calls, tuning or retries.

Fetched `origin/main` and starting HEAD:
`53af3488809063a8fddf16a9bb9f7f35bd19aa94`. Read the current repository instructions,
README, work log, formulation/common-B/Native/trackability reports and implementations.
The two unrelated Stage 0 config edits are preserved and excluded from commits.
The historical common-B saved-only validator passed before implementation edits.

Run: `data/osa03_relative_factor_ablation_01/primary_20260929T050000Z/`.
The directory identifier is fixed; actual UTC/host clocks are recorded separately.
Raw inputs stay in their sealed directories. All new arrays/rollouts are derived,
ignored data. Small source/config/code hash ledgers and summaries are tracked under
`results/osa03_relative_factor_ablation_01/`.

### Formulation and frames

A and B are fixed. The state-shift factor moves editable early FRESH nodes X_j
toward the transported target Ftilde_j = B A^{-1} F_j, equivalently encouraging
B^{-1} X_j ≈ A^{-1} F_j.

The historical L factor is described here as the **observation-to-application
state-shift / transport factor**, denoted S in mathematical discussion. Historical
code keys L are retained. `r_S,j = Log(Ftilde_j^{-1} X_j)`; relative and anchor
residuals remain exactly the existing implementation:

```
r_R,j = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A,j = Log(F_j^-1 X_j)
s_j = cumulative original-FRESH XY arc / total original-FRESH arc
w_S = (1-s)^2; w_A = s^2
normalization = diag(0.10 m, 0.10 m, 10 degrees)
```

S/A use normalized weighted block means; R uses the edge mean. Initialization is
exact original FRESH. No waypoint timestamps/dt, twist, GP, correspondence,
controller/command factor, obstacle cost or replacement smoothness term.
World XY is metres, yaw radians CCW, Z up; body x forward/y left.
`F_j = A F_local,j` remains observation-anchored. A-local derived representations
use `A^-1 X`; raw FRESH is never anchored at B.

| Fixed order | Reference | Planning calls |
|---|---|---:|
| M0_NATIVE | Original FRESH F | 0 |
| M1_TAPER | Existing `Exp((1-s) Log(B A^-1)) F` | 0 |
| FULL_LOCAL_SE2 | Exact authenticated frozen optimized world array | 0 |
| NO_RELATIVE | Optimize E_S + E_A; remove R from residual vector/cost | 1 |

The default LocalSE2 API still optimizes E_S+E_R+E_A. Only the explicit
`include_relative=False` option omits R. Raw R residuals remain available for
**diagnostic relative-edge distortion, not optimized cost**. No weight or solver
setting changes. Frozen Full is not reconstructed or re-optimized.

Unchanged LM: 80 iterations, central finite difference 1e-6, right-local SE(2)
updates, initial damping .001, rejection ×10, acceptance ×.3, maximum damping
1e12, gradient/step tolerances 1e-9, cost tolerance 1e-12. Same complete-polyline
candidate acceptance callback; only improving feasible steps may be accepted.
An optimizer exception records the trace and stops scientific execution; no fallback.

### Authenticated inputs and logical schedule

Source: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`, REPEAT_00.
Raw FRESH SHA256 `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`.
Frozen Full world SHA256 `bd459b0343fac1e78e4ac80831f4f99af9b86dd4876a3b0df1b45f41c21ce120`.
Official MPC SHA256 `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
The existing clean pinned external checkout and environment are used read-only.

The exact schedule is loaded from the previous common-B M0 rollout, authenticated
against its sealed result ledger and the tracked result summary. It is not generated
from nominal frequency or reconstructed from prose. The entire 180-interval schedule
is reused, including the primary 54 intervals (~.9 s). Full hash values, each pair,
and source paths are in the tracked freeze summary/config.

- B state/tick92, simulation time1.5666667483747005 s; original source dt
  `float32(1/60)=0.01666666753590107` s.
- Shared already-applied u_B_plus and advanced controller memory are restored
  exactly from saved solve000005; physical u_minus is recorded separately.
- There is no solve at B and no replay of future saved solve000006.
- Primary attempted/accepted submits: `[96,102,108,114,120,126,132,138,144]`.
- Their predetermined applications: `[99,103,109,115,121,127,133,139,145]`.
- Initial application92 belongs to the common saved first command.

Each independent tracker is initialized through the existing official observation-A
transform before its clock starts, with the saved generation/command/memory/reference/
prediction fields. Official MPC selection, H=5, objective, limits, initial guesses,
solve and poll behavior remain unchanged. No warmup solves.

At each authenticated submit tick, the research wrapper submits that tick's current
state and pauses the simulation while the unchanged worker finishes and polls.
The result waits in a release queue until its predetermined application tick;
previous commands are integrated/held in the meantime. No future state is used.
The official poll updates controller memory; every frozen release is strictly before
the next submit, so no new solve can observe an alternative memory ordering.
Generation/chunk/version rejection is preserved. Actual worker receipt and logical
release/application are separate records. Slow wall time cannot advance simulation.
A wall wait timeout (30 s) is a technical termination, never an invented command.

Before interpreting differences, the gate requires the identical attempted submits,
accepted submits, successful applications, all 54 integration intervals, and exact
initial state/command/memory provenance. Failure yields `SCHEDULE_CONTROL_FAILURE`;
no reference-causal interpretation is then permitted. Unsafe references are excluded
with explicit null metrics; the gate records whether all four were observed.

### Safety and evaluation

All complete references use the existing direct Hospital+cart geometry checker,
.20 m circular footprint, .05 m required footprint-edge clearance and unchanged
workspace/numerical uncertainty conventions. No B-to-first-row connector is added.
Independent full-union distance checks precede rollouts. Unsafe references are
recorded and skipped without repair. The existing abort-only execution guard
checks each new command's .1 s hold and each held command's next integration step.
An unsafe proposal is not applied; preserve the valid prefix and censor the rollout.

Primary evaluation is always against ORIGINAL FRESH with unchanged forward-only
continuous projection, shortest-angle yaw, and <=.10 m/15 degree tube over the
complete following .30 s sampled dwell. Missing attachment stays null. Report
initial errors, .5 s maximum/growth, .3/.9 s position/yaw AUC, attachment time/
original progress/remaining arc, clearance, endpoint, command TV/max omega and
termination. Secondary own-reference AUC/errors are separate, never substituted.
No scalar score or post-result threshold for superiority.

Planning diagnostics include node corrections, arc/segments, node-local lateral
correction and B-frame lateral extent (definitions explicit), yaw/relative edges,
closed-form rigid fit, full safety, B-to-row0 and continuous projection geometry,
first chord of at least .02 m relative to B heading, and Shapely self-intersection.
These are spatial pose deformations. The official MPC decides actual omega(t).

Nine figures with numeric/hash sidecars: references/world execution (equal axes,
cart+center exclusion boundary, A/B, attachment and minimum-clearance markers),
original-FRESH position/yaw/progress, clearance, commands, schedule, node/edge
distortion. Distinct markers/styles and numeric separation describe overlapping curves.

### Freeze and validation discipline

Preparation and zero-numerical-solve external restoration preflight passed. The
preflight tests the three already available references; the new reference uses
exactly the same generic installation/restoration path after its sole planning solve.
Synthetic tests cover the fourth method and are not experimental evidence.

Pre-execution regression: **384 passed, 1 skipped** (absent ignored historical
EXP-01B/EXP-02B corpus). This includes 19 new tests, historical Local-SE2/common-B,
SE(2), controller/generation, timing, guard, attachment and geometry tests. The
pre-edit Full synthetic residual/cost/solution golden is bit-exact. No new scientific
solve was used to test default preservation. Source authentication includes the
sealed R00 geometry and official source hashes; no R01 scientific evaluation.

Commit and normal push of implementation/config/tests/protocol/freeze ledger must
precede the one execution. The runner verifies live remote HEAD and committed file
bytes, and creates exclusive one-shot execution/optimization/method markers.
Validators reconstruct saved equations, retractions, safety, controller selection,
memory, integration, schedule and both metric targets without scientific solves.
Results, figures, interpretation and call counts are appended after execution.
