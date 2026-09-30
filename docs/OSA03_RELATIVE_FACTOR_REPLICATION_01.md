# OSA03_RELATIVE_FACTOR_REPLICATION_01

## Frozen protocol

**Timing-controlled offline replication** on sealed OSA03 **REPEAT_01 only**.
This is a **paired same-scenario replication**, also described as a technical
replication across OSA03 repetitions. It is not an online latency benchmark.
Starting fetched HEAD/origin/main: `0cc11dfb4d2048c758fc735509c2e8297124283b`.
The prior reports, implementation, tracked results and repository instructions were
inspected before edits. The two unrelated Stage 0 config edits remain untouched.

Run: `data/osa03_relative_factor_replication_01/primary_20260930T010000Z/`.
This fixed directory identifier is not an execution timestamp; records retain
actual UTC, host monotonic, and logical simulation clocks separately. All large
arrays, rollouts and figures stay under ignored data. Raw source files stay sealed.
Small provenance ledgers and numeric results are tracked under the same namespace.

### Source and scope

Only `data/obstacle_source_acquisition_03/primary_20260923T085200Z/` / REPEAT_01
supplies the scientific state and trajectory. The tracked acquisition summary pins
its saved validation digest; that validation pins every R01 episode file. The
sealed R01 bundle ledger, episode/raw/world parity, complete source qualification,
Hospital/cart geometry and official MPC source are authenticated before execution.
R00 artifacts supply the fixed schedule and previously measured contrasts only.
No R00 optimization or rollout is repeated. Its existing saved-only validator must
pass without modification, and all its result bytes must remain unchanged.

Independent acquisition sessions have different triggering RGB, observation poses,
world FRESH and controller states. **R00 and R01 have identical raw local FRESH
arrays**, and identical raw local OLD arrays. Therefore this experiment cannot
establish trajectory diversity, independent policy-output replication, population
replication or generalization across Navigation VLA trajectories.

Frames: world XY metres, +Z up, yaw radians CCW; observation-local x forward/y left.
`F_j = A F_local,j` is the original observation-anchored world FRESH. Raw arrays are
immutable and never re-anchored at B. Derived local references use `A^-1 X_j`.

### Formulation and solver

A and B are fixed. The **observation-to-application state-shift / transport factor**
moves editable early FRESH nodes X_j toward the transported target
`Ftilde_j = B A^-1 F_j`, equivalently encouraging `B^-1 X_j ≈ A^-1 F_j`.
Historical L keys are retained and mean S in this report.

```
r_S,j = Log(Ftilde_j^-1 X_j)
r_R,j = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A,j = Log(F_j^-1 X_j)
s_j = cumulative original-FRESH XY arc / total original-FRESH arc
w_S = (1-s)^2; w_A = s^2
normalization = diag(.10 m, .10 m, 10 degrees)
```

Weighted means/edge means remain exactly as implemented. Fixed rollout order:

| Condition | Reference | New R01 planning calls |
|---|---|---:|
| M0_NATIVE | Original FRESH | 0 |
| M1_TAPER | Existing `Exp((1-s) Log(B A^-1)) F` | 0 |
| FULL_LOCAL_SE2 | E_S + E_R + E_A | 1 |
| NO_RELATIVE | E_S + E_A | 1 |

Both planning conditions initialize exactly at original R01 FRESH, with the same
LocalSE2Problem, scales, weights, right-local SE(2) LM and safety callback. No R00
optimized reference enters either initialization. Default Full behavior remains
unchanged. No-relative removes R from the optimized vector and cost; its final old
R residual is **diagnostic relative-edge distortion, not optimized cost**.

Solver: maximum 80 iterations; central finite difference 1e-6; initial damping
1e-3; rejection ×10 / acceptance ×.3; maximum damping 1e12; gradient and step
tolerances 1e-9; cost tolerance 1e-12. Update `X_j <- X_j Exp(delta_j)`.
One call per condition, with exclusive markers; no restart, selected retry,
alternate initialization, weight/scale/lambda search or result-driven changes.
Nonconvergence/error is recorded and the condition is skipped, without repair.

Complete candidate polylines use the existing direct Hospital + cart checker:
circular radius .20 m, required footprint-edge clearance .05 m, unchanged workspace
and numerical/uncertainty conventions. Initial state must be feasible; a candidate
is accepted only when objective decreases and the complete candidate is feasible.
No B-to-first-waypoint connector, clipping, projection, cropping or obstacle cost.
An independently checked unsafe complete reference is recorded and not executed.

### Schedule and controller semantics

The exact full 180-step R00 logical schedule is authenticated through its tracked
result digest and original common-B M0 events; no schedule is synthesized from
rounded prose or nominal frequency. R01 compatibility checks B tick/time, dt,
next legal submit, original grid, version semantics, generation validity, memory,
settings and release-before-next-submit ordering. Any incompatibility blocks
scientific execution as `TECHNICAL_BLOCKED`.

Authenticated preparation facts:

```
A = [19.203260368199164, 24.346668368545373, -1.5689754090345875]
B = [19.20364874270168, 24.133335375688848, -1.5689760469373495]
u_minus = [0.8, -1.2790285101476235e-06]
u_B_plus = u_mem_B = [0.8, 0.4999987283797167]
B tick = 92; B simulation time = 1.5666667483747005 s
dt = 0.01666666753590107 s (original float32 1/60)
R01 generation = 6; fresh reference version = 1
first saved FRESH result = REPEAT_01_solve_000005
```

R00's generation is 3. Generation counters need not numerically match across
independent acquisitions; R01's exact generation 6 is restored for all four methods,
and each result must match it. The official zero-numerical-solve installation and
restoration preflight passes with generation 6. No R00 B/memory is substituted.
The initial development compatibility check unnecessarily required generation
counter equality across repetitions; this was corrected before freeze and before
any scientific call. The source state and controller semantics were unchanged.

Primary submits `[96,102,108,114,120,126,132,138,144]` and releases
`[99,103,109,115,121,127,133,139,145]` are loaded from authenticated files. The
application at tick92 is the shared saved R01 command; there is no new solve at B
and no replay of the future saved R01 result at tick96. Full schedule is in the
freeze ledger. All methods use identical common state, command, memory and clock.

The unchanged R00 execution function is reused directly. It installs each reference
through the official observation-A transform before the logical clock, restores
R01 generation/command/memory/diagnostic fields, and uses the unchanged official
worker, H=5 nearest/+1 selector, MPC objective, limits and command computation.
Its inherited per-rollout `label` names the causal-reference-comparison wrapper;
this experiment's protocol, summary and plots label the R01 study as an offline
replication. No historical implementation file is edited.

At each submit tick the simulation waits for the official solve/poll while state
and simulation clock remain fixed. The result is withheld until its predetermined
logical tick. No future state enters a solve. The previous command remains held.
Official poll updates memory before the next solve; every scheduled release
precedes the next submit, preserving the existing ordering. Stale generation,
controller failure, missed release or timeout fails closed. Wall time does not
choose the application tick; this is not real asynchronous deployment timing.

The primary causal gate requires identical attempted/accepted submits, successful
applications, all 54 integration intervals (~.9 s), initial B/held command and
controller-memory provenance. Report full 180-step parity as an additional check.
A schedule-control failure prevents reference-causal attribution and is
`TECHNICAL_BLOCKED`. Missing/censored conditions remain explicit, with no invented
attachment or continuation.

Execution keeps the existing abort-only guard: new command .1 s preview and held
command next-dt check. An unsafe proposal is never applied; retain the valid prefix.
No steering, repair or interpolated commands. Cap 180 intervals, no later FRESH.

### Evaluation and diagnostics

Primary evaluation targets **ORIGINAL R01 FRESH**, with the unchanged continuous
forward-only projection and shortest-angle yaw. Attachment requires <=.10 m and
<=15 degrees through the complete following .30 s sampled dwell. Null attachment
stays null. Report initial errors, max/growth first .5 s, position/yaw AUC .3/.9 s,
attachment time/fractional row/fractional original arc/remaining arc, swept clearance,
endpoint error, linear/angular command TV, max abs omega and termination.
Own-reference AUC .3/.9 and max .5 s error are separate secondary diagnostics.
No optimized-objective total is treated as an execution-performance score.

Planning diagnostics retain node/endpoint/per-node shifts, arc, segment lengths,
self-intersection, original-node-frame lateral correction, B-frame lateral extent,
yaw, relative-edge translation/yaw RMS/max, rigid fit, complete clearance, B-to-row0
and continuous projection distance/yaw, first >=.02 m chord versus B heading.
Both solves report costs, iterations, accepted/rejected/unsafe steps and termination.
Spatial yaw deformation is distinct from actual omega(t), chosen by official MPC.

Full and No-relative selector diagnostics record every solve's input pose, nearest
row, selected H=5 indices, produced command and logical application tick. They are
read from saved events after execution and cannot influence the controller. Report
the first differing selected-row tick, or explicitly report identical sequences.

Paired table uses signed **No-relative minus Full** for 13 metrics: max position
.5; position AUC .3/.9; yaw AUC .3/.9; attachment; remaining original arc; swept
clearance; endpoint; linear/angular TV; relative-edge translation RMS/max. Report
both repetitions and sign consistency; nulls remain null. No scalar score.

Predeclared categories, without outcome-driven thresholds:

- `REPLICATED_WITHIN_PAIRED_OSA03`: earlier attachment, nonpositive deltas for all
  three early position metrics, all references/executions safe through their caps,
  and higher relative-edge translation RMS/max without R.
- `PARTIAL_REPLICATION`: earlier attachment or consistent early position benefit,
  with the other required directions mixed or unavailable.
- `NOT_REPLICATED`: the main benefit reverses/disappears; also record scientifically
  observed planning/reference failure without relabelling it a technical failure.
- `TECHNICAL_BLOCKED`: source, schedule, controller or validation failure only;
  no reference-causal interpretation.

All exact signed metrics and safety outcomes remain visible regardless of category.

### Tests, freeze and outputs

Synthetic tests are mechanism checks, never experimental evidence. Required source,
frame, factor/default, solver/init/safety, immutable raw, schedule/release/stall,
memory/generation, abort, primary/secondary and selector checks precede execution.
The old R00 validator and Local-SE2/common-B regression tests must pass unchanged.
Compileall and diff review precede commit and normal push. The scientific runner
verifies remote HEAD and committed code bytes, then creates exclusive markers.
Exactly two planning calls and at most four rollouts are authorized after push.

Ten saved-only figures: R01 references/world execution (equal axes; cart + .25 m
center exclusion boundary, A/B, attachment/minimum-clearance markers), original
position/yaw/progress, clearance, commands, logical timeline, deformation and
paired signed-effect table. Distinct styles/markers and numerical maximum
separations describe overlapping curves. Each has a numeric and hash sidecar.
The local HTML review links all figures and is derived solely from saved records.

No new LightNav, RGB, Isaac episode, source search, external source/environment
change, MPC modification, new factor, time parameterization, GP, correspondence,
new margin, tuning, repeated online chunks or broader research stage.

### Exact commands

```bash
git fetch origin main
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode prepare --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode preflight --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLCONFIGDIR=/tmp/osa03-replication-mpl .venv/bin/python -m pytest -q tests/test_osa03_relative_factor_replication01.py
# Full regression command and counts are recorded in WORK_LOG.md before freeze.
.venv/bin/python -m compileall -q src scripts tests
git diff --check
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode freeze --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
# Review staged diff, focused commit and normal push BEFORE either solve or rollout.
.venv/bin/python scripts/run_osa03_relative_factor_replication01.py --mode execute --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
.venv/bin/python scripts/validate_osa03_relative_factor_replication01.py --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
MPLCONFIGDIR=/tmp/osa03-replication-mpl .venv/bin/python scripts/report_osa03_relative_factor_replication01.py --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
MPLCONFIGDIR=/tmp/osa03-replication-mpl .venv/bin/python scripts/report_osa03_relative_factor_replication01.py --validate-only --run data/osa03_relative_factor_replication_01/primary_20260930T010000Z
```

Results and the nine explicit research answers will be appended after the single
frozen execution; none are inferred from synthetic fixtures or preparation.
