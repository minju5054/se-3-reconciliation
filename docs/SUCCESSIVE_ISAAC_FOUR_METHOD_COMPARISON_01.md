# SUCCESSIVE_ISAAC_FOUR_METHOD_COMPARISON_01

**Result: FOUR_METHOD_MIXED_EVIDENCE.** RAW parity is exact and all four Isaac
rollouts are safe. RAW/B_ENTRY coincide; Hermite reduces error AUC with higher
angular TV; Graph gives no added tracking improvement in this isolated window.

## Frozen protocol

One DEVELOPMENT SOURCE, one C0→C1 obstacle-reveal handoff. This is a
**timing-controlled offline causal reference comparison in actual Isaac Sim**.
The agent is the existing logical SE(2) USD Xform, with real World physics steps
and USD pose readback. No wheel, actuator or physical robot dynamics are added.
This is not a real asynchronous latency benchmark.

Starting HEAD and fetched origin/main:
`d4831fa94474b1d138ca2f1eae3fce0a16c4d8cd`.
Source: `data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01`,
`source_bundle/handoffs/C0_to_C1`.
Source scientific freeze: `ee330244d867fad3749c60b6a5f1630b1471ec2a`;
source result: `e4f61465c13996cc8c5c70504a5735dabfa6e944`.
The previous blocked replay audit remains unchanged. Its full C1 physical-command
window included C2 installation at tick120. The user explicitly authorizes this
new, predeclared method-isolation horizon:

- integration intervals **[92,120)**: ticks92–119, 28 steps;
- states92–120, with no event at120 processed;
- saved dt = 0.01666666753590107 s; duration = 0.46666669100522995 s;
- submit ticks96/102/108/114, release/application ticks97/103/109/115;
- no C2 installation, generation change or submit in the horizon.

The input loader derives these from authenticated events, then asserts them against
the config. It does not construct the schedule from remembered timing.

## Pre-outcome repository-confirmed facts

The preparation authenticates the source's 401-file final seal, 297 episode input
files, C0/C1 raw and world arrays, scene/cart provenance and exact boundary states.
All values below are loaded from saved artifacts; runtime code does not copy them.

| Item | Saved value |
|---|---|
| A1 | [19.201407937708954, 25.363333351859286, -1.5689749934176171] |
| B1 | [19.20161431701553, 25.250000200523026, -1.5689755606609315] |
| P1 | [19.201602178508843, 25.25666685648663, -1.5689755391010196] |
| physical u_minus | [0.4, -1.2935946487914903e-6] |
| already-applied u_B_plus / previous_control | [0.4, -0.04625916114685449] |
| B simulation time | 1.5666667483747005 s |
| official generation / reference version | 3 / 1 |

The unchanged frozen C3 rule chooses **E*=F0**, segment0, beta0, original arc0 m:
`[19.201732994894844, 25.212761781627275, -1.5676130291471446]`.
There is **no removed stale prefix** in this source. B→E* distance is
0.03723860800693844 m; original remaining arc is 1.3574525636380002 m.
Hermite's existing spatial-density rule gives **M=2**. The Graph input S has ten
poses and eight editable internal poses. These facts precede scientific solves.

World XY uses metres, +Z up, yaw radians CCW. Observation-local +x is forward,
+y left. Original FRESH is `T_W_F = T_W_A T_A_F`. Derived adapter representations
are `T_A_X = inverse(T_W_A) T_W_X`. A and B are fixed. Raw FRESH is immutable and
never re-anchored at B. Rows have no intrinsic time or waypoint dt.

## Four frozen methods

1. **RAW**: original C1 raw local bytes and world installation.
2. **B_ENTRY**: existing `suffix_reference` plus
   `boundary_row_ablation04.staged_reference`; [B,E*,original suffix], preserving
   the existing duplicate convention and exact downstream local/world rows.
3. **HERMITE**: unchanged `b_to_entry_bridge.hermite_bridge`. Incoming tangent
   normalize(B−P), outgoing tangent normalize(F_next−E*), both derivative magnitudes
   equal ||E*−B||. Equal arc sampling with existing spatial row scale, shortest yaw
   interpolation. [B,bridge interior,E*,exact original suffix].
4. **GRAPH**: progress-aligned S=[E*,original suffix]. X0=B and Xlast=Slast are
   copied exactly; only X[1:-1] enters the existing right-local LM solver. E* defines
   S0; it is not an extra fixed Graph node after B.

### Graph objective and initialization

All lambda values are 1. Translation normalization is (0.10,0.10) m and yaw
normalization 10 degrees; transition direction normalization is 15 degrees.

- T: wrap(direction(B→X1) − direction(P→B)) / 15 degrees.
- R: Log(inverse(S_j^-1 S_(j+1)) (X_j^-1 X_(j+1))) / scales, every suffix edge.
- A: s_j Log(S_j^-1 X_j) / scales, editable internal nodes only. Thus squared
  cost weight is s_j², where s is normalized original suffix XY arc.
- Total cost is the sum of squared normalized T/R/A residuals.

Initialization is Exp((1−s_j) Log(B S0^-1)) S_j, followed by exact B/end copies.
This deterministic initialization is not a fifth scientific method.
No P→B distance matching, controller, speed, acceleration, waypoint time,
obstacle cost, GP or canonical full-FRESH BA^-1 transport factor is added.

`SolverConfig` defaults are frozen: max80 iterations, central difference1e-6,
initial damping1e-3, increase10, decrease0.3, max damping1e12, gradient/step
1e-9, cost1e-12. An improving proposal must be finite, have all XY edges >1e-12 m
and pass the source's complete reference checker. Returned Graph requires solver
convergence and those same checks. No last-iterate fallback, repair or retry.
A nonconverged/invalid Graph is GRAPH_PLANNING_FAILED and prevents Isaac execution.

## Restoration, controller and safety gates

All methods share saved B, simulation time, physical u_minus provenance, already
applied u_B_plus, previous_control, generation/version, dt and exact visible cart.
The official H=5 nearest+1 selector, objective, Q/R, acceleration limits and native
0.4 m/s source intervention are unchanged. Official MPC SHA256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Preflight constructs real official controller objects and installs references with
numerical solve calls forbidden; source settings and memory restoration must match.

Prefer exactly one SimulationApp. Before each method: World.reset, exact B pose
write, source cart visibility/transform assertion, and a fresh official controller
worker with no queued results. Saved simulation time is restored using bounded
zero-motion World physics steps before the comparison clock starts. Those steps
are reported separately and are not comparison intervals or fabricated timestamps.
No rendering/RGB read is requested during these steps or the comparison. Existing
scene construction may initialize render products; no new model image is captured.

Each interval uses the existing guard, unicycle logical agent integration, precise
USD write, `World.step(render=False)`, clock check and actual USD readback. A solve
uses its actual submit-tick state. Simulation waits while the official MPC solves;
LogicalRelease holds the genuine result until its frozen application tick. A
snapshot confirms that wall-clock waiting did not advance the pose or World time.
No interpolation, future state, modified controller command or fake result.

Safety follows the **current successive source**: circular radius0.20 m, physical
footprint/uncertainty/workspace conventions, with legacy0.05 m reported separately.
All four complete references must pass before Isaac. Derived B-to-entry bridge
segments are part of those references. Unsafe improving LM proposals are rejected.
During execution an unsafe proposed command remains unapplied; the valid prefix
is retained and its full-window metrics become N/A if incomplete.

RAW executes first and must match the saved original prefix: exact initial B,
per-tick pose/command, submit/application sequence, observable controller memory,
generation/reference identity and guard decisions/clearance. Reuse existing absolute
pose/command tolerance1e-10 and selection tolerance1e-12, relative tolerance0.
Material RAW mismatch stops the remaining methods with RAW_ISAAC_REPLAY_PARITY_FAIL.
Infrastructure failure is TECHNICAL_BLOCKED. No automatic retries.

## Metrics, interpretation and call budget

Primary target is immutable original C1: existing continuous forward-only XY
projection and shortest-angle yaw. Trapezoidal error AUC at exactly0.3 s (linear
interpolation at that boundary), plus full28-step duration. Incomplete windows are
null. Endpoint dwell is not a primary metric for this short window.

Report first NEW applied command and absolute changes from common u_B_plus;
post-B consecutive-command TV, max|v|/|omega|; complete reference and executed swept
clearance, overlap and abort; original row identity; initial reference gap/tangent
mismatch, row count and XY arc. Graph diagnostics include normalized T/R/A costs,
relative edge translation/yaw distortion, per-node anchor displacement and exact
zero endpoint deviation. Signed Graph-minus-RAW/B_ENTRY/Hermite differences are
reported separately. Shared u_B_plus is not a method-specific improvement.

There is no applicable frozen material-effect threshold for declaring a winner.
A valid full comparison uses FOUR_METHOD_MIXED_EVIDENCE and reports numeric signed
differences without adding a post-outcome threshold or scalar winner score.

Maximum science: one handoff, **one Graph solve**, one RAW/B_ENTRY/HERMITE/GRAPH
rollout each, **one SimulationApp launch**, 16 official MPC solves. LightNav calls,
RGB/model requests, new source acquisition and retries are all zero. Synthetic
fixtures and their numerical solves are tests only, never experimental evidence.
All scientific execution starts after pushed freeze. Saved-only validation and
reporting cannot solve or launch. Large traces/logs stay in ignored `data/`.

## Commands

Use the existing repository venv; no system/external environment changes.

```bash
RUN=data/successive_isaac_four_method_comparison_01/primary_20261009
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/four-method-mpl
.venv/bin/python scripts/run_successive_isaac_four_method01.py --run "$RUN" --mode prepare
.venv/bin/python -m pytest -q tests/test_successive_isaac_four_method01.py
.venv/bin/python -m pytest -q tests/test_successive*.py tests/test_long_continuous_obstacle_reveal_source01.py tests/test_continuous_obstacle*.py tests/test_online_mpc_adapter.py tests/test_osa03_common_b.py tests/test_osa03_relative_factor_ablation01.py tests/test_robotless*.py tests/test_join_online02*.py tests/test_se2*.py tests/test_b_to_entry*.py tests/test_spatial*.py tests/test_local_se2*.py tests/test_canonical_se2_graph.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
.venv/bin/python scripts/run_successive_isaac_four_method01.py --run "$RUN" --mode freeze
# Review/stage exact namespace + work log, commit and normal push before execute.
.venv/bin/python scripts/run_successive_isaac_four_method01.py --run "$RUN" --mode execute
.venv/bin/python scripts/validate_successive_isaac_four_method01.py --run "$RUN" --write
.venv/bin/python scripts/report_successive_isaac_four_method01.py --run "$RUN"
.venv/bin/python scripts/validate_successive_isaac_four_method01.py --run "$RUN"
.venv/bin/python scripts/report_successive_isaac_four_method01.py --run "$RUN" --check
```

Final PNGs only: `four_method_world_execution.png`, `transition_metrics.png`, under
`results/successive_isaac_four_method_comparison_01/figures/`. Inspect both images
and numeric sidecars. Unexecuted methods remain N/A; no fabricated paths.

## Claim boundary

Only this frozen C0→C1 handoff, identical scene/state/controller/logical schedule.
No general graph superiority, VLA improvement, navigation success, obstacle
avoidance superiority, real-world result or population claim. This task stops after
saved-only reporting and normal result push; no automatic multi-handoff experiment.

## Repository-confirmed results

Scientific freeze **7640c1126e24b69837dfcd6f97471a82c24eb26a** was normally pushed
before the single Graph solve and SimulationApp launch. All four complete
references passed; all four real Isaac rollouts completed the exact 28 intervals.
The saved-only validator reports `valid=true`, `schedule_comparable=true` and
`RAW_parity_pass=true`. No retry or post-freeze code/config change occurred.

### Actual Isaac parity, reset and schedule

RAW initial B is bit-exact. Maximum per-tick pose, command, observable memory and
guard-clearance errors against the original saved prefix are **0.0**. Guard
decisions, active reference identity/version, generation, attempted/accepted
submissions and physical new applications all match. This is new official MPC
execution with real World stepping and USD readback, not saved-command playback.

Each method has 29 actual states and 28 applied command intervals, four accepted
MPC submissions at 96/102/108/114 and four new applications at 97/103/109/115.
Each controller starts with identical memory/held command and empty future state.
All 16 solver waits leave Isaac pose/time unchanged; no C2 event is processed.
World.reset leaves time 0.03333333507180214 s; **92 additional zero-motion physics
steps per method** restore saved B time 1.5666667483747005 s, World step index 94.
These are pre-clock initialization steps, separate from 112 comparison intervals
across four rollouts. Exact B/cart checks pass after every reset. The one Isaac
process exits normally with return code 0.

### Transition metrics against original C1

All methods start with position error 0.03723860800693844 m and yaw error
0.0013625315137870686 rad. Full window is 0.46666669100522995 s. Lower error AUC
means less integrated geometric tracking error in this window.

| Method | Position AUC .3 (m s) | Position AUC full (m s) | Yaw AUC .3 (rad s) | Yaw AUC full (rad s) |
|---|---:|---:|---:|---:|
| RAW | .001834051770 | .002032905841 | .002246395122 | .004767249882 |
| B_ENTRY | .001834051770 | .002032905841 | .002246395122 | .004767249882 |
| HERMITE | .001814851697 | .001968172763 | .001723670764 | .003959796763 |
| GRAPH | .001834955860 | .002060661955 | .002284548170 | .005642477234 |

### Commands and execution safety

First NEW command applies at tick 97 for every method. Common u_B_plus is
[.4, −.04625916114685449]; v remains exactly .4 throughout every rollout.
Thus first |delta v| and linear TV are 0 for all methods, and max|v|=.4 m/s.

| Method | First NEW omega (rad/s) | First abs(delta omega) | Angular TV | Max abs(omega) | Swept clearance lower bound (m) |
|---|---:|---:|---:|---:|---:|
| RAW | −.0367487813751 | .00951037977179 | .0901891728750 | .0770850419113 | 1.040803137454 |
| B_ENTRY | −.0367487813751 | .00951037977179 | .0901891728750 | .0770850419113 | 1.040803137454 |
| HERMITE | −.00144785900904 | .0448113021378 | .144502590533 | .0836388207338 | 1.041118876121 |
| GRAPH | −.0380821957511 | .00817696539576 | .0995013093981 | .0818355647941 | 1.040461605923 |

Every termination is PRE_NEXT_INSTALL_CAP; guard abort=false and physical
overlap=false. All reference minimum clearances are **.821280919083059 m**;
all physical checks and legacy .05 m diagnostics pass. This short interval remains
far from contact; clearance differences do not establish an obstacle-avoidance gain.

### Signed paired differences: Graph minus comparator

Positive error AUC/TV means Graph is larger; positive clearance means Graph has
more clearance. These quantities are not combined into a scalar score.

| Metric | vs RAW | vs B_ENTRY | vs HERMITE |
|---|---:|---:|---:|
| Position AUC .3 (m s) | +.000000904090 | +.000000904090 | +.000020104162 |
| Position AUC full (m s) | +.000027756114 | +.000027756114 | +.000092489192 |
| Yaw AUC .3 (rad s) | +.000038153048 | +.000038153048 | +.000560877406 |
| Yaw AUC full (rad s) | +.000875227353 | +.000875227353 | +.001682680472 |
| First abs(delta omega) (rad/s) | −.001333414376 | −.001333414376 | −.036634336742 |
| Angular TV (rad/s) | +.009312136523 | +.009312136523 | −.045001281135 |
| Swept clearance (m) | −.000341531530 | −.000341531530 | −.000657270197 |

First |delta v| and linear TV paired differences are 0 throughout.

### Planning, intent proxies and selector identities

| Method | Rows | XY arc (m) | B→first gap (m) | Incoming/first-segment mismatch (rad) | Minimum edge (m) | Exact original rows retained |
|---|---:|---:|---:|---:|---:|---|
| RAW | 10 | 1.357452563638 | .037238608007 | .000379661219 | .149336927213 | F0–F9 |
| B_ENTRY | 11 | 1.394691171645 | 0 | .001366186470 | .037238608007 | F0–F9 |
| HERMITE | 12 | 1.394691171813 | 0 | .001271271279 | .018619303568 | F0–F9 |
| GRAPH | 10 | 1.394595535140 | 0 | .000052217392 | .149695504399 | F9 |

Graph converged in 3 iterations by cost_tolerance. Initial total cost
.05721558510371845 became .03120742223186173. Final costs:
T=3.978259363010347e−8, R=.023471344358307847, A=.0077360380909602565.
Its relative-edge translation RMS/max are .005104878790556552/
.00837541743006557 m; yaw RMS/max .00024362222584400025/
.00044436671954617424 rad. Endpoint world row remains bit-exact, displacement 0 m.

Per-node displacement from S, including fixed B replacing S0, in metres:
[.03723860800693844, .028864632645019577, .020849655649352018,
.013869274074193493, .008434917677988983, .004667774920780129,
.002335573088810043, .0010392438709875683, .00037165612406419554, 0].
Only the eight internal rows carry optimized A factors. The saved floating-point
SE(2) log at the exact endpoint has translation roundoff below 7e−15; the direct
endpoint row comparison and Euclidean displacement are exactly zero.

First official H5 selected identities at tick 96:

| Method | Identities |
|---|---|
| RAW | F1,F2,F3,F4,F5 |
| B_ENTRY | F1,F2,F3,F4,F5 |
| HERMITE | E*=F0,F1,F2,F3,F4 |
| GRAPH | X1,X2,X3,X4,X5 |

RAW/B_ENTRY select F1–F5 also at 102/108, then F2–F6 at 114. Hermite selects
F1–F5 at 102/108 and F2–F6 at 114. Graph selects X1–X5 at 102 and X2–X6 at 108/114.
Graph Xi denotes a deformed row associated with Si, not an untouched original pose.
Reference row insertion changes the unchanged nearest+1 selector's first H5 set
for Hermite. The measured differences therefore cannot be assigned solely to
smooth curve shape.

### Call accounting and figures

Graph scientific solves 1; SimulationApp launches 1; RAW/B_ENTRY/HERMITE/GRAPH
rollouts 1 each; new official MPC submissions/solves 16/16; LightNav 0,
RGB/model requests 0, source acquisition 0, retries 0. Validation/reporting adds 0
scientific calls. All execution, reference and result hashes are sealed.

Exactly two PNGs were generated and visually inspected:

- [World execution](../results/successive_isaac_four_method_comparison_01/figures/four_method_world_execution.png)
- [Transition metrics](../results/successive_isaac_four_method_comparison_01/figures/transition_metrics.png)

World panels have equal XY axes. Curves strongly overlap: RAW/B_ENTRY maximum
matched XY separation is exactly 0. RAW/Hermite .000322981136264662 m;
RAW/Graph .0003500901783163986 m; Hermite/Graph .0006730667535219206 m.
Markers/styles and the metric PNG/numeric sidecar expose these small differences.
No additional final PNG or HTML dependency was created.

## Research interpretation

**FOUR_METHOD_MIXED_EVIDENCE.** Graph gives a smaller first NEW angular-command
change, but higher position/yaw AUC than all three comparators. It also has higher
angular TV than RAW/B_ENTRY, and lower angular TV than Hermite. Graph's lower
planning objective and smaller initial tangent mismatch do not establish improved
execution. Its small spatial/yaw reference corrections are distinct from the
angular velocity commands selected by the official MPC.

RAW and B_ENTRY are numerically identical in this window. E*=F0 removes no prefix,
and their selected original poses match. Hermite has lower error AUC but a larger
first angular-command change and angular TV. This is an observed trade-off, not a
net improvement or evidence that optimization is necessary. Exact downstream
identity for RAW/B_ENTRY/Hermite and Graph's small relative-edge errors are geometry
proxies; neither establishes semantic intent preservation.

RAW parity and method isolation establish the technical ability to perform this
bounded Isaac comparison. **These results do not justify advancing Graph as an
improved method into a multi-handoff successive comparison on efficacy grounds.**
A separately specified future mechanism study could examine selector/row-density
and larger transition gaps before broader execution; this task performs none.

## Limitations and protocol deviations

One development source/handoff; E*=F0; B gap 3.72 cm; short .4667 s isolated window;
submillimetre matched execution differences; logical SE(2) agent without robot
actuator dynamics; offline controlled releases, not online timing. No downstream
completion, task success, general obstacle avoidance or population result is tested.

No scientific protocol deviation: no new source, no C2, no retry, no post-outcome
code/config/threshold change. The horizon correction and zero-motion clock
initialization were explicit before freeze. The prior replay remains blocked under
its original longer-window requirements. Only documentation/compact saved results
are added after science. Unrelated user changes remain uncommitted.

## Validation and completion record

Focused tests: **17 passed**. The full relevant regression command listed above
passed **1,159 tests before freeze (356.53 s)** and **1,159 again after science
(353.84 s)**. These are repeated runs of the same suite, not 2,318 unique tests.
Historical saved validators included in that suite pass. The source accounting
addendum was also checked directly with:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/four-method-mpl .venv/bin/python scripts/validate_successive_source02_accounting_addendum.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01
```

Post-science saved-only JSON/figure checks, source/code/raw-result hashes,
`python -m compileall -q src scripts tests`, `git diff --check` and staged diff
checks pass. Figures were visually inspected and CSV fields compared exactly to
validated JSON. The generated CSV's CRLF line endings were normalized to LF for
Git whitespace checks; no numeric field or frozen writer code changed.

Git actions: `git commit -m 'Freeze isolated C0 to C1 four-method Isaac comparison'`
and `git push origin main` preceded science. Completion uses
`git commit -m 'Report isolated four-method Isaac transition results'` and normal
`git push origin main`. No force push, history rewrite, environment edit or
additional experiment. The result commit is the commit containing this section.
