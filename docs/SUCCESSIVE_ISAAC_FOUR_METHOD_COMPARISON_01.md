# SUCCESSIVE_ISAAC_FOUR_METHOD_COMPARISON_01

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
