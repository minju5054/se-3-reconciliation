# GP-SE2-JOIN-01: controlled obstacle reveal and original-FRESH attachment

Question: when a newly observed obstacle changes FRESH, how quickly and safely
can the already executing state B attach to a valid future part of ORIGINAL
FRESH? This is one controlled handoff, not a long episode, population benchmark,
new controller, or continuation of endpoint/lateral threshold diagnostics.

Starting/fetched main: `7864ab26bf0c16be0b8575338c6c269874246a34`.
Two unrelated Stage-0 config edits are preserved. Numerical core, official
LightNav/MPC, prior runs, environment export and checkpoint remain unchanged.
The collector gains optional explicit intervention hooks; default None retains
historical behavior. JOIN-01 injects runtime reveal/visibility capture and
prevents a pre-reveal queued RGB from being the FRESH observation.

## Pre-primary protocol

Authoritative config: `configs/gp_se2_join_01.yaml`, combined with unchanged
acquisition and GP-SE2-01 configs; resolved copies/SHA256 in each exclusive run.
No primary has run at protocol authoring. Commit precedes actual collection.

Start `[19.2,26,-pi/2]` world metres/radians in the Hospital corridor. Existing
direct geometry verifies lateral free space (center clearance 1.348m at y=26,
free samples at x=18.2 and 20.2). Fixed instruction: "Continue straight down the
hallway. Avoid obstacles and keep going." Four live RGB bootstrap observations,
then genuine OLD and actual official-MPC execution. Reveal at first 4Hz capture
at least .10s after OLD activation. One FRESH per attempt, with asynchronous
OLD execution during inference, receipt and new-command activation. Source
recording stops .10s after activation, preserving actual B and u_minus/memory.
The method comparison, if qualified, starts independently from recorded B.

At most three ordered placements, predeclared before model outputs:

| Placement | OLD observation-to-path arc | Box forward/lateral/height, m |
|---|---:|---|
| 01 | 1.50m | .30/.60/.90 |
| 02 | 1.30m | .30/.50/.90 |
| 03 | 1.10m | .30/.40/.90 |

Arc includes observation-to-first-row; box yaw is the actual OLD segment tangent.
No extrapolation beyond OLD. Absolute world pose is resolved from OLD only,
before FRESH, with timestamp/state and USD readback matrix/size. Recent ~.67s
request latency motivates forward space; actual B must independently be valid.
Narrower/nearer later placements are predetermined upstream trials, not optimizer
retries. First qualifying event stops the remaining schedule.

Qualification: OLD future safe before reveal, unsafe afterward; B clearance-valid;
original FRESH suffix from official nearest row directly swept-clearance-valid;
goal beyond box front by footprint+.05m; >=4 remaining rows, >=.60m arc,
chord/arc >=.60; simple original XY, suffix segments >1e-6m; max existing
point-to-polyline disagreement >=.20m OR projected pose-yaw mismatch >=20deg.
OLD future is queried at <=.01m against original FRESH continuous segments.
Actual same-camera semantic masks require >=20 box pixels in the FRESH capture.
OLD observation precedes reveal; FRESH observation follows it. Exact inflight
state intervals require RTF [.8,1.2], max loop stall .25s and measured overlap.
Any crossed original finite gate remains. No artificial left/right box label.
If a gate is required, current GP derivative support is explicitly unsupported,
never bypassed. No qualifying actual handoff means no reconciliation solves.

## Formulation and execution

M0_NATIVE uses unchanged raw rows; M0_ADAPTER original suffix/resampling;
M1_RIGID unchanged solve_rigid/two starts. M3_CURRENT_GP maps to original
GPProblem plus verified supplied Jacobians, midpoint lateral equalities and
motion at u={0,.5,1}. No quarter rows, DIAG06 event-specific witnesses or DIAG08
endpoint reserve are treated as generic. M4 differs only in post-join FRESH
preservation and explicit attachment inequalities.

M3/M4 share byte-identical original FRESH finite-log and fixed same-curvature
deceleration seeds in common anchors. 150 variables/31 supports/3s/.1s, original
GP prior/interpolation/Qc/Sigma/lambda, .20m footprint/.05m clearance, final
.15m/15deg goal, lateral and motion tolerance and .20s final dwell are retained.
CPU float64 supplied-Jacobian SLSQP, 200 iterations, ftol1e-7, 30s prepared per
start, single-thread BLAS. No finite-difference fallback or new solver.

M4: j0 is official weighted wrapped-pose nearest on original FRESH at B;
J={j0..j0+4} clipped/deduplicated. Tau={.4,.6,.8,1,1.2,1.4}s. All candidates,
two identical original seeds each, run once sequentially; maximum60 starts.
No optimizer-outcome early exit or additional restart. Original FRESH[j:] is
interpolated across [tau,3] by the original linear-XY/shortest-yaw helper. This
is a planning convention; original row indices are never timestamps.

`J = J_GP + lambda_F * mean_{k>=k_a} ||Log(Fhat_k^-1 X_k)||²_SigmaF`.

Before tau there is no FRESH preservation cost. At tau,tau+.1,tau+.2,tau+.3:
`.10²-distance² >=0` plus signed 15deg yaw bounds (12 inequalities; zero new
equalities). Direct nominal tube check is separate from original solver numeric
allowance. Original full checker verifies boundary/twist/dense/offset motion,
environment/goal/route. Among full-valid initial/latest/actual grid-feasible
callback records, M4 selects by tau,J,j,seed,source label. M3 retains historical
dense-first selection and rejects a full-invalid selected result. Invalid GP
never executes or falls back. Native/adapter/rigid spatial-invalid diagnostic
rollouts retain the historical policy and their explicit invalid status.

Primitive optimizer distance=min(original conservative Hospital grid, exact
unsigned box distance), with original radius/clearance offsets. Independent
checker uses direct GEOS union/swept circle. Invalid grid remains invalid.
Min ties select original branch; inside-box distance is constant zero, not an
informative safety gradient. No smoothing/map rebuild. M4 CPU JAX supplies the
changed cost and appended tube derivatives; original Jacobians stay literal.
Derivative gates reuse DIAG02 tolerances, three frozen directions and steps
2e-4/2e-5/2e-6; both fine steps required. Failure blocks the start, no tuning.

All methods use original observation transform (never B reanchoring), physical
u_minus and separate previous_control. Official nearest/+1 H5/.1s computation,
30 solves at t=0..2.9, 60Hz exact held-command integration, 181 states. No GP
velocity feed-forward. Label: OFFLINE SINGLE-HANDOFF COUNTERFACTUAL. Optimizer
wall time does not move simulation; no online latency/navigation claim.

## Attachment metric and evidence

Closest projection onto the remaining original XY polyline is restricted to
previous fractional row progress or later; exact ties choose earliest segment.
Yaw interpolates shortest angle. First <=.10m and <=15deg interval whose entire
following .30s saved samples pass defines sustained join. One-sample crossing
fails; missing join is null. Execution grid includes <=5ms and 60Hz states;
this is sampled persistence, not continuous-time proof. GP planned metric uses
original interpolation at <=1ms plus .371-offset/support points. Untimed raw and
rigid references have planned join time N/A. Planned tau and measured joins differ.
Report post-join mean/max distance, pre-join AUC (N/A without join; full-window
AUC separate), safety/workspace/route, original goal/dwell, command TV/acceleration,
candidate availability, construction/warmup/gates/solve/check/MPC costs.

Primary plots show actual OLD prefix, original FRESH, B, box and clearance,
method reference/execution, measured joins and original goal, equal XY metres.
Distance/yaw versus full 0..3s plot all executed methods. No fabricated missing
traces. Qualification failures have source-only plots, not method results.

## Exact commands

After pre-primary tests, diff review and commit:

```bash
.venv/bin/python scripts/run_gp_se2_join01.py prepare --run "$JOIN_RUN"
.venv/bin/python scripts/lightnav/robotless_online_server.py start "$JOIN_RUN"
bash scripts/launch_gp_se2_join01_collect.sh --run "$JOIN_RUN"
.venv/bin/python scripts/lightnav/robotless_online_server.py stop "$JOIN_RUN"
# Only if collection_result.selected_episode is non-null:
.venv/bin/python scripts/run_gp_se2_join01.py freeze_event --run "$JOIN_RUN"
.venv/bin/python scripts/run_gp_se2_join01.py optimize --run "$JOIN_RUN"
/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python \
  scripts/lightnav/gp_se2_join01_rollout.py --run "$JOIN_RUN"
.venv/bin/python scripts/run_gp_se2_join01.py evaluate --run "$JOIN_RUN"
# Both qualified and nonqualified completed acquisition runs:
.venv/bin/python scripts/plot_gp_se2_join01.py --run "$JOIN_RUN"
.venv/bin/python scripts/validate_gp_se2_join01.py --run "$JOIN_RUN"
.venv/bin/python scripts/plot_gp_se2_join01.py --run "$JOIN_RUN" --package
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
bash -n scripts/launch_gp_se2_join01_collect.sh
git diff --check
```

Qualification failure, technical blockage, formulation limitation and scientific
recovery/regression remain distinct. This protocol does not authorize the next
long-episode stage or a controller redesign.
