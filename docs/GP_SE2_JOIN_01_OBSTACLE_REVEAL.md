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

## Actual result — 2026-09-21

Operational: **GP_SE2_JOIN_01_COMPLETED_WITH_LIMITATIONS**.
Research result: **NO_QUALIFYING_OBSTACLE_REVEAL_HANDOFF** / upstream
qualification failure. This is neither positive nor negative GP-performance
evidence. The prescribed stop condition was reached before method comparison.

Run: `data/robotless_gp_se2_join_01/primary_20260921T083025Z/`.
Execution freeze: `0de41faebe9a9bb662b0f6773123e4c6975a8d1e`.
Starting/fetched origin/main: `7864ab26bf0c16be0b8575338c6c269874246a34`.
Subsequent changes are reporting-only additions; every frozen numerical,
acquisition, config, environment and prior-source hash still verifies.

All three declared placements executed once, in order. Each has a real OLD and
FRESH from the pinned official service, actual rendered RGB and recorded OLD
commands during FRESH inference. The box was added at runtime, without changing
the Hospital asset. No fourth placement, altered instruction, fabricated route,
primary retry, trajectory optimizer or common-B execution was used.

| Attempt | B edge clearance m | OLD future / FRESH suffix clearance m | Max cross-track m | Max projected pose-yaw difference deg | Qualifies |
|---|---:|---:|---:|---:|---|
| 01 | .686667 | -.200000 / -.200000 | .000160226 | .0173707 | No |
| 02 | .653333 | -.200000 / -.200000 | .000047310 | .0052889 | No |
| 03 | .463333 | -.200000 / -.200000 | .000047310 | .0052889 | No |

The OLD future was valid before reveal (minimum edge clearance ~1.0145m).
After reveal the original OLD and original FRESH reference footprints overlap
the box; the -0.2m value is center-to-obstacle unsigned distance minus radius,
not a penetration-depth estimate. It describes unsafe future references, not
an executed collision. B passes the frozen .05m requirement in every attempt.
FRESH does not provide the required safe route around the box, and mismatch is
far below .20m/20deg. Attempts01/02 also fail the goal-beyond-box criterion.
No field was weakened or removed after these results.

| Attempt | OLD obs | OLD active | Reveal = FRESH obs | Ready seen | Application B | Request→receipt wall | FRESH obs→B travel |
|---|---:|---:|---:|---:|---:|---:|---:|
| 01 | .783333 | 1.216667 | 1.533333 | 1.800000 | 1.900000 | .290080s | .293333m |
| 02 | .783333 | 1.166667 | 1.283333 | 1.550000 | 1.683333 | .216171s | .266667m |
| 03 | .783333 | 1.166667 | 1.283333 | 1.550000 | 1.666667 | .199467s | .256667m |

Observation/activation/reveal/ready-seen/application columns are Isaac simulation
seconds; request and receipt preserve separate monotonic/UTC host clocks in
`aggregate/qualification.csv` and original context. They are not interchangeable.
B passes the first original FRESH point by about .142274/.115607/.105607m along
the nearly straight route because OLD continues after capture until the first
FRESH command is applied. FRESH stays observation-anchored. Actual u_minus and
controller previous_control are separately preserved, though equal here
(v=.8m/s and very small angular commands). Their equality was not imposed.

## Limits established by the saved evidence

1. All semantic masks contain only BACKGROUND/UNLABELLED, returning zero labelled
   obstacle pixels. Actual FRESH JPEGs were inspected and visibly contain the
   orange box; the OLD JPEGs do not. Thus the visibility instrument is not
   validated for this runtime primitive. Zero labelled pixels does **not** mean
   LightNav received an obstacle-free image. The six recorded wire requests were
   decoded locally and match the exact triggering JPEG bytes and SHA256; response
   sequence IDs match. See `verification/wire_rgb.json`. No new request was made.
   The renderer labelling cause remains unlocalized and was not repaired/retried
   after observing outcomes. Independently, all attempts already fail the
   essential safe-FRESH and nontrivial-disagreement conditions.
2. Attempt01 inflight RTF=1.903869 fails the frozen [.8,1.2] gate; attempt02/03
   RTF=1.161577/1.071644 pass. Full-episode pacing is not substituted for this
   request-local check. This first-attempt limitation remains in its ledger.
3. These are short, translation-dominant controlled source attempts, with only
   6–7 history frames at FRESH (not a full long-episode history). The experiment
   cannot establish why the upstream model retained straight decoded actions,
   general obstacle competence, or whether a different separately declared
   observation/instruction would produce the needed source.
4. M4's actual-event optimizer and common-state execution remain unvalidated.
   Implementation/AD tests are synthetic correctness evidence only. They do not
   show availability, feasibility, earlier join, or navigation improvement.

## Methods, calls and cost

| Method | Actual comparison run | Planned sustained join | Executed sustained join | Safety/motion/goal comparison |
|---|---|---|---|---|
| M0_NATIVE | Not run: source gate failed | N/A | N/A | N/A |
| M0_ADAPTER | Not run: source gate failed | N/A | N/A | N/A |
| M1_RIGID | Not run: source gate failed | N/A | N/A | N/A |
| M3_CURRENT_GP | Not run: source gate failed | N/A | N/A | N/A |
| M4_JOIN_GP | Not run: source gate failed | N/A | N/A | N/A |

No selected M4 time/index exists. GP and rigid solves=0, comparison rollouts=0,
comparison MPC solves=0. Actual source acquisition used 6 genuine LightNav
prediction requests (3 OLD + 3 FRESH), 22 saved RGB frames, 23 official MPC solve
results (9/7/7). Two results were rejected as stale through the original policy;
all 23 have reported solve duration, totaling .136773794s. Source states/steps
are 119/118, 106/105, 105/104. Optimizer/constructor/comparison MPC time is N/A,
not a measured zero-duration solve. The server separately logs one startup
warmup, which is not source evidence. Full testing also performs historical MPC
reproducibility audits, retained separately under `test_audits/`; these are not
new source episodes or counterfactual performance observations.

The actual headless Isaac renderer initialized and ran, with genuine RGB.
No comparison GUI or saved-trajectory execution figure was fabricated.

## Validation and evidence

The initial root `validation.json` preserves three reporting failures: its
"OLD during FRESH inference" predicate incorrectly started at OLD observation,
therefore included stationary bootstrap commands before OLD existed. The
additive `report_gp_se2_join01.py` admits only those exact three errors, checks
all other root assertions, re-authenticates raw manifests, repeats qualification
and checks actual client-inflight-to-B commands. All pass. Authoritative result:
`verification/validation.json` (`valid=true`); source qualification remains false.
This fixes a verifier scope error, not execution or research outcomes.

Tests before acquisition: relevant 126 passed, full 2,418 passed / 19 existing
skips. After reporting additions, two targeted reporting tests passed. Compileall,
shell syntax, diff checks, source verification and numeric/PNG/ZIP checks pass.
Final full-suite result is recorded in the append-only work log and retained log.

Primary review: `review/index.html`; all three source overlays:
`plots/placement_01_source_handoff.png` through `placement_03_source_handoff.png`.
Summary: `review/qualification_summary.png` with exact numeric/hash sidecar.
Tables: `aggregate/qualification.csv`, `aggregate/method_ledger.csv`.
Packet: `review_bundle.zip` (370,820 bytes; 27 members), SHA256
`66d916183f7469b5d336a0f66b676c8161b8c8c95cd729b9ef446df82ca96bea`.
`review_validation.json` verifies packet bytes and summary-plot numbers. The
packet excludes raw RGB; local HTML links retain actual before/after observations.
The later read-only wire-byte audit and final test log remain in the full run.

Actual command sequence was prepare → server start → collector → server stop →
source plots → root validator → reporting completion → review packet. The
conditional freeze_event/optimize/rollout/evaluate commands were **not** run.
Additional completion commands:

```bash
JOIN_RUN=data/robotless_gp_se2_join_01/primary_20260921T083025Z
.venv/bin/python scripts/report_gp_se2_join01.py --run "$JOIN_RUN"
.venv/bin/python scripts/review_gp_se2_join01.py --run "$JOIN_RUN"
```

All writers refuse existing outputs. To independently reproduce the wire audit,
base64-decode `requests/wire_*_next_request.json:data.image` at each saved chunk's
`metadata.json:seq`; compare bytes and SHA256 with `observation.path/sha256`, and
compare `response.json:data.seq`. This reads saved files only.

The single next uncertainty is obtaining a **genuine safe obstacle-responsive
original FRESH** under a separately frozen upstream observation protocol, with
validated visibility instrumentation. Only then can the frozen M0–M4 comparison
answer the attachment question. No new placement, controller, long episode or
subsequent formulation experiment is implemented here.
