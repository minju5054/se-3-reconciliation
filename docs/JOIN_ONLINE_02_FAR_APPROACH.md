# JOIN-ONLINE-02 — Far-approach successive native LightNav bypass audit

## Frozen question and scope

Starting far from the same fixed supply cart, do successive genuine native
LightNav chunks eventually provide a whole-path safe lateral bypass before the
next actual command becomes unsafe? This is upstream source diagnosis, with no
GP, rigid, reconciliation, controller changes, or handoff optimization.

Starting main and fetched origin/main: `a02b79750e22cd64b68351992fb82a2ce852141a`.
Source run: `data/robotless_join_online_02/primary_20260923T001500Z/`.
The run identifier is a label; actual UTC/host-monotonic timestamps are recorded
independently. SOURCE04/05 historical artifacts remain unchanged. The two existing
modified stage0 configs are unrelated user changes and are excluded from commits.

## Source and geometry-only start selection

Reuse SOURCE04 `persistence_20260922T050237Z` cart mesh (8,024 triangles), exact
world transform, Hospital composed layers, and camera. Illumination is the same
`/World/JOINSource03Fill` RectLight: intensity 1000, exposure 0, white, normalize
false, width 4 m, height 10 m, translation [19.2,24,2.6] m. No RGB postprocessing.
The cart is present before every ON episode and absent throughout OFF.

Candidate centre distances, frozen in order: **5.0, 4.5, 4.0 m**. All three pass
original direct Hospital/cart footprint and OFF approach checks. Select 5.0 m:
`R0=[19.19599771672821,27.640298097840173,-1.5685636348381817]`.
Hallway forward `[0.002232690101752685,-0.9999975075443486]`;
cart centre `[19.207161167236976,22.64031056011843]`.
At R0, cart-only edge clearance 4.579380595 m; combined environment clearance
1.350752635 m. OFF straight approach clearance 1.016922068 m. These are geometric
checks, without querying LightNav. Both direct lateral passage checks pass;
they are evaluator diagnostics, never model inputs or controller waypoints.

Technical Isaac preflight passed exact static-layer/cart-transform/mesh identity,
OFF hidden pixels=0, ON cart pixels=372, same camera, unchanged state, physics
clock, and official MPC import provenance. No model prediction or MPC solve was
used. Six zero-motion technical physics steps only.

## Native provenance and schedule

Official LightNav source `c6f40e3220edbf7011e4f17eaf2c865416737d4d`, checkpoint
revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, VLN task. Exact instruction:

> Avoid the supply cart and continue to the end of the hallway.

Greedy generation: temperature=0, top_p=1, top_k=0, traj_top1=0. Launcher argv and
actual inherited process environment are recorded; checkpoint file hashes must
match SOURCE04. Official MPC SHA256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1` is unchanged.
HORIZON=5, dt=.1 s, control rate=10 Hz. Original nearest/+1 selector, computation,
limits, memory updates, observation anchoring, and command holds are preserved.
The research collector adds only an optional abort hook; its default behavior
and all external source files remain unchanged.

Run once in this order:

1. OFF_REPEAT_00
2. ON_REPEAT_00
3. OFF_REPEAT_01
4. ON_REPEAT_01

Independent sessions, four live bootstrap images at 4 Hz; three buffer requests,
fourth predicts C0. A subsequent prediction requires .5 simulation seconds of
actual activation of the prior reference. Live OLD commands continue while FRESH
is pending. Original 10 Hz asynchronous MPC and exact held-command SE(2)
integration at resolved PhysX 60 Hz dt. Stop at 20 activations after C0 or 25 s
since C0, whichever first; original attempt cap is also 20. Natural model STOP
terminates separately. No outcome retry or new distance. Optimizers are absent.

World x/y in metres, yaw CCW radians. Local forward/left/yaw CCW. Each raw Nx3 is
anchored only at its triggering observation; LightNav intrinsic waypoint dt is
null. B is the pre-integration state of the first **applied** current-generation
command. Ready, install, and apply timestamps remain distinct. Physical previous
command and controller previous-command memory are stored separately.

## Abort-only guard

Before applying a newly solved command, inspect its nominal next .1 s hold using
exact unicycle samples at no more than 1/60 s and the existing sagitta allowance.
Each subsequent held 60 Hz step is checked again, including delayed-result holds.
The direct Hospital plus actual cart-state checker uses radius .20 m, required
edge clearance .05 m, and existing geometry uncertainty. A negative result ends
with `SAFETY_ABORT_BEFORE_UNSAFE_COMMAND`; the proposed command is logged but not
applied, and cannot create B or activate a reference. No substitute steering or
new post-abort prediction. Already-sent responses may be retained as unactivated
and excluded from prospective source events. The .1 s check is a nominal hold;
a later asynchronous command can replace it earlier. This conservative oracle
is an acquisition abort, not a real-robot safety controller.

**An unsafe raw FRESH alone never triggers the guard.** Raw future safety and
actual applied-prefix safety are evaluated separately.

## Frozen geometry classifications

Whole raw returned polyline is checked, without trimming or an observation-to-
first-row connector. Generic N. Nominal radius/margin and numerical uncertainty
are unchanged. Meaningful lateral change is >=.20 m, or a reliable >=.10 m chord
at >=20 degrees to the hallway with at least .02 m actual lateral change. Yaw-only
response is insufficient. Positive returned forward progress and a previously
validated free side are required for bypass onset. Full bypass additionally
requires the endpoint beyond the actual cart rear plane by .25 m. Whole-path
safety is always required; endpoint success alone cannot pass.

Classification priority: official STOP (safe raw => SAFE_STOP, otherwise unsafe),
PAST_OBSTACLE (observation beyond rear+.25 and path safe), unsafe, full bypass,
onset, shorten, straight/baseline. Safe shortening means no meaningful side
movement, end before front, and either reaches front-.50 m influence region or
raw arc <=.50 m; normal far straight chunks remain distinct. All continuous
measurements are reported, including returned arc and endpoint front/rear offset.

OFF labels involving the cart are **hypothetical cart-ON checks**; actual OFF
execution is evaluated in cart-OFF geometry. Descriptive OFF match uses nearest
observation longitudinal coordinate, <=.25 m and <=20 degree heading gap. No
forced chunk-index matching and no independent-sample statistical claims.

Primary event: first full SAFE_BYPASS received before episode termination.
First SAFE_BYPASS_ONSET is separate. Episode outcomes preserve STOP, abort, and
technical/episode limits. Candidate source selection: earliest full bypass in
ON_REPEAT_00, else earliest onset there, then the same order in ON_REPEAT_01.
All candidates are retained. An unactivated candidate has no fabricated B and
cannot become a moving handoff source.

## Presentation and validation protocol

Static all-chunk RGB/world contact sheets, all observations/execution overlays,
clearance/lateral/arc vs observation distance, classification timelines, and OFF
longitudinal comparisons. First-onset/bypass snapshots only if available, else
explicit N/A. PNG sidecars include plotted numbers, source and image hashes.

After collection, an interactive Isaac viewer reads saved states and RGB only:
`RECORDED ONLINE FAR-APPROACH REPLAY`. Four-episode dropdown, chunk slider,
previous/next, play/pause, preceding/following chunk toggles; original trigger RGB,
B/observation markers, raw safety/onset/full-bypass fields and numeric timings.
No inference, MPC, or new integrated execution in replay. Orthographic equal XY
world axes; following recorded data are explicitly retrospective.

Saved-only validation reuses the original stream/history/wire/official-selection
validator and independently reconstructs commands, all states, B, lifetime,
guard decisions, activation non-application, same-product masks, observation
anchoring, raw geometry/classification and plotted numbers. Scientific failure
is not artifact corruption. Timing validity remains a separate limitation.

## Commands

```bash
.venv/bin/python scripts/run_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z --mode init
# Isaac commands use the existing clean Isaac environment; see run logs.
/home/gpuadmin/isaacsim/python.sh scripts/isaac/join_online02_collect.py --run data/robotless_join_online_02/primary_20260923T001500Z --mode preflight
.venv/bin/python scripts/run_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z --mode freeze
# Commit/push before either start or collect; code/input hashes checked again.
.venv/bin/python scripts/run_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z --mode start
/home/gpuadmin/isaacsim/python.sh scripts/isaac/join_online02_collect.py --run data/robotless_join_online_02/primary_20260923T001500Z --mode collect
.venv/bin/python scripts/run_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z --mode stop
.venv/bin/python scripts/analyze_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z
.venv/bin/python scripts/report_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z
.venv/bin/python scripts/validate_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z
/home/gpuadmin/isaacsim/python.sh scripts/isaac/join_online02_replay.py --run data/robotless_join_online_02/primary_20260923T001500Z --output data/robotless_join_online_02/replay_20260923 --verify
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_join_online02.py tests/test_robotless_online.py tests/test_join_source02_acquisition.py tests/test_online_mpc_adapter.py tests/test_online_history.py tests/test_join_source04.py tests/test_join_source05.py tests/test_online_ipc.py tests/test_robotless_online_validator.py tests/test_robotless_online_replay.py tests/test_online_handoff_analysis.py tests/test_join_source05_completion.py
.venv/bin/python -m compileall src scripts tests
git diff --check
```

Scientific execution pending the pushed freeze commit. No favorable outcome is
assumed. The source/geometry-only gates have passed.
