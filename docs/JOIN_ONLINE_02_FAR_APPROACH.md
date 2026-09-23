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

Scientific execution completed once after pushed freeze commit
`f3170c76042e9a2fd62dfa515f74723a7fbeaafd`. Actual acquisition interval including
Isaac startup: 2026-09-23 01:15:28.663247–01:16:52.085748 UTC.
The result commit is the Git revision containing this report; full final SHA is
also recorded in the ignored run completion record after push.


## Observed result

**PROGRESSIVE_SHORTENING_OR_STOP**, with timing limitations. Both ON repeats end
by native `MODEL_STOP`, not the oracle guard. Neither emits SAFE_BYPASS_ONSET or
SAFE_BYPASS. No reconciliation candidate source is available. All ON returned
raw polylines and applied command prefixes satisfy the unchanged .05 m edge
requirement. The cart is visible in the very first ON capture (372 instance
pixels, simulation t=.033333335 s), at the selected 5 m start. This establishes
rendered presence, not hidden neural recognition.

The seven ON raw local arrays match bitwise across repetitions. The sequence is
near-straight C0/C1/C2, nearly identical C3, shorter C4 (1.05583 m arc), shorter C5
(.510854 m arc), then C6 STOP (one zero-pose action row). Observation distance at
C4 is 2.233–2.237 m, at C5 1.433–1.437 m, at STOP .765 m. Maximum centreline lateral
magnitude remains under .00461 m. No meaningful side response or yaw-only bypass
is inferred. Last executed ON centre distances are about .762 m; minimum actual
edge clearances .355180979 / .354753227 m. Last ON forward commands are about
6.5e-9 / 0 m/s. No actual cart traversal occurred.

The frozen **SAFE_SHORTEN label** additionally required arc <=.50 m or reaching
front-.50 m. C5 has arc .510854 m and ends about .550 m before the front. Thus
ON C0–C5 retain `STRAIGHT_OR_BASELINE` and first formal SAFE_SHORTEN is **N/A**,
although numerical shortening is directly measured. No threshold was adjusted
to relabel the result. OFF_REPEAT_01 C4 meets the spatial SAFE_SHORTEN gate despite
unchanged raw arc; the label is not a causal relative-shortening metric. The
primary STOP/no-bypass result does not depend on that descriptive category.

Cart-OFF runs continue beyond the hypothetical cart location, later ending with
MODEL_STOP farther down the corridor. OFF C5/C6 raw references would intersect
the cart if it were present; this is **not an actual OFF collision**. Actual OFF
execution clears the static Hospital. OFF/ON chunk indices are not forcibly
aligned; `aggregate/analysis.json` records longitudinal/heading-matched pairs.
At comparable C4/C5 observation locations in repeat01, OFF arc stays1.3562 m while
ON arc drops to1.0558/.5109 m. Both remain nearly straight. This supports an
obstacle-associated extent/STOP response in this scenario, not a lateral detour
or a claim about internal reasoning.

| Episode | responses | applied chunks | final outcome | actual min edge m | final longitudinal m | guard abort |
|---|---:|---:|---|---:|---:|---|
| OFF_REPEAT_00 | 20 | 19 | NATURAL_MODEL_STOP_BEFORE_BYPASS | 0.613669968 | 10.449664 | No |
| ON_REPEAT_00 | 7 | 6 | NATURAL_MODEL_STOP_BEFORE_BYPASS | 0.355180979 | -0.762302 | No |
| OFF_REPEAT_01 | 21 | 20 | NATURAL_MODEL_STOP_BEFORE_BYPASS | 0.617518095 | 10.172862 | No |
| ON_REPEAT_01 | 7 | 6 | NATURAL_MODEL_STOP_BEFORE_BYPASS | 0.354753227 | -0.761874 | No |

All four first SAFE_BYPASS_ONSET / first SAFE_BYPASS events are N/A. The two ON
repeats agree on response sequence, lack of bypass, and natural STOP. This is
only two independent online repetitions of one scene, not a population result.

## Complete successive-chunk ledger

Times below are saved simulation clock seconds. C6 STOP has no t_apply/B/lifetime;
no moving boundary is fabricated for an unapplied response. `edge` uses cart-ON
geometry for every row, hypothetical for OFF. Actual prefix edge uses the actual
episode environment. World transforms, full yaw/tangent and raw-array hashes,
request/receipt/ready/install clocks, all physical u-minus and controller memory,
and OFF matching are in `aggregate/analysis.json`. `aggregate/chunks.csv` gives
machine-readable table values. Labels: S=STRAIGHT_OR_BASELINE,
U=UNSAFE_INTERSECTING, H=SAFE_SHORTEN, STOP=SAFE_STOP, P=PAST_OBSTACLE.

### OFF_REPEAT_00

| Ck | obs / apply s | obs centre m | B edge m | obs→B travel m | raw edge m | arc m | max lateral m | final yaw wrt hallway rad | endpoint rear m | lifetime s | OLD → next | actual prefix edge m | class |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| C0 | 0.7833 / 1.4667 | 5.0000 | 4.5794 | 0.0000 | 1.3465 | 1.3562 | 0.0004 | 0.0007 | -3.7161 | 0.9167 | N/A → C1 | 1.3496 | S |
| C1 | 2.0333 / 2.3833 | 4.6633 | 3.9645 | 0.2800 | 1.2940 | 1.3562 | 0.0008 | 0.0010 | -3.3795 | 1.0000 | C0 → C2 | 1.3461 | S |
| C2 | 3.0333 / 3.3833 | 3.8633 | 3.1677 | 0.2800 | 1.0319 | 1.3562 | 0.0015 | 0.0013 | -2.5795 | 1.0000 | C1 → C3 | 1.1596 | S |
| C3 | 4.0333 / 4.3833 | 3.0633 | 2.3728 | 0.2800 | 1.0192 | 1.3562 | 0.0027 | 0.0017 | -1.7795 | 0.9833 | C2 → C4 | 1.0190 | S |
| C4 | 5.0333 / 5.3667 | 2.2633 | 1.5895 | 0.2667 | 0.3488 | 1.3562 | 0.0043 | 0.0021 | -0.9795 | 1.0167 | C3 → C5 | 1.0321 | S |
| C5 | 6.0333 / 6.3833 | 1.4633 | 0.7762 | 0.2800 | -0.2000 | 1.3562 | 0.0061 | 0.0025 | -0.1795 | 1.1000 | C4 → C6 | 1.3377 | U |
| C6 | 7.0333 / 7.4833 | 0.6634 | -0.1022 | 0.3600 | -0.2000 | 1.3562 | 0.0083 | 0.0029 | 0.6205 | 0.9167 | C5 → C7 | 0.9832 | U |
| C7 | 8.0333 / 8.4000 | 0.1368 | 0.0324 | 0.2933 | -0.0838 | 0.5607 | 0.0107 | 0.0091 | 0.6457 | 1.3000 | C6 → C8 | 0.8518 | U |
| C8 | 9.0333 / 9.7000 | 0.8673 | 0.4932 | 0.0240 | 0.6188 | 0.5109 | 0.0140 | 0.0058 | 1.2978 | 1.1833 | C7 → C9 | 0.8434 | P |
| C9 | 10.2833 / 10.8833 | 1.2479 | 1.1593 | 0.3096 | 0.8334 | 1.3562 | 0.0219 | 0.0070 | 2.5317 | 1.3000 | C8 → C10 | 0.8333 | P |
| C10 | 11.5333 / 12.1833 | 1.9898 | 2.1083 | 0.5200 | 0.9018 | 1.3562 | 0.0274 | 0.0075 | 3.2736 | 1.2000 | C9 → C11 | 1.0599 | P |
| C11 | 12.7833 / 13.3833 | 2.9898 | 3.0486 | 0.4673 | 1.2996 | 1.3562 | 0.0347 | 0.0078 | 4.2735 | 1.2833 | C10 → C12 | 1.2996 | P |
| C12 | 14.0333 / 14.6667 | 3.9771 | 4.0711 | 0.5067 | 1.2898 | 1.3562 | 0.0423 | 0.0081 | 5.2608 | 1.2167 | C11 → C13 | 1.2900 | P |
| C13 | 15.2833 / 15.8833 | 4.9771 | 5.0420 | 0.4800 | 1.2765 | 1.3562 | 0.0503 | 0.0084 | 6.2608 | 1.3000 | C12 → C14 | 1.2766 | P |
| C14 | 16.5333 / 17.1833 | 5.9771 | 6.0773 | 0.5171 | 0.8877 | 1.3562 | 0.0584 | 0.0086 | 7.2608 | 1.1000 | C13 → C15 | 0.9610 | P |
| C15 | 17.7833 / 18.2833 | 6.9742 | 6.9562 | 0.4000 | 0.6136 | 1.3562 | 0.0667 | 0.0088 | 8.2578 | 1.1000 | C14 → C16 | 0.6137 | P |
| C16 | 18.7833 / 19.3833 | 7.7742 | 7.8353 | 0.4800 | 0.6135 | 1.3562 | 0.0739 | 0.0092 | 9.0578 | 1.2833 | C15 → C17 | 0.6137 | P |
| C17 | 20.0333 / 20.6667 | 8.7742 | 8.8582 | 0.5037 | 0.6163 | 1.2603 | 0.0815 | 0.0064 | 9.9611 | 1.2333 | C16 → C18 | 0.6618 | P |
| C18 | 21.2833 / 21.9000 | 9.7713 | 9.7838 | 0.4328 | 0.9365 | 0.5369 | 0.1022 | 0.2951 | 10.2085 | 1.1500 | C17 → N/A | 1.1082 | P |
| C19 | 22.5333 / N/A | 10.4501 | N/A | N/A | 1.1478 | 0.0000 | 0.0937 | 0.1744 | 10.2259 | N/A | C18 → N/A | N/A | STOP |

### ON_REPEAT_00

| Ck | obs / apply s | obs centre m | B edge m | obs→B travel m | raw edge m | arc m | max lateral m | final yaw wrt hallway rad | endpoint rear m | lifetime s | OLD → next | actual prefix edge m | class |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| C0 | 0.7833 / 1.2167 | 5.0000 | 4.5794 | 0.0000 | 1.3465 | 1.3562 | 0.0004 | 0.0007 | -3.7161 | 0.9667 | N/A → C1 | 1.3496 | S |
| C1 | 1.7833 / 2.1833 | 4.6367 | 3.8980 | 0.3200 | 1.2888 | 1.3562 | 0.0008 | 0.0010 | -3.3528 | 0.9833 | C0 → C2 | 1.3460 | S |
| C2 | 2.7833 / 3.1667 | 3.8367 | 3.1146 | 0.3067 | 1.0283 | 1.3562 | 0.0016 | 0.0013 | -2.5528 | 1.0167 | C1 → C3 | 1.1322 | S |
| C3 | 3.7833 / 4.1833 | 3.0367 | 2.3067 | 0.3200 | 1.0189 | 1.3549 | 0.0024 | 0.0006 | -1.7555 | 0.9833 | C2 → C4 | 1.0188 | S |
| C4 | 4.7833 / 5.1667 | 2.2367 | 1.5228 | 0.3067 | 0.6792 | 1.0558 | 0.0046 | 0.0225 | -1.3646 | 1.0333 | C3 → C5 | 0.6979 | S |
| C5 | 5.7833 / 6.2000 | 1.4367 | 0.6979 | 0.3316 | 0.3721 | 0.5109 | 0.0041 | -0.0059 | -1.0061 | 0.8500 | C4 → N/A | 0.3552 | S |
| C6 | 6.7833 / N/A | 0.7652 | N/A | N/A | 0.3581 | 0.0000 | 0.0013 | -0.0054 | -0.9890 | N/A | C5 → N/A | N/A | STOP |

### OFF_REPEAT_01

| Ck | obs / apply s | obs centre m | B edge m | obs→B travel m | raw edge m | arc m | max lateral m | final yaw wrt hallway rad | endpoint rear m | lifetime s | OLD → next | actual prefix edge m | class |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| C0 | 0.7833 / 1.2167 | 5.0000 | 4.5794 | 0.0000 | 1.3465 | 1.3562 | 0.0004 | 0.0007 | -3.7161 | 0.9667 | N/A → C1 | 1.3496 | S |
| C1 | 1.7833 / 2.1833 | 4.6333 | 3.8947 | 0.3200 | 1.2882 | 1.3562 | 0.0008 | 0.0010 | -3.3495 | 1.0000 | C0 → C2 | 1.3460 | S |
| C2 | 2.7833 / 3.1833 | 3.8333 | 3.0980 | 0.3200 | 1.0279 | 1.3562 | 0.0016 | 0.0013 | -2.5495 | 0.9833 | C1 → C3 | 1.1361 | S |
| C3 | 3.7833 / 4.1667 | 3.0333 | 2.3166 | 0.3067 | 1.0191 | 1.3562 | 0.0027 | 0.0017 | -1.7495 | 1.0167 | C2 → C4 | 1.0190 | S |
| C4 | 4.7833 / 5.1833 | 2.2333 | 1.5062 | 0.3200 | 0.3188 | 1.3562 | 0.0043 | 0.0021 | -0.9495 | 1.0000 | C3 → C5 | 1.0440 | H |
| C5 | 5.7833 / 6.1833 | 1.4333 | 0.7062 | 0.3200 | -0.2000 | 1.3562 | 0.0061 | 0.0025 | -0.1495 | 1.0000 | C4 → C6 | 1.3395 | U |
| C6 | 6.7833 / 7.1833 | 0.6334 | -0.0924 | 0.3200 | -0.2000 | 1.3562 | 0.0082 | 0.0029 | 0.6505 | 1.0000 | C5 → C7 | 0.9580 | U |
| C7 | 7.7833 / 8.1833 | 0.1668 | 0.0889 | 0.3200 | -0.0541 | 0.5607 | 0.0106 | 0.0090 | 0.6757 | 1.0167 | C6 → C8 | 0.8492 | U |
| C8 | 8.7833 / 9.2000 | 0.8986 | 0.5249 | 0.0244 | 0.6501 | 0.5109 | 0.0138 | 0.0057 | 1.3291 | 0.9833 | C7 → C9 | 0.8413 | P |
| C9 | 9.7833 / 10.1833 | 1.2797 | 1.1740 | 0.2926 | 0.8331 | 0.5109 | 0.0163 | 0.0056 | 1.7101 | 1.0167 | C8 → C10 | 0.8329 | P |
| C10 | 10.7833 / 11.2000 | 1.9480 | 1.5661 | 0.0163 | 0.8892 | 0.5109 | 0.0203 | 0.0055 | 2.3785 | 0.9833 | C9 → C11 | 0.8572 | P |
| C11 | 11.7833 / 12.1833 | 2.3176 | 2.2154 | 0.3002 | 1.0375 | 1.3562 | 0.0277 | 0.0068 | 3.6014 | 1.0000 | C10 → C12 | 1.1209 | P |
| C12 | 12.7833 / 13.1833 | 3.0893 | 3.0012 | 0.3200 | 1.3010 | 1.3562 | 0.0331 | 0.0072 | 4.3730 | 1.2000 | C11 → C13 | 1.3020 | P |
| C13 | 13.7833 / 14.3833 | 3.8893 | 3.9572 | 0.4800 | 1.2933 | 1.3562 | 0.0390 | 0.0076 | 5.1730 | 1.2833 | C12 → C14 | 1.2934 | P |
| C14 | 15.0333 / 15.6667 | 4.8893 | 4.9621 | 0.4875 | 1.2797 | 1.3562 | 0.0465 | 0.0079 | 6.1730 | 1.2167 | C13 → C15 | 1.2797 | P |
| C15 | 16.2833 / 16.8833 | 5.8685 | 5.9251 | 0.4732 | 0.9623 | 1.3562 | 0.0540 | 0.0081 | 7.1521 | 1.3000 | C14 → C16 | 0.9584 | P |
| C16 | 17.5333 / 18.1833 | 6.8616 | 6.9638 | 0.5200 | 0.6176 | 1.3562 | 0.0619 | 0.0083 | 8.1453 | 1.2000 | C15 → C17 | 0.6175 | P |
| C17 | 18.7833 / 19.3833 | 7.8616 | 7.9228 | 0.4800 | 0.6175 | 1.3549 | 0.0697 | 0.0075 | 9.1426 | 1.3000 | C16 → C18 | 0.6204 | P |
| C18 | 20.0333 / 20.6833 | 8.8616 | 8.9580 | 0.5159 | 0.6217 | 0.3028 | 0.0708 | 0.0128 | 9.1045 | 1.2000 | C17 → C19 | 0.6996 | P |
| C19 | 21.2833 / 21.8833 | 9.4943 | 9.0746 | 0.0000 | 0.8021 | 0.5369 | 0.0976 | 0.2991 | 9.9314 | 1.1667 | C18 → N/A | 0.7449 | P |
| C20 | 22.5333 / N/A | 9.9009 | N/A | N/A | 0.9265 | 0.0000 | 0.0774 | 0.0251 | 9.6769 | N/A | C19 → N/A | N/A | STOP |

### ON_REPEAT_01

| Ck | obs / apply s | obs centre m | B edge m | obs→B travel m | raw edge m | arc m | max lateral m | final yaw wrt hallway rad | endpoint rear m | lifetime s | OLD → next | actual prefix edge m | class |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| C0 | 0.7833 / 1.2167 | 5.0000 | 4.5794 | 0.0000 | 1.3465 | 1.3562 | 0.0004 | 0.0007 | -3.7161 | 0.9500 | N/A → C1 | 1.3496 | S |
| C1 | 1.7833 / 2.1667 | 4.6333 | 3.9080 | 0.3067 | 1.2882 | 1.3562 | 0.0008 | 0.0010 | -3.3495 | 1.0167 | C0 → C2 | 1.3460 | S |
| C2 | 2.7833 / 3.1833 | 3.8333 | 3.0980 | 0.3200 | 1.0279 | 1.3562 | 0.0016 | 0.0013 | -2.5495 | 1.0000 | C1 → C3 | 1.1309 | S |
| C3 | 3.7833 / 4.1833 | 3.0333 | 2.3034 | 0.3200 | 1.0189 | 1.3549 | 0.0025 | 0.0007 | -1.7521 | 1.0000 | C2 → C4 | 1.0188 | S |
| C4 | 4.7833 / 5.1833 | 2.2333 | 1.5062 | 0.3200 | 0.6759 | 1.0558 | 0.0045 | 0.0225 | -1.3612 | 1.0000 | C3 → C5 | 0.7078 | S |
| C5 | 5.7833 / 6.1833 | 1.4333 | 0.7078 | 0.3184 | 0.3688 | 0.5109 | 0.0039 | -0.0058 | -1.0028 | 0.8667 | C4 → N/A | 0.3548 | S |
| C6 | 6.7833 / N/A | 0.7653 | N/A | N/A | 0.3581 | 0.0000 | 0.0013 | -0.0054 | -0.9890 | N/A | C5 → N/A | N/A | STOP |

## Timing, actual calls, and limitations

The first ON C5 FRESH was captured at t=5.783333635 s and applied at t=6.200000323 s.
During that interval actual OLD execution travelled .331616 m. Its B was
[19.203817257,23.745368153,-1.563185566], cart edge clearance .697902536 m, physical
u-minus [0.793132279,0.027432911]; recorded controller memory happens to match
this value. Original FRESH remains anchored to the earlier observation. The
second repeat's C5 B is [19.203921910,23.755253835,-1.563651023], travel .318397 m,
cart edge .707788771 m. These are genuine moving boundaries, but neither FRESH
is a bypass candidate.

| Episode | whole RTF | request-local RTF range | maximum loop stall s | terminal RTT sum s | submitted MPC / same-episode saved results |
|---|---:|---|---:|---:|---|
| OFF_REPEAT_00 | 0.991688 | 0.4497–1.3966 | 0.324587 | 8.996025 | 217 / 216 |
| ON_REPEAT_00 | 0.967177 | 1.3542–1.3884 | 0.254805 | 2.138234 | 60 / 59 |
| OFF_REPEAT_01 | 0.991832 | 0.8085–2.2595 | 0.322056 | 8.884610 | 220 / 219 |
| ON_REPEAT_01 | 0.967176 | 1.3426–1.3739 | 0.257387 | 2.106017 | 60 / 59 |

**All14 ON prediction request-local RTF checks exceed1.2.** Whole-episode RTF near
.967 does not repair this failure. Camera/readback stalls and absolute-time
catch-up remain visible in saved streams; no hidden slowdown compensation or
post-outcome retry was added. ON maximum loop stalls are .2548/.2574 s, just over
the prior .25 s gate. Thus this is actual recorded online execution with a pacing
limitation, not a timing-qualified handoff-source success or real-world latency
validation. The moving geometry and STOP sequence are observable; stronger
real-time generalization is unsupported.

Actual primary calls: **55 terminal LightNav predictions**,183 buffer-only image
requests. One separate official server startup warmup; technical preflight0
scientific predictions/0 MPC solves. **557 accepted official MPC submissions**;
556 unique solve-result records are preserved across episode boundaries, with
2.342283712 s summed saved solver time. The final
`ON_REPEAT_01_solve_000059` has no persisted completion record at shutdown;
its duration/status are N/A and total solver time is a recorded lower bound.
Every applied command has a validated saved result. Accepted submission coverage
must not be confused with result-record coverage. End-to-end collection including
Isaac startup is83.422505 s; terminal client RTT sum22.124886393 s. These overlap
execution and are not added to simulation time. `aggregate/call_counts.json`
recomputes unique IDs and costs.

No GP/rigid/reconciliation experimental solves. Focused tests call no real
LightNav or MPC. No trajectory correction, bypass policy, raw trimming, or
post-STOP execution. A raw-safe short path is not an optimization-ready bypass.
The source-bundle manifest explicitly reports zero candidates and no moving
bypass handoff source. Existing SOURCE04/05 failures remain unchanged.

## Saved validation and presentation

Authoritative `validation.json` passes every episode's original history/wire
JPEG/observation/reference/command/state checks, all same-render visibility
records, each direct guard query and applied exact-integration prefix, saved
analysis/classification, plotted values and source hashes. Scientific timing
limitations remain in the report and do not become corruption errors.

Post-run reporting corrections only: a NumPy boolean in OFF-match metadata was
serialized as text by the historical generic JSON helper; the new reporting
wrapper now casts it explicitly. The first validator log is retained. The world
legend was moved outside the data and GUI RGB panel docked left. Call accounting
now includes late solve results stored in the following episode. These did not
change raw records, classification thresholds, collector/core, or any scientific
prediction. All frozen reporting code and earlier derived reports are preserved
under `reporting_development_01/`; `reporting_revisions.json` records exact old/new
hashes. Strict pre-primary verification is still enforced for collection; only
explicit saved-report verification admits these listed reporting revisions.

Focused new + strongest relevant online/SOURCE04/05 regressions: **276 passed**.
Compileall and diff whitespace checks pass. An unnecessarily broad all-repository
test invocation was interrupted during unrelated diagnostic fixtures (975 passed,
18 skipped); it is not reported as a completed full-suite run. Initial sandbox
socket-test failures were rerun successfully with local socket access. No shell
launcher was added/changed.

Static review: `review/index.html` and `review_bundle.zip` in the primary run;
all55 chunks have original RGB/world contact sheets and numeric/source sidecars.
Full per-episode world overlays, clearance/lateral/arc vs distance, classification
and OFF/ON longitudinal panels are included. No missing bypass figures are
fabricated; first-onset/bypass remain N/A.

Interactive Isaac GUI (saved-only) was run and validated for every dropdown
entry, chunk selection and playback; RGB stays unmodified. Final screenshot and
runtime evidence directory:
`data/robotless_join_online_02/replay_20260923_final/`.
The verification uses GUI callbacks and the actual renderer, not a claim of
OS mouse automation. The GUI is left open at ON_REPEAT_00 C5. Six final screenshots
include both ON STOP frames and last-applied C5 frames. The earlier GUI layout
verification remains in `replay_20260923/`.

```bash
# Clean Isaac launch prefix used for preflight, collection and replay:
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 \
  /home/gpuadmin/isaacsim/python.sh scripts/isaac/join_online02_replay.py \
  --run data/robotless_join_online_02/primary_20260923T001500Z \
  --output data/robotless_join_online_02/replay_20260923_final --verify
# Use a NEW output directory for a later saved replay; overwrite is refused.
```

The bounded result is safe forward extent reduction and native STOP before the
cart, without observed lateral bypass. The largest remaining uncertainty is
whether this behavior persists under request-local real-time pacing; no further
scientific episode or reconciliation is implemented here.
