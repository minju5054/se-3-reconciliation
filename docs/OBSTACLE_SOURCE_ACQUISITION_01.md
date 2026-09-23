# OBSTACLE-SOURCE-ACQUISITION-01

Question: can a suddenly revealed cart cause a genuine straight OLD to change
into a whole-path-safe, task-progressing turning FRESH, while OLD continues to a
moving application boundary B? This task ends at source acquisition. No GP,
rigid, graph, splice, correspondence search or method comparison is authorized.

Starting/fetched main: `1cfffed886357d082a5a7824cddadc28a916173c`. The two
unrelated Stage0 config edits are preserved and excluded. Existing JOIN and
SOURCE/ONLINE results remain immutable; no failed historical source is promoted.

## Pre-prediction protocol

Config: `configs/obstacle_source_acquisition_01.yaml`. Primary preparation:
`data/obstacle_source_acquisition_01/primary_20260923T073600Z/`.
The earlier `primary_20260923T073500Z` contains only a failed preparation: YAML
parsed unquoted OFF/ON as booleans. No session/model/MPC call occurred. Quoting
the enum and a regression test corrected this before any prediction. No raw
record was overwritten; no condition/input/gate changed after model output.

A new renderer is unnecessary for Phase A. Reuse the exact immutable BRIGHT
OFF/ON bank from SOURCE04 `persistence_20260922T050237Z`, referring to original
render files in `persistence_20260922T044703Z`. Its final validator is rerun,
and all RGB/mask/depth bytes and scene/input provenance are authenticated.
This is **COUNTERFACTUAL MOVING-POSE HISTORY**, not a new genuine online event.

All four candidate poses are actual chronological camera poses in that bank.
They were chosen by distance/available saved pose geometry, not new output:

| Order | Saved bank index | Observation x,y,yaw (m,m,rad) | Cart longitudinal distance m |
|---|---:|---|---:|
| POSE10 | 10 | 19.2027486292,24.6233345800,-1.5689905996 | 1.9830289291 |
| POSE11 | 11 | 19.2031207316,24.4233349158,-1.5689742328 | 1.7830289326 |
| POSE12 | 12 | 19.2034697495,24.2233352102,-1.5691535799 | 1.5830289462 |
| POSE13 | 13 | 19.2037830537,24.0233354453,-1.5691825214 | 1.3830289803 |

Same natural supply cart `/World/Hospital/SM_SupplyCart_02a7`, runtime
`/World/JOINSource02Obstacle`, declared centre `[19.2071611672,22.6403105601]`m.
Exact actual USD wrapper matrix and 8024 triangles are unchanged. The recorded
actual mesh centre differs slightly from the requested centre; actual triangles
are the collision oracle. Cart extents along hallway are [-.232527154,.223749934]m,
lateral [-.425365100,.417002892]m relative to the declared centre. No bounding
box replaces mesh geometry. Camera, materials, authored Hospital and BRIGHT
RectLight are unchanged. BRIGHT: position [19.2,24,2.6]m, 4x10m, intensity1000,
exposure0, white, normalize=false; authored lights unchanged. Camera480x270,
HFOV112.2deg, .09m forward/.65m up, JPEG95. No image postprocessing.

Each pair contains H=8 consecutive saved moving poses ending at its candidate:
seven identical OFF history JPEGs and terminal OFF vs ON. Between candidates
pose/history lineage changes; only the within-candidate contrast isolates
terminal obstacle presence. H8 is explicit, not a padding/reset claim. No future
frames, repeated JPEG padding or persistent-cart history variant. Every branch
uses an independent official session: seven empty-instruction buffer requests,
then one terminal prediction. Historical capture clock values are preserved;
current request clocks are separate, with cross-boot age N/A. Official sampler
uses delivered indices; LightNav rows have no intrinsic time.

Use existing direct mesh side probes. Both pass .05m edge clearance. RIGHT's
minimum is .0737180275m versus LEFT .0600000000m. Choose larger checked minimum,
exact tie RIGHT. This is a geometric instruction choice, not a generated path
or full free-space topology proof. Local +y is LEFT, +x forward, yaw CCW.

Only instruction, identical OFF/ON and in conditional online acquisition:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

No metric, speed, angle, waypoint or alternate wording. Official source
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`, checkpoint revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, VLN, greedy temperature0/top_p1/
top_k0/traj_top1=0. Actual process argv/environment and checkpoint files must
match pinned SOURCE04 before requests. MPC SHA256 remains
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.

## Phase 0 technical qualification

One separate actual OFF episode starts at the unchanged ONLINE03 5m pose.
Same new instruction, live RGB4Hz, official MPC10Hz, exact logical integration
60Hz, actual native history/request/install/application. Maximum3 handoffs after
bootstrap,5s active, .10s postroll: at most4 technical predictions. No cart reveal,
no representative source selection from this episode. Existing abort-only guard
is active with cart absent. No pacing patch or controller change is included in
this initial protocol. Its purpose is to measure the present infrastructure;
a failed gate blocks Phase B and never prevents timing-independent Phase A.

Require >=3 complete post-bootstrap request windows, each reconstructed from
saved states within actual client request/receipt host timestamps, RTF[.8,1.2],
and maximum episode loop stall<=.25s. Whole-episode RTF cannot substitute.
One short technical episode is bounded qualification, not a worst-case guarantee.
Technical predictions/MPC calls and server warmup are counted separately from
Phase A. No automatic retry or hidden engineering change after outcomes.

## Phase A gates and execution

Run all eight terminal sessions once in order: POSE10 OFF/ON, POSE11 OFF/ON,
POSE12 OFF/ON, POSE13 OFF/ON. No sham/extra prompt/side/candidate retry. Four
premodel pair/camera/mask/clearance gates pass. Before inference, preserve each
observation clearance and straight observation-to-cart geometry design probe;
that straight probe is explicitly not model output. Actual OLD/OFF obstruction
is determined from the genuine OFF output, not assumed from the probe.

A: OFF non-STOP, whole raw safe, positive returned hallway progress, and roughly
straight: max local lateral<.10m, max wrapped yaw<20deg, all reliable >=.02m
segment tangents<20deg. This rejects text-only steering.

B: same OFF path is clearance-invalid when actual cart occupancy is added.
C: entire ON raw polyline passes unchanged .20m radius/.05m edge clearance,
workspace and uncertainty. No observation connector or suffix trimming.
D: existing influence-region bidirectional projection reports interior lateral
separation>=.10m OR reliable tangent/yaw>=20deg, chord>=.02m. Finite endpoint
gaps do not count. Influence is cart front-.50m through rear+.50m.
E: ON non-STOP, positive returned forward progress, raw arc>=.60m and reaches
the front-.50m interaction region. Short truncation or pure yaw is insufficient.
F: original weighted nearest at observation leaves>=4 raw rows and>=.60m XY arc.
This is an observation-based future proxy, not a guarantee at a nonexistent B.

All flags and continuous numbers remain in the ledger. Select the first all-pass
candidate in the fixed order only after all four pairs finish. Zero all-pass
means `LIGHTNAV_OBSTACLE_SOURCE_QUALIFICATION_FAILED` and no Phase B. Complete
bypass is a descriptor, not a gate. A local avoidance source is not traversal.

## Conditional Phase B (not authorized to bypass either gate)

Only Phase A selection AND Phase0 qualified permit a separate selected-source
freeze/commit/push. Two independent live repetitions, both retained. Initial
pose = selected observation translated .40m backwards along hallway, yaw
unchanged. Reveal at the first scheduled4Hz capture at/beyond its longitudinal
plane with OLD active. Prior queued OFF frames stay buffer-only; first eligible
post-reveal frame is FRESH. Same session/instruction, one post-reveal prediction,
maximum4s active and .10s post-switch postroll. Actual pose may differ from bank;
no output matching, teleport, reset or second FRESH retry.

All Phase A behavioral gates are recomputed on genuine OLD/FRESH. Actual B must
have physical v_minus>.20m/s, obs-to-B traveled arc>=.02m, original clearance,
>=4 relevant rows and remaining raw arc>=.60m. RTF/stall gates remain exact.
Physical command and controller memory (including before/after first FRESH
solve/application phase) are separate fields. B is first actual FRESH command
application, never receipt/install pose. FRESH world=T_world_observation*T_local.
Oracle guard only aborts before unsafe command application; it does not repair
raw FRESH or establish future safety. No successful Phase A replay alone creates
an optimization-ready moving handoff. First qualifying repeat is representative;
second repeat still runs because both are frozen. Then stop.

## Reproduction

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_obstacle_source_acquisition.py tests/test_join_source05.py tests/test_join_source04.py tests/test_join_online02.py tests/test_online_history.py tests/test_online_mpc_adapter.py tests/test_online_switch.py tests/test_online_ipc.py
.venv/bin/python scripts/run_obstacle_source_acquisition.py --run "$RUN" --mode init
.venv/bin/python scripts/run_obstacle_source_acquisition.py --run "$RUN" --mode freeze
# Review, commit and normal push before any new model request.
.venv/bin/python scripts/run_obstacle_source_acquisition.py --run "$RUN" --mode start
# Unset external ROS/CUDA/Python library overrides using the existing launch convention.
/home/gpuadmin/isaacsim/python.sh scripts/isaac/obstacle_source_pacing.py --run "$RUN"
.venv/bin/python scripts/run_obstacle_source_acquisition.py --run "$RUN" --mode pacing-result
.venv/bin/python scripts/run_obstacle_source_acquisition.py --run "$RUN" --mode execute
.venv/bin/python scripts/run_obstacle_source_acquisition.py --run "$RUN" --mode evaluate
.venv/bin/python scripts/run_obstacle_source_acquisition.py --run "$RUN" --mode stop
.venv/bin/python scripts/report_obstacle_source_acquisition.py --run "$RUN"
.venv/bin/python scripts/report_obstacle_source_acquisition.py --run "$RUN" --validate
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

All writers use exclusive creation. Validator repeats saved source hashes,
branch input/wire/session/instruction parity, world anchoring, original geometry,
qualification/selection, and technical state integration/guard. No new solver or
model call. Figures bind numerical/hash sidecars. Scientific rejection is not
artifact corruption. Results, actual counts, timing and remaining uncertainty
will be appended after this fixed schedule.
