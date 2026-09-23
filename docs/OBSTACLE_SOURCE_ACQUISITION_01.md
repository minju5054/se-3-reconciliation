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

## Completed outcome

**PACING_BLOCKED_ONLINE_ACQUISITION.** Phase A supplies two qualifying safe
local-avoidance prediction pairs, but the separately measured collector fails
request-local pacing. Therefore **zero Phase B repetitions were attempted** and
**no genuine obstacle-induced moving-B representative source was obtained**.
This is not an online failure trial or a reconciliation result.

Pushed execution freeze: `9190c70f3b305eab092b853d5aa478f3da5e32a1`.
Authoritative run: `data/obstacle_source_acquisition_01/primary_20260923T073600Z/`.
All eight declared scientific terminal requests completed once. The official
process environment confirms temperature0/top_p1/top_k0/traj_top1=0 and exact
historical scientific server argv/checkpoint identity. Source/core/geometry and
the two unrelated user edits retain their hashes. No controller, pacing,
selector, external LightNav, checkpoint or evaluation threshold was changed.

### Phase 0: actual OFF technical episode

Same native collector, live RGB4Hz, MPC10Hz, exact60Hz held-command integration.
Four real technical predictions and three post-bootstrap handoffs completed.
All physical guard checks pass; minimum curve-bounded actual edge clearance
1.064697433 m, zero safety abort or collision. These OFF technical handoffs are
not obstacle-response sources.

| Request | Client RTT s | Request-local RTF | Timing gate |
|---|---:|---:|---|
| C0 bootstrap | .253778107 | 1.357520585 | outside retained bound; context |
| C1 | .293739212 | 1.378365284 | FAIL |
| C2 | .302946885 | 1.393438269 | FAIL |
| C3 | .308652606 | 1.387401764 | FAIL |

Maximum loop stall .216155667 s passes <=.25 s. Whole-episode RTF .999249833
does not replace request-local RTF. The local intervals include faster-than-real
catch-up integration within the absolute-paced acquisition loop. This is a
saved timing observation, not a new estimate of model latency or a change in
integration dt. No runtime engineering correction was introduced in this bounded
protocol; the measured current infrastructure is not qualified for Phase B.
There was no technical or scientific retry after these results.

The episode saved256 states/255 exact integration intervals,17 new live RGBs,
13 buffer-only requests,31 accepted MPC submissions and30 saved results.
`PACING_OFF_00_solve_000031` was accepted near termination without a saved
completion; no command from that solve was applied. Do not report31 completed
solves. Recorded MPC solve wall time .154098410 s is a lower bound, not complete
compute for all submissions. Technical terminal RTT sum1.159116810 s. The
collector's measured post-SimulationApp-startup wall span is14.618949200 s;
Isaac initialization time is separate. No cart reveal occurred in this episode.

### Phase A: all four fixed pairs

The cart transform is identical in all candidates; observation poses differ as
listed above. Included terminal cart pixels are2976/3765/4821/6810; OFF counts0.
Source04 authoritative validation, unchanged bank SHA256 and same-camera/mask
state checks pass. Seven preceding OFF frame bytes match exactly within each
pair. Independent sessions share task/instruction/settings and terminal pose;
only the terminal JPEG encodes cart presence. No new Phase A render or MPC.

| Candidate | Cart distance m | OFF actual edge m | OFF hypothetical cart-ON edge m | ON whole edge m | ON signed final lateral m | ON XY arc m | Qualifies |
|---|---:|---:|---:|---:|---:|---:|---|
| POSE10 | 1.983029 | 1.057500 | +.074073 | +.097559 | +.388397 | 1.354194 | No: OFF not obstructed |
| POSE11 | 1.783029 | 1.117514 | −.125215 | +.205296 | +.584514 | 1.348502 | Yes |
| POSE12 | 1.583029 | 1.203975 | −.200000 | +.100168 | +.644361 | 1.352735 | Yes |
| POSE13 | 1.383029 | 1.125477 | −.200000 | +.725907 | +.000002 | .100066 | No: short/nonturning, insufficient future |

All eight responses are non-STOP and contain10 rows in this runtime; code is
generic N. Negative hypothetical OFF clearance means the same returned OFF
path would overlap the newly present cart; no physical collision was executed.
The −.20m unsigned-distance-minus-radius value is not a penetration estimate.
All four ON paths pass the unchanged whole-polyline footprint requirement;
safety alone is insufficient for source qualification.

| Candidate | OFF max lateral m / yaw deg | ON max yaw deg | Interior mismatch m | Reliable tangent difference deg | ON forward progress m | ON end relative to cart front m | Observation-proxy rows / arc m |
|---|---:|---:|---:|---:|---:|---:|---:|
| POSE10 | .000004 / .000901 | 15.002604 | .388394 | 14.872723 | 1.308500 | −.296320 | 10 / 1.354194 |
| POSE11 | .000004 / .000901 | 29.919087 | .608727 | 19.603996 | 1.210529 | −.340224 | 10 / 1.348502 |
| POSE12 | .024627 / 6.980638 | 30.224079 | .721359 | 28.712400 | 1.188372 | −.162205 | 10 / 1.352735 |
| POSE13 | .024627 / 6.980638 | .037865 | N/A | N/A | .099836 | −.900609 | 10 / .100066 |

POSE13's endpoint-projection differences are not misreported as lateral response.
All candidates end before the cart front plane, so **no complete bypass is
observed**. POSE11/12 nonetheless meet the predeclared local-interaction range,
positive progress, full safety and future-proxy gates. Their ~1.35m arc is not
mere truncation compared with OFF. They are raw spatial futures, not proved
motion-feasible transitions from a moving B.

Frozen first-pass selection is **POSE11**, not the largest mismatch/most
attractive plot. Gates A–F all pass on POSE11/12. POSE10 fails B only. POSE13
fails D/E/F. All four were executed as declared before selection; no fifth
candidate, changed cart, changed side, added instruction or threshold was used.

**Observed side caveat:** POSE10/11/12 ON arrays turn local +y, which is LEFT.
The exact text says "pass the supply cart on your right". We neither rewrite
this as a confirmed robot-right bypass nor change the instruction afterward.
A cart-on-right interpretation and robot-pass-right interpretation are not
resolved by the token trace. Side compliance was not an extra frozen gate;
it is not added post hoc to remove otherwise qualifying geometry. Both original
side probes were clearance-valid, and actual ON whole-path safety is checked
against the complete Hospital plus cart mesh.

APOS OFF10/11=[245,155], OFF12/13=[255,155], all unclamped. ON10/11/12 are
[235,165]/[175,185]/[155,185], unclamped; ON13=[245,265], clamped and censored.
All ON OPOS are not_visible. These observable fields are preserved but do not
prove internal obstacle recognition, correct intent grounding or decoder cause.
No sham repeats were declared in this bank; same-input stochastic variability
was not newly measured despite greedy generation.

### Phase B and source availability

| Frozen repetition | Executed | OLD/FRESH, B, u_minus, memory, travel, RTF/stall | Reason |
|---|---|---|---|
| 00 | No | N/A | Phase0 request-local timing gate fails |
| 01 | No | N/A | Phase0 request-local timing gate fails |

There is no selected online configuration freeze or online source bundle.
`paired_candidate_bundle/manifest.json` seals POSE11's exact OFF/ON inputs,
responses, raw/local/world files, scene and hashes only. It explicitly has
`online_optimization_ready=false`, B/u_minus/controller_memory=null. It must
not be substituted for a genuine moving handoff. The technical OFF episode's B
must not be attached to this independently predicted ON path.

### Calls, validation and artifacts

- Scientific Phase A: **8 terminal predictions,56 buffer-only requests,8 saved
  responses**,36.962635267 s process-loop wall; terminal RTT sum2.159152857 s.
- Phase0 technical: **4 terminal predictions,13 buffers,31 accepted MPC
  submissions/30 saved results**, as detailed above.
- Server: **one separately logged synthetic startup warmup**, not source evidence.
- PhaseB:0 requests/episodes. Scientific MPC0; optimizer/GP/rigid/reconciliation0.
  Regression tests call no real model/MPC/optimizer.
- Prefreeze focused164 pass; additional242 related pass. Final combined relevant
  suite **480 passed**,6.74 s; compileall and diff checks pass.
- Original SOURCE04 saved validator passes. Primary `validation.json` and
  authoritative additive `validation_final.json` pass: all source hashes,
  eight independent sessions/wire bytes, seven empty buffers/terminal instruction,
  observation anchors, whole geometry, deterministic selection, technical
  integration/guard, pacing, CSV/JSON/PNG numerical parity and ZIP bytes.
  Validation makes zero new inference/solver calls. Timing failure remains failure.

Review: `index_final.html`; four `review/POSE*.png` pair overlays with exact
numeric/source JSONs; `review_bundle_complete.zip`. Source-only primary tables:
`aggregate/phaseA.csv`, `aggregate/analysis.json`, `aggregate/completion.json`.
Small Git-readable summary: `docs/results/obstacle_source_acquisition_01.json`.
Raw scientific data: `phaseA/POSE*/rgb`, `requests`, `chunks/terminal/raw_local.npy`
and `response.json`; derived observation-anchored `world.npy` is separate.
Technical online streams: `phase0/episodes/PACING_OFF_00/`. Unavailable online
repetitions: `phaseB_availability.json`. Original RGB bank is referenced by hash,
not re-rendered or edited. No RGB/array/ZIP/environment is committed.

Additive completion command after frozen report/validator:

```bash
.venv/bin/python scripts/finalize_obstacle_source_acquisition.py --run data/obstacle_source_acquisition_01/primary_20260923T073600Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q tests/test_join_*.py tests/test_gp_se2_join01*.py tests/test_online_*.py tests/test_robotless_online*.py tests/test_se2.py tests/test_obstacle_source_acquisition.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

### Repository-confirmed facts

One geometry-selected exact instruction and four immutable paired conditions
produced two all-pass local raw source candidates. POSE11 is first. Current
actual collector fails request-local pacing; no online sudden-reveal acquisition
ran. Source/core/thresholds remain unchanged; all failures are retained.

### Research interpretation

A safe, substantial, obstacle-conditioned local response is available under
these controlled inputs; raw generation is no longer an absolute blocker in
this tested bank. It is not enough to establish the requested asynchronous
moving-B source. The single remaining uncertainty is whether this paired
response survives **genuine sudden reveal with a timing-qualified moving B**.
No timing fix, online retry or next-stage optimization is implemented here.

### Not demonstrated

Reconciliation benefit, complete obstacle avoidance, graph superiority,
closed-loop navigation improvement, real-robot feasibility or general LightNav
obstacle capability. The selected record is a source-only candidate, not the
requested final online representative source.
