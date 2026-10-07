# SUCCESSIVE_NATIVE_SOURCE_ACQUISITION_02

## Repository-confirmed result

**SUCCESSIVE_SOURCE_MAX_REACHED — DEVELOPMENT SOURCE.** CANDIDATE_01 generated
and actually applied all 12 consecutive native chunks C0–C11. There are 11
applied-to-applied handoffs, including 10 after reveal (C1→C2 through C10→C11).
No rejected chunk, STOP, safety abort, skipped identity or later reset occurred.
The episode continued past six/eight and ended at the frozen 12-request limit.
CANDIDATE_02/03 were not run: the first candidate qualifies.

All frozen scientific provenance, timing, controller and geometry gates pass.
A post-freeze saved-only accounting addendum was needed for a shutdown-drained
submission; the original outer validator's TECHNICAL_EXECUTION_BLOCKED record is
preserved, with the distinction explained below. No scientific rerun or tuning.

- Starting HEAD: `e39d465762e0680d0c806f58079127b313312855`.
- Audit-fix commit: `3c940abe1fbaea9081f1eb1f985ccf774cca1734`.
- Pushed scientific freeze: `ee330244d867fad3749c60b6a5f1630b1471ec2a`.
- Final result commit: the commit containing this report, identified by subject
  `Report twelve genuinely applied native chunks and complete saved accounting`;
  its SHA is given in the completion response (not a self-referential file hash).
- [Frozen protocol](SUCCESSIVE_NATIVE_SOURCE_ACQUISITION_02_PROTOCOL.md),
  [config](../configs/successive_native_source_acquisition_02.yaml),
  [candidate manifest](../results/successive_native_source_acquisition_02/candidate_manifest.json),
  [search outcome](../results/successive_native_source_acquisition_02/search_summary.json).

## Candidate geometry and bounded search

`T_candidate=T_POSE11 Trans(-d,0,0)`, with yaw unchanged. Coordinates are world
XY metres, +Z up, CCW yaw radians; local +x forward and +y left. Same Hospital,
cart asset/transform, BRIGHT lighting, camera, instruction and model/history.
All three geometry-only checks passed before live predictions; camera placement
used inherited calibration, finite rigid transforms and conservative XY footprint
checks, followed by actual USD matrix parity before model requests.

| Order | Offset m | Exact initial [x,y,yaw] | Initial clearance m | Corridor clearance m | Execution |
|---|---:|---|---:|---:|---|
| 1 | 1.0 | `[19.20129863861204, 25.42333325575427, -1.5689742328041627]` | 1.158566166 | 1.017023008 | One attempt; selected |
| 2 | 0.75 | `[19.201754161857664, 25.17333367075747, -1.5689742328041627]` | 1.069021045 | 1.017023008 | Not run; first candidate qualified |
| 3 | 0.5 | `[19.20220968510329, 24.923334085760665, -1.5689742328041627]` | 1.021942771 | 1.017023008 | Not run; first candidate qualified |

No failed live candidate, extra pose, cart movement, instruction change, source
retry, LightNav response-based geometry selection or full preflight Isaac launch.
Cart reveal occurs once at C1 observation, sim1.2833334002643824 s / state75,
strictly after C0 application. It remains static and visible in every later
terminal RGB. Direction and immediate C1 response do not affect selection.

## Scheduler audits and accounting deviation

Before science, the new capture-phase rule was tested in cases A–E: a scheduled
capture is required only if that iteration reaches capture. A final zero-step
abort from an already in-flight unsafe/STOP terminal result before capture adds
no capture requirement. Exact actual capture state/time and normal cadence are
still checked. No collector or historical validator was changed.

Saved LONG_SOURCE_01 passes this corrected scheduler audit; its final state120
was an abort-only loop. Its original technical classification included this
audit-definition limitation. The original result remains unchanged, including its
two-chunk prefix. [Saved-only audit](../results/successive_native_source_acquisition_02/historical_long01_corrected_audit.json).

This new episode passes the frozen corrected scheduler, with zero burst steps,
all 12 request RTFs between .989470019 and .991076714, maximum loop stall
.229252355 s (frozen bound .25 s), and 403 completed integration intervals.

The frozen outer call check initially failed: its inner audit counted 57 accepted
submission replies in `controller/events.jsonl`, while the existing outer auditor
counted 58 including `post_episode_messages.json`. The extra accepted solve is
`EPISODE_00_solve_000057`, input state402, drained after termination. Both its
submission and result were saved. It was never applied and did not advance time.
This was a count-scope mismatch, not an extra launch, busy retry or altered command.

The separate [accounting addendum](../results/successive_native_source_acquisition_02/accounting_addendum.json)
reruns every frozen scientific check, authenticates the original failed outer
result and corrects only the accepted-submission total to include shutdown drain.
It requires unique matched submit/result IDs and verifies late results are
unapplied. The frozen outer result remains in
`CANDIDATE_01/attempt_validation.json` and its immutable seal. Its saved error is
`AssertionError: ('MPC_submissions', 58, 57)`; it is not silently relabeled.
The complete accounting supports the unchanged predeclared qualification.

**Protocol deviation:** this accounting addendum and exact-clock reporting helper
were added after science. They make zero model/MPC/Isaac/optimizer calls and alter
no raw data, collector, controller, source geometry, thresholds or selection rule.
The frozen plotting CSV's optional `request_host_s`/`receipt_host_s` fields expect
another stamp schema and remain blank; use the exact native `monotonic_ns`/UTC
columns in [event_clocks.csv](../results/successive_native_source_acquisition_02/event_clocks.csv)
or the original JSON instead. No clock is fabricated.

A read-only `/proc` observer encountered PermissionError during process shutdown;
the already-frozen handler recorded it. Initial environment and actual bundled
CUDA library maps were saved, launch exited0 and all library checks pass. No retry.

## Controller, safety and call accounting

Effective native OBJNAV_V_MAX and maximum actual applied |v| are both **.4 m/s**.
The same pre-tracker configuration is reused byte-for-byte. No new post-solve
clipping/scaling. All 58 previous_control checks pass; applied official commands
match accepted results. A stale solve is rejected under unchanged generation
semantics. H5, dt .1, selector, Q/R, acceleration/angular limits and external
controller source remain unchanged.

External MPC SHA256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
External checkout commit: `c6f40e3220edbf7011e4f17eaf2c865416737d4d`.

| Quantity | Count |
|---|---:|
| Candidate episodes / full Isaac launches | 1 / 1 |
| Terminal LightNav predictions / generated / applied | 12 / 12 / 12 |
| Buffer-only requests / startup warmups / RGB captures | 15 / 1 / 27 |
| MPC submit attempts / accepted submissions / solved results | 58 / 58 / 58 |
| Successful results / stale-rejected results | 57 / 1 |
| New physical command applications | 50 |
| Integration intervals / reveals / initialization resets | 403 / 1 / 1 |
| Resets after initialization / retries | 0 / 0 |
| Graph / canonical / B_ENTRY / Hermite / V2 / GP / rigid / correspondence optimization | 0 each |
| Saved-only validation/report model / MPC / Isaac / optimizer calls | 0 each |

Some successful results are superseded before physical application; success count
is not an application count. Final drained solve is also unapplied. All 12 chunks
have actual first-command boundaries, independent of those command-level counts.

Robot footprint radius .20 m, required extra reserve0; legacy .05 m diagnostic.
All complete raw references pass both thresholds; minimum is C9 at
.06382605574432249 m. Actual swept execution and guard-lookahead lower bounds are
both .3329766668456972 m. All 403 actual intervals pass the legacy diagnostic too.
No physical overlap was executed. No unsafe command or rejected chunk was applied.
World installation remains each observation A times its immutable raw array;
B does not re-anchor it. Model rows have `waypoint_dt=null`.

## Exact A/B boundaries and inference lineage

Each A is the actual observation pose; each B is first physical command
application. C0 is stationary bootstrap; every Ck, k>=1, is observed after
C(k-1) application and generated while that preceding chunk remains active.
Original full precision and controller memory/P/u_minus are in the JSON/bundle.

| Chunk | A [x,y,yaw] | B [x,y,yaw] | Active during inference |
|---|---|---|---|
| C0 | `[19.20129863861204, 25.42333325575427, -1.5689742328041627]` | `[19.20129863861204, 25.42333325575427, -1.5689742328041627]` | none (bootstrap) |
| C1 | `[19.201407937708954, 25.363333351859286, -1.5689749934176171]` | `[19.20161431701553, 25.250000200523026, -1.5689755606609315]` | C0 |
| C2 | `[19.20137427327621, 25.163334122679647, -1.5773308765452005]` | `[19.200148261966227, 25.05000809379713, -1.589632898016459]` | C1 |
| C3 | `[19.202021578622336, 24.963382215646583, -1.5155987442959007]` | `[19.211629399142247, 24.85047677502895, -1.4474301054362901]` | C2 |
| C4 | `[19.23045766339444, 24.76602784226726, -1.2555844659456998]` | `[19.27389403118432, 24.661447483473033, -1.1106901492252321]` | C3 |
| C5 | `[19.31537929222634, 24.58537456142413, -1.042210094933905]` | `[19.391835629986183, 24.46021740723268, -1.0157335574895257]` | C4 |
| C6 | `[19.419507276626742, 24.414625279192347, -1.0339144511119294]` | `[19.49227873662385, 24.287294703852076, -1.0709447759986226]` | C5 |
| C7 | `[19.51856517286296, 24.24089159568664, -1.041234916070099]` | `[19.594178723299457, 24.115223153802393, -1.0431171499844467]` | C6 |
| C8 | `[19.62026185370495, 24.068705780759256, -1.0745814019413018]` | `[19.6856688723953, 23.937472338905074, -1.1522804733807532]` | C7 |
| C9 | `[19.706538691916546, 23.888394303268907, -1.1831769918483561]` | `[19.757482233022174, 23.750897469161924, -1.2594372560698994]` | C8 |
| C10 | `[19.77296666244312, 23.699863895566263, -1.2909558125803564]` | `[19.80875441379089, 23.557668078097482, -1.3680537791296452]` | C9 |
| C11 | `[19.819712707861573, 23.50547283007218, -1.3597090546721613]` | `[19.850725453528458, 23.362124000413758, -1.368135140793049]` | C10 |

## Distinct clocks

Simulation seconds and host monotonic nanoseconds are different clocks. Receipt
is a host event; ready-seen is the later collector observation of that event.
Ready-seen and install happen to coincide here but remain separate records.
These timestamps describe execution events, not intrinsic waypoint timing.

| Chunk | Observation sim s | Request host ns | Receipt host ns | Ready-seen sim s | Install sim s | Application sim s |
|---|---:|---:|---:|---:|---:|---:|
| C0 | 0.783333374 | 452691669519490 | 452691885854091 | 1.033333387 | 1.033333387 | 1.116666725 |
| C1 | 1.283333400 | 452692328649469 | 452692542768553 | 1.516666746 | 1.516666746 | 1.566666748 |
| C2 | 1.783333426 | 452692977515074 | 452693199754551 | 2.033333439 | 2.033333439 | 2.066666774 |
| C3 | 2.283333452 | 452693622946721 | 452693846450094 | 2.533333465 | 2.533333465 | 2.566666801 |
| C4 | 2.783333478 | 452694271895832 | 452694500278932 | 3.033333492 | 3.033333492 | 3.066666827 |
| C5 | 3.283333505 | 452694924557738 | 452695159375237 | 3.550000185 | 3.550000185 | 3.650000190 |
| C6 | 3.783333531 | 452695583654853 | 452695824674718 | 4.050000211 | 4.050000211 | 4.150000216 |
| C7 | 4.283333557 | 452696248428558 | 452696484838419 | 4.550000237 | 4.550000237 | 4.650000243 |
| C8 | 4.783333583 | 452696908063838 | 452697160265757 | 5.050000263 | 5.050000263 | 5.150000269 |
| C9 | 5.283333609 | 452697580720637 | 452697842729943 | 5.550000289 | 5.550000289 | 5.650000295 |
| C10 | 5.783333635 | 452698273227962 | 452698546457695 | 6.050000316 | 6.050000316 | 6.150000321 |
| C11 | 6.283333661 | 452698974976779 | 452699275421717 | 6.550000342 | 6.550000342 | 6.650000347 |

Termination: `ATTEMPT_LIMIT` after C11 application and .1 s postroll, final sim
6.750000352039933 s; configured10 s cap did not bind. No C12, STOP or raw rejection.

## Raw geometry and descriptive evolution

Every chunk has ten untimed rows in this acquisition (the interface remains
arbitrary N). Endpoint components below are each chunk's own observation frame.
Positive lateral means left, negative means right; neither sign is a gate.
All legacy5cm entries pass. Full coordinates/hashes/active duration are also in
[chunks.csv](../results/successive_native_source_acquisition_02/chunks.csv) and
[result_summary.json](../results/successive_native_source_acquisition_02/result_summary.json).

| Chunk | Local endpoint [x,y,yaw] | XY arc m | Cart pixels | Ref clearance m | Legacy5cm |
|---|---|---:|---:|---:|---|
| C0 | `[1.501985788345337, 3.947936875192681e-06, 8.423162398685236e-06]` | 1.351820588 | 0 | 1.017021019 | PASS |
| C1 | `[1.4947317838668823, -0.13291215896606445, -0.25827035307884216]` | 1.357452564 | 1494 | 0.821280919 | PASS |
| C2 | `[1.417783260345459, 0.38496825098991394, 0.5225185751914978]` | 1.352387716 | 1703 | 0.669477863 | PASS |
| C3 | `[1.3555278778076172, 0.6297799944877625, 0.5259516835212708]` | 1.354701251 | 2065 | 0.641978479 | PASS |
| C4 | `[1.4774906635284424, 0.2404809594154358, -0.11331233382225037]` | 1.355421529 | 3053 | 0.460310628 | PASS |
| C5 | `[1.4917658567428589, -0.13928474485874176, -0.296600341796875]` | 1.356084348 | 5909 | 0.303271105 | PASS |
| C6 | `[1.2833104133605957, -0.17393547296524048, -0.6308086514472961]` | 1.177894498 | 8754 | 0.286555298 | PASS |
| C7 | `[1.4440888166427612, -0.325979620218277, -0.512310266494751]` | 1.359875071 | 10446 | 0.142946730 | PASS |
| C8 | `[1.4347742795944214, -0.3930123448371887, -0.5260758399963379]` | 1.360943127 | 11959 | 0.135383537 | PASS |
| C9 | `[1.4347742795944214, -0.3930123448371887, -0.5260758399963379]` | 1.360943127 | 14457 | 0.063826056 | PASS |
| C10 | `[1.505447268486023, -0.19841521978378296, -0.2557354271411896]` | 1.376053451 | 17551 | 0.147161981 | PASS |
| C11 | `[1.505447268486023, -0.19841521978378296, -0.2557354271411896]` | 1.376053451 | 20593 | 0.120088413 | PASS |

| Chunk | Raw SHA256 |
|---|---|
| C0 | `6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae` |
| C1 | `6f6b97c922b9e6cf8d455cf4cc4b1686375b93a40a0a67f557278137a751450b` |
| C2 | `641bd0b4c77c65d0351506dee584639529db857715fe0078bf95da256fb5d25f` |
| C3 | `0c43ff4629f9ce25df390104813258ac8fbe611b8ad8a71706623b5e13c2b39e` |
| C4 | `65f5227e962784fee02ac43559ccf2ed6323fdd8916edadc124089ba756ef4b9` |
| C5 | `8224b6f6d5456f4f58b4d065e2bf617ecfd3fdf2745abea8740813468587e6ad` |
| C6 | `5426ff4e99097821e56dade5f7039886fc128e47056a6e8eae09e0c618ee20b8` |
| C7 | `fb8d3dca48d4c825cccb836468378cfdbc2a48427f1a8f3ac4c1d824712af353` |
| C8 | `ccc1c8613424d4d1a32fe8e98f2ceaad1d39cf17010fbe2f5a89ebe2877250a1` |
| C9 | `ccc1c8613424d4d1a32fe8e98f2ceaad1d39cf17010fbe2f5a89ebe2877250a1` |
| C10 | `f9b6029bd888f6df3de916a449ce436c0e3256b650bf4ea2aa8e2e75e2dc686a` |
| C11 | `f9b6029bd888f6df3de916a449ce436c0e3256b650bf4ea2aa8e2e75e2dc686a` |

First descriptive reaction onset is **k_react=1**, observation state75 at
sim1.2833334002643824 s, A1 as above, 1494 cart pixels, .0600000034475 m actual
travel from initial pose and cart-only footprint clearance2.313163009814311 m.
It was not required to occur at C1. There are nine EVOLVING and two STABLE pairs.

C8/C9 and C10/C11 raw arrays are respectively byte-identical (zero local path
separation and yaw/lateral difference), so their local plot curves coincide.
Their observation RGB/history and world installations differ. This is genuine
successive invocation with stable outputs, not substituted/replayed data.

| Pair | Local symmetric separation m | Endpoint lateral delta m | Endpoint yaw delta deg | Net yaw delta deg | Descriptor |
|---|---:|---:|---:|---:|---|
| C0->C1 | 0.132916025 | -0.132916107 | -14.798284 | -14.876422 | EVOLVING |
| C1->C2 | 0.487187342 | 0.517880410 | 44.735910 | 44.678923 | EVOLVING |
| C2->C3 | 0.252603489 | 0.244811743 | 0.196703 | -10.492500 | EVOLVING |
| C3->C4 | 0.397998959 | -0.389299035 | -36.627130 | -40.805040 | EVOLVING |
| C4->C5 | 0.380033908 | -0.379765704 | -10.501629 | 4.478733 | EVOLVING |
| C5->C6 | 0.211315747 | -0.034650728 | -19.148726 | -23.530768 | EVOLVING |
| C6->C7 | 0.221285150 | -0.152044147 | 6.789457 | 11.175451 | EVOLVING |
| C7->C8 | 0.067676782 | -0.067032725 | -0.788709 | 0.634245 | EVOLVING |
| C8->C9 | 0.000000000 | 0.000000000 | 0.000000 | 0.000000 | STABLE |
| C9->C10 | 0.206141338 | 0.194597125 | 15.489365 | 14.150743 | EVOLVING |
| C10->C11 | 0.000000000 | 0.000000000 | 0.000000 | 0.000000 | STABLE |

[All adjacent-pair descriptors](../results/successive_native_source_acquisition_02/evolution.json)
include observation deltas, world separation, clearance/pixel changes and hash
identity. Exact RGB bytes, serialized request/response and history are preserved.
Server history contract is audited; internal frame-selection implementation is
not claimed to be directly observed.

## Bundle and figures

Selected source root:
`data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01/`.
Bundle: `source_bundle/` under that root. Integrity PASS: 297
source files and 59 derived files, 11 indexed handoffs,
10 post-reveal. Each ready_record includes candidate/episode, A/B/P, OLD/FRESH
raw/world hashes, distinct clocks, physical u_minus, controller previous_control,
first command, generation/version, actual OLD-to-B segment, exact history/request/
response provenance, scientific freeze and model/checkpoint source.
Large arrays/RGB/logs remain ignored under data; only compact results are tracked.

Exactly four final PNGs, visually inspected with numeric/hash parity PASS:

1. [World execution](../results/successive_native_source_acquisition_02/figures/continuous_world_episode.png)
   — equal axes, raw dashed, actual active-chunk colors, every A/B, cart and .20/.25 contours.
2. [Own-observation local chunks](../results/successive_native_source_acquisition_02/figures/observation_local_evolution.png)
   — all 12 chunks, +x forward/+y left; identical pairs overlap as reported above.
3. [Event and active timeline](../results/successive_native_source_acquisition_02/figures/episode_timeline.png)
   — OLD remains active through the next inference, application defines transitions.
4. [Exact terminal RGBs](../results/successive_native_source_acquisition_02/figures/request_rgb_sequence.png)
   — all12 request images, metadata outside pixels; raw JPEGs unchanged.

The source-summary CSV substitutes for an extra fifth PNG. No HTML dependency.

## Validation and commands

Pre-freeze: focused37 passed; relevant full regression652 passed, one skip for
absent immutable DATA02 v1 corpus. Audit-only stage37 included historical LONG
regressions. Post-freeze accounting/clock helper tests: 8 passed; final relevant regression:
660 passed, 1 same absent-corpus skip. Compileall and diff checks pass. Saved-only source/bundle,
accounting, exact-clock and report parity checks pass with zero new science.
Original outer validator's accounting failure is retained as described above.

From repository root, commands executed (all paths literal):

```bash
git fetch origin main
git status --short
git branch --show-current
git rev-parse HEAD origin/main
.venv/bin/python scripts/audit_successive_scheduler02.py
.venv/bin/python scripts/run_successive_native_source_acquisition02.py --run data/successive_native_source_acquisition_02/primary_20261007 --mode prepare
.venv/bin/python scripts/run_successive_native_source_acquisition02.py --run data/successive_native_source_acquisition_02/primary_20261007 --mode freeze
git commit -m 'Correct saved-only audit for terminal loops before capture'
git push origin main
git commit -m 'Freeze bounded twelve-chunk successive native source acquisition'
git push origin main
.venv/bin/python scripts/run_successive_native_source_acquisition02.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01 --mode verify
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_successive_native_source_acquisition02.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01 --mode start
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_successive_native_source_acquisition02.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01 --mode launch
.venv/bin/python scripts/run_successive_native_source_acquisition02.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01 --mode stop
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/successive-source02-mpl .venv/bin/python scripts/validate_successive_native_source_acquisition02.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01 --seal
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/successive-source02-mpl .venv/bin/python scripts/validate_successive_source02_accounting_addendum.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01 --seal
.venv/bin/python scripts/report_successive_source02_event_clocks.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01
```

The audit-fix commit/push preceded preparation; scientific freeze commit/push
preceded server startup and the only launch. Environment sanitation and actual
Isaac argv are frozen in each candidate's launch_environment.json; no system or
external environment/source edits. The owned server pid570280 was stopped.

Frozen report functions were invoked directly after the accounting addendum,
because its CLI checks the preserved failed outer-attempt record:

```python
import sys
from pathlib import Path
sys.path[:0]=['src','scripts']
from reconciliation.join_source03 import read,save
from run_successive_native_source_acquisition02 import OUT
from report_successive_native_source_acquisition02 import report,validate_report
run=Path('data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01').resolve()
assert read(run/'accounting_addendum.json')['valid']
report(run,OUT)
save(OUT/'report_validation.json',validate_report(run,OUT))
```

Invocation prefix: `OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/successive-source02-mpl .venv/bin/python -`.
Final checks (saved-only, not a collection rerun):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/successive-source02-mpl .venv/bin/python -m pytest -q tests/test_successive*.py tests/test_long_continuous_obstacle_reveal_source01.py tests/test_continuous_obstacle_reveal_exploratory02.py tests/test_continuous_obstacle_reveal_episode01.py tests/test_continuous_obstacle_reveal_episode01b.py tests/test_obstacle_source*.py tests/test_join_online*.py tests/test_online*.py tests/test_robotless_online*.py tests/test_data02*.py tests/test_se2*.py tests/test_join_source02*.py tests/test_gp_se2_environment*.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/successive-source02-mpl .venv/bin/python scripts/validate_successive_source02_accounting_addendum.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01
.venv/bin/python scripts/audit_successive_scheduler02.py --check
.venv/bin/python scripts/report_successive_source02_event_clocks.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01 --check
git commit -m 'Report twelve genuinely applied native chunks and complete saved accounting'
git push origin main
```

## Claim boundary

A development source with 12 consecutive genuinely applied native LightNav chunks
was acquired. Every later chunk was generated from a new observation causally
reached by executing the preceding chunk, while that preceding chunk remained
active during inference. Cart visibility and evolving/stable outputs are
reported descriptively.

This establishes no right/left instruction compliance, semantic obstacle
understanding, complete bypass, navigation task success, general safety,
reconciliation benefit, graph superiority, generalization or real-world
performance. It is one selected development source, not held-out evidence.
No graph, bridge, B_ENTRY, GP, transport or future benchmark was implemented.
Work stops after result validation, documentation and normal push.
