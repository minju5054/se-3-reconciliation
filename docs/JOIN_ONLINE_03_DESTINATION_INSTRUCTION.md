# JOIN-ONLINE-03 — Destination-first instruction rerun

## Frozen question and single intervention

User instruction, used verbatim for every prediction in every new episode:

> Go to the far end of the hallway. Pass around the supply cart without touching it, and stop only when you reach the end of the hallway.

Does this destination-first wording change the shortening/STOP behavior observed
under "Avoid the supply cart and continue to the end of the hallway."?
This is a new native online source experiment, not reconciliation or a new
avoidance/controller policy. Historical JOIN-ONLINE-02 is immutable.

Starting HEAD and freshly fetched origin/main:
`68ebd7ec32428ede040dcd715f0925a6de2fd9de`.
Baseline: `data/robotless_join_online_02/primary_20260923T001500Z/`.
New run: `data/robotless_join_online_03/primary_20260923T031300Z/`.
Actual UTC, monotonic, and simulation times are recorded separately from run ID.

## Preserved setup

Reuse the baseline's exact resolved config; change only its instruction and
experiment label. The inherited `join_online02` configuration key is retained
for the unchanged collector. New wrapper never changes legacy defaults.

- Exact selected 5 m start: [19.19599771672821,27.640298097840173,-1.5685636348381817].
- Exact Hospital layers, cart mesh/transform, BRIGHT light, camera, footprint,
  required edge clearance .05 m, direct environment and curve allowances.
- Cart absent throughout OFF; present before the first capture throughout ON.
- Official LightNav c6f40e3220edbf7011e4f17eaf2c865416737d4d; checkpoint revision
  7221d418bfff55cfcbadd09f7a26aaab81e1f8a6; VLN. Actual argv/config/hashes verified.
- temperature0, top_p1, top_k0, traj_top1=0, unchanged history sampler.
- Unchanged official MPC source SHA256
  2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1.
- Native nearest/+1 selector, gains, limits, objective, 10 Hz solves,
  60 Hz exact held-command SE(2) execution, 4 Hz live RGB.
- Four bootstrap frames (first three buffer-only); later predictions require
  .5 simulation seconds after actual activation of the preceding reference.
- OLD continues while FRESH is pending. World poses remain observation anchored;
  readiness, installation, first command application/B, physical previous
  command and controller memory remain separately recorded.
- Original oracle abort guard only: no steering, reference editing or unsafe
  execution. Unsafe raw prediction alone does not trigger an abort.

Frozen order, once each: OFF_REPEAT_00, ON_REPEAT_00, OFF_REPEAT_01, ON_REPEAT_01.
Retain the original 20-activation/attempt and 25 s bounds and natural STOP
termination. All four planned episodes run regardless of earlier scientific
outcome. No prompt/placement/threshold tuning or outcome retry.

The exact former `scenario.json`, `start_selection.json`, and `side_passages.json`
bytes are copied; no start search is repeated. Previous raw RGB/responses are
not reused as new model inputs. Every new episode captures new current RGB and
has an independent session. Therefore this is an instruction intervention on
the same declared scenario, **not an identical-history paused inference pair**.

## Frozen evaluation and interpretation

Reuse full raw-polyline footprint clearance, without suffix trimming or an
observation-to-first-row connector; generic N. Keep the previous meaningful
lateral/tangent criteria, SAFE_BYPASS_ONSET and full SAFE_BYPASS conditions.
Physical cart traversal, safe raw future, STOP and hallway goal completion
remain distinct. There is no oracle hallway-end target, so model STOP alone
does not prove arrival at the instruction's destination.

Report every response, actual execution, boundary/timing/clearance, original
classifications, and actual model/MPC calls. Compare both ON repetitions with
historical ON and new OFF controls. Preserve the original request-local RTF
[.8,1.2] and .25 s stall checks; do not repair pacing after seeing outcomes.
If paths shorten or STOP again, this does not establish a hidden neural cause.
If bypass appears, it is evidence on this scenario only. No GP/rigid solve.

The read-only STOP/pointing auditor can inspect saved new outcomes afterward;
OPOS/cart overlap remains a grounding hypothesis, not proof of goal identity.
No model justification is fabricated or elicited as a substitute for evidence.

## Implementation and commands

Only a new config, preparation/report/validation wrapper and tests are added.
Unchanged `join_online02_collect.py` performs real preflight and collection;
unchanged online history, transform, solver and environment functions execute.
The new validator uses the original full stream validator and independently
reconstructs every guard, mask, config comparison and plotted series.
The original plot generator is reused literally; `review/index.html` is its
legacy-labelled helper output. The primary correctly labelled new index is
`index.html`, with `instruction_review_bundle.zip`.

```bash
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode init
# Same clean Isaac environment as JOIN-ONLINE-02, no system changes:
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 /home/gpuadmin/isaacsim/python.sh scripts/isaac/join_online02_collect.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode preflight
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode freeze
# Review, commit, normal push before model startup and actual collection.
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode start
# Repeat the exact clean Isaac command above with --mode collect.
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode stop
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode analyze
MPLCONFIGDIR=/tmp/join_online03_mpl .venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode report
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode validate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_join_online03.py tests/test_join_online02.py tests/test_join_online02_stop_audit.py tests/test_join_source04.py tests/test_online_history.py tests/test_se2.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Pre-model technical rendering/import and unit tests are separate from
scientific calls. No MPC solve/model inference is used in preflight or tests.
