# OSA03_FRESH_TRACKABILITY_CONTROL_01

## Frozen question and scope

How difficult is the exact same OSA03 FRESH to execute from its original observation state, compared with the saved delayed B-start Native continuation? This is an offline same-FRESH observation-to-application sensitivity diagnostic, not an additive decomposition or a recoverable reconciliation-cost bound.

Starting revision: `0842f5583d9295b436a47c7da9d26ef6ed054143`. Source: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`, representative `REPEAT_00`. Comparator: `data/osa03_native_continuation_01/primary_20260923T103000Z/`, revalidated saved-only, never re-solved. New unique run: `data/osa03_fresh_trackability_control_01/primary_20260924T012000Z/`.

Original FRESH SHA256: `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`. Original OLD: `6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae`. World coordinates remain observation-anchored Isaac Z-up SE(2), metres/radians, yaw CCW; body x forward, y left. Neither raw array changes. No crops, re-anchoring, resampling, selector intervention, model update, or optimization.

## Saved observation phase, before execution

Observation is state/tick 75, simulation time `1.2833334002643824`; pose `[19.203242163778583,24.356668352984542,-1.5689752526514715]`. Both physical incoming command and independently reconstructed controller memory happen to equal `[0.6000000287746985,-4.733615847732774e-06]`; their provenance remains separate. This differs from B's incoming .8 m/s and already-advanced .5 rad/s controller memory.

OLD solve `REPEAT_00_solve_000002` was submitted at tick 72 and applied at tick 73. Its result was already seen before observation. There is no pending OLD solve at the observation cut. The next historical OLD submit at tick 78 is after the cut, and becomes the first counterfactual FRESH submit; it is not replayed as an obsolete pending solve. Tick 75 is off-grid, so no solve is permitted there.

A fresh independent official tracker is constructed, OLD is installed, and the last accepted OLD controller state is restored before simulated time begins. Official FRESH installation increments generation 2 to 3 and preserves this nonzero memory. No numerical solve is used to restore it. Official generation rejection of stale OLD results is retained and tested; no result exists to discard in this source. Missing or ambiguous observation state fails closed.

The unchanged official source hash is `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`. Nearest/+1 selection, Q/R, limits, horizon, asynchronous worker, hold/failure policy, and exact held-command integration are reused. Original absolute control ticks modulo six and source float32-resolved `dt=0.01666666753590107` remain fixed.

## Pre-execution protocol

Exactly one new scientific observation-start rollout after the code/config/tests freeze commit is pushed. No B-start solves. Availability is counterfactually immediate at observation; physical OLD command persists until an actual new FRESH result is received and applied. Original asynchronous MPC compute/IPC delay is measured, not deleted. The newly constructed solver may have first-call construction/cache costs, unlike an already warm live worker; no extra warmup solve is invented.

The observation cap is 180 integration intervals from **availability**, `3.0000001564621925 s`. It is not an intrinsic duration of the LightNav rows. Two origins are stored: observation availability and first FRESH command application. Primary paired .30/.90 s metrics use the latter. Its complete 3 s window is N/A if unavailable. Each achieved full horizon is labelled, and the saved B trajectory is also evaluated over a common execution-prefix exposure, without another controller solve.

Safety reuses the direct Hospital/cart checker and abort-only guard, physical radius .20 m, required edge clearance .05 m, original uncertainty/curve allowance. Attachment reuses original-FRESH forward-only polyline projection, shortest-angle yaw, .10 m/15 degrees for a full following .30 s of samples. No continuous-time attachment proof or completed obstacle bypass claim.

Spatial descriptors include row0 distance/bearing/yaw, reliable first-chord direction, row spacings/yaw increments, meaningful XY curvature, lateral amplitude and arc. Untimed rows are never converted into required rates or accelerations.

Signed B-minus-observation contrasts cover initial geometry, early growth, .30/.90 s AUCs, attachment time/progress, clearance and commands. No scalar score or percentages of intrinsic versus latency cost. Disagreement between metrics is reported as interaction. Reconciliation would start at B and cannot undo preceding motion; future intent-preservation constraints would need to prevent silently replacing upstream intent or repairing arbitrary source infeasibility.

## Commands

```bash
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode prepare
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode preflight
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_osa03_trackability.py tests/test_osa03_native.py tests/test_online_mpc_adapter.py
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode freeze
# Commit and normal push, then exactly once:
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode execute
.venv/bin/python scripts/run_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z --mode validate
.venv/bin/python scripts/report_osa03_trackability.py --run data/osa03_fresh_trackability_control_01/primary_20260924T012000Z
```

Results will be appended after the one frozen execution. Tests using synthetic fixtures are implementation checks only. Source-state, baseline validation and restoration preflight use saved records and zero numerical MPC solves. LightNav, RGB, Isaac, GP, rigid, graph, splice and reconciliation calls remain zero.
