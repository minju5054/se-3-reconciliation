# SUCCESSIVE_ISAAC_RECONCILIATION_REPLAY_01

**TECHNICAL_BLOCKED — stopped at the saved-only schedule prerequisite.**

The source's complete C1-active interval contains a C2 controller installation
and submission. Reproducing those events requires a second reference; removing
them changes the requested original schedule and generation history. The task
permits RAW C1 only and explicitly requires stopping when exact scheduling cannot
be preserved. No Isaac launch, replay, new MPC solve, or method comparison occurred.

## Repository-confirmed source facts

- Fetched `origin/main`; starting HEAD and remote main were
  `e4f61465c13996cc8c5c70504a5735dabfa6e944`.
- Selected source only:
  `data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01`,
  `source_bundle/handoffs/C0_to_C1`.
- Source scientific freeze: `ee330244d867fad3749c60b6a5f1630b1471ec2a`.
  Its result commit is the starting HEAD above.
- The tracked integrity record matches that result commit byte-for-byte. The
  final source seal authenticates 401 files; the bundle authenticates 297 episode
  input files. The original saved-only accounting-addendum validator passes with
  the historical `SUCCESSIVE_SOURCE_MAX_REACHED` classification unchanged.
- Both C0 and C1 raw/world hashes authenticate. C1 world SHA-256 is
  `a93c28ce754a81f814f2ac94f4d5aa2497de28b27ffd15c2d96670622bcdbb14`.
  C1 raw SHA-256 is
  `6f6b97c922b9e6cf8d455cf4cc4b1686375b93a40a0a67f557278137a751450b`.
  Full input hashes and clocks are in the machine-readable artifacts.

### Boundary and coordinate convention

World XY is metres, +Z up; yaw is radians CCW. Observation-local +x is forward,
+y is left. The saved world reference is `T_W_F = T_W_A1 T_A1_F`; no B re-anchor,
new row, smoothing, interpolation, or waypoint time was introduced.

| Saved quantity | Value |
|---|---|
| A1 | `[19.201407937708954, 25.363333351859286, -1.5689749934176171]` |
| B1 | `[19.20161431701553, 25.250000200523026, -1.5689755606609315]` |
| P1 | `[19.201602178508843, 25.25666685648663, -1.5689755391010196]` |
| physical u_minus | `[0.4, -1.2935946487914903e-06]` |
| already-applied u_B_plus | `[0.4, -0.04625916114685449]` |
| previous_control at B1 | `[0.4, -0.04625916114685449]` |
| B tick / simulation time | `92 / 1.5666667483747005 s` |
| resolved integration dt | `0.01666666753590107 s` |
| C1 version / generation at B | `1 / 3` |
| C1 physical integration interval | ticks `92..121`, 30 steps, states `92..122` |
| stop before C2 command application | tick `122`, simulation time `2.0666667744517326 s` |

The first C1 solve was submitted at tick90 and applied at92. Its already accepted
command and memory are restoration inputs, not a new solve at B. The memory
provenance is controller event row14. Physical u_minus and controller memory
are distinct quantities.

The saved obstacle reveal was at tick75, before B1. The saved cart transform and
presence match the C1 record. The unchanged official MPC source hash is
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
The saved intervention sets only native `OBJNAV_V_MAX` to 0.4 before tracker
construction. H=5, nearest+1 selection, Q/R, angular and acceleration limits,
previous-control and stale-result behavior remain the historical definitions.

### Schedule evidence

All ticks below come from authenticated source records, not a nominal control rate.

| Submit tick | Reference | Version / generation | Application tick |
|---:|---|---|---:|
| 96 | C1 | 1 / 3 | 97 |
| 102 | C1 | 1 / 3 | 103 |
| 108 | C1 | 1 / 3 | 109 |
| 114 | C1 | 1 / 3 | 115 |
| 120 | **C2** | **2 / 4** | **122** |

At tick120 (`2.0333334393799305 s`), C2 is installed and submitted while the
physical command is still held from C1 solve000009. The installation reply is
seen at121; C2 solve000010 is first seen/applied at122. C1 commands therefore
remain active during ticks120 and121 despite the controller generation change.

Evidence locations within `episodes/EPISODE_00`:

- `controller/events.jsonl`: zero-based row23 C2 install, row24 submit;
  install context records tick120's simulation time separately from reply receipt.
- `scheduler.jsonl`: start_state_id120 records C2 ready and MPC submission;
  121 records replies; 122 records the command result.
- `commands.csv` and `guard.jsonl`: C1/version1/solve000009 is held through121;
  C2/version2/solve000010 first applies at122.

The extracted attempted and accepted submit sequence is `[96,102,108,114,120]`.
The post-B applications strictly inside the horizon are `[97,103,109,115]`.
The last submit must not disappear just because its application is at the boundary.

## This experiment's frozen protocol

The config and `freeze_manifest.json` form a **blocked preflight snapshot**.
They are not a pushed scientific execution freeze: `scientific_freeze_sha=null`
and `scientific_launch_authorized=false`. The prerequisite failed before such a
freeze could authorize a run. This report and code are one focused audit commit.

The snapshot pins the selected source/handoff, config, exact input hashes,
relevant implementation hashes, extracted full horizon and event sequence,
restoration rules, parity tolerances and budget. Values are loaded from saved
sources; boundary coordinates and schedules are not copied into runtime code.

The intended protocol uses the original 30-step window and RAW C1 world rows.
It requires all original in-window controller events, including generation
changes, and cannot substitute a filtered single-reference schedule. Existing
pose/command absolute tolerances of `1e-10` and selection tolerance of `1e-12`
come from `osa03_common_b_method_comparison_01.yaml`; relative tolerance is zero.
Raw/world files, restored saved values, ticks, generation and version require
exact equality. No tolerance was selected from a replay outcome.

Inspected reuse points:

| Concern | Existing implementation | Audit finding |
|---|---|---|
| Logical release | `osa03_relative_ablation.LogicalRelease` | Fixed generation; wall wait cannot advance sim state |
| Common-B memory restore | `osa03_common_b_mpc_worker.initialize` | Restores command/memory/diagnostics; no future-result replay |
| Runtime install semantics | same worker's `dispatch` | Disallows installs/resets during the rollout |
| Official MPC | `online_mpc_adapter`, `online_mpc_worker`, `long_source_mpc01` | Saved source/settings/speed intervention authenticated |
| Scene | `isaac/join_online02_collect.setup_scene` | Existing Hospital/cart setup; not launched here |
| Integration | `isaac/robotless_online_handoffs` | Existing actual USD pose write/readback and world step |
| Launch sanitation | `run_continuous_obstacle_reveal_episode01b`, successive launcher | Existing CUDA/library sanitation must be retained; not exercised here |
| Guard | `join_online02.guard_check`, source environment adapter | Reused for saved C1 intervals |

The existing logical scheduler/common-B worker cannot directly reproduce the
in-window C2 install/generation switch. Extending it to install/solve C2 would
also exceed RAW C1-only scope. Filtering the event, replacing it with a C1 solve,
or ending at120 would change the protocol. None was implemented.

## Actual Isaac scientific execution result

No scientific execution occurred. The explicit prerequisite STOP rule was used.
All new counts are zero: Isaac launches, RAW rollouts, MPC solves, LightNav calls,
RGB/model requests, optimizer calls, B_ENTRY/Hermite calls and retries.

The new script is a saved-only auditor with write/validation modes. It has no
launcher, model, solver or live scene entry point. Tests use synthetic adapters
where controller restoration is exercised; those tests are not experimental
evidence. No display-only playback was substituted for scientific execution.

## Parity result

**TECHNICAL_BLOCKED.** Every scientific parity quantity is null/NOT_RUN in
`parity.json`: initial restored B error, per-tick pose error, command error,
application-tick equality, memory equality, guard equality, replay swept clearance
and horizon termination. These are not zeros and not a RAW parity failure after
execution. Live Isaac B/cart restoration and live controller restoration remain
unverified.

Saved-data checks only:

- Recomputing the 30 historical unicycle steps from saved states/commands yields
  maximum component difference **0.0**. This is source consistency, not Isaac replay.
- C1 complete raw-reference footprint-edge clearance: **0.821280919083059 m**.
- Minimum saved C1 guard clearance lower bound: **1.0379373625059845 m**.
  All 30 guard checks recompute successfully and pass the legacy 0.05 m diagnostic.
- The unchanged source uses radius0.20 m, zero extra reserve for its exploratory
  physical-overlap guard, and reports the historical 0.05 m margin separately.
  No safety threshold changed; no new command was applied.

No final PNG is generated because there is no scientific replay trace to compare.
This is allowed by the task's technical-failure output rule.

## Limitations

The incompatibility is between strict full controller-event parity and the
RAW C1-only scope. It does **not** establish that C1's physical path cannot be
reproduced by four C1 solves followed by holding. That alternative would require
an explicit protocol decision about the boundary-tail controller events; this
task does not silently authorize it. No Isaac startup, CUDA loading, scene
restoration or live solver determinism has been tested in this task.

The authenticated episode is one development source. The source's prior
accounting addendum and historical results remain unchanged. Unrelated local
configuration changes and the GPU-memory plotting script were preserved.

## What is NOT demonstrated

RAW Isaac replay parity, reconciliation benefit, graph superiority, VLA or
navigation improvement, endpoint completion and real-world behavior are not
demonstrated. It is **not scientifically valid to proceed to B_ENTRY comparison**
on the basis of this audit. No next method was implemented.

## Verification and commands

Initial plain pytest startup encountered an unrelated ROS auto-loaded plugin
requiring unavailable `lark`. The repository's established
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` setting resolved this without environment edits.
Focused tests: **16 passed**. Relevant regression: **900 passed** in 54.36 s,
including the new tests. Compileall, saved-only artifact equality and diff checks
pass. The original source validator also passes without any scientific call.

```bash
git fetch origin main
git status --short
git branch --show-current
git rev-parse HEAD origin/main
.venv/bin/python scripts/validate_successive_source02_accounting_addendum.py --run data/successive_native_source_acquisition_02/primary_20261007/CANDIDATE_01
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLCONFIGDIR=/tmp/mpl-replay01 .venv/bin/python -m pytest -q tests/test_successive_isaac_replay01.py
MPLCONFIGDIR=/tmp/mpl-replay01 .venv/bin/python scripts/audit_successive_isaac_replay01.py --write
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLCONFIGDIR=/tmp/mpl-replay01 .venv/bin/python -m pytest -q tests/test_successive*.py tests/test_long_continuous_obstacle_reveal_source01.py tests/test_continuous_obstacle*.py tests/test_online_mpc_adapter.py tests/test_osa03_common_b.py tests/test_osa03_relative_factor_ablation01.py tests/test_robotless*.py tests/test_join_online02*.py tests/test_se2.py tests/test_se2_lie.py
.venv/bin/python -m compileall -q src scripts tests
MPLCONFIGDIR=/tmp/mpl-replay01 .venv/bin/python scripts/audit_successive_isaac_replay01.py
git diff --check
git diff --cached --check
git commit -m 'Audit RAW C1 replay prerequisite and record in-window C2 schedule blocker'
git push origin main
```

Artifacts: `results/successive_isaac_reconciliation_replay_01/` contains source
authentication, full schedule audit, blocked snapshot manifest, result summary,
validation summary, call accounting and null scientific parity JSON. README stays
unchanged because no scientific replay capability or parity result was established.
