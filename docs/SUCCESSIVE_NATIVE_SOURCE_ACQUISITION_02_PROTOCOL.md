# SUCCESSIVE_NATIVE_SOURCE_ACQUISITION_02 — frozen protocol

Development source acquisition only. Starting HEAD:
`e39d465762e0680d0c806f58079127b313312855`.
Audit-fix commit: `3c940ab` (full SHA recorded in protocol_inputs.json).
No held-out or method evidence. This protocol is frozen before any new scientific
call; results belong in a separate report so this file remains hash-stable.

## Authority and permitted changes

Authenticate saved LONG_SOURCE_01, its original validation, report, raw/derived
files, previous source chain, external controller/model hashes and launch setup.
The old result is not reclassified. Corrected audit passes saved LONG_SOURCE_01:
only the final zero-step loop receiving an in-flight unsafe/STOP terminal response
before capture is exempt. Normal cadence, flags and exact rendered state/time
remain checked; no collector capture is fabricated. Old prefix is still C0–C1.

All three candidates are predeclared in
`results/successive_native_source_acquisition_02/candidate_manifest.json`:

| Order | Backward offset m | World x m | World y m | Yaw rad |
|---|---:|---:|---:|---:|
| 1 | 1.00 | 19.20129863861204 | 25.42333325575427 | -1.5689742328041627 |
| 2 | 0.75 | 19.201754161857664 | 25.17333367075747 | -1.5689742328041627 |
| 3 | 0.50 | 19.20220968510329 | 24.923334085760665 | -1.5689742328041627 |

Transform `T_candidate=T_POSE11 Trans(-d,0,0)`: local +x forward, +y left;
world XY metres, Z up, CCW yaw radians. Cart transform, asset, Hospital export,
BRIGHT lighting, camera calibration/height, instruction, model/checkpoint/history
and generation settings remain unchanged. Robot is logical; physical denotes the
integrated command-driven state rather than contact dynamics of a robot body.

Geometry-only preflight uses the existing checker, .20 m footprint, zero extra
reserve, known workspace and unchanged numerical uncertainty/reserve. All three
initial footprints, camera XY conservative footprint checks and corridors to
original POSE11 pass. Corridor clearance minimum is 1.01702300774 m. Camera uses
the historical calibrated rigid transform; each actual USD agent/camera matrix
must match its manifest within 1e-12 before model requests. No extra full Isaac
preflight launches. No cart relocation or new candidate is permitted.

The exact instruction remains:
“Go to the far end of the hallway. If the path is blocked, pass the supply cart
on your right without touching it. Continue straight after passing it and stop
at the end of the hallway.”

Right/left agreement, lateral sign, initial C0/cart overlap and immediate C1
reaction are not qualification gates. Historical first-response diagnostics may
be retained descriptively; they do not determine selection.

## Collection, speed and budget

One episode / full Isaac launch per candidate, maximum three, no retries. Execute
in manifest order, skipping geometry-invalid candidates without a live launch.
Seal failed attempt files before proceeding. Stop search after the first fully
validated candidate with at least six consecutive applied chunks. Within an
episode never stop merely at six or eight: maximum C0–C11 (12 terminal requests,
11 handoffs), active simulation cap 10.0 s, existing final postroll .1 s.
No C12. Natural raw overlap rejection, genuine STOP, safety guard or technical
termination ends an episode and preserves its valid prefix.

Reuse the exact LONG_SOURCE_01 speed_intervention.json and loader: effective
OBJNAV_V_MAX .4 m/s set before tracker construction and used inside native solve.
No new output scaling/clipping. All other controller settings, H5, dt .1,
nearest+1, limits, weights, solver, memory and generation/stale semantics unchanged.
External source SHA256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.

Reuse unchanged asynchronous loop: 60 Hz integration, 10 Hz submit, 4 Hz RGB,
no simulation pause/catchup. Cart OFF for C0 observation/inference, reveal once at
first scheduled capture strictly after C0 actual application. That exact frame
is C1 observation. Cart ON/static thereafter. Each later request uses the first
eligible scheduled observation after preceding actual application, one terminal
in flight, OLD physically active through inference. No replay, reset after
initialization, frame substitution or view search. All rows untimed;
`waypoint_dt=null`. World reference is its own observation A times immutable raw;
B is actual first command application and never a re-anchor.

Whole raw reference safety excludes any invented connector. Guard checks proposed
commands before application, with actual executed interval recheck. Legacy .05 m
reserve is diagnostic only. Rejected/STOP chunks retain exact model records but
have no application/B/active interval/next request. STOP creates no fake path.

## Frozen qualification and categories

Require consecutive applied prefix >=6, safe applied references and actual prefix,
no skipped/rejected chunk inside it, no later reset, new causal observation and
OLD-active inference, valid source/history/timing/controller/guard provenance,
and >=1 cart pixel in at least one post-reveal terminal RGB. STABLE and EVOLVING,
left and right, all qualify equally. Safety-aborted proposals remain unapplied;
a valid six-plus prefix may qualify if all its provenance/execution gates pass.

Priority: infrastructure/controller/timing/provenance failure →
TECHNICAL_EXECUTION_BLOCKED; all gates with 12 / 8–11 / 6–7 applied →
SUCCESSIVE_SOURCE_MAX_REACHED / SUCCESSIVE_SOURCE_TARGET_REACHED /
SUCCESSIVE_SOURCE_MINIMUM_REACHED. Genuine STOP before six →
MODEL_STOP_BEFORE_MINIMUM. Other 2–5 prefixes → PARTIAL_SUCCESSIVE_SOURCE.
Otherwise candidate is NO_USABLE_SUCCESSIVE_SOURCE; bounded search with no
selectable candidate uses that same overall category, preserving technical and
natural termination details per attempt. No side-based category.

Every generated adjacent pair including C0→C1 uses the old evolution thresholds:
symmetric local separation >=.02 m OR absolute endpoint lateral delta >=.02 m OR
wrapped endpoint/net yaw delta >=5 degrees. First meaningful pair's later index
is k_react, descriptive only. Record its observation, pixels, A, distance from
start and actual travelled arc. No evolution remains a valid stable source.

## Saved-only outputs and validation

Validate all candidate geometry, immutable source/runtime hashes, exact initial
pose/cart, request/history chronology, observation anchors, actual application
boundaries and OLD inference, memory/commands, .4 m/s bound, safe reference and
execution geometry, corrected scheduler, consecutive prefix and handoff bundle.
Bundle every actually applied handoff, including candidate/episode IDs, OLD and
FRESH raw/world hashes, A/B/P, distinct clocks, u_minus, controller memory, first
command, actual OLD-to-B states, RGB/history/request/response hashes, generation,
freeze and model provenance. No generated-only B.

Four final PNGs for first qualified source; if none qualifies, last completed
candidate is shown as descriptive unsuccessful acquisition. Keep all attempt
JSON/bundles and search accounting. World plot equal axes with raw dashed curves,
A/B, cart/.20/.25 contours, rejected label and actual segments colored by active
command identity. All local chunks, event/active timeline, and exact request RGB
pixels with external metadata. `chunks.csv` is the compact source-summary table,
including active duration, endpoints, clocks, pixels and clearance. No extra
method plots, HTML or scientific calls during validation/reporting.

Test synthetic audit A–E, saved historical parity, 12 requests/no C12, causal
first frames/one in-flight, rejection/STOP, direction independence, bounded search
and immutable failed attempts, geometry/camera, unchanged speed/controller memory,
2/6/12-chunk reporting, historical regressions, compileall and diff check. Freeze
code, config, candidate poses, input/source hashes; commit, normal push, verify
HEAD==origin/main before scientific launch. Result/docs commit follows saved-only
validation and PNG inspection. Do not implement reconciliation or next stage.
