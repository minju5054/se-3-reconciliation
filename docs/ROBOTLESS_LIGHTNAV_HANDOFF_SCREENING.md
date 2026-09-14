# Robotless LightNav handoff screening across Hospital transitions

Decision: **ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING_VALIDATED**.

All **30 predeclared episodes** produced genuine successive OLD/FRESH responses
from actual Isaac RGBs; invalid episodes: **0**. They contain **10 unique OLD
arrays, 12 unique FRESH arrays and 16 unique ordered raw pairs**. At controlled
delay tau=1 s, the episode median cross-track distance is 0.000139967 m, while
p75 is 0.246077 m and the maximum is 0.249942 m. Median absolute tangent-direction
mismatch is 0.00991582 degrees, p90 is 72.6667 degrees, and the maximum is
75.0703 degrees. The previously observed small mismatch does not describe every
transition in this frozen bank. These are geometric observations, not failure
labels or evidence that reconciliation is necessary.

## Research question and provenance

Does the small handoff mismatch in the preceding single transition also occur
across varied successive LightNav pairs, or do some scene/instruction contexts
repeatedly produce larger spatial, direction or pose-yaw differences?

Starting HEAD and fetched `origin/main` were both
`dff29251d670544445b95f180e7e162a687c78ed`. README, AGENTS, WORK_LOG and the
successive, controlled-staleness and projection reports were read before edits.
The focused completion commit is this document's introducing commit, with the
resolved final SHA and normal push result recorded in the ignored run's
`git_completion.json`; a commit cannot contain its own final hash.

Generated run: [data/robotless_handoff_screening/20260914T101519Z/](../data/robotless_handoff_screening/20260914T101519Z/).
Raw RGBs, raw model outputs/responses, derived arrays, metrics, plots, screenshots,
logs, source snapshots and checkpoint inventories remain outside Git. The two
preexisting changes in `configs/stage0_jackal_controller_validation.yaml` and
`configs/stage0_lightnav_single_chunk.yaml` are preserved and excluded from this
commit. All 131 files in the preceding successive (54), controlled (37) and
projection (40) runs were checked unchanged, including their filename sets.

## Geometry-first bank design and freeze

The scene is Isaac Sim 6.0.1 Hospital, asset
`/Isaac/Environments/Hospital/hospital.usd`. Before any model inference, actual
Hospital RGB views and instance-aware mesh bounds were inspected. The bank uses
the east patient corridor/open west-facing patient-room doorway, the western
hallway fork around a broad column, and the east lobby pillar/reception area.
A north landing resembling a corridor junction, closed or cart-obscured doors,
and blank/wall-intersecting candidate views were rejected during preparation.
No prior LightNav prediction or metric was a scenario selection criterion.

Preparation is preserved under `preparation/inspection*`. One exploratory
inspection ended at a blank candidate without a final completion JSON; its
partial files and log remain. A subsequent geometry-only inspection recorded
that candidate's failure. All 60 final draft R0/R1 views were then rendered and
visually checked in `preparation/bank_camera_check`, before freeze. These are
preparation views, separate from the 60 actual frozen-bank inputs.

[Versioned bank](../configs/robotless_handoff_screening.yaml) and
[full frozen manifest](../data/robotless_handoff_screening/20260914T101519Z/manifest_frozen.json)
contain all 30 ordered IDs, categories, R0, local displacement, R1 and literal
instructions. The common root instruction/agent pose are inherited adapter
fields; the capture loop always substitutes the individual `episodes[]` entry,
and actual wire requests are checked against that entry. Categories describe
visible scene/instruction context, not output classes. Six doorway episodes
sample **one** open doorway at nearby poses; the bank is balanced by category,
not an exhaustive or random sample of Hospital geometry.

Freeze UTC: **2026-09-14T10:31:41.272991Z**. Manifest SHA-256:

`139ce7ad6c212584a0e9d680d81b9dbe12e3be3d6ae7ae0f19bc8df2bd800e33`

`manifest_freeze.json` binds the manifest bytes; the manifest binds the exact
configuration and pre-inference geometry review. Each collection phase checks
those hashes again. No episode, pose, category, instruction or displacement
changed after freeze. No episode was replaced or collected twice.

Every row uses Delta_local=[0.30,0,0], with **R1=R0*Delta_local**. World XY is in
metres and yaw is in radians. Rounded values follow; the manifest retains full
precision. Instruction codes below are report shorthand only.

- I01: Continue straight down the hospital hallway.
- I02: Turn left into the open patient room ahead.
- I03: At the junction, turn left into the hallway.
- I04: Turn right into the open patient room ahead.
- I05: At the junction, turn right into the hallway.
- I06: Go forward through the open patient-room doorway.
- I07: Take the passage to the right of the central pillar in the lobby.
- I08: Pass to the right of the reception desk and continue down the hallway.

| Episode | Category | R0=(x,y,yaw) | R1=(x,y,yaw) | Instruction |
| --- | --- | --- | --- | --- |
| episode_000 | straight | (19, 23.5, -1.57079633) | (19, 23.2, -1.57079633) | I01 |
| episode_001 | straight | (19.2, 22, -1.57079633) | (19.2, 21.7, -1.57079633) | I01 |
| episode_002 | straight | (18.8, 20.5, -1.57079633) | (18.8, 20.2, -1.57079633) | I01 |
| episode_003 | straight | (19, 14, 1.57079633) | (19, 14.3, 1.57079633) | I01 |
| episode_004 | straight | (19.2, 15.5, 1.57079633) | (19.2, 15.8, 1.57079633) | I01 |
| episode_005 | straight | (18.8, 17, 1.57079633) | (18.8, 17.3, 1.57079633) | I01 |
| episode_006 | left_turn | (19, 25.7, 1.57079633) | (19, 26, 1.57079633) | I02 |
| episode_007 | left_turn | (19.2, 26, 1.57079633) | (19.2, 26.3, 1.57079633) | I02 |
| episode_008 | left_turn | (18.9, 26.3, 1.57079633) | (18.9, 26.6, 1.57079633) | I02 |
| episode_009 | left_turn | (-30.8, 7, 0) | (-30.5, 7, 0) | I03 |
| episode_010 | left_turn | (-30.5, 7.15, 0) | (-30.2, 7.15, 0) | I03 |
| episode_011 | left_turn | (-30.2, 6.85, 0) | (-29.9, 6.85, 0) | I03 |
| episode_012 | right_turn | (19, 28.1, -1.57079633) | (19, 27.8, -1.57079633) | I04 |
| episode_013 | right_turn | (19.2, 27.9, -1.57079633) | (19.2, 27.6, -1.57079633) | I04 |
| episode_014 | right_turn | (18.9, 27.6, -1.57079633) | (18.9, 27.3, -1.57079633) | I04 |
| episode_015 | right_turn | (-30.7, 7.1, 0) | (-30.4, 7.1, 0) | I05 |
| episode_016 | right_turn | (-30.4, 6.9, 0) | (-30.1, 6.9, 0) | I05 |
| episode_017 | right_turn | (-30.1, 7, 0) | (-29.8, 7, 0) | I05 |
| episode_018 | doorway | (18.8, 26.8, 3.14159265) | (18.5, 26.8, -3.14159265) | I06 |
| episode_019 | doorway | (18.8, 27, 3.14159265) | (18.5, 27, -3.14159265) | I06 |
| episode_020 | doorway | (19, 26.9, 3.14159265) | (18.7, 26.9, -3.14159265) | I06 |
| episode_021 | doorway | (18.5, 26.9, 3.14159265) | (18.2, 26.9, -3.14159265) | I06 |
| episode_022 | doorway | (19.2, 26.8, 3.14159265) | (18.9, 26.8, -3.14159265) | I06 |
| episode_023 | doorway | (19.2, 27, 3.14159265) | (18.9, 27, -3.14159265) | I06 |
| episode_024 | route_choice | (14.6, 6.9, 3.14159265) | (14.3, 6.9, -3.14159265) | I07 |
| episode_025 | route_choice | (14.3, 7.1, 3.14159265) | (14, 7.1, -3.14159265) | I07 |
| episode_026 | route_choice | (14, 7, 3.14159265) | (13.7, 7, -3.14159265) | I07 |
| episode_027 | route_choice | (9.6, 6.9, 3.14159265) | (9.3, 6.9, -3.14159265) | I08 |
| episode_028 | route_choice | (9.3, 7.1, 3.14159265) | (9, 7.1, -3.14159265) | I08 |
| episode_029 | route_choice | (9, 7, 3.14159265) | (8.7, 7, -3.14159265) | I08 |

## Actual capture, sessions, coordinates and clocks

The actual bank was captured in **one Isaac process / one scene load**, then
inference began after all captures completed. Timeline stayed stopped at 0 s.
Runtime inventory: 1936 prims, 126 collision prims, zero robot-named paths,
articulations, rigid bodies and physics scenes. A logical USD agent/camera pose
was assigned directly between observations; no robot motion was simulated.

World is right-handed +Z up. Agent +X is forward, +Y left, +Z up; yaw is CCW about
+Z. Transform matrices use column vectors and `T_target_source`. USD camera
+X is image-right, +Y image-up, -Z optical-forward. The fixed camera translation
in the agent frame is [0.09,0,0.65] m, and its rotation maps optical-forward to
agent +X, image-right to -Y and image-up to +Z. Logical camera height 0.65 m is
explicitly distinct from the official body-offset value. Actual intrinsics are
saved for both observations of every episode: RGB JPEG 480x270, horizontal FOV
112.1999983 degrees, fx=161.2733123, fy=161.2733097, cx=240, cy=135 pixels,
no distortion. Extrinsics and before/after static pose checks are retained.

For each episode, a new connection performed exactly
`connect -> login -> reset -> next(seq=0, RGB0) -> next(seq=1, RGB1) -> disconnect`.
There are 30 distinct connection IDs and every server response has history
steps 1 then 2. Instruction is identical across the two requests in its session.
No internal reset, reconnect or retry occurred. One persistent official server
served the bank and was stopped afterward using the existing task-owned process
identity guard. The reused shutdown helper's generic reason string mentions a
“two-request session”; the batch/protocol evidence records all 30 such sessions.

Actual raw arrays are all shape (10,3); processing also supports unequal/arbitrary
OLD/FRESH N and tests exercise that support. Raw arrays are cumulative local
poses. No additional integration, observation-to-first-row connector, endpoint
anchor or boundary anchor is introduced:

```text
T_world_OLD_i   = T_world_R0 * T_R0_OLD_i
T_world_FRESH_j = T_world_R1 * T_R1_FRESH_j
```

Every episode preserves both actual JPEGs, response JSONs, protocol wire log,
raw NPY arrays, separate world NPY arrays, camera metadata, observation/send/
receive clocks, server provenance and status. Host UTC and monotonic clocks are
explicit. Observation time means render-readback completion of a bracketed
static capture, not a hardware exposure timestamp. Execution time is null.

| Event | UTC |
| --- | --- |
| Manifest frozen | 2026-09-14T10:31:41.272991Z |
| First actual RGB readback | 2026-09-14T10:32:05.900859Z |
| Capture batch completed | 2026-09-14T10:32:14.637535Z |
| Server readiness confirmed by launcher | 2026-09-14T10:34:44.735361Z |
| Inference batch started | 2026-09-14T10:35:11.646661Z |
| Inference batch completed | 2026-09-14T10:37:31.009062Z |
| Post-shutdown upstream audit | 2026-09-14T10:39:09.533844Z |
| Representative captures completed | 2026-09-14T10:39:45.223329Z |

Measured client RTT for the 60 requests is min **188.823081 ms**, median
**191.677229 ms**, max **204.301606 ms**. RTT is request-send to response-receive
on the host monotonic clock. It is separate from controlled tau, batch duration,
checkpoint verification overhead and simulation time. No actual asynchronous
OLD execution or latency-induced failure rate is measured.

The official checkout remains clean at
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`. Checkpoint is
`LightOriginsHQ/LightNav-0`, revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`; model SHA-256 is
`ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
Final audit rehashed all 16 checkpoint files and verified unchanged bytes, sizes
and mtimes. Source/checkpoint checks before and after each session also passed.

## Counts and exact raw duplication

| Category | Attempted | Valid | Invalid |
| --- | ---: | ---: | ---: |
| straight | 6 | 6 | 0 |
| left_turn | 6 | 6 | 0 |
| right_turn | 6 | 6 | 0 |
| doorway | 6 | 6 | 0 |
| route_choice | 6 | 6 | 0 |
| **Total** | **30** | **30** | **0** |

All statuses are VALID_PAIR, with null failure reasons; there are no unreported
stop/model/protocol/capture/nonfinite exclusions. Invalid attempts would retain
their ID and reason and would not be replaced. Protocol failure preserves the
completed prefix without retry; unavailable geometry is recorded explicitly.

Unique OLD: **10**. Unique FRESH: **12**. Unique ordered pair: **16**.
Most frequent pair: **5/30 = 0.166666667 (16.67%)**, episodes 001–005, hash
`ba08a84e10338fde6293933a29ada67e3ae60911221e8fba7a38801bd2a83398`.

OLD/FRESH hashes are SHA-256 of immutable raw NPY file bytes, without rounding or
tolerance clustering. Ordered pair digest is
`SHA256(b"OLD\0" + OLD_digest_bytes + b"FRESH\0" + FRESH_digest_bytes)`.
The same pair group is used at all four controlled delays.

[Full transition CSV](../data/robotless_handoff_screening/20260914T101519Z/aggregate/transitions.csv)
and JSON contain **all 120 rows**, including each raw/pair hash, category, tau,
B, Q, segment, alpha, distances, signed/absolute angular residuals, progress and
availability flags. [Unique-pair CSV](../data/robotless_handoff_screening/20260914T101519Z/aggregate/unique_pairs.csv)
contains **64 group-by-tau rows** with member IDs/categories, multiplicity,
available counts and each within-pair metric median. Full hashes also appear in
`aggregate/diversity.json`; the plot's group labels map as follows.

| Plot group | Pair SHA-256 prefix | Count | Episode suffixes |
| --- | --- | ---: | --- |
| P01 | `ba08a84e10338fde` | 5 | 001, 002, 003, 004, 005 |
| P02 | `5c99d7cb8793e1f8` | 3 | 020, 022, 023 |
| P03 | `64118d42ea7b2742` | 3 | 006, 009, 011 |
| P04 | `08d497b097305551` | 2 | 007, 008 |
| P05 | `405d83b85fc0cebf` | 2 | 013, 014 |
| P06 | `68bb4aab83466cc3` | 2 | 016, 017 |
| P07 | `815ba6c4e4e50b76` | 2 | 027, 028 |
| P08 | `c2b2183c1a55e3d0` | 2 | 018, 021 |
| P09 | `c7f51ab14f1e4fbd` | 2 | 025, 026 |
| P10 | `33acf740cd9a2b7b` | 1 | 010 |
| P11 | `75250d33a28cdfc7` | 1 | 012 |
| P12 | `796c6f66c139d536` | 1 | 019 |
| P13 | `798f2b7304a068b2` | 1 | 000 |
| P14 | `a569e480369896e6` | 1 | 024 |
| P15 | `c11f6de1d1dee0b2` | 1 | 015 |
| P16 | `e316ad6c66718091` | 1 | 029 |

## Controlled geometry and aggregation

The unchanged validated `robotless_controlled_staleness.characterize` and
`robotless_projection_handoff.characterize_projection` functions are reused:

```text
R_obs = R1
v = 0.25 m/s; omega = 0 rad/s; tau = [0,0.2,0.5,1] s
B(tau) = R_obs * [v*tau,0,0]
```

This is exactly a **controlled body-forward boundary-motion surrogate**. OLD
provides visual context; its predicted endpoint or executed motion does not
define B. FRESH remains fixed at its own observation pose for all tau.

Q is the continuous closest clamped segment projection. Exact distance ties
choose the lowest segment index. e_perp=||B.xy-Q|| includes endpoint distance;
it is not necessarily perpendicular to an infinite line. s_Q is cumulative XY
arc length to Q, normalized progress=s_Q/total_FRESH_arc_length.
phi_in=yaw(B), phi_F is the winning segment tangent, e_dir=wrap(phi_F-phi_in).
theta_Q interpolates endpoint pose yaw along the shortest wrapped angle, and
e_yaw=wrap(theta_Q-yaw(B)); wrap is [-pi,pi). Tangent and pose yaw are separate.
Zero-length winning segments have null tangent/direction with a reason; zero
total arc length is rejected. All 120 actual rows have finite available metrics.
The consistency tolerance 1e-12 m checks equality with prior d_poly, not quality;
the difference is exactly zero in every actual row.

Percentiles use `numpy.percentile(method="linear")` at [0,50,75,90,100].
Episode weighting uses all 30 valid values per tau. Unique-pair weighting takes
one within-pair median for each metric and tau, then equally weights the 16
medians. Each statistic reports available/unavailable counts (30/0 or 16/0 here).
Neither weighting estimates deployment frequency. No threshold, weighted score,
bootstrap or significance test is used. Tables below are rounded to 9 significant
figures; [statistics.json](../data/robotless_handoff_screening/20260914T101519Z/aggregate/statistics.json)
retains full precision.

### Episode-weighted distributions (30 values per condition)

| Metric | Controlled tau (s) | Min | Median | p75 | p90 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| e_perp (m) | 0 | 3.52404667e-05 | 0.150510373 | 0.151303706 | 0.151385085 | 0.151385085 |
| e_perp (m) | 0.2 | 0.0434216803 | 0.100510389 | 0.101303706 | 0.101385086 | 0.101385086 |
| e_perp (m) | 0.5 | 0.0240112024 | 0.0263850906 | 0.122138054 | 0.124249888 | 0.124942118 |
| e_perp (m) | 1 | 4.62390602e-06 | 0.000139966676 | 0.246076792 | 0.2492498 | 0.249942118 |
| abs e_dir (deg) | 0 | 0.00027120758 | 0.00991581733 | 13.4301263 | 90.0002543 | 90.0081703 |
| abs e_dir (deg) | 0.2 | 0.00027120758 | 0.00991581733 | 22.7035481 | 72.6667258 | 75.0702769 |
| abs e_dir (deg) | 0.5 | 0.00027120758 | 0.00991581733 | 22.7035481 | 72.6667258 | 75.0702769 |
| abs e_dir (deg) | 1 | 0.00027120758 | 0.00991581733 | 22.7035481 | 72.6667258 | 75.0702769 |
| abs e_yaw (deg) | 0 | 0.00102295767 | 0.0431481459 | 27.0253905 | 72.0071413 | 72.0666613 |
| abs e_yaw (deg) | 0.2 | 0.00102295767 | 0.0431481459 | 49.455711 | 72.1066264 | 72.5045822 |
| abs e_yaw (deg) | 0.5 | 0.00102295767 | 0.0431481459 | 49.455711 | 72.474773 | 72.5045822 |
| abs e_yaw (deg) | 1 | 0.000292367327 | 0.0649809645 | 49.455711 | 72.5045822 | 73.0883507 |
| normalized progress | 0 | 0 | 0 | 6.64843086e-05 | 0.000522750556 | 0.00148678984 |
| normalized progress | 0.2 | 0 | 0 | 0.000355371282 | 0.00605519028 | 0.0242474055 |
| normalized progress | 0.5 | 0 | 0 | 0.000355371282 | 0.00828197027 | 0.0599950611 |
| normalized progress | 1 | 1.86333339e-05 | 0.0727130982 | 0.0746522775 | 0.0804889558 | 0.119574487 |

### Unique-pair median distributions (16 equally weighted groups)

| Metric | Controlled tau (s) | Min | Median | p75 | p90 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| e_perp (m) | 0 | 3.52404667e-05 | 0.150113399 | 0.15077816 | 0.151385085 | 0.151385085 |
| e_perp (m) | 0.2 | 0.0434216803 | 0.100113422 | 0.10077816 | 0.101385086 | 0.101385086 |
| e_perp (m) | 0.5 | 0.0240112024 | 0.0263850906 | 0.120635165 | 0.124172974 | 0.124942118 |
| e_perp (m) | 1 | 4.62390602e-06 | 0.000202451429 | 0.24246829 | 0.249172876 | 0.249942118 |
| abs e_dir (deg) | 0 | 0.00027120758 | 0.0496788879 | 25.0504663 | 90.0037725 | 90.0081703 |
| abs e_dir (deg) | 0.2 | 0.00027120758 | 0.0496788879 | 24.6999925 | 66.2891061 | 75.0702769 |
| abs e_dir (deg) | 0.5 | 0.00027120758 | 0.0496788879 | 24.6999925 | 66.2891061 | 75.0702769 |
| abs e_dir (deg) | 1 | 0.00027120758 | 0.0496788879 | 24.6999925 | 66.2891061 | 75.0702769 |
| abs e_yaw (deg) | 0 | 0.00102295767 | 0.0491543185 | 33.7464286 | 72.0335946 | 72.0666613 |
| abs e_yaw (deg) | 0.2 | 0.00102295767 | 0.0491543185 | 53.9419176 | 63.5077111 | 72.5045822 |
| abs e_yaw (deg) | 0.5 | 0.00102295767 | 0.0491543185 | 53.9419176 | 64.4412242 | 72.5045822 |
| abs e_yaw (deg) | 1 | 0.000292367327 | 0.108662183 | 53.9419176 | 65.7051951 | 73.0883507 |
| normalized progress | 0 | 0 | 0 | 0.00013815308 | 0.00095121246 | 0.00148678984 |
| normalized progress | 0.2 | 0 | 0 | 0.000467617265 | 0.0101513181 | 0.0242474055 |
| normalized progress | 0.5 | 0 | 0 | 0.000467617265 | 0.021285218 | 0.0599950611 |
| normalized progress | 1 | 1.86333339e-05 | 0.0728523063 | 0.075690229 | 0.0872284448 | 0.119574487 |

## Deterministic representatives and actual Isaac evidence

Selection uses **only tau=1 s**: maximum e_perp, maximum absolute e_dir, maximum
absolute e_yaw, and minimum absolute distance from median e_perp. Exact ties
choose lowest episode ID. This is visualization selection only: all 30 episodes
remain in the tables. Rules B and C select the same episode, leaving three unique
representatives, each with one actual overview and one actual detail PNG.

| Rule | Episode | Selected metric | Value |
| --- | --- | --- | ---: |
| max_e_perp | episode_006 | e_perp (m) | 0.249942118396 |
| max_abs_e_dir | episode_016 | abs e_dir (deg) | 75.0702768731 |
| max_abs_e_yaw | episode_016 | abs e_yaw (deg) | 73.0883506693 |
| nearest_median_e_perp | episode_027 | e_perp (m) | 0.000123017760833 |

Median e_perp target is **0.00013996667595648604 m**; episode_027 is closest.
Rules use full precision, so approximately equal world-transformed duplicate
pairs need not tie exactly at floating-point precision.

| Episode | e_perp (m) | abs e_dir (deg) | abs e_yaw (deg) | Normalized progress | Segment | Alpha |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| episode_006 | 0.249942118396 | 16.7142148659 | 35.9970913412 | 1.86333339298e-05 | 0 | 1 |
| episode_016 | 0.240664039602 | 75.0702768731 | 73.0883506693 | 0.074652277535 | 3 | 0.425856706195 |
| episode_027 | 0.000123017760833 | 0.127546213344 | 0.230980648443 | 0.072737473097 | 0 | 0.659944831345 |

Episode_006's winning segment 0 is only **0.000016796965365369586 m** long,
with alpha=1, s_Q equal to that length, and total FRESH arc length 0.9014471285 m.
Its local tangent is therefore sensitive to the geometry of nearly coincident
rows; it is not the direction of the later, visibly long turning path. We did
not merge points, smooth yaw, invent a minimum-length threshold or change the
projection rule. Episode_016 projects inside segment 3 of length 0.1503720381 m;
its signed direction/yaw differences are -75.07027687/-73.08835067 degrees.
Episode_027 projects inside segment 0 and has submillimetre distance.

| Screenshot feature | Meaning |
| --- | --- |
| Blue line | OLD world poses, anchored at R0; context only |
| Magenta line | FRESH world poses, anchored at R1 |
| Red point | R_obs=R1, fixed observation pose |
| Yellow point/arrow | B(tau=1) and incoming body-forward direction |
| White point | Q, closest continuous projection onto FRESH |
| Green connector | B.xy to Q.xy; length equals e_perp |
| Cyan arrow | Tangent of the winning FRESH segment at Q |
| Orange arrow | Interpolated FRESH pose yaw at Q |

Paths draw at z=.12 m; points/connector at .13 m; incoming/tangent/yaw arrows at
.16/.22/.28 m with lengths .12/.18/.15 m. Vertical stems identify their common
XY origins. These display layers and arrow lengths do not change world XY,
residuals or angles. Geometry scaling and angular magnification are both 1;
no scene objects were hidden. Overview cameras anchor at R_obs; detail cameras
anchor at B. The original projection display convention is retained.

- episode_006: [overview](../data/robotless_handoff_screening/20260914T101519Z/evidence/representative_episode_006_overview.png), [detail](../data/robotless_handoff_screening/20260914T101519Z/evidence/representative_episode_006_detail.png).
- episode_016: [overview](../data/robotless_handoff_screening/20260914T101519Z/evidence/representative_episode_016_overview.png), [detail](../data/robotless_handoff_screening/20260914T101519Z/evidence/representative_episode_016_detail.png).
- episode_027: [overview](../data/robotless_handoff_screening/20260914T101519Z/evidence/representative_episode_027_overview.png), [detail](../data/robotless_handoff_screening/20260914T101519Z/evidence/representative_episode_027_detail.png).

All six actual images were inspected. Episode_016 overview includes partial OLD
context near the lower edge; detail views intentionally concentrate on B/Q and
arrows. The full OLD and FRESH arrays drawn are recorded in `visualization.json`.
Episode_027's 0.123 mm connector and sub-degree angular differences cannot be
measured from these screenshots; numerical tables provide those values. The
legend above is needed because the viewport PNGs do not include the UI panel.

## Plots and visibility of every raw episode

- [e_perp by controlled delay](../data/robotless_handoff_screening/20260914T101519Z/evidence/distribution_cross_track.png).
- [absolute e_dir by controlled delay](../data/robotless_handoff_screening/20260914T101519Z/evidence/distribution_direction.png).
- [absolute e_yaw by controlled delay](../data/robotless_handoff_screening/20260914T101519Z/evidence/distribution_yaw.png).
- [normalized progress by controlled delay](../data/robotless_handoff_screening/20260914T101519Z/evidence/distribution_progress.png).
- [e_perp versus absolute e_dir at controlled tau=1](../data/robotless_handoff_screening/20260914T101519Z/evidence/scatter_cross_track_direction.png).
- [e_perp versus absolute e_yaw at controlled tau=1](../data/robotless_handoff_screening/20260914T101519Z/evidence/scatter_cross_track_yaw.png).
- [exact raw pair multiplicity](../data/robotless_handoff_screening/20260914T101519Z/evidence/pair_multiplicity.png).
- [all raw episode values, separated by ID](../data/robotless_handoff_screening/20260914T101519Z/evidence/episode_metrics_by_id.png).

Each distribution retains 30 raw points per tau with deterministic display-only
horizontal offsets and a black median bar. Axes say controlled delay. Scatter
coordinates are exact; duplicate/near-duplicate points can coincide, and crowded
points in the original distributions can overlap. The extra 4x4 episode-ID
panels show **all 480 individual metric values** at distinct ID positions with
the same metric scale across tau and an external legend. They supplement the
required plots without perturbing any metric. Pair memberships/counts are also
explicit in the full tables. The first supplemental layout had a title/legend
overlap and is archived under `preparation/panels_initial_layout`; final layout
was corrected before review without changing inputs or results.

`plot_manifest.json` and `episode_points_manifest.json` record every plotted
value and source hash. `visual_review.json` binds all 14 final evidence images
(six Isaac screenshots plus eight plots) and all 60 actual RGB hashes to visual
checks. The validator recomputes the point inventory and image hashes.

## Interpretation, limitations and next uncertainty

At tau=1, the episode and unique-pair medians are near zero for distance and
angles, but their upper quantiles remain much larger. The equal-pair p75 e_perp
is 0.242468 m and p90 absolute direction is 66.2891 degrees. Thus the distribution
shape is not merely the fivefold repetition of the dominant straight pair.
The 0.249942 m maximum-distance raw pair recurs in episodes 006/009/011; the
75.0703-degree maximum-direction pair recurs in episodes 016/017. These are
repeated outputs at predeclared related views, not independent estimates of
an environmental frequency. The categories themselves do not prove what motion
the model chose or whether that motion would successfully follow the instruction.

Cross-track distance need not increase with tau: many trajectories start about
0.15 m ahead of their observation anchor, so a forward boundary approaches the
first point, then moves into the polyline. Turning trajectories can diverge
from this deliberately straight body-forward surrogate. At tau=0 the maximum
absolute tangent mismatch is 90.0082 degrees; representative rules still use
tau=1 as specified. Small early spatial distance can coexist with a large angular
residual. Tangent versus pose yaw must remain separate, especially at endpoints
and very short segments such as episode_006's 16.8 micrometre winner.

This is one predeclared balanced Hospital bank, one fixed model/checkpoint,
nearby views within each context, one displacement, and a prescribed straight
surrogate. There is no natural deployment sampling, robot/controller, collision
execution test, real-robot frequency, actual asynchronous execution, actual
latency-induced failure rate, navigation success rate, reconciliation-necessity
threshold, graph superiority or closed-loop improvement claim. No case was
added or altered to obtain a challenging result. No graph optimization,
correspondence factor, rigid correction, objective weight or gate is implemented.

The next uncertainty is how representative these conditional geometries are
under a separately predeclared broader scene/instruction bank, and how local
tangent interpretation depends on near-coincident pose rows. Answering that
would require a new stated experiment; this task does not silently smooth,
resample or expand the bank, and does not implement a future research stage.

## Implementation and reproducibility

The existing `capture_observation` and stopped-timeline check moved verbatim
from the successive script into `robotless_runtime.py` so both entry points
reuse them; independent AST comparison confirms identical function definitions.
The existing successive inference client/server and controlled/projection
geometry modules are unchanged. New code provides immutable bank handling,
30-session orchestration, descriptive summaries, representative views and
strict artifact audits.

Committed changes comprise the frozen YAML; the screening pure module;
geometry-inspection, capture/view, inference, aggregate, supplementary plot and
validation scripts with two launchers; three pure/mock test files; this report,
the append-only work-log entry and the two-sentence README addition. Only the
two existing Isaac files needed the shared-capture refactor. The exact list and
final commit SHA are saved in `git_completion.json` after commit/push.

To reproduce, use a **new output directory** and review actual geometry before
freezing it. Existing bank/capture/inference/aggregate outputs refuse overwrite.
The following are the phase commands used with this run; do not rerun mutation
phases against the completed evidence directory:

```bash
screening_run=data/robotless_handoff_screening/20260914T101519Z
# geometry inspection and 60-view bank check precede this freeze
.venv/bin/python scripts/screen_robotless_handoffs.py freeze "$screening_run" --config configs/robotless_handoff_screening.yaml
scripts/isaac/run_robotless_handoff_screening.sh capture "$screening_run"
.venv/bin/python scripts/lightnav/robotless_successive_server.py start "$screening_run" --timeout-s 180
scripts/lightnav/run_robotless_screening_inference.sh "$screening_run" --timeout-s 120
.venv/bin/python scripts/lightnav/robotless_successive_server.py stop "$screening_run"
.venv/bin/python scripts/screen_robotless_handoffs.py analyze "$screening_run"
scripts/isaac/run_robotless_handoff_screening.sh visualize "$screening_run"
.venv/bin/python scripts/plot_robotless_screening_episode_points.py "$screening_run"
# Record independent visual review and final upstream audit, then read-only check:
.venv/bin/python scripts/validate_robotless_handoff_screening.py "$screening_run"
```

`preparation/geometry_review.json` binds the reviewed bank config before freeze.
The external final audit reuses the existing source/checkpoint provenance
helpers; no external source or weights are modified. Runtime phase source hashes
match the committed processing code. `independent_audit.py` in the generated run
records the separate scalar check and its own source hash; it does not import
project geometry functions. Process/log evidence and pre-inference source
snapshots remain in the generated run. View-only mode avoids rewriting evidence.

## Validation results

- Full required host suite:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest` —
  **958 passed in 37.74 s** (47 tests added over the 911-test baseline).
- `python -m compileall src scripts tests`, both new shell launcher syntax
  checks, and `git diff --check` passed.
- Pure/mock tests cover bank freeze, exactly 30 unique IDs, instruction/session
  ordering, no reconnect, raw preservation, deterministic ordered hashes,
  unequal/arbitrary N, projection reuse, nonfinite rejection, invalid retention,
  linear percentiles, within-pair medians and deterministic representatives.
  Aggregate corruption, session reuse, dropped/mutated plot values and malformed
  PNG regression tests fail closed. These synthetic fixtures are tests only.
- Actual runtime validator passed with no missing artifacts or failures:
  30 actual RGB pairs, 30 distinct seq0/1 sessions, 120 verified metric rows,
  16 pair groups, all required distributions and deterministic actual views.
- Independent scalar arithmetic verified all own-anchor transforms, 120 condition
  rows and 160 summary quantiles: 3340 comparisons, maximum absolute difference
  **1.4210854715202004e-14**. All phase source hashes and 131 prior evidence files
  were unchanged. This numerical tolerance is a consistency check, not a gate.
- All 60 actual RGBs and 14 final evidence images were inspected. A PNG audit
  initially decoded pixels before PIL structural verification; this ordering
  was corrected and regression-tested before the final successful validation.
  It did not change captures or metrics. Retained runtime warnings include
  rendering/readback/plugin-release and temporary Matplotlib cache messages;
  successful artifacts and independent checks establish completion.

Final artifact decision: **ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING_VALIDATED**.
