# OLD-consistent successive LightNav observation pilot

Decision: **ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_VALIDATED**.

All five frozen diagnostic cases produced actual new Isaac RGB1 observations and
five genuine independent final LightNav sessions. Every final seq0 OLD exactly
reproduced its planning OLD in shape, dtype, values and deterministic raw-array
hash. Valid cases: **5**; invalid cases: **0**; insufficient OLD arcs: **0**.

Direction/yaw disagreement does not uniformly disappear when the observation
pose follows OLD geometry. Cases 006 and 016 retain about 90 degrees in both
local and windowed direction residuals and about 72 degrees in pose-yaw residual.
Cases 007 and 013 have small local direction residuals but about 74.5 degrees
in the 0.10 m window diagnostic. Their very short first FRESH segments therefore
do not describe the broader FRESH direction. These are descriptive observations,
not good/bad labels, correspondence decisions or navigation failure claims.

## Question and fixed source

Does spatial/direction/yaw disagreement remain when FRESH is observed at a pose
on OLD's intended spatial progression, instead of the previous forced
`R1=R0*[0.30,0,0]` observation displacement?

This is a diagnostic pilot, not a new population screening bank. The user froze
exactly these five cases before new capture or inference:

| Case | Selection provenance |
| --- | --- |
| episode_006 | Prior spatial/directional disagreement |
| episode_007 | Prior OLD-conditioned direction disagreement |
| episode_013 | Prior spatial and tangent-scale diagnostic |
| episode_016 | Prior straight residual decreased under OLD-conditioned continuation |
| episode_027 | Prior benign reference at the selection condition |

The source is `data/robotless_handoff_screening/20260914T101519Z/`; selection
provenance is `data/robotless_old_conditioned_handoff/20260915T021149Z/`.
Both existing complete validators passed before freeze and during final audit.
All **805 source files** and **57 selection-run files** retain the exact same
file sets, sizes and SHA-256 values. Source RGB0, instruction, R0, raw OLD,
OLD world arrays and previous metrics remain unchanged.

Source manifest SHA-256:
`139ce7ad6c212584a0e9d680d81b9dbe12e3be3d6ae7ae0f19bc8df2bd800e33`.

New five-case manifest SHA-256:
`68befb7f528506bf943302af5141ef20156fdcbf6dcb059ff2daa0da276bba69`.

Starting HEAD and fetched origin/main were
`e4bd9c7f6db924296e5cc846c8b5ee8920cb574b`. Repository rules, README, work-log,
screening report and OLD-conditioned report were inspected before implementation.
The final SHA is the introducing commit of this report and is recorded in the
ignored run's `git_completion.json` after the single commit and normal push.
The two preexisting user edits in the Jackal controller and single-chunk configs
are preserved and excluded from this commit.

## Spatial observation construction

World coordinates use metres and right-handed +Z up; yaw is CCW radians, wrapped
into [-pi,pi). Agent +X is forward and +Y left. Transform notation uses column
vectors and `T_target_source`. The two raw predictions remain cumulative local
SE(2) poses, never increments to integrate.

```text
P_old = [R0, O0, O1, ..., O(N-1)]
delta_s_obs = 0.30 m
R1_old = interpolate P_old at XY arc progress 0.30 m
Delta_fixed_to_old = inverse(T_world_fixed_R1) * T_world_R1old
```

R0->O0 is included **once** as an explicit simulation-side spatial connector,
not a model waypoint. Position is linear within the selected arc segment and
yaw follows the shortest wrapped endpoint-yaw difference. Zero-length XY
segments add no spatial arc duration. At an exact vertex, interpolation chooses
the lowest-index positive-length segment ending at that progress. No intrinsic
waypoint timestamps or timed in-place rotation are invented.

If total augmented length is below 0.30 m, the case is
`OLD_ARC_INSUFFICIENT`, with total/available arc, requested progress and reason
retained; R1_old is unavailable and no endpoint clamp or case replacement occurs.
All five actual paths have sufficient length. The pose is an
**OLD-consistent spatial observation surrogate**, not actual robot execution.

The new logical-agent pose is assigned as a double-precision USD matrix. Its
column-vector matrix is transposed for USD's row-vector convention. This changes
only representation precision, not the requested transform. Actual pose readback
is checked at unchanged absolute tolerance 1e-7; all five final readbacks equal
the planned pose **exactly** in the saved float64 arrays.

## All five poses and interpolation records

Pose columns are [world x m, world y m, yaw rad]. The actual new capture pose
equals R1_old in every row. The relative transform is expressed in fixed R1's
local axes, with metres/radians. Full-precision values remain in
[manifest_frozen.json](../data/robotless_old_consistent_observation/20260915T043415Z/manifest_frozen.json).


| Case | R0 | Previous fixed R1 | R1_old / actual readback | inverse(fixed R1) * R1_old |
| --- | --- | --- | --- | --- |
| episode_006 | [19, 25.7, 1.57079632679] | [19, 26, 1.57079632679] | [18.7109269123, 25.6294518241, -2.8881586975] | [-0.37054817587, 0.289073087684, 1.82423028288] |
| episode_007 | [19.2, 26, 1.57079632679] | [19.2, 26.3, 1.57079632679] | [18.9109269123, 25.9294518241, -2.8881586975] | [-0.37054817587, 0.289073087684, 1.82423028288] |
| episode_013 | [19.2, 27.9, -1.57079632679] | [19.2, 27.6, -1.57079632679] | [18.9109377988, 27.8223105188, -2.88338775584] | [-0.222310518751, -0.289062201168, -1.31259142905] |
| episode_016 | [-30.4, 6.9, 0] | [-30.1, 6.9, 0] | [-30.4771642806, 6.6189316525, -1.83225134457] | [-0.377164280628, -0.281068347501, -1.83225134457] |
| episode_027 | [9.6, 6.9, 3.14159265359] | [9.3, 6.9, -3.14159265359] | [9.34023062399, 7.05002757806, 2.61781316406] | [-0.0402306239906, -0.150027578056, -0.523779489533] |


All target progress values are 0.30 m. Segment indices are zero-based on the augmented path; segment 0 is the explicit connector.


| Case | Connector m | Model-only OLD arc m | Augmented total m | Segment | Alpha | Remaining m |
| --- | --- | --- | --- | --- | --- | --- |
| episode_006 | 0.000250972735129 | 0.702847467202 | 0.703098439937 | 6 | 0.923852056834 | 0.403098439937 |
| episode_007 | 0.000250972735129 | 0.702847467202 | 0.703098439937 | 6 | 0.923852056834 | 0.403098439937 |
| episode_013 | 0.000310137951969 | 0.867735306694 | 0.868045444645 | 5 | 0.98917930342 | 0.568045444645 |
| episode_016 | 0.00621522156045 | 0.742282611184 | 0.748497832744 | 6 | 0.935434686394 | 0.448497832744 |
| episode_027 | 1.17016658125e-05 | 1.35380412471 | 1.35381582637 | 2 | 0.993224509029 | 1.05381582637 |


## Actual RGB capture and final session protocol

The final five new RGB1 images were captured in one Isaac 6.0.1 process and one
Hospital scene load. Source RGB0 was **not recaptured**: its original JPEG bytes
were copied exactly for the unchanged successive-client path contract, and its
complete original observation metadata/timestamp was reused unchanged.

The same logical agent and child camera use [0.09,0,0.65] m agent-relative
translation. Camera optical-forward/image-right/image-up map to agent +X/-Y/+Z.
The source and new captures have identical configured camera parameters, actual
intrinsics and agent-relative extrinsics (the latter within 1e-10 roundoff).
Images are RGB JPEG 480x270, approximately 112.2-degree horizontal FOV, with
fx/fy approximately 161.2733 pixels and principal point [240,135]. Every capture
records world camera/agent matrices, requested pose, actual before/after pose,
readback timestamp, JPEG hash and source link.

The final process inventory has 1936 prims and 126 collision prims, with zero
robot-named paths, articulations, rigid bodies and physics scenes. Timeline
remained stopped at zero. Logical poses were assigned directly; no physics,
controller or inference ran during capture.

For **each** episode the existing official successive client executed:

```text
connect -> login -> reset
        -> next(seq=0, SOURCE RGB0, unchanged instruction)
        -> next(seq=1, NEW RGB1 at R1_old, unchanged instruction)
        -> disconnect
```

There are five distinct connection IDs, one login/reset and two next requests
per connection, no internal reset/reconnect/retry, and response history steps
1 then 2. Exact request/response wire bytes, decoded arrays and send/receive
clocks are preserved. All ten final outputs are finite float64 (10,3), stop=false.
One persistent external official server served all five sessions and was stopped
using its recorded process identity before visualization. Its built-in startup
warm-up is separate from these ten observation requests and is not evidence.

The unmodified official checkout remains clean at
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`; checkpoint
`LightOriginsHQ/LightNav-0` revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`.
The model file SHA-256 is
`ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
All 16 checkpoint files were rehashed after inference; bytes, sizes and mtimes
match the server-start inventory. No upstream source, weight or environment
was changed.

Host UTC and monotonic clocks are recorded separately; capture timestamps denote
render-readback completion, not hardware exposure. RGB0 retains its original
source time. All new captures precede final-session requests. Request RTT is
recorded only; it does not schedule motion or represent model-only latency.
Execution time and waypoint time base are explicitly null.


| Event | UTC |
| --- | --- |
| Manifest frozen | 2026-09-15T04:34:17.521940Z |
| Final new RGB1 batch complete | 2026-09-15T04:36:20.030977Z |
| Final inference batch starts | 2026-09-15T04:40:26.415444Z |
| Final inference batch complete | 2026-09-15T04:40:49.907020Z |
| Analysis complete | 2026-09-15T04:42:33.672556Z |
| Final overview batch complete | 2026-09-15T04:46:57.218147Z |


| Case | New RGB1 readback UTC | New RGB1 SHA-256 |
| --- | --- | --- |
| episode_006 | 2026-09-15T04:36:19.025410Z | e82a1c67d2da61242bb6b8a285fa9eb61f536c1519f1c402694d1310efbb8e86 |
| episode_007 | 2026-09-15T04:36:19.274992Z | 89719a834c73fb83362e7096e6dd2f0815bd8dda3dcd045b1cabc35631c4a224 |
| episode_013 | 2026-09-15T04:36:19.524055Z | f8f3be7f21e16e8e8a6002187b9bab556e782553ef068e89ced3978cadcfb38b |
| episode_016 | 2026-09-15T04:36:19.772068Z | 5c106492778d356c5f399c7323c44101ff9329ff6a23a7ee98b2d1eb6e0dd1f4 |
| episode_027 | 2026-09-15T04:36:20.023542Z | a6f73eedaaa71872906309ae6dd12dfc6382ae18e205d404a12d4371f03d41b8 |


## Exact planning OLD reproduction and new FRESH hashes

Raw-array hashing is SHA-256 over deterministic `np.save(..., allow_pickle=False)`
bytes, without rounding, dtype conversion or reordering. Shape, dtype, exact
array values and this hash must all match. Raw file hashes are also retained.
A mismatch would record `PLANNING_OLD_MISMATCH` and exclude that case's primary
metrics while preserving FRESH and all failure evidence. All five actual cases
are `PLANNING_OLD_REPRODUCED` and `VALID_PAIR`.

The shared OLD column below is **both** the source OLD hash and final seq0 OLD
hash, which are exactly equal for every row. In 013, new FRESH happens to have
the same raw hash too; equal local arrays at different observation anchors do
not imply equal world trajectories.


| Case | Source OLD = final OLD SHA-256 | New FRESH SHA-256 | Reproduced / status |
| --- | --- | --- | --- |
| episode_006 | 9e0fbd41d976258f361e84b0d06c8e62da0ffd969e86d91f7c7e5523f6665ab6 | 1b74b6c498549b154f49ea5bb54ade7d91e88e246030e221e12ad098e83a70cc | yes / VALID_PAIR |
| episode_007 | 9e0fbd41d976258f361e84b0d06c8e62da0ffd969e86d91f7c7e5523f6665ab6 | cb8000fe4677dc4877586bd4d51907fb82a5bc4b1bc5a284704ed6ee4a2b5e0a | yes / VALID_PAIR |
| episode_013 | 222da8ffc1a495c2d88fd2ee46f0949d13050f6020db8297e6071770f6f10092 | 222da8ffc1a495c2d88fd2ee46f0949d13050f6020db8297e6071770f6f10092 | yes / VALID_PAIR |
| episode_016 | a3733a632901f265037cf1fd0f57a020b813ec96d4bf80bc8543859f675cdf23 | 16bf53115e10aa20998f240442a7f0d98995c949176bba0cbbb4d484f92f73cd | yes / VALID_PAIR |
| episode_027 | a9e2a7e27fb117c59a5ec4f42482cd6ce57f79bf9a8c1a19cbf63e9fde3d1c6b | 203a78fad06ec0ccb3b8bd2f7fd068c281da45ceac7de36cf621a45787c1a0d9 | yes / VALID_PAIR |


## Primary immediate handoff geometry and paired comparison

Only **tau=0** is evaluated. There is no extra 0.25 m continuation and no added
controlled latency:

```text
B = R1_old
T_world_OLD_i = T_world_R0 * T_R0_OLD_i
T_world_FRESH_j = T_world_actual_R1old * T_R1old_FRESH_j
```

FRESH uses its own actual observation readback, which exactly equals the planned
R1_old in this run. It is never anchored at the source fixed R1, an OLD endpoint,
a projection or a later boundary. The unchanged validated `projection_geometry`
finds the continuous closest FRESH XY segment and stores its index, alpha, Q,
e_perp, arc progress, normalized progress and shortest-angle interpolated yaw.
Exact projection ties use the helper's existing lowest-index rule.

Incoming OLD direction is the **geometric tangent of P_old at B**, separate
from yaw(B). Local direction residual is `wrap(phi_fresh_local-phi_old_local)`;
pose-yaw residual is `wrap(theta_Q-yaw(B))`. The nested projection helper's
original yaw(B)-referenced direction field is labeled as a helper reference;
the pilot's top-level local residual is the geometric OLD/FRESH comparison.

Both OLD and FRESH also retain a 0.10 m arc-window tangent: the heading of the
chord from s_minus=max(0,s-.05) to s_plus=min(total,s+.05). This is diagnostic
only and does not replace local tangent. Span or chord <=1e-12 m is numerically
undefined and is reported unavailable with a reason. A zero-length FRESH winner
has unavailable local tangent. All actual local/window values are available.

**Comparison caveat:** previous tau=0 metrics are loaded unchanged from the
fixed-forward source. Their incoming direction is yaw(fixed R1), whereas the
new local residual uses augmented OLD's geometric tangent. Thus the paired
direction difference includes both the observation protocol and this explicit
incoming-reference change. It is not a controlled estimate of observation
position alone. Source tau=1 results are selection/context only; none is used
as a paired baseline here.

The following table contains every frozen case. Distances are metres and
absolute angles are degrees. No thresholds, weighted score or case exclusion
based on residual size are used.


| Case | Previous tau0 e_perp | New e_perp | Previous tau0 abs dir | New local abs dir | New window abs dir | Previous tau0 abs yaw | New abs yaw |
| --- | --- | --- | --- | --- | --- | --- | --- |
| episode_006 | 3.52404667317e-05 | 3.52404667282e-05 | 89.9993747453 | 90.1106542623 | 89.9246318737 | 72.0005279288 | 72.0005279288 |
| episode_007 | 3.70883039787e-05 | 0.000110123546376 | 13.4301262736 | 1.18235200954 | 74.5060815446 | 27.0253905427 | 18.0257947103 |
| episode_013 | 0.000462856646644 | 0.000310137951971 | 90.0081703325 | 0.552479067572 | 74.5383885372 | 72.0666613149 | 17.9972325667 |
| episode_016 | 0.000310137951969 | 0.000462856646645 | 0.533264486061 | 90.520649538 | 90.1390083079 | 17.9972325667 | 72.0666613149 |
| episode_027 | 0.150510373132 | 0.151385084978 | 0.127546213344 | 0.00682017276877 | 0.00577342530685 | 0.0431481459364 | 0.0036176586358 |


Paired differences are new minus previous tau=0, not improvement scores.


| Case | Delta e_perp m | Delta abs local dir deg | Delta abs yaw deg |
| --- | --- | --- | --- |
| episode_006 | -3.52827847234e-15 | 0.111279516944 | -2.41584530158e-13 |
| episode_007 | 7.30352423977e-05 | -12.2477742641 | -8.99959583244 |
| episode_013 | -0.000152718694673 | -89.455691265 | -54.0694287482 |
| episode_016 | 0.000152718694676 | 89.987385052 | 54.0694287482 |
| episode_027 | 0.000874711845964 | -0.120726040575 | -0.0395304873006 |


Local versus window diagnostics depend on the spatial scale of the selected FRESH segment:


| Case | OLD local segment m | FRESH segment | FRESH local segment m | FRESH alpha | FRESH s_Q m | FRESH normalized progress |
| --- | --- | --- | --- | --- | --- | --- |
| episode_006 | 0.154739211981 | 3 | 0.150303970267 | 9.25644924269e-05 | 5.9932089065e-05 | 6.6484308586e-05 |
| episode_007 | 0.154739211981 | 0 | 5.81067249378e-05 | 0 | 0 | 0 |
| episode_013 | 0.150201634181 | 0 | 0.000224793490899 | 0 | 0 | 0 |
| episode_016 | 0.147957661799 | 3 | 0.150026185799 | 0.00314371805677 | 0.00133367530088 | 0.00148678983975 |
| episode_027 | 0.150475087122 | 0 | 0.151119278351 | 0 | 0 | 0 |


## Actual Isaac evidence and line legend

All five final 1280x720 overview screenshots and five new RGB1 JPEGs were
visually inspected. Their exact hashes are bound by `visual_review.json`.
The final visualization process loads the Hospital once and renders saved
geometry only. The logical agent stays at each case's B during its capture.


- [episode_006 actual overview](../data/robotless_old_consistent_observation/20260915T043415Z/evidence/episode_006_overview.png) · [new RGB1](../data/robotless_old_consistent_observation/20260915T043415Z/episodes/episode_006/raw/observation_001.jpg)

- [episode_007 actual overview](../data/robotless_old_consistent_observation/20260915T043415Z/evidence/episode_007_overview.png) · [new RGB1](../data/robotless_old_consistent_observation/20260915T043415Z/episodes/episode_007/raw/observation_001.jpg)

- [episode_013 actual overview](../data/robotless_old_consistent_observation/20260915T043415Z/evidence/episode_013_overview.png) · [new RGB1](../data/robotless_old_consistent_observation/20260915T043415Z/episodes/episode_013/raw/observation_001.jpg)

- [episode_016 actual overview](../data/robotless_old_consistent_observation/20260915T043415Z/evidence/episode_016_overview.png) · [new RGB1](../data/robotless_old_consistent_observation/20260915T043415Z/episodes/episode_016/raw/observation_001.jpg)

- [episode_027 actual overview](../data/robotless_old_consistent_observation/20260915T043415Z/evidence/episode_027_overview.png) · [new RGB1](../data/robotless_old_consistent_observation/20260915T043415Z/episodes/episode_027/raw/observation_001.jpg)


| Display | Meaning |
| --- | --- |
| Blue path | Final OLD world trajectory, exactly reproduced from source OLD |
| Magenta path | New FRESH, transformed only by actual R1_old observation pose |
| Yellow point | R0 |
| Lavender point/line | Previous fixed-forward R1 and R0-to-fixed-R1 displacement |
| Green path | Augmented OLD prefix from R0 to R1_old; explicit connector included |
| Orange point/arrow | R1_old=B and geometric incoming OLD tangent |
| White point/connector | FRESH projection Q and B-to-Q |
| Cyan arrow | FRESH winning-segment tangent |

Raw path Z=.12 m; green prefix Z=.17 m. R0/fixed R1/B/Q markers use Z=.20/.34/
.24/.30 m, and OLD/FRESH tangent arrows use Z=.24/.30 m with .18 m arrow length.
Vertical stems reveal overlapping XY positions. These display-only layers are
not 3D measured distances. XY geometry and angular values are unchanged;
there is no geometry scaling, angle magnification or scene hiding. Camera
positions are saved per case in R0's frame.

## Retained preparation failures and technical corrections

The manifest and all five cases remained fixed throughout. No final inference
session was repeated.

1. The first capture pass produced four RGB1s and a capture failure at 027.
   The shared runtime's float32 Euler rotation stores its requested yaw with a
   calculated -1.1470309e-7 rad error, just beyond the unchanged 1e-7 tolerance.
   That calculation is a diagnosis, not a fabricated pose readback. The pass,
   four images, statuses and log remain under `preparation/capture_float32_yaw/`.
   The pilot now assigns the same requested poses using a double USD matrix;
   the final five-case capture pass has zero pose-readback error. No RGB0 was
   recaptured, no model request preceded this correction, and no source moved.
2. An initial client launch stopped at import before connecting or creating an
   inference-start record: the isolated client has no Matplotlib. Validator
   imports were made lazy so the protocol client needs no plotting dependency.
   No environment or package was installed. The failed launch log is retained.
3. Initial overviews cropped the far FRESH tail at 013. A wider camera trial
   exposed the full tail but was occluded by Hospital geometry at 007. Both
   complete trial image sets are archived in `preparation/initial_overviews/`
   and `preparation/wide_camera_occlusion/`, including the black occluded image.
   The final view uses the clear original camera at 007 and wider cameras
   elsewhere; every final path and required marker is visible. Scene visibility,
   model outputs, case membership and computed metrics were not changed.

Executed versions that differ from final source are retained by SHA-256 in
`processing_source_history/`; current processing sources are also snapshotted.
The post-freeze artifact changes only separated plotting dependencies and made
CSV output retain invalid first rows; they did not change the frozen planning
geometry or analysis formulas. Validators check phase source hashes against
current bytes or their preserved executed snapshots.

## Research interpretation and next uncertainty

006 keeps approximately 90.11/89.92 degrees local/window direction residual,
with a 0.1503 m FRESH winning segment; 016 has about 90.52/90.14 degrees with a
0.1500 m segment. Both retain about 72 degrees of pose-yaw difference. Therefore
these directional disagreements are not explained solely by fixed-forward
observation generation, nor solely by a near-coincident winning FRESH segment.
This conclusion is conditional on the specified connector, interpolation and
camera protocol, without assigning any threshold-based category.

007 and 013 show the opposite of a large-local/small-window artifact: their
local direction residuals are only about 1.18 and 0.55 degrees, while windows
are about 74.51 and 74.54 degrees. Winning FRESH lengths are only about 58.1 and
224.8 micrometres. A small very-local tangent difference therefore does not
establish broader directional agreement. Pose-yaw residuals are about 18 degrees.

At tau=0, spatial distance is small in 006/007/013/016 under both protocols.
The large previous tau=1 spatial residual must not be described as a decrease
caused by this protocol change: that would confound different boundary times.
027 remains a small-direction/yaw reference, but its immediate e_perp is about
0.1514 m, similar to the previous 0.1505 m. This reflects the first forward
FRESH waypoint being ahead of the observation; it is not a failure label.

At B equal to the FRESH observation anchor, e_perp and pose-yaw residual are
invariant to a common SE(2) transformation of B/FRESH. Their changes between
protocols therefore come from the changed local FRESH prediction; the direction
comparison additionally depends on the explicit incoming OLD tangent. The pilot
changes camera position and yaw together, and does not separate their effects.

Next uncertainty is whether these patterns persist over repeated independent
sessions and a broader predeclared collection with the same incoming-direction
reference in both protocols, and how they depend on the connector and spatial
advance. No additional cases, inference repeats or future research stages are
implemented here.

## Validation, reproduction and scope limits

- Full host suite: **1044 passed in 40.57 s**, including **41 new pure/mock tests**.
  The earlier 1040-test pass preceded the additional provenance corruption tests.
  Synthetic/mock fixtures are contract checks, not experimental evidence.
- Tests cover all requested spatial/SE(2), zero-length, exact-vertex, exhaustion,
  raw immutability, exact OLD reproduction/mismatch, own-anchor FRESH, local/
  window, tau=0-only comparison and nonfinite-input contracts. Additional tests
  cover five independent session lifecycles, invalid-first-row CSV retention,
  no overwrite/retry, double USD pose representation and phase-source integrity.
- Compileall, both launcher syntax checks and `git diff --check` passed.
- Actual artifact validator passed: 5 retained/valid cases, 5 unique final
  sessions, zero missing artifacts/failures, both frozen source validators
  unchanged, all images reviewed, no extra primary latency condition.
- Independent scalar audit, importing no repository geometry helpers, checked
  **380 scalar values**, exact OLD arrays and all **862 source/selection files**.
  Maximum numerical difference was **1.1102230246251565e-16**. Projection uses
  the exact saved B after its separate scalar arc-construction check.

Commands (each write phase requires a fresh destination; existing raw outputs
and completed phases refuse overwrite):

```bash
pilot_run=data/robotless_old_consistent_observation/<new_run_id>
MPLCONFIGDIR=/tmp/old-consistent-matplotlib .venv/bin/python scripts/pilot_robotless_old_consistent.py freeze "$pilot_run"
scripts/isaac/run_robotless_old_consistent_observation.sh capture "$pilot_run"
.venv/bin/python scripts/lightnav/robotless_successive_server.py start "$pilot_run" --timeout-s 180
scripts/lightnav/run_robotless_old_consistent_inference.sh "$pilot_run"
.venv/bin/python scripts/lightnav/robotless_successive_server.py stop "$pilot_run" --timeout-s 30
MPLCONFIGDIR=/tmp/old-consistent-matplotlib .venv/bin/python scripts/pilot_robotless_old_consistent.py analyze "$pilot_run"
scripts/isaac/run_robotless_old_consistent_observation.sh visualize "$pilot_run"
# Inspect every actual RGB1/overview and save the hash-bound visual_review.json.
MPLCONFIGDIR=/tmp/old-consistent-matplotlib .venv/bin/python scripts/validate_robotless_old_consistent.py "$pilot_run" --write
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

The actual run is
`data/robotless_old_consistent_observation/20260915T043415Z/`. It retains source
inventories, frozen configuration/manifest, raw final replies and arrays, separate
world arrays/metrics, camera and session metadata, all five case rows, screenshots,
preparation failures, executed source versions, independent audit and logs.
Generated data, screenshots, weights and external sources are not committed.

This pilot establishes genuine successive predictions and their immediate
handoff geometry at OLD-consistent **spatial observation surrogates**. It does
not establish actual robot execution/controller behavior, inference-latency
failure, correspondence correctness, graph necessity/superiority or navigation
success improvement. The five selected cases are not deployment frequencies.
Remote Hospital assets are not recursively content-pinned. No graph optimization,
rigid reconciliation, automatic correspondence, weights, gates, controller or
robot dynamics were implemented.

Final status: **ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_VALIDATED**.
