# Straight versus OLD-conditioned robotless handoff continuation

Decision: **ROBOTLESS_OLD_CONDITIONED_HANDOFF_VALIDATED**.

The straight surrogate influences some residuals substantially, but it does not
explain all disagreement in this fixed bank. At tau=1 s, episode_016 changes from
0.240664 to 0.100690 m cross-track distance and from 75.0703 to 24.3140 degrees
absolute direction difference. Episode_006 retains 0.212648 m and about 58.095
degrees in both local and windowed direction diagnostics. Episode_007 retains
about 71.980 degrees in both diagnostics. These results are conditional on the
specified alignment: the observation-to-OLD anchor distance has median
**0.204968 m** and maximum **0.304540 m**, so many aligned continuations start
from a reference significantly displaced from the observation pose.

## Question, frozen source and integrity

Are the preceding screening residuals mainly a consequence of assuming straight
body-forward continuation, or do OLD/FRESH differences remain when the boundary
uses OLD's relative future geometry?

Source: [frozen screening run](../data/robotless_handoff_screening/20260914T101519Z/),
`data/robotless_handoff_screening/20260914T101519Z/`. The existing full screening
validator passed before analysis and again during final validation. **All 805
source files**, including raw OLD/FRESH arrays and responses, observation RGBs,
world arrays, manifest, previous straight metrics and evidence inventories,
are unchanged. Exact filename sets, sizes and SHA-256 values are retained in
[source.json](../data/robotless_old_conditioned_handoff/20260915T021149Z/source.json).

Source manifest SHA-256:
`139ce7ad6c212584a0e9d680d81b9dbe12e3be3d6ae7ae0f19bc8df2bd800e33`.

Starting HEAD and fetched origin/main were both
`653b9e7fc9cb69c4899e6ea40460b73e3b1ffd4d`. Required README, repository rules,
work-log and screening/projection reports were inspected before edits. The
final SHA is the introducing commit of this report and is written to the
ignored run's `git_completion.json` after commit/push. A commit cannot include
its own final SHA. The preexisting user edits in the Jackal controller and
single-chunk configs are preserved and excluded.

All **30** predeclared episodes remain: six each of straight, left_turn,
right_turn, doorway and route_choice. The source has 10 unique OLD arrays,
12 unique FRESH arrays and **16 ordered raw pairs**; the largest group remains
5/30 episodes. The pair grouping is reused without rounding or recategorization.
No LightNav inference, RGB recollection, model loading, episode replacement or
source-artifact edit occurred. New outputs live in
[data/robotless_old_conditioned_handoff/20260915T021149Z/](../data/robotless_old_conditioned_handoff/20260915T021149Z/).

## Formulation and frame conventions

World XY is in metres, right-handed +Z up; yaw is CCW about +Z in radians.
Transforms use column vectors and `T_target_source`. The source OLD and FRESH
world arrays retain their original R0 and R1 observation anchors. Agent +X is
forward and +Y left. Both arrays are read-only inputs; no integration or FRESH
reanchoring is performed. Their original camera extrinsics/intrinsics and
observation/readiness timestamps remain in the source.

First project R_obs=R1 onto the OLD XY polyline with the **unchanged validated
`projection_geometry`** helper. Save its winning segment, alpha_obs, Q_old_obs,
s_old_obs, normalized OLD progress, segment length and d_obs_to_old. Interpolate
OLD pose yaw along the shortest wrapped angle to form the reference
A_obs=[Q_old_obs.x,Q_old_obs.y,theta_old_obs]. A_obs is not a measured robot pose.

```text
T_align = T_world_Robs * inverse(T_world_Aobs)
C_old(s) = T_align * T_world_A(s)
C_old(s_old_obs) = R_obs

v = 0.25 m/s
tau = [0, 0.2, 0.5, 1] s
requested delta_s = v*tau = [0, 0.05, 0.125, 0.25] m
s_B = s_old_obs + delta_s
B_old(tau) = C_old(s_B)
```

T_align maps source-world OLD geometry into a separate counterfactual
continuation in the same fixed world coordinate axes. It does not modify,
optimize or replace the raw OLD prediction. The green visualization is only
the aligned **future suffix**, starting at A_obs, of this derived surrogate.

At zero delay the projection winner's exact A_obs/segment/alpha are reused,
including its pose-yaw convention. After checking `T_align*A_obs` against R_obs
with absolute numerical tolerance 1e-10, B_old(0) is stored as an exact copy of
R_obs. This removes floating-point composition noise without snapping R_obs
to Q_old_obs. Every actual zero-delay boundary equals its original R_obs exactly.

For positive advance, position is linear within the selected arc-length segment
and pose yaw uses shortest-angle interpolation. At an exact arc vertex the
lowest nonzero segment ending there is chosen. Zero-length segments have no arc
duration and cannot encode timed in-place rotation; at tau=0 a degenerate
projection winner retains null local tangent. No minimum-length filter replaces
short positive segments. Paths with zero total XY arc length are rejected
explicitly, not assigned fabricated progress. Exact distance ties in projection
use the lowest segment index; near ties can change winner under floating-point
perturbation, so no coordinate transform or tolerance grouping is silently used.

This is a **counterfactual OLD-conditioned geometry-based execution surrogate**.
It assigns no intrinsic timestamps to waypoint rows and performs no OLD robot
execution. The previous boundary remains exactly the saved
`B_straight(tau)=R_obs*[v*tau,0,0]`.

## Exhaustion, local direction and window diagnostic

If `requested_delta_s > total_OLD_length - s_old_obs`, the condition is
`OLD_CONTINUATION_EXHAUSTED`; B_old and its handoff metrics are null, with the
requested advance, available remaining length and reason retained. Equality is
allowed; requests beyond the endpoint are never silently clamped. Exhausted
conditions retain their episode and tau and are not replaced. Actual results:

| Controlled tau (s) | Retained conditions | Available | OLD exhausted |
| --- | ---: | ---: | ---: |
| 0 | 30 | 30 | 0 |
| 0.2 | 30 | 30 | 0 |
| 0.5 | 30 | 30 | 0 |
| 1 | 30 | 30 | 0 |

Each available B_old is projected onto the unchanged FRESH world polyline using
the same validated helper. Q, FRESH segment/alpha, e_perp, s_Q, normalized
progress and theta_Q are retained. The OLD-conditioned incoming direction is
the **aligned OLD geometric segment tangent**, not yaw(B_old):

```text
phi_old_local = wrap(yaw(T_align) + tangent_of_OLD_segment_at_s_B)
e_dir_old = wrap(phi_fresh_local - phi_old_local)
e_yaw_old = wrap(theta_fresh_Q - yaw(B_old))
```

The reused helper also emits a yaw(B)-referenced direction difference; the nested
`fresh_projection` explicitly labels that as a helper reference, not e_dir_old.
The top-level `e_dir_old_rad` and `abs_e_dir_old_deg` are the intended comparison.
This distinction matters even at tau=0: position and pose yaw residuals equal
the straight baseline exactly, but direction residuals can differ because the
incoming-direction definition changes without any boundary motion.

For each OLD and FRESH progress s, the fixed **0.10 m** window uses
`s_minus=max(0,s-.05)`, `s_plus=min(total,s+.05)` and the heading of the chord
between interpolated XY positions. OLD's heading is rotated by T_align before
comparison. e_dir_window wraps the FRESH-window minus aligned-OLD-window angle.
Local and window angles, sampled endpoints, arc span, chord length and
availability reasons are all saved separately. Window endpoints are truncated
at path ends by definition; this is distinct from forbidden continuation clamp.

A window with arc span or chord length at or below **1e-12 m** is unavailable
with an explicit reason. These numerical resolutions avoid undefined headings;
they are not quality thresholds and exclude no actual episode/condition here.
All 120 actual local and window residuals are finite and available. A windowed
heading is a diagnostic at a different spatial scale, not a replacement tangent
or an inferred correspondence.

## Anchor diagnostics

No anchor-distance threshold removes episodes. All reference poses, projection
segment lengths/progress and remaining lengths are in
[anchor_diagnostics.csv](../data/robotless_old_conditioned_handoff/20260915T021149Z/derived/anchor_diagnostics.csv).
The table below summarizes d_obs_to_old in metres.

| Weighting | Min | Median | p75 | p90 | Max |
| --- | ---: | ---: | ---: | ---: | ---: |
| 30 episodes | 4.28249788333e-06 | 0.204967663787 | 0.29787964761 | 0.299709538721 | 0.30453964934 |
| 16 within-pair medians | 4.28249788333e-06 | 0.274437464538 | 0.29787964761 | 0.301856254552 | 0.30453964934 |

The camera observations were originally separated by the fixed 0.30 m
body-forward displacement, regardless of OLD's suggested turn. Therefore an
OLD reference can be about 0.30 m away, and alignment may rotate its future
relative motion appreciably. The results must be read conditional on this
reference construction, not as proof of actual executable motion from R_obs.

## Aggregate and paired distributions

[episode_metrics.json](../data/robotless_old_conditioned_handoff/20260915T021149Z/derived/episode_metrics.json)
retains 30 complete episode records; its CSV contains **all 120 conditions**,
including exact nested previous straight rows, new geometry, directions,
window diagnostics and availability. [unique_pairs.csv](../data/robotless_old_conditioned_handoff/20260915T021149Z/derived/unique_pairs.csv)
contains 64 pair-by-tau rows, member IDs, multiplicities, metric medians and
available counts. No top-N subset replaces these tables.

Each paired difference is **OLD-conditioned minus straight** for the same
episode and tau. Negative means a smaller residual under that surrogate;
positive means a larger residual. It is a paired difference, not an improvement
score. In particular, median(new-old) is not generally median(new)-median(old).
Unique-pair aggregation takes within-pair medians of each metric **and each
paired difference separately**, then gives the 16 pair medians equal weight.
These are descriptive bank distributions, not deployment-frequency estimates.
No bootstrap, significance test, quality threshold or weighted score is used.

All tables use linear percentiles, rounded to 9 significant figures. Each
statistic has available/unavailable counts: 30/0 for episode weighting and
16/0 for pair weighting. The full-precision
[statistics.json](../data/robotless_old_conditioned_handoff/20260915T021149Z/derived/statistics.json)
remains the numerical reference. Angular columns are absolute degrees unless
marked as a signed paired difference; e_perp and its difference are metres.

### Episode-weighted results

| Metric | Controlled tau (s) | Min | Median | p75 | p90 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Straight e_perp (m) | 0 | 3.52404667e-05 | 0.150510373 | 0.151303706 | 0.151385085 | 0.151385085 |
| Straight e_perp (m) | 0.2 | 0.0434216803 | 0.100510389 | 0.101303706 | 0.101385086 | 0.101385086 |
| Straight e_perp (m) | 0.5 | 0.0240112024 | 0.0263850906 | 0.122138054 | 0.124249888 | 0.124942118 |
| Straight e_perp (m) | 1 | 4.62390602e-06 | 0.000139966676 | 0.246076792 | 0.2492498 | 0.249942118 |
| OLD e_perp (m) | 0 | 3.52404667e-05 | 0.150510373 | 0.151303706 | 0.151385085 | 0.151385085 |
| OLD e_perp (m) | 0.2 | 0.0171981379 | 0.100510389 | 0.101385087 | 0.104080382 | 0.127489673 |
| OLD e_perp (m) | 0.5 | 0.0240112371 | 0.0320510092 | 0.106397327 | 0.121165665 | 0.12418022 |
| OLD e_perp (m) | 1 | 7.63714708e-06 | 0.0248817497 | 0.207666132 | 0.23710097 | 0.249179908 |
| Straight abs direction (deg) | 0 | 0.00027120758 | 0.00991581733 | 13.4301263 | 90.0002543 | 90.0081703 |
| Straight abs direction (deg) | 0.2 | 0.00027120758 | 0.00991581733 | 22.7035481 | 72.6667258 | 75.0702769 |
| Straight abs direction (deg) | 0.5 | 0.00027120758 | 0.00991581733 | 22.7035481 | 72.6667258 | 75.0702769 |
| Straight abs direction (deg) | 1 | 0.00027120758 | 0.00991581733 | 22.7035481 | 72.6667258 | 75.0702769 |
| OLD abs local direction (deg) | 0 | 8.29658456e-05 | 6.6977872 | 73.912774 | 89.8372312 | 89.9879247 |
| OLD abs local direction (deg) | 0.2 | 0.000429061417 | 5.68712653 | 34.1885691 | 58.6194295 | 72.5045822 |
| OLD abs local direction (deg) | 0.5 | 0.000429061417 | 5.68712653 | 34.1885691 | 58.6194295 | 72.5045822 |
| OLD abs local direction (deg) | 1 | 0.000610460765 | 5.85537894 | 34.0439961 | 58.0945404 | 71.9796931 |
| OLD abs window direction (deg) | 0 | 0.000254609939 | 3.19879164 | 58.7901611 | 73.9173983 | 90.0277711 |
| OLD abs window direction (deg) | 0.2 | 0.000422099831 | 5.68668612 | 56.6735787 | 71.9717085 | 90.037974 |
| OLD abs window direction (deg) | 0.5 | 0.000472110215 | 5.72459081 | 56.413114 | 72.5399957 | 90.040582 |
| OLD abs window direction (deg) | 1 | 0.000610460763 | 5.85537894 | 56.2681287 | 73.6202868 | 90.0484583 |
| Straight abs yaw (deg) | 0 | 0.00102295767 | 0.0431481459 | 27.0253905 | 72.0071413 | 72.0666613 |
| Straight abs yaw (deg) | 0.2 | 0.00102295767 | 0.0431481459 | 49.455711 | 72.1066264 | 72.5045822 |
| Straight abs yaw (deg) | 0.5 | 0.00102295767 | 0.0431481459 | 49.455711 | 72.474773 | 72.5045822 |
| Straight abs yaw (deg) | 1 | 0.000292367327 | 0.0649809645 | 49.455711 | 72.5045822 | 73.0883507 |
| OLD abs yaw (deg) | 0 | 0.00102295767 | 0.0431481459 | 27.0253905 | 72.0071413 | 72.0666613 |
| OLD abs yaw (deg) | 0.2 | 0.000637707894 | 1.92403174 | 50.0073479 | 53.0839025 | 69.3126871 |
| OLD abs yaw (deg) | 0.5 | 5.25169027e-05 | 4.74261174 | 49.9757463 | 52.5344584 | 64.662789 |
| OLD abs yaw (deg) | 1 | 0.000303043885 | 5.93594519 | 51.8663281 | 55.7864576 | 65.3802121 |
| Paired delta e_perp (m) | 0 | 0 | 0 | 0 | 0 | 0 |
| Paired delta e_perp (m) | 0.2 | -0.0302173902 | 4.28081993e-10 | 3.20665484e-09 | 0.00269529679 | 0.0269357751 |
| Paired delta e_perp (m) | 0.5 | -0.0709497406 | 3.63250537e-09 | 3.04955886e-08 | 0.0165125647 | 0.0952774864 |
| Paired delta e_perp (m) | 1 | -0.139974316 | 3.9496175e-06 | 1.55856542e-05 | 0.0590350135 | 0.192471647 |
| Paired delta abs direction (deg) | 0 | -1.73649515 | 0.000653150546 | 0.00274744846 | 25.3943384 | 78.6259254 |
| Paired delta abs direction (deg) | 0.2 | -50.0370009 | 0.00344215433 | 1.73649515 | 41.9052147 | 49.8619639 |
| Paired delta abs direction (deg) | 0.5 | -50.0370009 | 0.00344215433 | 1.8529528 | 41.9052147 | 49.8619639 |
| Paired delta abs direction (deg) | 1 | -50.7562277 | 0.00182973484 | 1.59149979 | 41.3803256 | 50.6976484 |
| Paired delta abs yaw (deg) | 0 | 0 | 0 | 0 | 0 | 0 |
| Paired delta abs yaw (deg) | 0.2 | -41.8010414 | 0.000385249776 | 0.00467350728 | 17.3057414 | 42.2989047 |
| Paired delta abs yaw (deg) | 0.5 | -46.6597065 | 0.0014269396 | 0.0122429267 | 16.4307538 | 48.0841421 |
| Paired delta abs yaw (deg) | 1 | -49.185215 | 0.00192908418 | 0.0160500141 | 21.4123619 | 50.8573377 |

### Unique-pair-weighted results

| Metric | Controlled tau (s) | Min | Median | p75 | p90 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Straight e_perp (m) | 0 | 3.52404667e-05 | 0.150113399 | 0.15077816 | 0.151385085 | 0.151385085 |
| Straight e_perp (m) | 0.2 | 0.0434216803 | 0.100113422 | 0.10077816 | 0.101385086 | 0.101385086 |
| Straight e_perp (m) | 0.5 | 0.0240112024 | 0.0263850906 | 0.120635165 | 0.124172974 | 0.124942118 |
| Straight e_perp (m) | 1 | 4.62390602e-06 | 0.000202451429 | 0.24246829 | 0.249172876 | 0.249942118 |
| OLD e_perp (m) | 0 | 3.52404667e-05 | 0.150113399 | 0.15077816 | 0.151385085 | 0.151385085 |
| OLD e_perp (m) | 0.2 | 0.0171981379 | 0.100113422 | 0.101140948 | 0.109169759 | 0.127489673 |
| OLD e_perp (m) | 0.5 | 0.0240112371 | 0.0433254487 | 0.109001078 | 0.122502249 | 0.12418022 |
| OLD e_perp (m) | 1 | 7.63714708e-06 | 0.0749904719 | 0.1977018 | 0.242466256 | 0.249179908 |
| Straight abs direction (deg) | 0 | 0.00027120758 | 0.0496788879 | 25.0504663 | 90.0037725 | 90.0081703 |
| Straight abs direction (deg) | 0.2 | 0.00027120758 | 0.0496788879 | 24.6999925 | 66.2891061 | 75.0702769 |
| Straight abs direction (deg) | 0.5 | 0.00027120758 | 0.0496788879 | 24.6999925 | 66.2891061 | 75.0702769 |
| Straight abs direction (deg) | 1 | 0.00027120758 | 0.0496788879 | 24.6999925 | 66.2891061 | 75.0702769 |
| OLD abs local direction (deg) | 0 | 8.29658456e-05 | 18.6235632 | 64.4704865 | 89.0544532 | 89.9879247 |
| OLD abs local direction (deg) | 0.2 | 0.000429061417 | 17.972739 | 30.4172171 | 54.284456 | 72.5045822 |
| OLD abs local direction (deg) | 0.5 | 0.000429061417 | 17.972739 | 30.4172171 | 54.284456 | 72.5045822 |
| OLD abs local direction (deg) | 1 | 0.000610460765 | 17.9414173 | 30.0348142 | 54.4398537 | 71.9796931 |
| OLD abs window direction (deg) | 0 | 0.000254609939 | 14.7238276 | 53.6816051 | 80.3105194 | 90.0277711 |
| OLD abs window direction (deg) | 0.2 | 0.000422099831 | 17.21431 | 53.1870987 | 79.2160896 | 90.037974 |
| OLD abs window direction (deg) | 0.5 | 0.000472110215 | 18.0898265 | 52.2287376 | 79.493786 | 90.040582 |
| OLD abs window direction (deg) | 1 | 0.000610460763 | 17.9414173 | 52.6128208 | 80.1826616 | 90.0484583 |
| Straight abs yaw (deg) | 0 | 0.00102295767 | 0.0491543185 | 33.7464286 | 72.0335946 | 72.0666613 |
| Straight abs yaw (deg) | 0.2 | 0.00102295767 | 0.0491543185 | 53.9419176 | 63.5077111 | 72.5045822 |
| Straight abs yaw (deg) | 0.5 | 0.00102295767 | 0.0491543185 | 53.9419176 | 64.4412242 | 72.5045822 |
| Straight abs yaw (deg) | 1 | 0.000292367327 | 0.108662183 | 53.9419176 | 65.7051951 | 73.0883507 |
| OLD abs yaw (deg) | 0 | 0.00102295767 | 0.0491543185 | 33.7464286 | 72.0335946 | 72.0666613 |
| OLD abs yaw (deg) | 0.2 | 0.000637707894 | 13.8851043 | 44.9051594 | 53.4623792 | 69.3126871 |
| OLD abs yaw (deg) | 0.5 | 5.25169027e-05 | 17.6064878 | 48.7514505 | 53.1585239 | 64.662789 |
| OLD abs yaw (deg) | 1 | 0.000303043885 | 17.7639977 | 51.2952301 | 54.8712772 | 65.3802121 |
| Paired delta e_perp (m) | 0 | 0 | 0 | 0 | 0 | 0 |
| Paired delta e_perp (m) | 0.2 | -0.0302173902 | 4.28081993e-10 | 1.7961578e-06 | 0.00778467299 | 0.0269357751 |
| Paired delta e_perp (m) | 0.5 | -0.0709497406 | 3.63250537e-09 | 1.83742288e-06 | 0.0372355131 | 0.0952774864 |
| Paired delta e_perp (m) | 1 | -0.139974316 | 3.9496175e-06 | 2.04067381e-05 | 0.0981409139 | 0.192471647 |
| Paired delta abs direction (deg) | 0 | -1.73649515 | 0.000349318951 | 5.86415753 | 29.6222073 | 78.6259254 |
| Paired delta abs direction (deg) | 0.2 | -50.0370009 | 0.00218255899 | 4.11150435 | 38.9857337 | 49.8619639 |
| Paired delta abs direction (deg) | 0.5 | -50.0370009 | 0.00218255899 | 4.19884759 | 38.9857337 | 49.8619639 |
| Paired delta abs direction (deg) | 1 | -50.7562277 | 0.00182973484 | 4.0838231 | 38.7104602 | 50.6976484 |
| Paired delta abs yaw (deg) | 0 | 0 | 0 | 0 | 0 | 0 |
| Paired delta abs yaw (deg) | 0.2 | -41.8010414 | 0.000179107113 | 0.942934267 | 20.2833037 | 42.2989047 |
| Paired delta abs yaw (deg) | 0.5 | -46.6597065 | 0.000451826089 | 2.35775504 | 23.7905572 | 48.0841421 |
| Paired delta abs yaw (deg) | 1 | -49.185215 | 0.00192908424 | 2.91616653 | 27.9043444 | 50.8573377 |

## Critical cases at controlled tau=1 s

All three requested cases remain available. Triples below are e_perp in metres,
absolute local direction in degrees, absolute pose-yaw residual in degrees.
The window column is absolute windowed direction in degrees.

| Episode | Straight (distance / direction / yaw) | OLD-conditioned (distance / direction / yaw) | Window direction | R_obs-to-OLD (m) | Availability |
| --- | --- | --- | ---: | ---: | --- |
| episode_006 | 0.249942118 / 16.7142149 / 35.9970913 | 0.212648298 / 58.0945404 / 55.7864576 | 58.0957826 | 0.297879648 | AVAILABLE |
| episode_016 | 0.24066404 / 75.0702769 / 73.0883507 | 0.100689724 / 24.3140492 / 23.9031357 | 24.3140492 | 0.304539649 | AVAILABLE |
| episode_027 | 0.000123017761 / 0.127546213 / 0.230980648 | 0.000143424485 / 0.141972515 / 0.247030663 | 0.141972515 | 0.150036566 | AVAILABLE |

- **006:** distance falls by 0.0372938 m, while absolute local direction rises
  by 41.3803 degrees and yaw by 19.7894 degrees. At the new B, the winning FRESH
  segment is 0.150304 m long and the OLD incoming segment is 0.154739 m long.
  Thus this 58-degree local/window residual is not merely the earlier
  straight-boundary winner's 16.8 micrometre segment. The anchor gap is 0.297880 m.
- **016:** distance falls by 0.139974 m, local direction by 50.7562 degrees and
  yaw by 49.1852 degrees. Local/window residuals both remain 24.3140 degrees,
  with OLD/FRESH segment lengths 0.147958/0.150202 m. The straight assumption
  substantially affects this case, but the specified OLD-conditioned result
  does not collapse to zero. Its 0.304540 m anchor gap is the bank maximum.
- **027:** both surrogates retain small numerical spatial/angular residuals:
  distance changes by +0.0000204067 m, direction by +0.0144263 degrees and yaw
  by +0.0160500 degrees. Local and window agree to numerical precision. Its
  anchor gap is 0.150037 m, even though the aligned FRESH handoff is close.

[critical_cases.csv](../data/robotless_old_conditioned_handoff/20260915T021149Z/derived/critical_cases.csv)
retains the full rows plus each requested anchor distance.

## Deterministic representatives and Isaac evidence

The two previous cases 006/016 are mandatory. Additional tau=1 rules choose
maximum OLD-conditioned e_perp, maximum absolute local direction, and largest
absolute paired change in absolute local direction. Exact ties choose lowest
ID. These rules select **006, 007, 013, 016** after deduplication; no manually
chosen case is substituted.

| Rule | Episode | Actual selected value |
| --- | --- | --- |
| Previous maximum distance | 006 | Source e_perp 0.249942118396 m |
| Previous maximum direction | 016 | Source abs direction 75.0702768731 deg |
| Maximum OLD-conditioned distance | 013 | 0.249179908167 m |
| Maximum OLD-conditioned abs local direction | 007 | 71.9796931279 deg |
| Largest absolute direction change | 016 | abs(-50.7562277170) = 50.7562277170 deg |

Actual Isaac Sim 6.0.1 loaded the unchanged Hospital scene, using saved geometry
only. Each rendering process loaded the scene once. Runtime inventory is 1936
prims, 126 collision prims, no robots, articulation roots, rigid bodies or
physics scenes. Timeline remained stopped at 0 s; the logical agent stayed at
that episode's R_obs during each overview/detail pair. Four representatives
produce **eight final actual viewport PNGs**.

| Feature | Meaning |
| --- | --- |
| Blue path | Original OLD world path, still anchored at its original observation |
| Magenta path | Original fixed FRESH world path |
| Green path | Derived aligned OLD future suffix beginning at R_obs |
| Red point | R_obs |
| Lavender point and thin connector | OLD projection Q_old_obs and its gap to R_obs |
| Yellow point/stem | B_straight(tau=1) |
| Orange point/arrow | B_old(tau=1) and aligned OLD local tangent |
| White point/connector | Q on FRESH and B_old-to-Q distance |
| Cyan arrow | FRESH local tangent at Q |

Raw paths draw at z=.12 m, aligned future at .17 m. OLD/FRESH tangent arrows
use z=.23/.29 m and length .18 m. Marker drawing heights are R_obs .14,
Q_old_obs .13, B_straight .34, B_old .145 and Q_fresh .20 m. Vertical stems
connect elevated markers/arrows to their actual XY origins. All world XY and
angles are unchanged; these heights are display layers, not a 3D distance or
physical boundary lift. No geometry scaling, angular magnification or scene
hiding was used.

The initial render made near-coincident B_straight/B_old and R_obs/Q markers
hard to distinguish in case013. Its eight images and metadata are preserved
under `preparation/initial_marker_overlap`. The final rendering changed only
marker drawing heights/stems, without changing the analysis or source data.
The final actual viewport completion was 2026-09-15T02:15:35.826541Z.

- episode_006: [overview](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_006_overview.png), [detail](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_006_detail.png).
- episode_007: [overview](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_007_overview.png), [detail](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_007_detail.png).
- episode_013: [overview](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_013_overview.png), [detail](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_013_detail.png).
- episode_016: [overview](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_016_overview.png), [detail](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/representative_episode_016_detail.png).

All final images were inspected. Full paths are visible in overviews; close
detail views crop some remote tails. Tiny segment structure and sub-degree
angles cannot be measured visually. The UI legend is separate from the saved
viewport, so the table above accompanies the screenshots. `visualization.json`
records exact drawn raw arrays, complete episode geometry, source hashes,
marker layers, static agent poses and rendering-code hashes.

The five required plots were also inspected:

- [straight versus OLD distance](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/straight_vs_old_cross_track.png).
- [straight versus OLD local direction](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/straight_vs_old_direction.png).
- [straight versus OLD pose yaw](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/straight_vs_old_yaw.png).
- [local versus window direction](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/local_vs_window_direction.png).
- [anchor distance distribution](../data/robotless_old_conditioned_handoff/20260915T021149Z/evidence/observation_to_old_distance.png).

Paired plots give every episode its own ID position; circles and crosses are
connected within an episode, including duplicate raw pairs. Colours retain
source categories: blue straight, green left_turn, orange right_turn, purple
doorway, red route_choice. No metric is jittered. Axes/titles identify controlled
delay, not measured LightNav latency. The anchor plot includes every raw point
and an empirical distribution for this fixed bank. `visual_review.json` binds
all **13 final images** to their hashes and review checks.

## Interpretation and remaining uncertainty

The evidence supports a mixed, conditional conclusion. Some residuals are
strongly influenced by the straight surrogate (notably 016), while substantial
local and windowed direction differences persist for 006/007. The episode
median OLD distance is 0.0248817 m versus straight 0.000139967 m; its p90 drops
from 0.249250 to 0.237101 m. Absolute local-direction median rises from
0.00991582 to 5.85538 degrees while p90 drops from 72.6667 to 58.0945 degrees.
A single “improvement” statement would obscure these differing paired changes.

Equal-pair weighting also retains differences: OLD distance median/p90 are
0.0749905/0.242466 m, absolute local direction 17.9414/54.4399 degrees, and
window direction 17.9414/80.1827 degrees. Thus the result is not just the
fivefold repetition of the dominant raw pair. This weighting is descriptive,
not a natural-frequency correction.

Windowing does not uniformly suppress direction residuals. For 006/007/016,
local and window values largely agree. In case013, the FRESH projection's
winning segment is only **0.000272335 m** long; local direction is 26.2915
degrees but the 0.10 m window yields 88.3856 degrees. Case012 similarly changes
24.6905 to 90.0485 degrees. Here the local short-segment direction understates
the broader window discrepancy. The framework's “large local, small window”
possibility must not be assumed in advance; neither tangent definition is
silently substituted for the other. No A/B/C/D threshold classifier is coded.

The bank still has the source's limited geometry coverage, related nearby
views, one doorway, one model checkpoint and one fixed observation displacement.
OLD reference gaps and interpolated pose yaw affect T_align, so alignment may
be a weak execution surrogate even when its final residual is small. Geometric
projection is not verified correspondence. Neither surrogate measures actual
OLD robot execution, latency failure, navigation success or improvement,
reconciliation necessity, graph superiority or controller performance.

The next uncertainty is how these findings change when observation poses are
consistent with OLD's intended continuation, and how reference ambiguity and
local-versus-window spatial scale affect interpretation. That needs a separate
predeclared study. This task performs no new inference, graph optimization,
rigid reconciliation of model outputs, correspondence factor, objective weight,
threshold gate, controller or robot dynamics.

## Implementation, commands and validation

New files implement a pure continuation/window/summary module, frozen-source
artifact handling, one analysis/plot pipeline, a strict validator, an Isaac
viewer/launcher, a versioned config and two test files. Existing SE(2), source
validators, projection metrics, CSV/statistic helpers and Hospital runtime are
reused without modification. README adds only two success-state sentences;
WORK_LOG is append-only. Raw/generated evidence, model/source checkouts and
virtual environments are excluded from the focused commit.

The actual analysis was created at 2026-09-15T02:11:50.650263Z and completed at
02:11:51.425757Z. Source observation/readiness events are retained per episode;
new-inference readiness and actual execution are explicitly null. UTC,
monotonic host time and stopped simulation time remain separate from controlled
tau. No sleeping for tau or intrinsic waypoint timing is introduced.

```bash
comparison_run=data/robotless_old_conditioned_handoff/20260915T021149Z
# Read-only source validation:
.venv/bin/python scripts/validate_robotless_handoff_screening.py data/robotless_handoff_screening/20260914T101519Z
# Use a NEW output directory to reproduce this immutable analysis:
.venv/bin/python scripts/characterize_robotless_old_conditioned.py "$comparison_run"
scripts/isaac/run_robotless_old_conditioned_handoff.sh "$comparison_run" --no-hold
# Inspect PNGs and record visual_review.json, then audit:
.venv/bin/python scripts/validate_robotless_old_conditioned.py "$comparison_run"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

The shown run already exists and mutation commands deliberately refuse to
overwrite it. Use `--view-only` to reopen saved geometry without rewriting
evidence, optionally without `--no-hold` for a persistent UI. Source/output
paths cannot overlap. Logs, configuration/processing hashes and the final
file inventory are retained in the new ignored run.

- Full required host suite: **1003 passed in 40.07 s**, including 45 new pure/mock
  tests. Compileall, shell syntax and whitespace checks passed.
- Synthetic tests cover projection/interpolation, exact zero-delay alignment,
  curved and arbitrary-yaw continuation, strict exhaustion including exact
  endpoint availability, source immutability, fixed FRESH, local/window and
  near-zero/degenerate handling, paired arithmetic, pair medians, deterministic
  representatives, nonfinite rejection, artifact corruption and visual guards.
  These fixtures are tests, not runtime evidence.
- Actual artifact validator passed with no missing artifacts or failures:
  all 805 source files, 30 retained episodes, 120 conditions, all anchors,
  availability, paired CSV/JSON, 64 pair rows, five plots and eight actual
  screenshots checked. Previous straight rows are loaded unchanged.
- Independent scalar audit, without importing project geometry helpers:
  **2950 comparisons**, all 120 conditions and **400 summary quantiles**,
  maximum absolute difference **1.0177969578251123e-13**. Boundaries are first
  independently checked; projections use their checked saved coordinates to
  avoid amplifying harmless roundoff at exact segment ties. Numerical
  tolerances are consistency checks, not handoff classifications.
- Rendering warnings about DLSS resolution, readback and plugin release remain
  in logs; actual viewport artifacts and final validation completed. The
  existing source validator is read-only, and no LightNav process was started.

Final status: **ROBOTLESS_OLD_CONDITIONED_HANDOFF_VALIDATED**.
