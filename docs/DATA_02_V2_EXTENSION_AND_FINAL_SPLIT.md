# DATA-02 v2 Extension and Final v1+v2 Assessment

## Scope and decision

DATA-02 v2 is a predeclared, independent extension cohort for the same online-successive
OLD/FRESH research question as DATA-02 v1. It adds data structure and coverage; it does not add
or evaluate a reconciliation formulation. No graph, objective, residual, correspondence,
selector, gate, smoothing, obstacle-avoidance method, controller tuning, physics tuning, or
LightNav change was introduced.

The immutable v1 run remains
`data/data02_online_successive_v1/data02-online-successive-primary-v1/`. The immutable v2 run is
`data/data02_online_successive_v2/data02-online-successive-extension-v2/`. The final combined
directory, `data/data02_combined_v1_v2/data02-combined-v1-v2-final/`, is a reference-only index:
it copies no raw observation, action, or telemetry artifact and records the hashes and paths of
both source runs.

The exact combined decision is:

`DATA02_COMBINED_DIVERSITY_INSUFFICIENT`

The requested component assignment is frozen and hashed, with its development/held-out access
markers recorded, but it is not a readiness-passing split and does not authorize EXP-02D. The
fixed sample-size, raw-pair-count, chunk-diversity, geometry, difficulty, and artifact gates pass,
while duplicate domination and isolated-split feasibility fail.

## Frozen system and provenance

The scientific system is unchanged from v1: Isaac Sim 6.0.1 Hospital, official Clearpath Jackal,
480 x 270 HWC uint8 RGB at 112-degree HFOV and 4 Hz, LightNav checkout
`a645828d81a8439651172197ca80a75dc1377977`, checkpoint revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, the existing `TrajectoryFollower`, frozen Stage 0-D
`pi_strong`, and Isaac `DifferentialController`. FRESH still triggers at 0.50 simulation seconds,
the RTF gate is still `[0.90, 1.10]`, and the raw observation-anchored FRESH path is activated
unchanged. LightNav waypoint rows have no intrinsic timestamp.

v2 was collected from a clean detached worktree at collector commit
`ee87d3897f03a81371af494ed258fe263d750ebe`; recorded `collector_git_status` is empty. Important
v2 SHA-256 values are:

| Artifact | SHA-256 |
|---|---|
| frozen config | `8e5ed277729adb479e15309f9843780e22138080e0560b62ea227560245dfbe4` |
| protocol | `f395409b9fb86e0671f12e6b216fe503e2829274e43c965798c1e74b40b2d064` |
| collection manifest | `96c89d0a923341b93ba9edac7594cbe03a85bf8bf5a05514bc1ab9770b789362` |
| strict validation | `e48e515073559955a5384ccdc96980d21137d6d761bcaea70d0c7f5e1784f561` |
| combined manifest | `26b391feb0053ab7c44f4882a26f6602e75b84d4807bb61e411f650a1241f331` |

The v1 strict validator was rerun before v2 work and again after collection. It confirmed all 84
episodes, all 480 attempts, all raw/world transforms, observation anchors, clocks, P/B samples,
successive chains, and hashes with zero errors. Its 248 historical eligible transitions and 219
historical `TIMING_INVALID` transitions were not relabeled, and no v1 artifact was overwritten.

## v1 timing-invalid diagnosis

The read-only diagnosis is under
`data/data02_online_successive_v2/data02-v1-timing-diagnosis/`. Of 480 v1 attempts, 219 were
timing-invalid: 158 were below RTF 0.90 and 61 were above RTF 1.10. The saved distributions do not
support model latency alone as an explanation. LightNav-reported latency overlaps between valid
and invalid groups. The strongest saved linear associations with RTF were response-detection delay
`r=-0.561`, simulation-ready latency `r=0.506`, and model-reported latency `r=0.282`; these are
descriptive correlations, not causal attribution.

v1 synchronously encoded, wrote, and hashed every lossless PNG inside the online loop. An offline,
deterministic sample of 40 existing frames measured median PNG encoding at 14.506 ms and file hash
at 0.049 ms. These are offline estimates, not missing v1 online measurements. v1 telemetry was
already buffered and was not a per-step file-I/O bottleneck.

v2 therefore makes only a runtime cleanup: it polls for a response immediately after capture, and
defers lossless PNG encoding/writing/hashing until the episode ends. RGB content, sampling,
ordering, exact inputs delivered to LightNav, hashes, controller execution, clocks, timing gate,
and all scientific variables remain unchanged. The primary v2 run persisted 14,813 frames after
their episode loops; the episode-median mean persistence cost was 22.017 ms/frame and the maximum
observed single-frame cost was 27.673 ms. These measurements show that work was moved out of the
online loop; they do not establish a causal treatment effect on RTF.

## Predeclared independent template bank

The bank contains 24 templates, exactly four for each of straight, left, right, doorway, detour,
and compound. The same seven deterministic pose variants as v1 yield 168 planned episodes.
Templates span central upper/lower corridors, room rows, north/south junction approaches, farther
compound approaches, and the north/south supply-cart regions. The exact instructions are frozen in
`configs/data02_online_successive_v2.yaml` and the run's config snapshot.

Before primary inference, all 168 template/variant initial states passed collision and settling
checks. The bank contains no v1 template ID, uses unique new IDs, and has no candidate satisfying
the predeclared practical near-duplicate conjunction: translation strictly below 0.4 m, wrapped
yaw strictly below 0.15 rad, and semantic equivalence. `template_bank/manifest.json` records every
nearest-v1 comparison, preview, and feasibility result. Template selection used no LightNav
output.

| Family | ID | Region | Instruction |
|---|---|---|---|
| straight | `V2_S00_CENTRAL_SOUTHBOUND` | central mid-upper | Continue straight south along the hospital corridor. |
| straight | `V2_S01_CENTRAL_NORTHBOUND` | central mid-upper | Continue straight north along the hospital corridor. |
| straight | `V2_S02_CENTRAL_LOWER_SOUTHBOUND` | central lower hall | Continue straight south past the patient-room doors. |
| straight | `V2_S03_JUNCTION_APPROACH_NORTHBOUND` | south junction approach | Continue straight north toward the end of this corridor. |
| left | `V2_L00_SOUTH_JUNCTION_APPROACH` | south junction approach | At the upcoming junction, turn left into the side corridor. |
| left | `V2_L01_NORTH_JUNCTION_APPROACH` | north junction approach | At the upcoming junction, turn left and continue along the cross corridor. |
| left | `V2_L02_ROOM_WEST_SOUTH` | central room row south | Turn left into the open patient room on the east side. |
| left | `V2_L03_SERVICE_ROOM_SOUTH` | central room row north | Turn left into the visible service-room opening. |
| right | `V2_R00_SOUTH_JUNCTION_APPROACH` | south junction approach | At the upcoming junction, turn right into the side corridor. |
| right | `V2_R01_NORTH_JUNCTION_APPROACH` | north junction approach | At the upcoming junction, turn right and continue along the cross corridor. |
| right | `V2_R02_ROOM_WEST_NORTH` | central room row south | Turn right into the open patient room on the east side. |
| right | `V2_R03_ROOM_EAST_NORTH` | central room row north | Turn right into the open patient room on the east side. |
| doorway | `V2_D00_MID_ROOM_LEFT_SOUTHBOUND` | central room row south | Enter the open patient-room doorway on the left. |
| doorway | `V2_D01_NORTH_ROOM_RIGHT_NORTHBOUND` | central room row north | Enter the open patient-room doorway on the right. |
| doorway | `V2_D02_MID_ROOM_LEFT_NORTHBOUND` | central room row south | Go through the open doorway on the left into the patient room. |
| doorway | `V2_D03_NORTH_SERVICE_ROOM_LEFT_SOUTHBOUND` | central room row north | Go through the visible service-room doorway on the left. |
| detour | `V2_T00_CART_NORTHBOUND_LEFT` | supply cart south | Pass the supply cart on the left and continue north. |
| detour | `V2_T01_CART_NORTHBOUND_RIGHT` | supply cart south | Pass the supply cart on the right and continue north. |
| detour | `V2_T02_CART_SOUTHBOUND_LEFT` | supply cart north | Pass the blue supply cart on the left and continue south. |
| detour | `V2_T03_CART_SOUTHBOUND_RIGHT` | supply cart north | Pass the blue supply cart on the right and continue south. |
| compound | `V2_C00_SOUTH_APPROACH_LEFT` | south junction far approach | Continue north to the junction, turn left, then follow the side corridor. |
| compound | `V2_C01_SOUTH_APPROACH_RIGHT` | south junction far approach | Continue north to the junction, turn right, then follow the side corridor. |
| compound | `V2_C02_NORTH_APPROACH_LEFT` | north junction far approach | Continue south to the junction, turn left, then follow the cross corridor. |
| compound | `V2_C03_NORTH_APPROACH_RIGHT` | north junction far approach | Continue south to the junction, turn right, then follow the cross corridor. |

## v2 collection result

All 168 planned episodes were attempted and completed in one invocation. The persistent server
built the model once, reset episode history 168 times, served 1,167 predictions including
bootstrap requests, and exited cleanly. There were zero server restarts and no OOM.

| Status | Count |
|---|---:|
| `ELIGIBLE_MOVING` | 711 |
| `MODEL_STOP` | 1 |
| `OLD_EXHAUSTED` | 4 |
| `CHUNK_EXHAUSTED_BEFORE_TRIGGER` | 0 |
| `TIMING_INVALID` | 282 |
| `NONFINITE_MODEL_OUTPUT` | 0 |
| `EXECUTION_COLLISION` | 1 |
| `EXECUTION_OUT_OF_ENVELOPE` | 0 |
| `TECHNICAL_INVALID` | 0 |

The 711 eligible transitions contain 52 unique OLD raw chunks, 53 unique FRESH raw chunks, and
113 unique ordered raw pairs. Eighteen ordered raw-pair identities overlap with v1. The largest v2
pair occurs 500 times, or 70.323% of eligible v2. Geometry is 617 `STRAIGHT_LIKE`, 55
`POSITIVE_TURNING`, 34 `NEGATIVE_TURNING`, and 5 `OTHER`; difficulty is 605 `BENIGN`, 55
`INTERMEDIATE`, and 51 `CHALLENGING`. These are descriptive labels, not instruction-success
scores.

The very large repeated pair is an observed LightNav output property under the frozen bank, not a
collector error to be repaired by deleting or reweighting samples. v2 strict validation
reconstructed all 168 frame streams and all 999 actual transition attempts with zero errors.

## Combined reference corpus and split

The reference-only combined corpus contains 1,479 attempts and 959 eligible transitions. It has 82
unique OLD raw chunks, 94 unique FRESH raw chunks, and 191 unique ordered raw pairs. The largest
pair appears 593 times, or 61.835% of eligible combined data.

Combined geometry counts are 786 `STRAIGHT_LIKE`, 94 `POSITIVE_TURNING`, 68
`NEGATIVE_TURNING`, and 11 `OTHER`. Combined difficulty counts are 685 `BENIGN`, 178
`INTERMEDIATE`, and 96 `CHALLENGING`. The complete status union is 959 eligible, 501
timing-invalid, 10 OLD-exhausted, 5 collision, and 4 model-stop attempts.

The deterministic splitter connects transitions sharing either an episode or an exact ordered raw
pair, then assigns whole connected components only. It finds 12 components; the largest contains
920 eligible transitions because the dominant pair links many episodes across both cohorts. The
best deterministic whole-component assignment has 39 development and 920 held-out transitions
(`95.933%` held-out versus the `25%` target), with 14 and 236 episodes respectively. Episode
leakage and raw-pair leakage are both zero. Geometry and difficulty coverage occur in both sides,
but the development side lacks the straight semantic family. This is recorded as an infeasible
isolated split, not fixed by splitting the dominant component.

## Original readiness gates A--H

| Gate | Result | Evidence |
|---|---|---|
| A minimum eligible | PASS | 959 |
| B minimum unique ordered raw pairs | PASS | 191 |
| C largest pair fraction | **FAIL** | 593/959 = 61.835%, above 20% |
| D unique OLD and FRESH | PASS | 82 OLD, 94 FRESH |
| E geometry coverage | PASS | all four bins meet frozen requirements |
| F difficulty coverage | PASS | all three bins meet frozen requirements |
| G isolated split | **FAIL** | 12 components; largest 920; 39/920 split; incomplete semantic coverage |
| H strict artifact validity | PASS | both sources and combined references/hash chain valid |

The gates and thresholds were not changed after observing v1 or v2.

## Generated analysis and visual review

The combined `plots/` directory contains exactly the ten requested plots:

- `v1_vs_v2_vs_combined_counts.png`
- `v1_vs_v2_pair_frequency.png`
- `combined_pair_frequency.png`
- `combined_geometry_distribution.png`
- `combined_command_jump_distribution.png`
- `combined_latency_distribution.png`
- `timing_invalid_by_transition_index.png`
- `component_size_distribution.png`
- `final_split_summary.png`
- `representative_combined_transitions.png`

All ten were visually inspected. The frequency and component/split plots expose rather than hide
the dominant-pair problem. `representatives.json` contains 36 deterministically selected records.

A real Isaac GUI saved-only replay used the challenging v2 representative
`episode_000061_transition_04` (`POSITIVE_TURNING`, `|delta omega|=1.738 rad/s`). Hospital and the
official Jackal mesh were visible; the Jackal traversed its recorded wheel-driven pose history;
OLD, raw FRESH, actual history, observation/P/B markers, and live phases were visible; and the
non-black capture was saved as `gui_replays/episode_000061_transition_04.png`. Replay performs no
LightNav inference and no physics re-execution.

## Commands

Primary v2 collection from the clean frozen collector commit:

```bash
./scripts/isaac/run_data02_v2_online_successive.sh \
  --phase primary --run-id data02-online-successive-extension-v2
```

Strict v2 validation and summary:

```bash
.venv/bin/python scripts/validate_data02_online_successive.py \
  data/data02_online_successive_v2/data02-online-successive-extension-v2
.venv/bin/python scripts/summarize_data02_online_successive.py \
  data/data02_online_successive_v2/data02-online-successive-extension-v2
```

Build, plot, and validate the reference-only union:

```bash
.venv/bin/python scripts/build_data02_combined.py \
  --v1-run data/data02_online_successive_v1/data02-online-successive-primary-v1 \
  --v2-run data/data02_online_successive_v2/data02-online-successive-extension-v2 \
  --output data/data02_combined_v1_v2/data02-combined-v1-v2-final
.venv/bin/python scripts/plot_data02_combined.py \
  data/data02_combined_v1_v2/data02-combined-v1-v2-final
.venv/bin/python scripts/validate_data02_combined.py \
  data/data02_combined_v1_v2/data02-combined-v1-v2-final
```

Saved-only GUI replay with automatic exit:

```bash
./scripts/isaac/run_data02_v2_online_successive.sh \
  --replay-run data/data02_online_successive_v2/data02-online-successive-extension-v2 \
  --episode episode_000061 --transition 4 --gui --no-hold
```

GUI legend: blue is OLD, magenta is raw observation-anchored FRESH, green is recorded actual
motion, yellow is the FRESH observation, orange is P, and red is B/switch. The moving official
Jackal is a replay of saved physical execution, not a new command or trajectory simulation.

## Claim limitation

This evidence establishes corpus integrity, actual wheel-driven collection, observed timing,
geometry/difficulty coverage, raw-output identity, duplicate structure, and split infeasibility.
It does not validate instruction satisfaction, estimate natural deployment frequencies, show that
reconciliation works, compare reconciliation methods, improve transition discontinuity, validate
obstacle safety, or identify a causal simulator/controller/LightNav failure. In particular, the
technical timing cleanup and lower timing-invalid fraction are associated observations across two
different cohorts, not a controlled causal estimate.
