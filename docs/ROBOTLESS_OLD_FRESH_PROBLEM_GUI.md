# Saved OLD/FRESH handoff problem GUI

Decision: **ROBOTLESS_OLD_FRESH_PROBLEM_GUI_VALIDATED**.

This persistent Isaac demonstration makes the saved OLD-to-FRESH transition
visible in three stages: OLD spatial replay, a pause at the handoff, and a
reveal of the unchanged FRESH trajectory. It reuses the validated
[OLD-consistent observation pilot](ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT.md).
It performs no new LightNav inference or research experiment.

This GUI demonstrates saved trajectory-level handoff geometry.
It does not demonstrate robot failure or graph improvement.

## Frozen evidence and case selection

The only primary input is
`data/robotless_old_consistent_observation/20260915T043415Z/`.
The existing pilot validator must return
`ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_VALIDATED` before an Isaac stage is
created. Missing or invalid source data stops the launcher; there is no
synthetic fallback. All 208 source files, including raw predictions, derived
arrays, metrics, RGB and metadata, are inventoried and left unchanged.

The source manifest hash is
`68befb7f528506bf943302af5141ef20156fdcbf6dcb059ff2daa0da276bba69`.
The starting research HEAD and fetched origin/main were both
`e7f2f6a295f8937b0d5fcd3f823a08cfb283d3a3`.
The final commit is the introducing commit of this document; the ignored run's
`git_completion.json` records its full SHA after the single commit and push.

Only `episode_006`, `episode_016` and `episode_027` are offered, with all three
case buttons always present. The first two were selected to show that large
local/window direction disagreement remains in two genuine successive pairs.
027 provides an orientation-benign reference, labeled
`BENIGN DIRECTION / YAW REFERENCE`.

The panel reads these values from each source `derived/metrics.json`:

| Case | Cross-track m | Absolute local direction deg | Absolute 0.10 m window direction deg | Absolute pose-yaw deg | FRESH segment | Alpha |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| episode_006 | 0.0000352404667282 | 90.1106542623 | 89.9246318737 | 72.0005279288 | 3 | 0.0000925644924269 |
| episode_016 | 0.000462856646645 | 90.5206495380 | 90.1390083079 | 72.0666613149 | 3 | 0.00314371805677 |
| episode_027 | 0.151385084978 | 0.00682017276877 | 0.00577342530685 | 0.00361765863580 | 0 | 0 |

These numbers are evidence summaries, not constants in the GUI implementation.
The GUI also reconstructs the measurements independently from the source
arrays and fails closed if they disagree with the saved metrics.

027's distance is an endpoint projection distance: the first FRESH spatial
reference begins ahead of its observation anchor. Its direction and pose yaw
nearly agree with OLD. The GUI does not classify it as a failure, navigation
success, or perfect handoff. There are no threshold-based GOOD/BAD labels.

## Replay, frames and measurements

The source raw arrays are cumulative local `[x, y, yaw]` references: +X forward,
+Y left, yaw CCW radians. World arrays use the fixed Isaac XY frame, metres,
+Z up. `T_world_observation` uses column vectors and each chunk's own actual
saved observation pose: R0 for OLD and R1 for FRESH. Both world transforms are
checked against raw arrays; the loaded world arrays are then drawn unchanged.
FRESH is never translated, rotated, scaled or re-anchored at B for display.

The replay path is reconstructed as `[R0, O0, ..., O(N-1)]`. R0-to-O0 is included
once as the pilot's simulation-side connector, not a model waypoint. The marker
advances through its first 0.30 m of XY arc, with linear segment position and
shortest wrapped endpoint-yaw interpolation. Positive-length segments use the
pilot's lowest-index exact-vertex convention. Zero-length XY segments receive
no invented rotation duration. Insufficient arc is rejected, never clamped.
The initial marker is exactly R0 and the final marker is exactly the
independently reconstructed `B = R1_old`, which must equal the saved boundary.

The simple yellow marker is labeled visibly
`OLD-CONSISTENT SPATIAL SURROGATE REPLAY`. It is a DebugDraw point, with no robot
mesh, articulation, controller or dynamics. The Hospital is loaded once with
its authored geometry and lighting. The logical observation Xform is placed
using the pilot's double-precision USD transform; USD's row-vector matrix is
the transpose of the documented column-vector transform. The simulation
timeline stays stopped at zero. No historical execution is being replayed.

| Display time | State |
| --- | --- |
| 0–3 s | OLD-only marker motion through 0.30 m |
| 3–4 s | Exact B pause, OLD incoming tangent visible, FRESH hidden |
| 4–8 s | Unchanged FRESH revealed, static comparison |
| After 8 s | Comparison stays visible until another control or app close |

This uses a monotonic host display clock and `display_playback_only=true`.
It is not waypoint timing, model latency, physical speed or execution time.
Original observation, request and response/readiness timestamps are retained
as source provenance, separately from GUI action/capture timestamps.
Execution time remains null. `Replay OLD` stops at B without revealing FRESH.

Q is the closest clamped projection onto the original FRESH polyline, with
exact ties choosing the lowest segment index. Cross-track is `||B.xy-Q.xy||`,
including endpoint cases. Local direction compares the incoming augmented OLD
segment at 0.30 m with the winning FRESH segment. The separate window diagnostic
compares XY chords over a centered 0.10 m arc window, truncated at path ends.
Pose-yaw compares yaw(B) with shortest-angle interpolated FRESH yaw at Q;
it is not a tangent-direction measurement.

## Controls and views

Run from the repository root:

```bash
./scripts/isaac/run_old_consistent_problem_gui.sh
```

The default case is 006; the GUI stays open until the user closes Isaac.
The repository `.venv` supplies the source validator and the local Isaac
installation supplies the GUI runtime. `ISAACSIM_ROOT` can select the Isaac
installation; its default is `~/isaacsim`.

| Control | Effect |
| --- | --- |
| 006 Challenging / 016 Challenging / 027 Benign Reference | Switch source case and reset to R0, FRESH hidden |
| Replay OLD | Restart the three-second spatial marker replay; stop at B |
| Pause at Handoff | Place marker exactly at B and hide FRESH |
| Reveal FRESH | Place marker at B and reveal the original FRESH comparison |
| Replay Full | Automatically run motion, pause, reveal and static comparison |
| Reset | Restore R0, hide FRESH and window overlays, stop playback |
| Show RGB1 | Toggle actual saved `raw/observation_001.jpg`, labeled FRESH OBSERVATION |
| Toggle Local / Window Tangent | Add/remove separately labeled longer pale window diagnostic arrows |
| Overview Camera | Full OLD/FRESH paths and Hospital context |
| Handoff Camera | Nearly top-down close-up around B and the arrows |

RGB visibility and camera choice are presentation preferences retained across
case changes. Window overlays reset on case change/replay/reset. RGB is shown
without trajectory overlays or image-coordinate projection. The window overlay
retains the local arrows; similar local/window directions can overlap.

Supported CLI options include `--case episode_006|episode_016|episode_027`,
`--auto-replay`, `--show-rgb`, `--no-hold`, `--capture-evidence`, and `--output`.
`--no-hold` exits after playback and a twelve-second runtime observation.
`--capture-evidence` exercises the same callbacks as all controls for all three
cases and writes screenshots before returning to the requested initial case.
An existing output directory is rejected to prevent overwriting evidence.

The final evidence command was:

```bash
./scripts/isaac/run_old_consistent_problem_gui.sh \
  --capture-evidence --show-rgb --auto-replay \
  --output data/robotless_old_consistent_problem_gui/20260915T064814Z
```

## Color legend and camera conventions

| Color | Geometry |
| --- | --- |
| Blue | Original OLD path, including the explicit initial connector |
| Magenta | Original FRESH world path |
| Yellow point | Moving marker; at handoff, B |
| White point | Closest FRESH projection Q |
| Green line | B-to-Q connector |
| Lime arrow | OLD incoming local tangent, based at B |
| Cyan arrow | FRESH local tangent, based at Q |
| Orange arrow | FRESH pose yaw, based at Q |
| Short gold arrow | B pose yaw, distinct from OLD geometric direction |
| Pale green / pale cyan longer arrows | OLD / FRESH 0.10 m window diagnostics |

All path XY values and arrow headings are unchanged. Paths are drawn at
display Z=0.12 m and annotations at Z=0.15 m, solely to lift them above the
floor. Local tangent and FRESH yaw annotation arrows are 0.24 m long, window
arrows 0.34 m, and the B yaw arrow 0.18 m; these are annotation lengths, not
trajectory scaling or physical motion. Point sizes and line widths are screen
pixels. Very small B/Q separations are allowed to overlap visibly rather than
being enlarged. Collinear arrows in 027 naturally overlap; line widths retain
the cyan tangent around the orange yaw arrow.

Overview eye/target are `[-2.3,-1.6,2.7]` / `[0.3,0,0.12]`, with XY offsets in
R0's local axes and Z in world metres. Handoff eye is
`[B.x,B.y-0.001,1.6]`, targeting `[B.x,B.y,0.15]`. The near-normal view makes the
true planar direction comparison clear. Only camera transforms change between
views. XY scale and angular magnification are both 1. No scene geometry is
hidden. Editor panels are closed to make space for the demonstration panel;
UI DPI is explicitly 1.0 for the recorded application window.

## Evidence and validation

Generated evidence remains outside Git at
`data/robotless_old_consistent_problem_gui/20260915T064814Z/`.
Each case has `before_fresh.png`, `after_fresh.png`, `window_tangents.png` and
`overview.png`, prefixed by its full episode ID. These are actual Isaac
application swapchain captures, including the interactive panel and saved RGB
inset. They are not reconstructed diagrams or composites.

| Case | Before FRESH | After FRESH | Overview |
| --- | --- | --- | --- |
| 006 | [OLD pause](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_006_before_fresh.png) | [Direction disagreement](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_006_after_fresh.png) | [Context](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_006_overview.png) |
| 016 | [OLD pause](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_016_before_fresh.png) | [Direction disagreement](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_016_after_fresh.png) | [Context](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_016_overview.png) |
| 027 | [OLD pause](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_027_before_fresh.png) | [Direction/yaw reference](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_027_after_fresh.png) | [Context](../data/robotless_old_consistent_problem_gui/20260915T064814Z/evidence/episode_027_overview.png) |

Each screenshot JSON records the pilot run, episode ID, raw OLD and FRESH
hashes, metrics JSON hash, screenshot hash, camera transform/mode, reveal/window
state, capture timestamp, research Git SHA and independently reconstructed
geometry. `processing_sources.json` pins the executed code/configuration;
`geometry_audit.json` preserves reconstruction results. `runtime_controls.json`
contains actual-renderer callback checks and marker samples; `events.jsonl`
appends display actions. `persistence.json` records the still-running app after
the full eight-second presentation. `visual_review.json` identifies inspected
image hashes and the visual observations. Layout trials are separately retained
as ignored `gui_trial_01`, `gui_trial_02`, and `gui_trial_03` runs.

Revalidate without writing to the source or replacing any output:

```bash
.venv/bin/python scripts/validate_old_fresh_problem_gui.py \
  data/robotless_old_consistent_problem_gui/20260915T064814Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Pure/mock tests cover the fourteen requested contract categories: allowed
cases, source hashes, augmented OLD reconstruction, exact arc endpoint, local
and window tangents, FRESH projection, metric agreement, case reset, reveal,
replay timing, immutable display geometry, invalid source and mismatched
metrics/source rejection. Those synthetic tests are software checks, not
experimental evidence.

The final validator passed with 208 unchanged source files, 1,974 checked
runtime marker samples and all 12 manually inspected application screenshots.
Persistent hold mode rendered beyond the full eight-second presentation;
the separate `--no-hold` trial exited after its documented observation period.
The primary process later ended with exit143 after about 340 seconds; the
cause of termination was not established. A separate terminal launch also
passed its persistence observation and later closed. Optional OS mouse
injection did not establish a click while other desktop windows were
foreground. Control validation therefore refers to the actual Isaac button
callbacks exercised in the renderer, not a claimed successful native mouse test.
The full repository suite passed **1,087 tests** (including 43 new GUI contract
tests), and compileall and diff whitespace checks passed.

The evidence supports the descriptive claim that under OLD-consistent
observation construction some genuine successive LightNav transitions show
large direction/orientation disagreement while another selected transition
remains aligned. It does not establish navigation failure, collision,
executability, a need for graph optimization, graph improvement, a general
LightNav defect, or natural deployment frequencies. No optimizer, rigid
correction, correspondence, factor weighting, controller or dynamics was added.
