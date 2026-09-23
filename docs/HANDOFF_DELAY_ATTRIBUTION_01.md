# HANDOFF-DELAY-ATTRIBUTION-01

Does an observation-to-application delay-sensitive execution gap remain after
using the frozen REF-02 reference interface? This is an offline single-handoff
counterfactual diagnostic, not reconciliation, a new selector, or online navigation.

Starting and fetched main: `718a72641ec12df1b8ec92af0d6d34846e291f7f`.
The complete 13-event source cohort from
`data/genuine_source_moving_mismatch/scan_20260922T073107Z/` is retained in its
original order, including two endpoint cases. The additional expected eight
sources are C1–C4 applications in ON_REPEAT_00 and ON_REPEAT_01 from
`data/robotless_join_online_03/primary_20260923T031300Z/`. No reselection occurs.
The final saved-loss comparator is `audit_20260922T091708Z`, not the earlier
retained `audit_20260922T091255Z` development pass.

## Frozen protocol before scientific execution

Four conditions, in the following order for each frozen event:

| Condition | Initial state | Reference interface |
|---|---|---|
| DELAYED_NATIVE | actual B-time state | unchanged raw FRESH; official nearest/+1 |
| DELAYED_LOOKAHEAD | actual B-time state | prepared FRESH + frozen REF-02 source progress |
| LATENCY_FREE_NATIVE | actual observation-time state | same raw FRESH; official nearest/+1 |
| LATENCY_FREE_LOOKAHEAD | actual observation-time state | same prepared FRESH + source progress |

The lookahead condition is explicitly
**PREPARED_REFERENCE_PLUS_SOURCE_PROGRESS_SELECTOR**. On raw rows with `s_i=i`,
REF-02 equals the official next-row rule. Therefore native-to-lookahead contrasts
include preparation/resampling and selector, and cannot isolate selector alone.
The existing preparation runs once at B. Its 30 float64 future rows and REF-01
lineage are held identical across the two initial states. No Oracle re-cutting,
reanchoring, rounding, stride change, or accumulated progress is allowed.
The .1–3s preparation convention is not intrinsic LightNav waypoint timing; the
actual evaluated exposure remains .9s.

Every source uses its original observation transform:
`T_world_F = T_world_R(obs) * T_obs_F`. World: Z-up, metres, yaw CCW radians;
local: forward/left/yaw CCW. Native installation uses unchanged original local
rows. Dense installation uses the existing inverse observation-frame transform,
with roundtrip audit. Raw arrays and historical runs remain immutable.

### Controller state semantics and an important limitation

Physical u_minus comes from the incoming command that reconstructs each cut's
saved state. Controller memory comes from the actual worker state at that host
monotonic cut, independently of physical velocity. A successful worker result
updates official `previous_command` during poll; the update is bounded by the
worker result timestamp (before poll) and receipt in Isaac (after poll). A later
submit snapshot can tighten that bound. A cut straddling a value-changing update
is unavailable. Stale results do not change memory. Ambiguous controller errors
are unavailable. No zero reset or B-memory substitution for observation is used.

In the 21 reconstructed sources, observation memory is available and equals
the physical incoming command. At B, the first FRESH solve has already completed
and updated worker memory before its command is physically applied. **All 21
B-time memories differ from physical u_minus.** This differs from the legacy
REF-02 rollout policy, which intentionally restores the *pre-FRESH solve input*
memory. Both values and the exact source rows/timestamps are retained here.
The present experiment follows the requested actual B-time memory, then performs
a new synchronous solve at B. It does not reproduce the historical asynchronous
first command or pending future. Initial motion violations remain in evaluation
and must not be attributed uniquely to the selector or to delay geometry.
Delay contrasts compare the complete recorded state packages, not position alone.

Each condition constructs an independent official tracker. The pinned MPC file
SHA256 is `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
REF-02 per-instance selection/input recorder, official objective, Q/R, horizon5,
MPC dt=.1, limits and failure handling remain unchanged. No source defaults are
patched. The existing official-demo Python environment hosts MPC; research Python
hosts geometry/evaluation, communicating via a small JSON-line bridge.

### Exposure, safety and metrics

Exactly 54 intervals of the saved float32 1/60 timestep, total
`.9000000469386578 s`; nine solves at ticks0,6,…,48. The solve clock is nominal
10Hz; the float32 integration convention makes each six-step hold
`.10000000521540642 s`. All sources have at least this saved reference lifetime.
No secondary extension, future VLA update, or retry is planned. Initial maximum:
21×4=84 rollouts, 84×9=756 solves. These are planned counts, not completed work.

The existing JOIN-ONLINE02 abort-only guard checks each new command's next nominal
.1s hold and each subsequent integration step against direct Hospital geometry
(and the original runtime cart for ONLINE03), radius.20m and required edge
clearance.05m with unchanged geometry/curve allowances. It never changes commands.
An unsafe proposed command is saved as unapplied; no continuation is fabricated.
Aborted prefixes retain diagnostics but equal-exposure primary metrics are N/A.

Original forward-only geometric projection and .10m/15deg/.30s sampled dwell are
reused. Position, excess-position, wrapped-yaw and excess-yaw AUCs retain separate
units. No weighted score. First tube entry, sustained join, right censoring,
command TV, nominal-grid acceleration, clearance, workspace and failures remain
separate. N/A is never replaced with .9 or3s. No terminal-goal success is inferred
from this short observation window.

For each AUC, report delayed-minus-latency-free within each reference interface;
native-minus-prepared+SP within each initial state; and their difference-in-
differences. These are descriptive paired diagnostics, not population causal
estimates. Categories are shown explicitly; transient versus censored dwell is
not ordered by a fabricated rank. Event and episode-equal summaries are separate
for each cohort. All-event geometry figures are saved. Predeclared representatives
are013/01/024,008/01/023,001/01/013 and ON_REPEAT_00/C1.

Interpretation is based on all tables and safety/coverage, without an automatic
scalar cutoff: delay sensitivity remains, reference interface explains most
cost, mixed attribution, or insufficient Oracle coverage. The Oracle gap is
**not** a recoverable maximum: an optimizer would start at B, not observation.

## Source coverage and preflight

All13+8 expected sources have usable application B, observation state and exact
physical/controller memory provenance; all84 conditions are available. Source
lifetimes range58–93 intervals in the13 and59–61 in ONLINE03. Original ONLINE03
pacing limitations are preserved. Neither cohort establishes obstacle-constrained
reconciliation: the13 have zero obstacle-near retained suffixes at <=.20m;
ONLINE03 consists of safe partial onset, with C4 still about.42–.46m clearance,
not complete bypass. No new obstacle, observation, instruction or source is made.

The initial source-only preparation `primary_20260923T130000Z` is retained with
zero scientific execution. It referenced the earlier saved-loss development
validation and was superseded before freeze by
`data/handoff_delay_attribution_01/primary_20260923T054600Z/`, using the documented
final authoritative audit. No source/threshold/cohort/rollout outcome was changed.

## Commands

```bash
.venv/bin/python scripts/run_handoff_delay_attribution.py prepare --run data/handoff_delay_attribution_01/primary_20260923T054600Z
# Review, commit and normal push the protocol/code/tests before the following:
.venv/bin/python scripts/run_handoff_delay_attribution.py freeze --run data/handoff_delay_attribution_01/primary_20260923T054600Z
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MPLCONFIGDIR=/tmp/handoff_delay_mpl .venv/bin/python scripts/run_handoff_delay_attribution.py execute --run data/handoff_delay_attribution_01/primary_20260923T054600Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/handoff_delay_mpl .venv/bin/python scripts/run_handoff_delay_attribution.py analyze --run data/handoff_delay_attribution_01/primary_20260923T054600Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_handoff_delay_attribution.py validate --run data/handoff_delay_attribution_01/primary_20260923T054600Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Scientific VLA, GP, rigid, graph, Isaac and RGB calls are zero. Synthetic tests are
implementation checks only. Results and research interpretation will be appended
after the pushed freeze and one bounded execution.

## Preserved technical failure and correction

Pushed freeze `c823be8b0184a02ba2c61abcabf37ae1f39bc4d2` reached the first MPC
request in run054600. Native solver stdout contaminated the JSON bridge; close
parsing then failed before a result file was saved. One solve request was issued,
zero numerical responses were preserved, and zero state integration steps ran.
All later conditions were unattempted. `execution_failure.json` preserves this
limitation. The numerical outcome of that first solve is unavailable.

The correction adopts the exact file-descriptor separation already present in
`scripts/online_mpc_worker.py`, adds a durable IPC journal, and prevents a close
error from discarding a collected result. A native-print subprocess regression
uses no CasADi/model call. It changes no scientific semantics. After a separate
commit/push, the same84-condition schedule uses a new run
`data/handoff_delay_attribution_01/primary_20260923T055300Z/`.
The additional failed request is counted separately from that run's solve count.

## Completed measurements

Corrected execution freeze: `b9d44afd65676d97c890eea99e224af4db60c322`. Main run: `data/handoff_delay_attribution_01/primary_20260923T055300Z`. All84 independent rollouts completed,54 intervals and9 solves each: **756 saved MPC solves,4536 integrated intervals**. VLA/GP/rigid/graph/RGB/Isaac calls: **0**. The preserved failed IPC run adds **one issued MPC request with no saved numerical response**; primary plus failed-attempt requests757. The two full test suites add two historical reproduction solves, giving759 total MPC requests. No scientific output was used to tune or replace a condition.

DN/DL = delayed native/prepared+source-progress; FN/FL = corresponding latency-free conditions. AUC units are m·s. Gap columns are DN−FN and DL−FL. N/A attachment remains N/A.

### GENUINE_13 — every frozen event

| Event | DN AUC | DL AUC | FN AUC | FL AUC | Native delay gap | Prepared+SP delay gap |
|---|---:|---:|---:|---:|---:|---:|
| episode_014_repeat_01/handoff_025 | 0.233324 | 0.233324 | 0.047517 | 0.104873 | 0.185807 | 0.128451 |
| episode_001_repeat_01/handoff_013 | 0.123889 | 0.124081 | 0.044184 | 0.043171 | 0.079706 | 0.080910 |
| episode_013_repeat_00/handoff_033 | 0.093072 | 0.093072 | 0.039420 | 0.095794 | 0.053653 | -0.002722 |
| episode_016_repeat_01/handoff_007 | 0.082413 | 0.080551 | 0.021677 | 0.028139 | 0.060735 | 0.052412 |
| episode_016_repeat_01/handoff_001 | 0.136389 | 0.136333 | 0.046591 | 0.043652 | 0.089799 | 0.092681 |
| episode_008_repeat_01/handoff_023 | 0.186935 | 0.186850 | 0.025917 | 0.023232 | 0.161019 | 0.163618 |
| episode_001_repeat_01/handoff_011 | 0.126534 | 0.126716 | 0.044054 | 0.042888 | 0.082480 | 0.083828 |
| episode_013_repeat_00/handoff_020 | 0.196260 | 0.205967 | 0.035640 | 0.028783 | 0.160620 | 0.177184 |
| episode_013_repeat_01/handoff_024 | 0.297773 | 0.321338 | 0.035443 | 0.027144 | 0.262330 | 0.294194 |
| episode_013_repeat_01/handoff_021 | 0.159467 | 0.159740 | 0.021878 | 0.031719 | 0.137589 | 0.128022 |
| episode_020_repeat_00/handoff_017 | 0.181051 | 0.181031 | 0.020003 | 0.020453 | 0.161047 | 0.160578 |
| episode_013_repeat_00/handoff_026 | 0.202362 | 0.210391 | 0.021702 | 0.029204 | 0.180660 | 0.181187 |
| episode_013_repeat_01/handoff_029 | 0.127178 | 0.127441 | 0.044097 | 0.042984 | 0.083081 | 0.084458 |

| Event | DN attachment | DL attachment | FN attachment | FL attachment |
|---|---|---|---|---|
| episode_014_repeat_01/handoff_025 | no entry | no entry | join 0.066667s | transient |
| episode_001_repeat_01/handoff_013 | no entry | no entry | join 0.066667s | join 0.066667s |
| episode_013_repeat_00/handoff_033 | no entry | no entry | join 0.050000s | transient |
| episode_016_repeat_01/handoff_007 | join 0.516667s | join 0.500000s | join 0.066667s | join 0.066667s |
| episode_016_repeat_01/handoff_001 | no entry | no entry | join 0.133333s | join 0.133333s |
| episode_008_repeat_01/handoff_023 | no entry | no entry | join 0.066667s | join 0.066667s |
| episode_001_repeat_01/handoff_011 | no entry | no entry | join 0.066667s | join 0.066667s |
| episode_013_repeat_00/handoff_020 | no entry | no entry | join 0.133333s | join 0.133333s |
| episode_013_repeat_01/handoff_024 | no entry | no entry | join 0.116667s | join 0.116667s |
| episode_013_repeat_01/handoff_021 | no entry | no entry | join 0.066667s | join 0.066667s |
| episode_020_repeat_00/handoff_017 | no entry | no entry | join 0.066667s | join 0.066667s |
| episode_013_repeat_00/handoff_026 | no entry | no entry | join 0.066667s | join 0.066667s |
| episode_013_repeat_01/handoff_029 | no entry | no entry | join 0.066667s | join 0.066667s |

| Condition | Mean position AUC | Mean yaw AUC [rad·s] | Sustained attach | Motion invalid | Mean linear TV [m/s] | Mean angular TV [rad/s] |
|---|---:|---:|---:|---:|---:|---:|
| DELAYED_NATIVE | 0.165127 | 0.225799 | 1/13 | 13/13 | 0.364975 | 3.206753 |
| DELAYED_LOOKAHEAD | 0.168218 | 0.229754 | 1/13 | 13/13 | 0.316384 | 3.272199 |
| LATENCY_FREE_NATIVE | 0.034471 | 0.106688 | 13/13 | 0/13 | 0.123077 | 2.421072 |
| LATENCY_FREE_LOOKAHEAD | 0.043234 | 0.158695 | 11/13 | 0/13 | 0.123077 | 2.856510 |

| Contrast | Event mean position | Episode-equal position | Event mean excess position | Event mean yaw | Event mean excess yaw |
|---|---:|---:|---:|---:|---:|
| delay_native | 0.130656 | 0.136697 | 0.074888 | 0.119111 | 0.056370 |
| delay_lookahead | 0.124985 | 0.127858 | 0.074352 | 0.071059 | 0.035461 |
| interface_delayed | -0.003091 | -0.001867 | -0.003218 | -0.003955 | -0.002768 |
| interface_latency_free | -0.008763 | -0.010706 | -0.003754 | -0.052007 | -0.023677 |
| interaction | 0.005671 | 0.008839 | 0.000536 | 0.048052 | 0.020909 |

All four metrics also have episode-equal values and event-level signed contrasts in `summary.json` and `paired_delay_gaps.csv`. Interface contrasts are native minus prepared+SP, not selector-only effects.

### ONLINE03_ONSET — every frozen event

| Event | DN AUC | DL AUC | FN AUC | FL AUC | Native delay gap | Prepared+SP delay gap |
|---|---:|---:|---:|---:|---:|---:|
| ON_REPEAT_00/C1 | 0.001668 | 0.003369 | 0.026674 | 0.034610 | -0.025006 | -0.031241 |
| ON_REPEAT_00/C2 | 0.006318 | 0.007992 | 0.023991 | 0.033231 | -0.017673 | -0.025239 |
| ON_REPEAT_00/C3 | 0.008593 | 0.008063 | 0.015377 | 0.017693 | -0.006784 | -0.009630 |
| ON_REPEAT_00/C4 | 0.021410 | 0.023318 | 0.018603 | 0.024608 | 0.002807 | -0.001290 |
| ON_REPEAT_01/C1 | 0.000916 | 0.002510 | 0.026674 | 0.034610 | -0.025758 | -0.032100 |
| ON_REPEAT_01/C2 | 0.005609 | 0.005377 | 0.014318 | 0.014491 | -0.008709 | -0.009114 |
| ON_REPEAT_01/C3 | 0.010269 | 0.011251 | 0.015239 | 0.017486 | -0.004970 | -0.006234 |
| ON_REPEAT_01/C4 | 0.017081 | 0.017718 | 0.015239 | 0.017486 | 0.001842 | 0.000232 |

| Event | DN attachment | DL attachment | FN attachment | FL attachment |
|---|---|---|---|---|
| ON_REPEAT_00/C1 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |
| ON_REPEAT_00/C2 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |
| ON_REPEAT_00/C3 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |
| ON_REPEAT_00/C4 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |
| ON_REPEAT_01/C1 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |
| ON_REPEAT_01/C2 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |
| ON_REPEAT_01/C3 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |
| ON_REPEAT_01/C4 | join 0.000000s | join 0.000000s | join 0.066667s | join 0.066667s |

| Condition | Mean position AUC | Mean yaw AUC [rad·s] | Sustained attach | Motion invalid | Mean linear TV [m/s] | Mean angular TV [rad/s] |
|---|---:|---:|---:|---:|---:|---:|
| DELAYED_NATIVE | 0.008983 | 0.020787 | 8/8 | 1/8 | 0.000000 | 0.811996 |
| DELAYED_LOOKAHEAD | 0.009950 | 0.023459 | 8/8 | 3/8 | 0.000000 | 0.767631 |
| LATENCY_FREE_NATIVE | 0.019514 | 0.016762 | 8/8 | 0/8 | 0.000000 | 0.586738 |
| LATENCY_FREE_LOOKAHEAD | 0.024277 | 0.028205 | 8/8 | 0/8 | 0.000000 | 0.527550 |

| Contrast | Event mean position | Episode-equal position | Event mean excess position | Event mean yaw | Event mean excess yaw |
|---|---:|---:|---:|---:|---:|
| delay_native | -0.010531 | -0.010531 | -0.001619 | 0.004025 | 0.000000 |
| delay_lookahead | -0.014327 | -0.014327 | -0.001619 | -0.004746 | 0.000000 |
| interface_delayed | -0.000967 | -0.000967 | 0.000000 | -0.002672 | 0.000000 |
| interface_latency_free | -0.004762 | -0.004762 | -0.000000 | -0.011443 | 0.000000 |
| interaction | 0.003796 | 0.003796 | 0.000000 | 0.008771 | 0.000000 |

All four metrics also have episode-equal values and event-level signed contrasts in `summary.json` and `paired_delay_gaps.csv`. Interface contrasts are native minus prepared+SP, not selector-only effects.


## Safety, motion and what the initialization actually measures

All84 paths remained inside the known workspace and passed the unchanged required
clearance check; zero physical overlaps, zero safety aborts and zero controller
failures. Minimum curve-adjusted clearance across all conditions was.119760m
for genuine13 and.493842m for ONLINE03. These are local .9s counterfactuals,
not completed obstacle bypasses or continuous-time real-robot safety guarantees.

Motion acceptance is **not** all-pass. Every genuine delayed condition (26/26)
has a first-command acceleration violation. Nominal first angular acceleration
is about9.99763–10.00000rad/s² against the unchanged5rad/s² limit; some first
linear changes reach about4m/s² against2m/s². Every violation is at interval0;
no later integration interval adds a speed/acceleration violation. Both genuine
latency-free conditions are motion-valid. ONLINE03 has one native-delayed
violation (ON00/C2,5.27183rad/s²) and three prepared+SP delayed violations
(ON00/C1,5.83641; ON00/C2,7.12299; ON01/C1,5.79827rad/s²). All are first-step;
all16 ONLINE03 latency-free executions are motion-valid. No thresholds changed.

This exposes the distinction between a source-exact controller memory snapshot
and a physically continuous first new command. The source worker had already
advanced memory with the historical first-FRESH result; a new synchronous solve
at the same B may advance it again before the stored physical command has caught
up. Official MPC constrains increments relative to **memory**, while the physical
motion checker compares against **u_minus**. We followed the requested actual
memory rather than resetting it, but this means the delayed conditions are not
historical native replay and are not all physically admissible baselines.
The entire genuine cohort therefore has **zero motion-valid complete delay
pairs**. AUC differences are retained as measurements; they do not isolate
geometric delay from memory/application-phase effects. No unrequested fifth
initialization condition or scientific retry was added to resolve this.

`motion_diagnostics.json/csv` provides every violating interval, magnitude and
source memory row. The original acceptance still rejects these motion violations.
Scientific rejection is separate from artifact validity.

## Cohort dependence and attachment changes

Genuine13 has seven episodes and13 unique ordered raw pairs (10 unique FRESH
raw arrays); ONLINE03 has two episodes, eight transitions, seven unique ordered
raw pairs and six unique FRESH arrays. Events are not IID. Original
observation-to-switch simulation delays are.383333–.900000s for genuine13 and
.383333–.400000s for ONLINE03. Host RTT and these simulation durations are not
subtracted across clocks; original source timestamps remain in the event ledger.

Genuine native comparison changes12 cases from no entry to observed sampled
attachment; one stays attached. Prepared+SP changes10 from no entry to observed
attachment, two from no entry to transient entry, and one remains attached.
The two transient Oracle cases are the retained endpoint-caveat sources
014/01/025 and013/00/033. Native latency-free attaches on both; the B-prepared
suffix limits the corresponding prepared+SP condition. This demonstrates why
preparation cannot be hidden under a selector-only label. No case is removed.

All eight ONLINE03 transitions attach in all four conditions. Delayed starts
are already inside the original-FRESH tube (join time0), whereas observation
starts enter at.066667s. The metric projects to the returned polyline, whose
first row is generally forward of the observation pose; no backwards extension
or invented observation→first-row model segment was added. Thus a negative
ONLINE03 position gap does not mean latency is universally beneficial.

The representative hard013/01/024 retains a position gap of.262330m·s under
native and.294194m·s under prepared+SP. Its delayed trajectory first increases
the already-present offset; both latency-free conditions attach at.116667s.
Case008/01/023 has gaps.161018/.163618m·s and latency-free attachment at.066667s.
The predeclared small-gap illustration001/01/013 is still a selected mismatch
case, not a new benign control: gaps.079705/.080910m·s, Oracle join.066667s.
The delayed paths' first-step motion failures qualify all of these observations.

## Research interpretation and decision support

Overall classification: **MIXED_ATTRIBUTION**.

Within genuine13, a delay-sensitive gap remains after the frozen reference
interface change: native positive gaps13/13, prepared+SP12/13; prepared+SP mean
position gap.124985m·s, episode-equal.127858m·s. The interface does not explain
most measured cost in this selected mismatch cohort. Its delayed mean AUC is
slightly higher (.168218 versus native.165127m·s), so this is not evidence of a
general lookahead benefit. Within ONLINE03, all conditions attach and the mean
position gap has the opposite sign. Together with the all-genuine first-step
motion limitation, this supports a bounded mixed interpretation, not a clean
causal estimate of delay alone or a claim that graph optimization is necessary.

| Research path | Evidence supporting consideration | Evidence/limitation against a strong conclusion |
|---|---|---|
| Path1: reconciliation method separated from upstream | Same saved FRESH is much easier from observation in most genuine mismatch cases; large gaps remain under both frozen interfaces; local geometry remains clear. | Every genuine delayed counterfactual violates first-step motion. Complete state packages differ, and native→lookahead also changes preparation. No same-B repair was tested. |
| Path2: problem definition/evaluation protocol | ONLINE03 onset already attaches in every condition; endpoint preparation and exact memory/application phase materially affect the interpretation. | Reference correction does not remove the genuine13 gap, so interface artifacts alone are not a sufficient explanation of that selected cohort. |

The research direction remains the user's decision. The most important remaining
uncertainty is whether a **motion-admissible, same-B comparison** retains reducible
attachment cost after accounting for the controller-memory/application boundary.
No such next experiment is implemented here.

Repository facts: all21 sources and84 conditions covered, exact saved inputs and
state reconstruction, completed primary756 solves, the signed metrics above,
safe local geometry, and the explicit motion failures. Research interpretation:
state-package delay sensitivity depends on the source regime and survives this
interface change in genuine13, with the stated initialization limitation.

Not demonstrated: graph/SE(2) optimization benefit; a recoverable maximum cost;
obstacle-constrained improvement; online closed-loop improvement; real-robot
benefit; or population prevalence. An Oracle starts at observation; a future
reconciliation would start at B. Their gap is never an achievable improvement
bound.

## Artifacts and compute

Authoritative main run:
`data/handoff_delay_attribution_01/primary_20260923T055300Z/`.
Pushed scientific execution SHA:
`b9d44afd65676d97c890eea99e224af4db60c322`.
Original freezec823be8 and failed run054600 remain preserved. Original source,
REF02 numerical/selector files, official MPC and two unrelated Stage0 edits
retain their hashes. No generated arrays, RGB, PNG or ZIP are tracked.

The complete main loop (including worker startup, guard queries, serialization)
took7.883226s; sum of756 official solver wall times2.571845s. Primary saved-only
validation took7.526908s and makes zero new MPC/VLA calls. An isolated historical
failed request's solver wall time is unavailable, not filled with zero.

The run contains source/controller-state reconstruction, cohort/availability,
all84 event metrics, signed paired gaps, categorical transitions, all source and
frozen code hashes,21 event overlays and six cohort figures with numeric/hash
sidecars, and `index.html`. `review_bundle_complete.zip` includes all27 figures
and their numeric sidecars. This is a static review; no GUI/Isaac was run.
The earlier `review_bundle.zip` omits the plot JSONs and is superseded for review
by the complete bundle. A small Git-readable84-row result table is preserved in
[`docs/results/handoff_delay_attribution_01.json`](results/handoff_delay_attribution_01.json).

Primary validation reconstructs the21 source cuts, anchors, exact FRESH/common
arrays, controller memories,84 independent tracker installations,756 selected
MPC inputs,54-step integration, command holds/guards, original metrics, CSV
arithmetic and plotted values; it checks before/after hashes and transcript
ordinals without a new solve. Original final saved-loss and ONLINE03 authoritative
saved-record validators also pass. Supplemental report validation independently
recomputes motion diagnostics and counts756 solve requests and84 initializations
from the durable IPC transcript.

Reproduction uses the earlier command block with run055300 substituted. After
primary validation, reporting only:

```bash
.venv/bin/python scripts/report_handoff_delay_attribution.py --run data/handoff_delay_attribution_01/primary_20260923T055300Z
```

All output writers reject overwrite. For a new scientific run the same protocol
requires another explicit freeze; this task runs no additional comparison.
Operational: **HANDOFF_DELAY_ATTRIBUTION_01_COMPLETED_WITH_LIMITATIONS**.

### Final validation and test-call accounting

Final full suite: **2719 passed,19 skipped**,175.65s; prefreeze full suite2718
passed/19 skipped,175.43s. Skips concern unavailable historical fixtures and an
existing representation-diagnostic fixture. Compileall and diff checks pass.
Focused tests use synthetic trackers/commands and make zero real MPC/VLA calls.
However, each **full** suite includes the existing
`test_real_official_historical_solve_reproducibility_when_available`: one real
historical MPC solve, no counterfactual integration. Thus task accounting is
**756 primary +1 failed IPC request +2 historical test solves =759 MPC requests**.
The early preflight zero-real-MPC statement applied only to focused/import checks,
not the full suite. `test_and_call_accounting.json` preserves this correction.
The final historical reproduction command matches literally; the first suite's
temporary numeric record was cleaned, so its solver time remains N/A.

`report_validation_final.json` binds the final reporting-only script, recomputes
all primary metrics and supplemental motion tables, checks27 figures/sidecars,
verifies complete ZIP members byte-for-byte and checks the public numeric table.
It does not call a solver. The preceding report validation remains as a retained
packaging revision, with unchanged numerical diagnostics. Final saved-only command:

```bash
.venv/bin/python scripts/report_handoff_delay_attribution.py --run data/handoff_delay_attribution_01/primary_20260923T055300Z --validate-only
```
