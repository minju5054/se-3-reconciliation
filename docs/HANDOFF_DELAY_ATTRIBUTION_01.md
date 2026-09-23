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
