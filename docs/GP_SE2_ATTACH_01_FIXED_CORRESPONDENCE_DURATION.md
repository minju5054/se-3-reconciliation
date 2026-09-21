# GP-SE2-ATTACH-01: fixed correspondence and externally imposed duration

Question: given a fixed OLD/FRESH correspondence, can the existing constrained
SE(2) GP construct an executable transition from actual B into FRESH, and how
does an externally imposed duration affect feasibility and sustained attachment?
This is one recorded obstacle-sensitive mechanism experiment. Correspondence and
duration selection are not research contributions or optimization variables.

Starting and freshly fetched origin/main: `44e204499cb18855de2236780a4244903cd35d8a`.
JOIN-01's qualification failure, acquisition, code and outputs remain unchanged.
No new reveal, inference, scene export, selector or controller development occurs.
The two pre-existing Stage-0 config edits remain untouched and uncommitted.

## Source-only freeze

Run: `data/robotless_gp_se2_attach_01/primary_20260921T103000Z/`.
Config: `configs/gp_se2_attach_01.yaml`; authoritative numerical config is the
GP-SE2-01 primary `config_snapshot.yaml`. Hospital geometry is the original
primary export, authenticated against preserved REF-03 provenance and original
authoritative validations. The complete 881-event ledger is saved. Historical
method outcomes and their selected-case rankings are not selection inputs.

Eligibility uses original GP01 source integrity and exact client-inflight timing,
the original `VALID_HANDOFF_MOVING` label, recorded B/physical command/controller
memory verification, no required finite gate and a known route, valid B, and
independently swept-valid original suffix **and** prepared common reference.
Required edge clearance remains .05m with original geometry uncertainty 1e-7m;
obstacle sensitivity means original suffix minimum clearance <=.20m. No B-to-row
connector is inserted in this suffix metric. Mismatch requires existing
e_perp>=.10m OR reliable window direction>=20deg OR finite projected pose-yaw
disagreement>=20deg on nondegenerate simple geometry. Direction reliability
retains original .05m incoming/.02m window chord checks.

Before scanning, future sufficiency was operationalized as >=2 original suffix
rows, positive XY arc >1e-6m, and the original 30-row .1..3s preparation; the
whole original XY polyline must be simple with each segment >1e-6m to avoid
ambiguous projection. This does not assert physical 3s timestamps in LightNav.
No criterion is relaxed after scanning. Integrity errors block the experiment
instead of silently excluding corrupted cases.

730 original eligible →710 no-gate/known-route →49 obstacle-sensitive →10 mismatch
→9 unambiguous eligible cases. Lexical SHA256(`ATTACH01-v1:`+full case ID), then
case ID, selects **episode_021_repeat_01/handoff_003**. All nine eligible IDs and
all 881 reasons are preserved, including one geometry rejection. No historical
GP pilot exclusions are added; this is not held-out evaluation.

Selected original suffix clearance .0984265005393m; prepared clearance
.0984252061139m; existing e_perp .141201449561m. Native has10 rows, nearest j=0,
existing suffix starts1 and retains9 rows; common has30. Route requires no gate.
The fixed common float64 value hash is
`c67a128205a70383fbc9b89fdb21bd3d0d377287e571e52204f855500e6f3a7f`.

Important source limitation: this record is moving under the collector's
xy/yaw displacement >1e-6 criterion, but physical speed immediately before B
is ~1.80e-9m/s (omega~-5.79e-5rad/s). OLD/FRESH local raw arrays have the same
hash; their observation poses differ. This is not obstacle-reveal response or
evidence of substantially moving B at switch. The predetermined source rule
is retained without replacing the event. e_perp is the existing projection
metric, not a newly asserted signed lateral error.

## Fixed formulation and protocol

One `prepare_reference` construction per source event during scanning. The
selected cached common array is saved directly, not rebuilt per duration. Raw
FRESH remains observation-anchored. A separate validator reconstructs preparation
only to verify byte identity. B is support0; **common row k-1 corresponds to
support k**, time k*.1s. There are no intrinsic LightNav waypoint timestamps.

External T={.4,.6,.8,1.,1.2,1.4}s, all six conditions once. Original chart has
150 variables/31 supports/3s horizon. M4 cost is unchanged GP prior plus original
lambda times mean squared whitened FRESH Log residual over supports k>=kT.
The target is the SAME common[k-1], never a retimed/compressed/new suffix.
There is no FRESH preservation term before T. Original midpoint lateral
equalities, motion at u={0,.5,1}, obstacle/workspace/goal constraints remain.
No DIAG06 witness, DIAG08 reserve, extra lateral row, scaling or tolerance change.

At T,T+.1,T+.2,T+.3 append `.10²-||p-reference||²>=0` and two signed 15deg yaw
bounds. These 12 inequality rows give30 equalities/915 inequalities; original
M3 has30/903. Original physical full checker and separate **nominal** tube check
are both required; solver numeric allowance is not a tube acceptance relaxation.
Continuous-time safety is not claimed from samples.

Methods: unchanged official Native, unchanged common Adapter, generic current
M3 (original formulation with verified supplied derivatives), six fixed-T M4.
Rigid is omitted under the permitted optional scope. Each GP condition uses
the same saved original FRESH and fixed same-curvature deceleration seeds,
two starts per condition; M3 two plus M4 twelve =14 scheduled starts.
No historical latest iterate or handoff-specific seed. Each start once,
CPU float64 JAX/analytic geometry derivatives, original SLSQP200 iterations,
ftol1e-7,30s prepared budget, single-thread BLAS. All original Jacobian prefixes
remain literal. M4 reuses JOIN01's masked-cost/tube **AD algebra only**; it never
calls JoinView, join_candidates or post_join_reference.

All14 seed/formulation gates passed original primal and three-direction,
three-step FD validation before primary; both fine steps must meet existing
DIAG02 tolerances. Runtime wrap-cut guards remain. Provider compilation may be
reused across seeds/shapes; warmup/construction/solve/check costs are separate.
Preflight compilation is additional offline cost, not hidden in prepared budget.
`source.json.official_mpc` retains the historical pin/settings provenance record
from REF-03 (which contains REF-02 instrumented-wrapper metadata). ATTACH-01
does not activate that wrapper; its live official import provenance is saved
separately in `rollouts/provenance.json`. Both source hash and loaded constants
are checked. No source-progress selector is used by this experiment.

Retention: inspect initial/latest/actual solver-grid-feasible callbacks through
the unchanged DIAG04 instrumented harness. M3 preserves its historical
dense-first selected record and requires independent full validity. M4 selects
only among retained full-valid plus nominal-tube-valid records, minimum J then
initialization label/source label, separately **within each T**. There is no
cross-T method selection or deployment choice, and no RAW fallback.

Primary order: M3 I0/I1, then T=.4,.6,.8,1,1.2,1.4 each I0/I1; after planning,
Native→Adapter→M3→all available M4 in ascending external T. Independent official
MPC trackers, unchanged nearest/+1 selector and H5/.1s, original B/u_minus and
separate controller memory,30 solves/180 exact held-command steps/181 states,
3s horizon. World reference is inverted through original capture pose for path
installation, never reanchored to B. GP velocities are not feed-forward.
Invalid GP does not execute. No safety stop/pose snap is introduced.

OFFLINE SINGLE-HANDOFF COUNTERFACTUAL: optimizer/MPC wall time never advances
simulation time. Native/Adapter keep original spatial-reference diagnostic
rollout policy; their rollout success is separate from GP continuous plan checks.

## Metrics and static output

Reuse JOIN01's forward-only continuous XY polyline projection with shortest yaw
interpolation, against ORIGINAL FRESH. Closest remaining projection cannot move
backward. Earliest full following .30s saved sample window within .10m and15deg
is sustained attachment; a crossing is insufficient, failure is null. Execution
queries include original <=5ms and60Hz boundaries; GP queries use the original
<=1ms/.371 offset grid. These are sampled metrics, not continuous certificates.

Per condition report plan validity, solver termination and failures, planned
and executed attachment, relation to external T, original safety/motion/goal/dwell,
post-attachment error, pre-attachment AUC (N/A without join), full-window AUC,
command TV/accelerations and compute. Rejected latest-iterate attachment is an
explicit diagnostic, never a returned valid-plan attachment. Shortest **tested**
full-valid duration is descriptive within this exact set, not optimal T.

Figures: selected source; common-axis method small multiples; execution-to-original
FRESH distance/yaw; duration validity/attachment/clearance/solve cost; command TV.
No unavailable execution curve. Every PNG carries numeric/source/config/common
hash sidecar. Index and small review ZIP are static; GUI runtime zero.

## Exact commands

```bash
.venv/bin/python scripts/run_gp_se2_attach01.py prepare --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/run_gp_se2_attach01.py preflight --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
# Unit/relevant tests, diff review, then normal pre-primary commit AND push.
.venv/bin/python scripts/run_gp_se2_attach01.py freeze --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/run_gp_se2_attach01.py optimize --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python scripts/lightnav/gp_se2_attach01_rollout.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/run_gp_se2_attach01.py evaluate --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/plot_gp_se2_attach01.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/validate_gp_se2_attach01.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z
.venv/bin/python scripts/plot_gp_se2_attach01.py --run data/robotless_gp_se2_attach_01/primary_20260921T103000Z --package
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

No primary optimization or MPC comparison had run when this protocol was written.
Source scan and derivative gates are preparation, not performance outcomes.
