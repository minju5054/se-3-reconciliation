# OSA03_COMMON_B_METHOD_COMPARISON_01

## Frozen protocol

One offline same-B comparison on sealed OSA03 `REPEAT_00` only. No new LightNav,
RGB, Isaac/source acquisition, graph/GP solve, ablation, R01 or other cohort.
Starting fetched `main`/`origin/main`: `f40f3de2dc2d0c114f65212f54c7207ec1d0ac61`.
Unrelated edits in the two stage0 controller/chunk configs are preserved and not
staged. Local run: `data/osa03_common_b_method_comparison_01/primary_20260925T162125Z/`.
Actual UTC/monotonic clocks are recorded independently of this run identifier.

### Source and fixed references

OSA03 source: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`.
OLD SHA256 `6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae`;
FRESH SHA256 `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`.
M3 comes from `data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/derived/optimized_world.npy`,
SHA256 `bd459b0343fac1e78e4ac80831f4f99af9b86dd4876a3b0df1b45f41c21ce120`.
All M3 saved result artifacts are authenticated, but no historical source manifest
is traversed into R01 and no optimization is repeated.

World: Isaac XY metres, yaw CCW radians. Body: forward +x, left +y.
`A=R(t_obs)`, `B=R(t_switch)`, `F=A F_raw`, `G=B A^-1`.
` s_j = cumulative original-FRESH XY arc / total original-FRESH XY arc `.
Generic N>=2 with positive XY arc, no timestamps, resampling or cropping.

| Fixed order | World reference | Construction |
|---|---|---|
| M0_NATIVE | F | Exact original world/raw local bytes |
| M1_SE2_TAPER | Exp((1-s_j) Log(G)) F_j | Deterministic spatial linear taper; no solve |
| M2_RIGID_TRANSPORT | G F_j | Exact full SE(2) transport; no solve |
| M3_LOCAL_SE2 | authenticated frozen X | Existing proposed output; no new solve |

M1/M2/M3 local representations use `A^-1 R_world`; this is **not B anchoring**.
Each exact local/world `.npy` byte hash is frozen before scientific execution.

### Common asynchronous controller phase

B = `[19.203630553052257,24.14333536015282,-1.5689760264727979]`,
absolute tick92, simulation time `1.5666667483747005` s.
Physical `u_minus=[.8,-1.5584691584098122e-06]`;
common already-applied `u_B_plus=[.8,.49999844893907003]`;
worker memory `u_mem_B=[.8,.49999844893907003]`.
These are loaded/authenticated, not rounded constants used as solver inputs.

The historical first-FRESH solve000005 (submitted90/applied92) supplies the common
initial held command and memory. **No solve at B. No replay of solve000006.**
The next new submit is the original absolute tick96, then102,108,...,270.
Original source dt=`float32(1/60)=0.01666666753590107` s, control stride6.
A new independent worker/tracker is created per method. Before the comparison
clock, the official observation-A installation transform materializes its reference;
then saved generation3, held command, memory and first-result diagnostic fields
are restored. There is no new install event at B or later. The official next solve
rebuilds nearest/+1 selected rows from the method reference, not cached rows.

The external official source/MPC remains read-only and unchanged:
LightNav `c6f40e3220edbf7011e4f17eaf2c865416737d4d`,
MPC SHA256 `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
H=5, solve dt=.1, Q=[10,10,1], R=[.1,.1], v<=.8, |omega|<=3,
nominal a_v=2 and a_omega=5 remain unchanged. No warmup solve.

### Safety, runtime and timing gate

Before execution, every entire returned polyline passes the existing direct mesh
Hospital+cart checker (radius .20 m, edge margin .05 m, original numerical reserve
1e-7 and known workspace). No invented B-to-first-row connector is treated as raw
path. An unsafe reference gets `REFERENCE_GEOMETRY_UNSAFE`, null execution metrics
and no fallback. Pre-execution minima:

| Method | Minimum edge clearance m | Limiting geometry | Run eligible |
|---|---:|---|---|
| M0 | .22948689016047874 | cart | yes |
| M1 | .22948689016047874 | cart | yes |
| M2 | .09144233803505619 | cart | yes |
| M3 | .22780920818555810 | cart | yes |

Each eligible method runs exactly once in the frozen order. Cap180 integration
intervals, ~3.000000156 s after common B, not LightNav waypoint timing.
Unchanged abort-only guard checks new command hold=.1 s and held command next dt,
with exact unicycle integration/curve allowance; unsafe proposals remain unapplied.
No steering/clipping, no later FRESH updates. Hold timeout .5 s and controller
failure terminate without a fabricated trajectory. No catch-up pacing.

Primary equal exposure is first54 intervals (~.900000047 s). Save actual submit,
accepted-submit and success-application ticks. The comparability gate requires
**identical full sequences through inclusive B+54** and complete exposure, not
just nominal 10 Hz. No schedule adjustment/retry to improve parity.
Priority: technical failure → `TECHNICAL_EXECUTION_BLOCKED`; differing schedules →
`TIMING_CONFOUNDED_COMMON_B_COMPARISON`; reference skips →
`REFERENCE_LIMITED_COMMON_B_COMPARISON`; otherwise `COMMON_B_COMPARISON_VALID`.
All applicable limitations remain separately visible.

Historical Native `data/osa03_native_continuation_01/primary_20260923T103000Z/`
is revalidated saved-only (R00 phase, prefix, all integrated states, guard, selections,
metrics). Historical continuation replayed solve000006 applied97; new M0 solves96
from a cold controller. Compare next selected rows/command, actual schedules,
.3/.9 metrics, attachment and clearance. If full application schedules match,
require state/command parity atol1e-10; otherwise show differences without forcing
parity. No historical Native rerun beyond the one newly authorized M0 condition.

### Evaluation and claims

All methods are evaluated against **original FRESH**, using unchanged Native
continuous forward-only projection, shortest-angle yaw interpolation and complete
following .30 s sampled dwell within .10 m /15 degrees. Null/censoring is preserved.
Primary: .9 s position/yaw AUC, sustained attachment time and original progress,
remaining original arc at attachment, execution swept clearance lower bound.
Secondary: .3/full windows, initial .5 s separation growth, endpoint/progress,
linear/angular command TV incl switch and post-B separately, nominal-grid versus
actual-application finite differences, maxima, selections and computation costs.
Signed method-minus-M0 differences are retained; no score/ranking/percentages.

Reference diagnostics: node/endpoint displacement, relative-edge translation/yaw
RMS/max, closed-form rigid-fit residual. This fit is diagnostic, no baseline solve.
Eight figures: world execution, reference-only geometry, position, yaw, progress,
clearance, commands and application timeline, each with numeric/hash sidecar.
Skipped/unavailable methods get no fabricated execution curves.

This comparison does not measure improvement in the already shared initial switch
command. Reference benefit attribution is restricted by observed application timing.
One source cannot establish generalization, complete bypass, obstacle traversal,
real-robot benefit, method necessity or superiority to an untested method.

### Reproduction

```bash
RUN=data/osa03_common_b_method_comparison_01/primary_20260925T162125Z
.venv/bin/python scripts/run_osa03_common_b.py --run "$RUN" --mode prepare
.venv/bin/python scripts/run_osa03_common_b.py --run "$RUN" --mode preflight
# Focused + relevant tests, diff review, freeze file generation, commit and push.
.venv/bin/python scripts/run_osa03_common_b.py --run "$RUN" --mode freeze
# Only after pushed freeze commit:
.venv/bin/python scripts/run_osa03_common_b.py --run "$RUN" --mode execute
.venv/bin/python scripts/validate_osa03_common_b.py --run "$RUN"
.venv/bin/python scripts/report_osa03_common_b.py --run "$RUN"
.venv/bin/python scripts/report_osa03_common_b.py --run "$RUN" --validate-only
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Preparation has completed without scientific solves. The original temporary run
folder name was normalized to the actual UTC preparation identifier before freeze;
only unfrozen reference path metadata changed. Source/artifact bytes are unchanged.
The zero-solve external preflight replaces `MPCController.solve` with an in-process
raising stub, instantiates/restores each tracker, and confirms all four installation
transforms and states. No external file is modified. Tests use mock controllers,
not genuine numerical MPC. Scientific results will be appended after the one run.
