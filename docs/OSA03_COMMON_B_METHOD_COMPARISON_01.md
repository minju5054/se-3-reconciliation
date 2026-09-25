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

## Completed result (2026-09-26 KST)

Scientific freeze SHA: `8a84b47f030f0e88f750f8044b56154ef6cc98a5` (pushed before execution).
Exactly four scientific rollouts ran once; no skips/retries/new optimization.
Saved-only validation PASS for all methods, source hashes, installed observation
anchors, restored state, command-memory evolution, nearest/+1 selection, exact
integration, guard decisions, metrics and signed arithmetic. Eight figure
sidecars and CSV/JSON parity PASS. No frozen implementation changed after execution.

**Final classification: `TIMING_CONFOUNDED_COMMON_B_COMPARISON`.**
M3's first new command applied one integration tick later than the other methods.
This prevents attributing its measured differences solely to the reference.
No timing repair/retry was performed.

### Exact frozen reference hashes

| Method | World .npy SHA256 | Observation-A local .npy SHA256 |
|---|---|---|
| M0_NATIVE | `9c8ea94f650d823ac3e1ada0a216dc36758432713fd5407cf18324069865bfc4` | `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521` |
| M1_SE2_TAPER | `50c086b767c2f507d381f13526fe3c76585edc39290f4df5c2bc6f1ea4dde23d` | `6e709a8dd56a2059a6dfff07d3f6fac328ab9f5a2cc42c3ec0f48c18671e6d36` |
| M2_RIGID_TRANSPORT | `057482c22d9dd489dd37b15c6bf1c64d93e2551b279b6dd969f6f3a48992f28b` | `dd3e9fac6184c31efb8c9cc8209e6a7d31adc3508020c52b61ee25386b5cbdd1` |
| M3_LOCAL_SE2 | `bd459b0343fac1e78e4ac80831f4f99af9b86dd4876a3b0df1b45f41c21ce120` | `b7133196bdcf9ff65e7ed99b4e9f0baf144f1a470ba5d8e139baa48935ce90c0` |

### Primary original-FRESH comparison

All methods begin at the same original-FRESH distance .106648569 m and yaw error
30.000556457 degrees. All completed the primary 54 intervals. Position AUC is m·s;
yaw AUC is rad·s. Sustained attachment is complete following .30 s sampled dwell.
Execution clearance is the swept lower bound (m), not centerline clearance.

| Method | .9s position AUC | .9s yaw AUC | Attach s | Original row progress /9 | Arc remaining at join m | Execution min clearance m | Termination |
|---|---:|---:|---:|---:|---:|---:|---|
| M0_NATIVE | 0.169065633 | 0.193368049 | 1.466666743 | 8.717846370 | 0.042484924 | 0.133561094 | OBSERVATION_CAP |
| M1_SE2_TAPER | 0.166853435 | 0.187911281 | 1.500000078 | 8.738964973 | 0.039305018 | 0.134208269 | OBSERVATION_CAP |
| M2_RIGID_TRANSPORT | 0.175414845 | 0.168074823 | N/A | N/A | N/A | 0.058965146 | SAFETY_ABORT_BEFORE_UNSAFE_COMMAND |
| M3_LOCAL_SE2 | 0.167335299 | 0.193304698 | 1.466666743 | 8.617009689 | 0.057668279 | 0.135997429 | OBSERVATION_CAP |

The rigid run's clearance is over its actual 1.583333416 s exposure; it is not a
3 s comparison. M0/M1/M3 complete 3.000000156 s. No method enters the attachment
tube within the primary .9 s. M0/M1/M3 first tube entry equals sustained onset,
then remains in the tube through the cap; M2 has no entry before abort (null, not 3 s).

| Method minus M0 | Δposition AUC .9 | Δyaw AUC .9 | Δattach s | Δrow progress | Δremaining arc m | Δmin clearance m |
|---|---:|---:|---:|---:|---:|---:|
| M0_NATIVE | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 |
| M1_SE2_TAPER | -0.002212197 | -0.005456769 | 0.033333335 | 0.021118603 | -0.003179907 | 0.000647175 |
| M2_RIGID_TRANSPORT | 0.006349212 | -0.025293226 | N/A | N/A | N/A | -0.074595948 |
| M3_LOCAL_SE2 | -0.001730334 | -0.000063351 | 0.000000000 | -0.100836681 | 0.015183355 | 0.002436334 |

### Additional windows, commands and endpoint

| Method | .3s position AUC | .3s yaw AUC | Full observed position AUC | Full yaw AUC | Max position error m | Max time s | Endpoint error m |
|---|---:|---:|---:|---:|---:|---:|---:|
| M0_NATIVE | 0.047920362 | 0.122763625 | 0.392619117 | 0.332788196 | 0.215262377 | 0.466666691 | 0.096439021 |
| M1_SE2_TAPER | 0.047473914 | 0.122779344 | 0.390854706 | 0.328381290 | 0.210303942 | 0.483333359 | 0.095981706 |
| M2_RIGID_TRANSPORT | 0.047920563 | 0.122794565 | 0.296410195 | 0.225683685 | 0.219047049 | 0.516666694 | 0.171107918 |
| M3_LOCAL_SE2 | 0.047557774 | 0.124232073 | 0.386756956 | 0.335504161 | 0.211756829 | 0.483333359 | 0.093879587 |

M2 full-window AUC covers only the aborted 1.583333416 s; its 180-interval result
is explicitly unavailable. All projections are forward-only/monotonic and reach
original final-row progress9 by termination; this does not by itself mean attachment.
M0/M1/M3 settle approximately .094–.096 m from the original endpoint, with final
speed near zero and omega about -.0035 to -.0037 rad/s. They do not meet the frozen
near-zero-all-command diagnostic. Original endpoint is not complete obstacle bypass.

All methods share Δu_B=[0,+.5000000074082285]. Nominal 10 Hz command-grid checks PASS.
Actual-application interval differences below are descriptive, **not continuous
physical acceleration** and not silently substituted for the nominal limiter.

| Method | linear TV incl switch | angular TV incl switch | post-B angular TV | max abs omega | max actual interval dv/dt | max actual interval domega/dt |
|---|---:|---:|---:|---:|---:|---:|
| M0_NATIVE | 0.800000000 | 3.629555771 | 3.129555763 | 1.548333658 | 1.999999955 | 7.499999701 |
| M1_SE2_TAPER | 1.199999972 | 3.513599927 | 3.013599920 | 1.499998444 | 2.999999639 | 7.499999426 |
| M2_RIGID_TRANSPORT | 0.000000000 | 3.097946604 | 2.597946596 | 1.499998430 | 0.000000000 | 7.499999300 |
| M3_LOCAL_SE2 | 1.199999860 | 3.508589892 | 3.008589885 | 1.499998441 | 3.999998400 | 9.999999267 |

M1/M3 increase linear command TV relative to Native (.8→1.2 m/s), while angular TV
is lower. This is a mixed measured pattern, not a weighted-score win. Historical
shared switch omega increment is at the existing nominal acceleration limit.

### Reference deformation (before controller execution)

| Method | First-node shift m | Endpoint shift m | Edge translation RMS m | Edge yaw RMS rad | Rigid-fit translation RMS m | Rigid-fit yaw RMS rad |
|---|---:|---:|---:|---:|---:|---:|
| M0_NATIVE | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 |
| M1_SE2_TAPER | 0.213333346 | 0.000000000 | 0.023703805 | 0.000000086 | 0.057411363 | 0.090991645 |
| M2_RIGID_TRANSPORT | 0.213333346 | 0.213333870 | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 |
| M3_LOCAL_SE2 | 0.211144018 | 0.002186831 | 0.026158510 | 0.001274846 | 0.066834803 | 0.103853184 |

### Actual asynchronous schedule

The following are the **complete** attempted submit and new-command application
sequences. Submitted requests were all accepted. Application92 is the common
saved first-FRESH command, not a new solve. No busy, stale rejection, hold timeout,
controller error or failed solve occurred. M2 solve186 returned but remained unapplied
because the guard aborted at187.

- **M0_NATIVE** submit: `[96, 102, 108, 114, 120, 126, 132, 138, 144, 150, 156, 162, 168, 174, 180, 186, 192, 198, 204, 210, 216, 222, 228, 234, 240, 246, 252, 258, 264, 270]`; apply: `[92, 99, 103, 109, 115, 121, 127, 133, 139, 145, 151, 157, 163, 169, 175, 181, 187, 193, 199, 205, 211, 217, 223, 229, 235, 241, 247, 253, 259, 265, 271]`.
- **M1_SE2_TAPER** submit: `[96, 102, 108, 114, 120, 126, 132, 138, 144, 150, 156, 162, 168, 174, 180, 186, 192, 198, 204, 210, 216, 222, 228, 234, 240, 246, 252, 258, 264, 270]`; apply: `[92, 99, 103, 109, 115, 121, 127, 133, 139, 145, 151, 157, 163, 169, 175, 181, 187, 193, 199, 205, 211, 217, 223, 229, 235, 241, 247, 253, 259, 265, 271]`.
- **M2_RIGID_TRANSPORT** submit: `[96, 102, 108, 114, 120, 126, 132, 138, 144, 150, 156, 162, 168, 174, 180, 186]`; apply: `[92, 99, 103, 109, 115, 121, 127, 133, 139, 145, 151, 157, 163, 169, 175, 181]`.
- **M3_LOCAL_SE2** submit: `[96, 102, 108, 114, 120, 126, 132, 138, 144, 150, 156, 162, 168, 174, 180, 186, 192, 198, 204, 210, 216, 222, 228, 234, 240, 246, 252, 258, 264, 270]`; apply: `[92, 100, 103, 109, 115, 121, 127, 133, 139, 145, 151, 157, 163, 169, 175, 181, 187, 193, 199, 205, 211, 217, 223, 229, 235, 241, 247, 253, 259, 265, 271]`.

Primary submit ticks for all: `[96,102,108,114,120,126,132,138,144]`.
M0/M1/M2 successful application ticks: `[99,103,109,115,121,127,133,139,145]`;
M3: `[100,103,109,115,121,127,133,139,145]`.
The first cold numerical solves took 45.596/47.242/48.337/48.149 ms respectively;
wall/IPC receipt alignment placed M3 one tick later. No inference about a systematic
method compute-time difference is justified from one solve per condition.

### Historical Native parity

The old saved Native was independently revalidated without rerun. New M0 matches
B/held-command/memory, next submit96, selected rows, and first result command exactly:
`[.8,.999998454469043]`. It cannot match historical application timing: historical
`[92,97,105,109,...]` versus new `[92,99,103,109,...]`. Therefore strict full trajectory
parity is not required by the frozen conditional gate, and is **not claimed**.
Max combined pose-component difference is .016666668; max command difference .500000006.

| Metric | Historical Native | New M0 |
|---|---:|---:|
| .3s position AUC | .047738553 | .047920362 |
| .9s position AUC | .168221794 | .169065633 |
| .9s yaw AUC | .191402386 | .193368049 |
| Sustained attachment s | 1.466666743 | 1.466666743 |
| Execution minimum clearance m | .133806153 | .133561094 |

### Safety outcome

M0/M1/M3 retained valid execution margin through the cap, with minima at the final
state. M2 stopped at tick187/time1.583333416, pose
`[19.784944041916173,23.0445461309621,-.978783294614642]`.
Its applied prefix remained valid, min lower bound `.05896514625340536` m.
Proposed command `[.615373073711719,-.10857069218404979]` predicted a next-.1 s lower
bound `.04952769860976136` m, below .05 m. **It was not applied**; no post-abort path
or attachment was fabricated. All reference geometries had passed preflight, showing
why reference safety alone cannot certify the B-start executable transition.

World figures show a `.25 m` cart buffer as a **robot-center exclusion boundary**
(radius .20 + edge margin .05). Plotted .20 m robot circles are physical footprints;
a circle overlapping the center exclusion band does not itself violate the .05 m
edge margin. Compare the center with the .25 m boundary, or the footprint with a
.05 m obstacle expansion, without counting the radius twice. Numeric direct checks
above determine validity.

### Calls, costs, validation and artifacts

| Method | New MPC solves | Rollout wall s | Summed numerical solve wall s |
|---|---:|---:|---:|
| M0_NATIVE | 30 | 3.115065 | 0.148219 |
| M1_SE2_TAPER | 30 | 3.110446 | 0.148255 |
| M2_RIGID_TRANSPORT | 16 | 1.690534 | 0.102345 |
| M3_LOCAL_SE2 | 30 | 3.109741 | 0.147005 |

Total **106** new official MPC solves, **4** scientific rollouts. LightNav=0, RGB=0,
graph optimization=0, GP=0, new source/Isaac episodes=0. Tests/preflight/validators
made zero real numerical MPC/VLA/optimizer calls. The existing M3 optimization cost
was not paid again and is not a new construction solve.

Relevant regression command from the pre-execution work-log entry: 248 tests PASS;
final compileall/diff check PASS. Saved-only validator and plot validator PASS.
All raw OSA03/M3/historical-Native artifacts and unrelated configs retain hashes.

- Authoritative local run: `data/osa03_common_b_method_comparison_01/primary_20260925T162125Z/`.
- `methods/<method>/`: raw worker/event records, exact applied states/commands,
  restoration, independently recomputed metrics and hashes.
- `summary.json`, `validation.json`, `report_validation.json`.
- [Static review](../data/osa03_common_b_method_comparison_01/primary_20260925T162125Z/index.html).
- [World figure](../data/osa03_common_b_method_comparison_01/primary_20260925T162125Z/review/world_execution.png).
- [Review ZIP](../data/osa03_common_b_method_comparison_01/primary_20260925T162125Z/review_bundle.zip).
- Tracked [numeric summary](../results/osa03_common_b_method_comparison_01/result_summary.json),
  [primary table](../results/osa03_common_b_method_comparison_01/primary.csv), and
  [signed contrasts](../results/osa03_common_b_method_comparison_01/signed_gaps.csv).

### Repository-confirmed facts

M0/M1/M3 safely sustained attachment late in the original FRESH; M2 was prevented
from violating margin. M3 did not attach earlier than Native. M1/M3 position AUCs
were slightly smaller, while command variation and yaw metrics show trade-offs.
M0/M1/M2 primary command schedules matched; M3's first new application did not.

### Research interpretation

This run provides execution characterization of four fixed references in one
source, but does not establish Local-SE2 superiority. M1 versus M0 has matching
primary application timing and a small AUC change with later attachment; M3's
small differences are timing-confounded. Rigid's endpoint transport brought the
reference/execution closer to cart and required abort in this tested condition.
The largest uncertainty is whether the small M3 differences would remain under
identical actual application timing; this task performs no corrective retry.

### Not demonstrated

Reference-only causal benefit for M3, general method superiority/necessity,
complete obstacle bypass, online closed-loop navigation benefit, real-robot
feasibility, ablation or replication. R01/Genuine13 remain unevaluated. No further
method, source or optimization experiment is implemented here.
