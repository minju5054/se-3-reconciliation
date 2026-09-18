# GP-SE2-01: environment-constrained OLD–FRESH handoff pilot

This is an offline counterfactual formulation pilot over preserved genuine online
LightNav handoffs. The question is whether switching from the actual pose and
motion state to FRESH can improve transition quality while preserving obstacle
clearance, the required local passage, and the original downstream goal. A small
FRESH deformation, zero boundary gap, or smoother stopping is insufficient.
Implementation completion and positive research evidence are separate outcomes.

## Implementation freeze and preserved source

Starting main: `ada33e145a412899ef5d33b05e9757fe4a780db7`, equal to fetched
`origin/main`. The source collector SHA is
`eed5f2c68f9bfc2b62d70a00a6d784647c32a971`. Primary input is
`data/robotless_online_handoffs_v1/primary_20260915T091900Z`; shape diagnostics are
`data/robotless_online_handoff_shape_analysis/primary_20260915T091900Z_v1`.
The source validator again passed all 881 events in 60 episodes and all 1,762
valid-event PNGs, with no errors. Rehashing the cleanup preservation inventory
matched all 36,684 retained data files. Source timing/history limitations remain.
Evidence is in `data/robotless_gp_se2_01/source_audit_20260918`.

The two unrelated Stage 0 configuration edits remain unmodified and uncommitted.
No new LightNav inference, observations, dataset, robot articulation, gain tuning,
external source modification, checkpoint modification, or model timing claim is
part of this pilot. Every derived run writes a new directory and refuses overwrite.
Code/config snapshots and hashes, selected raw inputs, frozen B/goal/gates,
candidates, per-start numerical diagnostics and all method outcomes are retained.

## Oracle environment and validation

The same configured Hospital 6.0 USD asset is loaded with native stage units,
full nested transforms, instance proxies and resolved references. The current
export contains 2,058 meshes, including 2,035 instance proxies. The evaluation
volume is a circular footprint of radius 0.20 m and ground-relative height band
[0.05, 0.65] m; these are experimental assumptions, not physical robot dimensions.
Clearance is centre-to-obstacle distance minus the radius. Required edge clearance
is 0.05 m; radius and margin are not applied twice.

Obstacle geometry uses height-clipped triangle projections and even-odd filled
lower-slab cross sections for solid interiors. Sibling material sections are
joined per actual asset; disconnected unresolved shells remain unknown. Floors
and overhead-only meshes are not indiscriminately projected as obstacles. Thin
surfaces remain; whole-wall AABBs do not replace door openings. Known evaluated
floor area is 3,245.396 m²; unresolved interior area of 5.471 m² is excluded.

The optimizer queries a 0.05 m distance grid with the conservative bilinear
Lipschitz error bound 0.03535534 m subtracted. The independent checker instead
uses direct projected geometry and swept circular footprints along each supplied
polyline. Its numerical uncertainty is 1e-7 m. Unknown space and borderline
clearance never count as a pass. GP evaluation samples at 0.005 s and refines to
0.00125 s when clearance is at most 0.10 m. Actual held-command rollout queries
include every 60 Hz boundary plus at most 0.005 s samples; the unicycle arc-to-chord
bound `max|v omega| dt²/8` is added to required clearance. This is a sampled
continuous-trajectory assessment, not a continuous-time safety proof.

Validated export:
`data/robotless_gp_se2_01/environment_technical_20260918_retry04`.
All 13 environment checks pass: free floor, wall/furniture overlap, real doorway,
unknown space, ground/overhead exclusion, transforms, independent distance/grid
checks, every gate and actual Isaac screenshots. Twenty-eight independent world
transform checks matched; 200 full-union distance checks differed by at most
3.10e-9 m. The largest observed error over 2,000 grid queries was 0.023728 m,
below the bound. All 293 non-anonymous USD layer hashes matched on actual reload.
Historical collection did not hash every layer: scene URL/configuration equality
is established, but historical byte identity of every remote asset is not proven.

Technical export attempts are preserved. Pre-primary fixes addressed JSON
serialization, an analytic horizontal GroundPlane, solid interiors missing from
surface-only projection, and overly connected unknown-shell sections that closed
a real doorway. They are implementation validation, not formulation outcomes.

## Cases, local goals and route proxy

Selection scans all 881 saved events without new optimizer or MPC outcomes.
Exact timing eligibility requires client-request-to-receipt matched-state RTF in
[0.8, 1.2], maximum enclosed loop stall at most 0.25 s, capture and actual
execution overlap, and recorded post-switch motion. Source integrity, valid B,
valid original FRESH endpoint, clearance-valid original future suffix and an
interpretable route relation are also required. Already colliding FRESH suffixes
remain in the diagnostic exclusion inventory.

Groups target three events each: A small mismatch/straight, B small mismatch/turn,
C large position or reliable direction mismatch, D shortcut/route sensitive.
The precise descriptive thresholds are versioned in `configs/gp_se2_01.yaml`.
Selection handles scarce groups in D, C, B, A order, then prefers an unused
episode, unused ordered raw pair, named priority, and lexical event order.
No event belongs to two selected groups. Missing coverage is reported rather
than filling the quota by relaxing conditions or replacing cases after outcomes.

The fixed goal is the last original FRESH pose, with 0.15 m and 15 degree
tolerances. The final 0.20 s must remain within both. Eight actual doorway
intervals and one independently verified broad corner-to-stairs passage form a
scene gate catalog. The latter is explicitly an open-corner passage proxy, not a
doorway: its safe-centre interval is 8.693 m wide, with 1 mm endpoint precision
accounted for, and full-interval swept clearance exceeds 0.05136 m. It is derived
from the building corner and actual stairs for episode_017_repeat_00/handoff_007;
the raw crossing clearance is 0.11552 m while the direct B-to-goal shortcut has
only 0.003889 m edge clearance. This is a clearance-sensitive case, not a claim
that the shortcut physically overlaps the footprint.

Remaining raw-suffix strict crossings establish ordered direction and fixed
crossing times. The solver constrains the gate plane at that time, the finite
safe-centre segment, and opposite signed sides at ±0.05 s (minimum progress
0.0001 m). Actual rollout crossings are checked independently. Reverse crossings,
grazing along the gate plane, and finite-segment endpoints do not pass. A reliable
clear shortcut with no remaining catalogued crossing needs no gate; a blocked
shortcut without a reliable gate is UNKNOWN and excluded. These are local
route-preservation proxies, not complete natural-language semantic ground truth.

## Reference preparation and methods

`F_native` is every original FRESH world row, unchanged. At B the official
weighted squared-pose distance `[10,10,1]`, wrapped yaw and next-row rule choose
a future suffix. The first row is assigned 0.1 s, the last 3.0 s, and intermediate
rows uniform row-order times. XY interpolation is linear, yaw shortest-angle;
rotation-only rows remain. A single row gives a constant future reference.
`F_common` contains exactly 30 samples at 0.1,...,3.0 s. No B connector is
inserted into either original or prepared reference. This is an evaluation
clock; intrinsic LightNav waypoint_dt remains null.

| Method | Reference and optimization |
|---|---|
| M0_NATIVE | Original full FRESH + official MPC |
| M0_ADAPTER | Common suffix and resampling only + official MPC |
| M1_RIGID | One left SE(2) transform of all common rows |
| M2_GP_NO_OBSTACLE | GP boundary/motion/workspace/goal/gates, no obstacle inequality |
| M3_GP_CONSTRAINED | Identical GP formulation plus obstacle inequality |

Rigid minimizes uniform FRESH preservation plus the finite-lookahead entry
residual against `B Exp(0.1 nu_minus)`. Goal, route, known workspace and obstacle
constraints apply without imposing an impossible hard initial boundary on a
three-parameter transform. Its undefined initial reference connector is assessed
through the actual MPC rollout.

## GP prior, interpolation and constraints

The prior follows the locally linear Lie-group construction of
[Dong et al. (2018)](https://dongjing3309.github.io/files/Dong18icra.pdf), with
[continuous-time GP planning background](https://www.roboticsproceedings.org/rss12/p01.pdf).
World agent poses use right-local perturbations `X <- X Exp(delta)` and body
velocity `nu = vee(X^-1 dX/dt)`. For `xi=Log(X_i^-1 X_j)` and interval h,

```
r = [xi - h nu_i; J_r(xi)^-1 nu_j - nu_i]
Q = [[h^3/3 Qc, h^2/2 Qc], [h^2/2 Qc, h Qc]]
J_GP = 0.5 sum ||Cholesky(Q)^-1 r||²
J = J_GP + mean ||Log(F_common^-1 X)||²_SigmaF
```

`Qc=diag(1,1,1)`, preservation standard deviations are `[0.1 m,0.1 m,10 deg]`,
and lambda is 1 uniformly over 30 future samples. These are design scales, not
learned uncertainty. The integrated-Wiener conditional local mean is evaluated
as cubic Hermite interpolation in local coordinates and their derivatives.
Body velocity is `J_r(xi) xi_dot`; acceleration includes the directional derivative
of J_r. The paper's printed Eq. 33 inverse is inconsistent with its Eq. 22;
the implemented forward-Jacobian relation is independently finite-difference
verified. It is not an exact global constant-body-acceleration or jerk guarantee.

Support states at 0,0.1,...,3.0 s fix `X(0)=B` and
`nu(0)=[v_minus,0,omega_minus]`. Support lateral velocity is structurally zero;
midpoint lateral velocity is an equality constraint, and dense intermediate
lateral velocity is independently checked. Limits match the resolved official
object-navigation MPC: v in [0,0.8] m/s, |omega|≤3 rad/s, |dv/dt|≤2 m/s²,
|domega/dt|≤5 rad/s². Support/midpoint checks include both sides of knot
acceleration discontinuities; final checks include denser samples and an
independent pose-derivative/body-velocity identity. Goal, ordered gates and known
workspace apply to both GP methods; only M3 adds obstacle clearance inequalities.

SciPy 1.18.1 SLSQP handles actual equality/inequality constraints. GP factors have
local sparse dependency, but SLSQP is a dense constrained backend, not the paper's
unconstrained sparse solver. Two deterministic initializations are shared by M2
and M3: FRESH support poses with finite-log initial velocities and a constant
boundary body-twist extrapolation. Both receive 200 iterations and 30 s each,
ftol 1e-7, equality/inequality acceptance tolerances 1e-5, seed 0 (no stochastic
search), single BLAS thread. Feasible initial/latest/intermediate iterates compete
by their own objective. A timeout may retain a checked feasible iterate but is
never called convergence. No feasible candidate means no MPC rollout for that
method; no RAW fallback or mathematical infeasibility claim follows.

Math tests cover Exp/Log, right retraction, J_r finite differences, covariance
SPD, zero-residual constant twists, endpoint/derivative identities, yaw wrapping,
chart boundaries, factor Jacobians and sparsity. A pre-primary SE(2) Exp/Log
small-angle series threshold correction from 1e-8 to 1e-4 avoids cancellation
near 4e-7 rad; independent matrix-exponential checks cover both sides. Synthetic
straight/curved solver qualification is retained separately and never treated as
dataset performance evidence.

## Common-state MPC execution and primary evaluation

The pinned external official module is unchanged at commit
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`, source SHA-256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Physical u_minus and recorded pre-first-FRESH-solve controller previous_control
are restored separately. Historical replay uses the historical solve input pose;
it does not incorrectly require a new solve at B to reproduce an earlier command.
Initial audits on all three named priority candidates reproduced commands exactly.

Every accepted candidate uses an independent official MPC instance and identical
B/u_minus/controller-memory policy. A fresh solve runs at t=0, then 0.1,...,2.9 s;
exact held-command unicycle integration runs at 60 Hz for 3.0 s. Solver wall time
is measured separately and does not advance simulation time. Candidate world rows
are inverse-transformed by the original observation pose and roundtrip-verified;
this is coordinate representation, not re-anchoring at B. GP velocities are not
fed forward. There is no teleport, snap, new observation, new VLA output, or
collision-triggered artificial stopping. Every controller-selected reference
stream and independently integrated state is saved.

Primary success requires actual rollout clearance, known workspace, motion
limits, route crossings, common goal and terminal dwell. GP plan validity is a
separate field. Motion acceleration for commands is a 10 Hz finite difference
including u_minus, not a physical actuator bound. Source and candidate hashes,
all held-command transitions and selection indices are independently checked.

Native-success→M3-failure and adapter-success→M3-failure regressions are reported
before secondary paired-success metrics. Time to goal, error, progress, path,
command changes/variation and acceleration retain separate linear/angular units;
clearance, deformation, optimizer and MPC compute time are also reported. No
cross-method objective comparison establishes superiority.

## Execution commands and result recording

Run commands from the repository root, using the exact new paths recorded in the
final result section. All numerical commands use `OPENBLAS_NUM_THREADS=1` and
`OMP_NUM_THREADS=1`; plots use `MPLCONFIGDIR=/tmp/gp_se2_01_mpl`.

```bash
.venv/bin/python scripts/run_gp_se2_01.py prepare --run "$GP_RUN" \
  --environment data/robotless_gp_se2_01/environment_technical_20260918_retry04
.venv/bin/python scripts/run_gp_se2_01.py optimize --run "$GP_RUN"
/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python \
  scripts/lightnav/gp_se2_mpc_rollout.py --request "$GP_RUN/mpc_request.json" --output "$GP_MPC"
.venv/bin/python scripts/run_gp_se2_01.py evaluate --run "$GP_RUN" --mpc-output "$GP_MPC"
.venv/bin/python scripts/plot_gp_se2_01.py "$GP_RUN"
./scripts/launch_gp_se2_comparison_replay.sh --run "$GP_RUN" --output "$GP_GUI" --verify --no-hold
.venv/bin/python scripts/validate_gp_se2_01.py "$GP_RUN" \
  --gui-validation "$GP_GUI/runtime_validation.json" --write
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
bash -n scripts/isaac/run_gp_se2_environment_export.sh scripts/launch_gp_se2_comparison_replay.sh
git diff --check
```

Every case/method receives eight required 160 dpi plots, GP methods three more,
plus a common actual-rollout overlay and linked index. Failed attempts are
explicitly labelled and never given fabricated zero traces. Isaac representative
selection is frozen as first native-success→M3-failure if present, first D, and
first A, deduplicated; absent categories are recorded. The GUI label is
**OFFLINE COUNTERFACTUAL HANDOFF COMPARISON**. It replays saved actual comparison
rollouts in the original Hospital; it does not imply live online optimization.

At implementation freeze the primary outcome is pending. The final report will
record actual case coverage, every method outcome, regressions, paired metrics,
compute times, image paths, runtime checks and separate operational/research
statuses without rewriting this pre-outcome protocol.
