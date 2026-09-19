# GP-SE2-02: hard-handoff transfer and fixed-MPC counterfactuals

This diagnostic transfer pilot asks whether the verified supplied-Jacobian GP
finds valid candidates on the three prescribed difficult handoffs, and whether
those references improve execution with the unchanged official pose-reference
MPC. It includes one benign control. Candidate convergence/availability, full
plan validity, original rollout success and objective improvement are separate.
These are dependent events from the development corpus, not a held-out benchmark
or an estimate of hard-case prevalence.

## Frozen scope

Starting main is `1462e81d9c8ebba04466a07d10898284d0fccf2e`, equal to fetched
origin/main. Both unrelated Stage 0 configuration edits remain untouched.
The original GP-SE2-01 run, both diagnostic runs, validated environment export,
existing images, source data and official MPC are retained and hashed.

| Role | Original event | Position mismatch m | Original direction diagnostic |
|---|---|---:|---|
| BENIGN_CONTROL | episode_001_repeat_01/handoff_002 | 0.00001205 | 0.00390 degrees |
| HARD_POSITION_AND_DIRECTION | episode_013_repeat_01/handoff_024 | 0.279113 | 36.5413 degrees |
| HARD_LARGE_TURN | episode_014_repeat_01/handoff_026 | 0.162573 | reliable window direction unavailable; raw yaw travel 130.191 degrees |
| HARD_POSITION_STRAIGHT | episode_000_repeat_01/handoff_019 | 0.323368 | 0.002115 degrees |

All four original routes contain no required gates. M2 keeps workspace, motion
and goal constraints; M3 additionally keeps the original obstacle inequalities.
Gate-required cases are outside this fixed scope; the provider is not extended.

The GP-SE2-01 configuration is copied byte for byte. T=3 s, support_dt=0.1 s,
31 supports, 150 variables, fixed original B and physical body twist, objective
weights, GP prior/interpolation, goal/route, tolerances, footprint 0.20 m and
clearance 0.05 m are unchanged. LightNav waypoint_dt remains null; the temporal
grid is an explicit downstream convention. There is no new terminal-stop
constraint. Plan terminal-pose acceptance differs from execution's final 0.20 s
goal dwell.

M0_NATIVE uses original F_native; M0_ADAPTER uses original F_common. M1 calls
the original solve_rigid with both original initializations. Each M2/M3 problem
uses unchanged FRESH plus the fixed same-curvature deceleration seed from its
own physical initial twist. No goal fitting, restoration or fallback is added.
SEED_ONLY independently checks this same deceleration vector without optimizing.

The planned order is the table's case order, then rigid identity/alignment,
M2 I0/I1 and M3 I0/I1: 16 GP and eight rigid starts, sequentially once. SLSQP,
CPU float64 JAX/analytic geometry, 200 iterations, 30 s prepared solve per GP
start and one BLAS thread remain fixed. Fresh providers compile per start;
construction/warmup and all validation costs are separate. Both initialization
costs contribute to each GP method's total.

## Derivative applicability and selection

The authoritative DIAG-02 root artifact validator and
`derivative_checks/authoritative_validation.json` are verified with their hashes,
distinct from preserved earlier diagnostic attempts. The provider, original
primal evaluator and full checker remain unchanged.

Before primary optimization, 32 point records are frozen: each case/method/seed
and one deterministic small perturbation. Original primal agreement and
multi-step, fixed-direction derivative tests reuse the original DIAG-02
tolerances/family rules. This is a compatibility check, not case selection.
Unsupported starts remain in the ledger; no guard is removed and no FD fallback
is supplied. Genuine mismatches block the affected start and remain evidence.

Each start retains the original minimum-objective dense-feasible candidate
among its initial/latest/actual callbacks. Across I0/I1 the same dense-first
minimum-cost rule applies, with initialization index breaking ties. The selected
candidate then needs the unchanged independent full and offset-grid acceptance.
A full-invalid selection is not replaced by an iterate with better MPC behavior.
M2 receives the same independent obstacle/known-workspace check.

## Counterfactual execution

Only full-valid GP and SEED_ONLY plans are submitted. Native/adapter/rigid use
the original GP-SE2-01 available-reference policy; any plan-invalid diagnostic
rollout is explicitly labelled and never called a validated plan.

Every candidate starts a new official MPC instance at original B, with physical
u_minus and recorded previous_control preserved separately. For
episode_000_repeat_01/handoff_019 these are respectively
[0.8,0.0001009450943702976] and [0.6192657763912551,0.00008006755974495329].
The historical solve is audited at its original input state, not forced to
match a new solve at B. World references use the inverse original capture
transform, with roundtrip validation, never a B reanchor. Planned GP velocity
is not sent as feed-forward to the pose-reference MPC.

The original schedule solves at t=0,0.1,...,2.9 s and integrates exact held
unicycle commands at 60 Hz through 3.0 s. Wall computation time does not advance
simulation. Failure/hold and collision-record-and-continue policies are unchanged.
No inference, new RGB/history, pose snap or extra stopping rule is introduced.
These offline counterfactuals do not test movement during optimizer latency or
online deployment readiness.

## Transfer compatibility before primary

All 32 frozen points pass, authorizing all 16 planned GP starts. Maximum primal
absolute difference is 4.88854e-12; maximum required scaled directional error is
0.157844 against the unchanged threshold 1. The smoke checks 170,686 required
smooth directional entries. Two obstacle entries at h=2e-5 for the straight
hard case's M3 deceleration seed cross a bilinear cell boundary and are separately
classified with branch metadata instead of treated as classical central
derivatives. No point is unsupported and no mismatch threshold changes.

Initial independent full checks show I0 infeasible for every case. I1 is
full-feasible only for the benign control; both GP methods' I1 seeds fail full
acceptance on all three hard cases. The seeds and goal are retained unchanged.
The transfer smoke takes 11.5103 s, separate from subsequent primary solve costs.

Implementation validation, frozen-run results, renderer evidence and final
reproduction commands are recorded below after their actual completion.
