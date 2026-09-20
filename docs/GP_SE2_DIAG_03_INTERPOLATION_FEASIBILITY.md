# GP-SE2-DIAG-03: saved GP interpolation feasibility audit

Question: where do the fixed hard-case GP curves violate the original motion
conditions, and is that discrepancy interpolation arithmetic or finite enforcement?
This audit does not optimize, change acceptance, or execute a rejected curve.

Starting revision: `da5b93151aec62ce88d4fa6bda58ca8ae5b7dd86`.
Source: `data/robotless_gp_se2_02/primary_20260919T024000Z/`, numerical execution
revision `bd273b22ed9ddcb35a3f2eadcf550895adee46b9`. Its final `validation.json`
is the authoritative historical artifact validation, rather than intermediate
reporting-only validator failures.

## Frozen scope and numerical protocol

The ledger contains initial and latest vectors for M2/M3 × I0_FRESH/I1_DECEL of
`episode_013_repeat_01/handoff_024` (eight records), plus the saved benign M3/I1
latest vector from `episode_001_repeat_01/handoff_002`. The four hard finals are
the analysis targets; initials and the benign reference are context, not new
independent performance samples. Null candidate fields do not discard saved
latest iterates. Missing vectors remain explicit and are never regenerated.

Original 150-variable right-local chart, 31 support states, 3 s horizon, 0.1 s
support spacing, B/physical initial twist, F_common, original goal, route and
validated environment are loaded through the original frozen-case loader.
Original core, inputs, results and images are hashed, not overwritten.
Unrelated Stage-0 configuration edits are separately preserved.

Original objective/collocation/dense reports are compared to saved records at
absolute 1e-8 / relative 1e-10 tolerance, with literal equality also reported.
Support reconstruction uses absolute 1e-10 and periodic yaw comparisons.
Physical tolerances remain the original 1e-5: equality in m/s, motion inequalities
in their respective units, and squared goal-distance margin in m². The separate
independent plan checker uses its original motion_numeric_tolerance, coincidentally
also 1e-5, and actual endpoint distance rather than a squared-distance residual.

All 30 intervals are queried using original collocation, dense and full-offset
grids and supplemental spacings 0.010/0.005/0.001 s. The last grid includes the
original 0.371 offset. Knot-side acceleration is retained separately. Sampled
extrema are not continuous extrema. Only observed safe/violating transition
brackets are refined to <=0.0001 s; safe endpoints alone prove nothing inside.

Independent cubic coefficients are compared against production Hermite values.
The algebraic expression shares frozen Exp/Log/right-Jacobian primitives; a
separate temporal finite difference of pose provides body velocity, and finite
differences of independently reconstructed velocity provide body acceleration.
Fixed interval fractions are .125/.25/.5/.75/.875, augmented by observed witnesses.
Epsilons are 1e-4, 3e-5, 1e-5 s, guarded from crossing knots or local Log cuts.
Frozen diagnostic absolute tolerances are coefficient pose/twist 1e-9,
coefficient acceleration 1e-8, FD twist 2e-6 and FD acceleration 2e-5 in component
units. Both finest valid epsilons must pass; all error trends remain recorded.
These are diagnostic agreement thresholds, never physical acceptance changes.

G0 uses original 0/.5/1 motion points and midpoint lateral equality. G1 also
queries quarter points. G2 adds post-hoc observed witnesses. They only detect
violations of the same saved vector; no new constraints are solved. Extra equality
counts do not imply extra independent rank or a feasible refined optimization.

New VLA inference, GP/rigid optimization, MPC solve, rollout and Isaac runtime
counts are all zero. The runner forbids optimizer and rollout entry points.
Numerical results, validation and evidence commands are appended after the frozen
audit; no positive scientific outcome is assumed by tests.
