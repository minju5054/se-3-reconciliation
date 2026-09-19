# GP-SE2-REF-04: saved prediction–execution stress audit

This audit asks where required clearance and the original route first fail for
`episode_017_repeat_00/handoff_007`. It compares the saved REF-03
`B_DENSE_ROW_STEP` and `C_DENSE_SOURCE_PROGRESS` records; `A_NATIVE` is historical
context. It does not change any outcome, trajectory, reference selector, MPC
calculation, physical acceptance or gate.

Source: `data/robotless_gp_se2_ref_03/primary_20260919T141000Z/`.
Starting revision: `fc51724ececc29ad00e1d62b832ce097821d16b7`.
The new runner resolves the case through its manifest and checks the authoritative
validation, transitive original files, source code, external MPC, configuration,
environment and copied inputs before auditing. Unrelated user configuration
changes are separately hashed and preserved.

## Frozen audit protocol

The audit uses three distinct layers:

1. The actual five selected world targets, controller-local targets, source-row
   identities and sequential yaw. Point checks, target-to-target connectors and
   current-pose-to-first-target connectors remain separate. These connectors are
   spatial diagnostics; their row parameter is not physical time.
2. The six saved MPC prediction poses: current state followed by five forward
   Euler states at 0.1 s spacing. The first interval is [issue, issue + 0.1 s];
   the remaining 0.4 s is an unexecuted prediction tail. Node clearance and direct
   swept clearance of the saved discrete prediction polyline are both reported.
3. The saved applied commands and execution states. The unchanged exact held
   unicycle integrator reconstructs each stored interval. It does not compute
   a new command or closed-loop rollout.

The original `evaluate_rollout` supplies full acceptance. Its dense sample grid,
60 Hz boundaries, geometry uncertainty and global curve allowance are retained.
Additional first-violation localization subdivides the earliest failing swept
segment, checking both halves rather than assuming safe endpoints imply a safe
interior. The target bracket width is at most 1 ms; unresolved brackets remain
explicit. Refinement never replaces the original acceptance result.

The original finite directed gate is a safe footprint-center interval. No second
footprint or clearance subtraction is applied. A prediction with no crossing in
its 0.5 s horizon does not thereby fail the full route. Already passed gates and
inherited invalid start states are identified separately.

First-interval comparisons retain four endpoints: saved prediction, Euler using
the applied command, exact held-command reconstruction and the next recorded
control state. Full solved control sequences, pre-clipping first controls and
original local predictions were not saved. Therefore solver numerical residuals
and command clipping cannot be uniquely separated. Beyond the first interval,
prediction/actual differences include replanning; beyond 3 s actual counterparts
are unavailable.

`PROTOCOL` in `gp_se2_ref04_audit.py` freezes numerical comparison tolerances,
missing-data handling and representative plots before the output run. Figures
use t=0, C's earliest prospective warning, first actual violation, minimum
clearance and invalid gate-crossing intervals, with duplicates removed and all
reasons retained. The entire 0–3 s execution and all 60 prediction records remain
in the audit and heatmaps.

No new VLA inference, GP/rigid optimization, MPC solve or closed-loop rollout is
part of this implementation. No GUI is required. Numerical findings and exact
output commands are recorded below after the saved-record audit.
