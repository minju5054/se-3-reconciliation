# DATA-02 Saved Collection GUI Demo

## Purpose and boundary

This is a short professor-facing explanation of how one already-recorded DATA-02 OLD/FRESH
transition was collected. It is visualization only, not a new experiment and not an optimization
or reconciliation result. It reads saved DATA-02 artifacts without modifying them. It does not
start or invoke LightNav, issue controller or wheel commands, advance physics to recreate the
motion, change any scientific timestamp, or regenerate scientific state. The official Jackal is
moved directly through saved world poses for presentation.

The one-command default is:

```bash
./scripts/isaac/run_data02_collection_demo.sh
```

It selects
`data/data02_online_successive_v2/data02-online-successive-extension-v2/`,
`episode_000061`, transition `4`, displays the exact saved RGB observation, runs for about 12
seconds, captures a non-black viewport image under the separate ignored
`data/data02_collection_demo/<UTC>/` root, and exits automatically. Options are:

```bash
./scripts/isaac/run_data02_collection_demo.sh \
  --episode episode_000061 --transition 4 --duration 12 \
  --no-hold --show-rgb
```

`--duration` is constrained to 10–15 seconds. Use `--hold` to keep the final state open or
`--no-show-rgb` if an RGB inset is undesirable. The default is `--no-hold --show-rgb`.

## Five presentation phases

The 12-second default maps the saved event sequence to a deliberately slower presentation clock:

| Presentation time | Phase | Meaning |
|---:|---|---|
| 0–2 s | OLD EXECUTING | the previously returned OLD chunk is already active |
| 2–5 s | FRESH INFERENCE / OLD CONTINUES EXECUTING | the saved FRESH request is in flight while saved Jackal poses continue along OLD |
| 5–7 s | FRESH READY | the raw FRESH path and the saved P/B switch context appear |
| 7–10 s | OLD → FRESH RAW SWITCH | the saved active chunk changes at B without reconciliation |
| 10–12 s | FRESH ACTIVE | saved post-switch Jackal poses are replayed |

The GUI and terminal explicitly say `PRESENTATION-TIME REPLAY` and
`NOT REAL-TIME SCIENTIFIC TIMING`. The numeric values shown for `t_obs`, LightNav-reported model
latency, effective latency, and `t_switch` are the immutable values in `transition.json`; only the
rate at which saved samples are displayed is changed. For the default transition those values are
`21.533334456384182 s`, `0.43086534799658693 s`, `0.5000000260770321 s`, and
`22.033334482461214 s`, respectively.

## Visual legend and semantics

- Blue: saved active OLD world trajectory.
- Magenta: saved raw FRESH world trajectory, transformed at its observation pose. It is not
  translated to B and contains no reconciliation.
- Green: accumulated saved actual Jackal pose history.
- Yellow: exact FRESH observation pose.
- Orange: P, the last saved control pose immediately before the switch.
- Red: B, the saved actual switch boundary.

Small arrows show the headings of FRESH, P, and B. The top-left panel gives the phase in large
text, the top-right panel displays the exact saved RGB frame used by the FRESH request (frame 81,
`rgb/frame_000081.png` for the default), and the bottom panel advances a presentation cursor over
observation, readiness, switch, and FRESH-active events. During the important inference phase the
progress bar advances while the Jackal continues through saved poses with `chunk_04` still active.
The default saved motion from observation to model readiness is `0.15539962298348772 m`.

## Replay correctness

Before opening the GUI, the saved-evidence loader verifies the transition selection and hashes of
OLD, raw FRESH, transition actual poses, transition telemetry, and the exact observation RGB. It
requires strictly ordered saved times, P strictly before B, OLD to remain active in every saved
`FRESH_IN_FLIGHT` row, actual motion during that interval, and the saved FRESH chunk to be active
after B. Returned NumPy arrays are copies marked read-only. Post-switch presentation stops before
the next saved FRESH request so the five-phase story cannot blend two transitions.

The raw OLD/FRESH waypoint rows remain spatial SE(2) references with no intrinsic timestamps.
Only recorded robot telemetry and transition events have time. Consequently, this demo must not
be interpreted as time-aligning LightNav waypoint row indices, measuring a new tracking result, or
reproducing real-time inference/physics behavior.
