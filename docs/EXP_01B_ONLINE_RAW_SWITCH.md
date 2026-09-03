# EXP-01B — Online Successive OLD/NEW Raw-Switch Discontinuity

## Question

When one preloaded and warmed LightNav process generates successive OLD and NEW
trajectory chunks, does a Jackal that continues executing OLD during NEW inference reach a
pose that is discontinuous from the raw NEW reference when NEW becomes ready?

This experiment measures whether the reconciliation problem exists. It does not implement or
evaluate reconciliation, correspondence, smoothing, interpolation, graph optimization, or
navigation-quality improvement.

## Why this experiment

Stage 0-C established the real LightNav-to-Jackal single-chunk interface, and Stage 0-B
established the closed-loop execution layer. EXP-01A then measured a warm LightNav median of
about 557 ms, shorter than one observed 2.3 s Stage 0-C chunk execution. EXP-01B therefore
tests the overlap directly instead of inferring a switch boundary from an assumed waypoint
period.

## Setup

The validated immutable run is `data/exp01b/exp01b-20260903T155402Z/` and is intentionally
ignored by Git.

- GPU: NVIDIA GeForce RTX 5060 Ti, 16,311 MiB
- Isaac Sim: 6.0.1, headless timing measurement with RGB rendering enabled
- robot: official `Clearpath/Jackal/jackal.usd`; its four runtime-discovered wheel joints
  are driven through the Stage 0-B controller path
- LightNav checkout: `a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0
- checkpoint: `LightOriginsHQ/LightNav-0`, revision
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`
- backend: `vllm_local`, vLLM 0.19.1, eager execution, 1 GiB explicit KV cache, prefix
  caching enabled
- instruction: `Go straight down the corridor.`
- history: 64 `256 x 448 x 3` uint8 RGB frames, initially primed while stationary; live
  frames appended at 4 Hz
- NEW trigger: 0.50 s of simulated time after OLD execution starts
- physics/control periods: 1/60 s and 0.10 s
- timing-validity range: RTF 0.90–1.10

The prior LightNav memory-utilization configuration was 0.90 with a 2 GiB explicit KV cache.
For coexistence it was explicitly changed to 0.65 and 1 GiB, respectively. Installed vLLM
reported that explicit KV bytes supersede the utilization setting. LightNav alone occupied
11,298 MiB after warm-up. LightNav plus loaded Isaac and camera occupied 13,270 MiB, leaving
2,559 MiB; the final snapshot was 13,346 MiB used. No OOM occurred. Model construction was
called exactly once, one saved Stage 0-C history supplied the explicit warm-up, and the live
process handled four episode resets and eight OLD/NEW predictions.

One first live attempt had RTF 0.654 and was retained as invalid rather than discarded. The
protocol continued until it obtained the required three valid transitions, within a configured
maximum of eight attempts. Absolute simulation-time deadline pacing targets 0.95 RTF so that
render stalls can be recovered and the one-physics-step ready-event quantization remains within
the stated validity range.

## Online architecture

Isaac and LightNav remain separate processes and environments. A versioned standard-library
Unix-domain-socket protocol carries explicit JSON headers and binary RGB/action payloads.
Each payload records shape and dtype; incoming frames must be contiguous `H x W x 3` uint8
RGB and returned actions must be finite `(N, 3)` arrays.

The LightNav server builds and warms one `TrackingAgent`, accepts a single episode reset,
observes the initial history, predicts OLD, continues accepting live observations, and predicts
NEW from the current buffer. It does not reset between OLD and NEW. While the server blocks in
NEW prediction, the Isaac client runs that request in a worker thread, continues physics and
OLD feedback control on the main thread, and queues later RGB frames for delivery after the
response. No ROS transport or shared Python environment is used.

## LightNav persistent-history semantics

Local source inspection confirms that `NavigationPolicy.observe()` appends monotonically
indexed frames and that the SlowFast configuration keeps the full episode buffer.
`NavigationPolicy.reset()` clears the frame ids, frame tensors, video cache, and engine episode
state. EXP-01B invokes it once at each live episode start, not between OLD and NEW. Initial
64-frame priming is excluded from overlap timing.

## Timing definitions

- **OLD observation:** pose and simulation time of the last initial-history frame.
- **OLD ready:** synchronous OLD response receipt before robot motion begins.
- **OLD execution start:** first activation of the Stage 0-B follower.
- **NEW observation:** the 4 Hz RGB sample at 0.50 s of OLD execution, recorded atomically
  with Jackal pose, simulation/host time, and follower progress.
- **NEW request:** asynchronous IPC prediction request after that frame is added.
- **NEW ready:** first completed physics/control opportunity after the client receives the
  response.
- **raw switch:** metrics evaluated before any controller-only reference construction.

The primary host interval is client request to response. The simulation interval is NEW
observation to NEW ready, and RTF is simulation interval divided by host interval. The LightNav
server's host prediction boundary and its narrower reported latency are retained separately.

## Coordinate frames

LightNav returns cumulative observation-frame local poses
`[forward_m, lateral-left_m, yaw-CCW_rad]`. These already match the canonical research axes
`[x, y, yaw]`. NEW is transformed only with the Jackal pose at NEW observation:

```text
T_world_new_i = T_world_robot_at_NEW_observation * T_robot_new_i
```

The ready pose is never used as this transform anchor. Raw action arrays, derived world paths,
and controller execution references are separate artifacts with raw SHA-256 checks.

## Raw switch policy

LightNav's public action rows have no intrinsic waypoint timestamps. The primary baseline
therefore uses NEW row 0 exactly as the first returned future pose and performs no stale-row
deletion. It compares that observation-anchored world pose with the measured Jackal pose at
NEW ready. For stable follower initialization only, the ready pose is prepended to a separate
controller reference after raw metrics have been computed; raw NEW and derived NEW are not
modified.

## Metrics

Translation/yaw gaps are pose mismatch at ready. Motion mismatch separately compares the last
actual control-interval translation/yaw increment with the first NEW segment increment.
Additional fields record observation-to-ready movement, OLD progress, OLD exhaustion, host and
simulation latency, RTF, and whether the descriptive 0.05 m/rad thresholds are exceeded.

## Actual results

The table contains the three timing-valid transitions. All had OLD progress `1 -> 2`, 26
in-flight timeline samples, and a nonzero OLD command in every in-flight sample.

| Trial | NEW obs / ready sim [s] | request-response [ms] | LightNav predict / reported [ms] | RTF | moved during inference [m] | yaw during inference [rad] | translation gap [m] | yaw gap [rad] | translation motion jump [m] | yaw-motion jump [rad] |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 17.5333 / 17.9667 | 421.654 | 421.419 / 354.301 | 1.0277 | 0.158705 | -0.000035 | 0.003854 | 0.004164 | 0.100789 | 0.006785 |
| 2 | 17.5333 / 17.9667 | 423.228 | 422.951 / 355.401 | 1.0239 | 0.161989 | -0.000043 | 0.161989 | 0.000043 | 0.039447 | 0.000001 |
| 3 | 17.5333 / 17.9667 | 423.663 | 423.401 / 356.020 | 1.0228 | 0.162930 | -0.000086 | 0.008079 | 0.004113 | 0.098742 | 0.006770 |

NEW row 0 world poses and the corresponding measured ready poses were:

- trial 1: NEW `[0.317299, 0.000725, -0.003360]`; ready
  `[0.321153, 0.000649, 0.000804]`
- trial 2: NEW `[0.163923, 0.000532, 0.000828]`; ready
  `[0.325912, 0.000654, 0.000785]`
- trial 3: NEW `[0.315918, 0.000716, -0.003404]`; ready
  `[0.323996, 0.000644, 0.000710]`

Valid-trial raw lists were:

- host NEW latency: `[0.421654, 0.423228, 0.423663] s`
- simulation NEW latency: `[0.433333, 0.433333, 0.433333] s`
- RTF: `[1.027700, 1.023876, 1.022826]`
- translation gap: `[0.003854, 0.161989, 0.008079] m`
- yaw gap: `[0.004164, 0.000043, 0.004113] rad`
- translation motion jump: `[0.100789, 0.039447, 0.098742] m`
- yaw-motion jump: `[0.006785, 0.000001, 0.006770] rad`

Translation-gap mean/median/min/max were `0.057974 / 0.008079 / 0.003854 / 0.161989 m`.
Translation-motion-jump mean/median/min/max were
`0.079659 / 0.098742 / 0.039447 / 0.100789 m`. Yaw-gap mean/median/min/max were
`0.002773 / 0.004113 / 0.000043 / 0.004164 rad`; yaw-motion-jump values were
`0.004519 / 0.006770 / 0.000001 / 0.006785 rad`.

One of three valid trials exceeded the 0.05 m translation-gap threshold and two of three
exceeded the 0.05 m translation-motion-jump threshold. No valid trial exceeded either yaw
threshold. Trial 2's actual NEW response was the RVQ stop token, producing an all-zero local
chunk; its world row 0 equaled the observation anchor, so its translation gap equaled the
robot's 0.161989 m inference-window motion. This response is preserved as returned and is not
treated as a navigation-quality judgment.

## Positive / negative / invalid evidence

The result is **positive evidence for a measurable translational raw-switch discontinuity**
under this setup. The robot demonstrably continued OLD during every valid NEW request, and
pose or motion mismatch crossed the descriptive threshold in all three valid transitions
(translation gap in one, translation motion jump in two). The yaw discontinuities were small,
which is negative evidence for a large yaw-boundary problem in this straight-corridor protocol.

The invalid trial is engineering evidence only: it moved 0.0858 m, but RTF was 0.654, so it is
excluded from all aggregate statistics and threshold counts. Earlier development runs that
failed the timing gate were likewise not used as research evidence.

## Limitations

- Three valid samples from one scene, instruction, initial pose, and trigger delay cannot
  characterize a general discontinuity distribution.
- Raw action variability was material even in the controlled setup, including one valid NEW
  stop chunk. This may combine render variation, model sensitivity, and serving nondeterminism;
  EXP-01B does not attribute it.
- Headless RGB rendering shares the GPU with LightNav. The first attempt's transient RTF
  failure shows that timing validity must remain per-trial rather than assumed from startup.
- The 0.50 s trigger is an experiment protocol, not a LightNav waypoint duration.
- Controller progress indices describe the execution layer; they do not give model action
  timestamps.
- This is simulation, not a real-robot result, and it does not assess collision avoidance or
  instruction following.

## What we can claim

Under the measured warmed LightNav + Isaac setup, NEW became ready after Jackal had continued
executing OLD by about 0.159–0.163 m, and naive switching exhibited measured translational pose
or motion discontinuity. Raw/derived separation, observation-pose anchoring, and timing gates
were independently validated from saved artifacts.

## What we cannot claim

This experiment does not show that graph optimization, correspondence, or any reconciliation
method improves execution. It does not establish navigation quality, real-world feasibility,
or a model-intrinsic horizon duration.

## Next decision

The positive translational evidence is sufficient to proceed to an EXP-02 oracle-
correspondence SE(2) graph prototype, while retaining raw-switch metrics as the baseline. The
largest uncertainty is the variability of successive LightNav chunks in a single simple scene,
especially occasional stop output; future work should separate that variability from latency-
induced frame mismatch without changing the EXP-01B artifacts.

## Reproduction

Run the concurrent headless measurement:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_exp01b_online_raw_switch.sh
```

Validate the immutable directory printed by the run:

```bash
.venv/bin/python scripts/summarize_exp01b.py data/exp01b/<experiment_id>
```

Add `--gui --hold` only for qualitative inspection. A GUI run uses its own newly measured
timing and must pass the same per-trial RTF gate; slow Stage 0-C playback settings are never
used by EXP-01B.
