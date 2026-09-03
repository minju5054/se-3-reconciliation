# EXP-01A — LightNav steady-state inference latency

EXP-01A is a latency microbenchmark, not a navigation-quality experiment. It does not run
Isaac Sim or a Jackal, and it does not implement successive OLD/NEW switching.

## Question

When one LightNav model/engine instance remains loaded in one process, how different are the
first and subsequent `predict_waypoints(...)` latencies? Is the warm latency longer or shorter
than the `2.300000 s` execution time of the one validated Stage 0-C chunk?

## Setup

- Benchmark ID: `exp01a-20260903T062524Z`
- GPU: NVIDIA GeForce RTX 5060 Ti, compute capability 12.0
- LightNav checkout: `a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0
- Checkpoint: `LightOriginsHQ/LightNav-0`, revision
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`
- Backend: `vllm_local`, vLLM 0.19.1, torch 2.10.0+cu128
- Runtime: eager vLLM, 2 GiB explicit KV cache, chunked prefill disabled, multimodal
  embeddings enabled
- Input source: validated Stage 0-C run `20260903T010425Z`
- Instruction: `Go straight down the corridor.`
- Input: the same 64 actual Isaac RGB frames, shape `64 x 256 x 448 x 3`, uint8, 4 Hz
- Trials: trial 0 first inference; trials 1–8 warm inference
- Contract: every trial performs `agent.reset(instruction=...)`, observes all 64 frames in
  order, and calls `predict_waypoints(...)` once. `build_tracking_agent(...)` is called once.

This is a **controlled repeated-input latency microbenchmark**. It is not a distribution over
different scenes, instructions, or observation histories.

### Timing boundary

The primary latency is host-observed elapsed time from immediately before
`predict_waypoints(...)` until it returns, measured with `time.monotonic_ns()`. Frame decoding
occurs before the benchmark. Reset and in-memory observation ingestion are timed separately
and are excluded from the primary number. Model/engine build is also separate. No extra
`torch.cuda.synchronize()` is inserted: the local source shows that the vLLM `LLM.generate`
path is blocking and decoded text/actions exist at return.

The LightNav-reported number is also retained, but the local `vllm_local` implementation
defines it as its internal `llm_ms`. It excludes LightNav data preparation and ViT work, so it
is not interchangeable with the primary host boundary. Across warm trials, those internal
components averaged `39.149 ms` data preparation, `202.892 ms` ViT, and `305.173 ms` vLLM.

### Cache inspection

Runtime introspection and the vLLM initialization log both report prefix caching enabled.
LightNav's ViT cache is enabled, but `NavigationPolicy.reset()` clears both the session and
direct-engine ViT caches before every trial; each completed request repopulated 23 entries.
The installed LightNav vLLM embedding patch assigns a new `time.monotonic()`-derived video
hash to each request. Therefore identical RGB reuse does not produce a cross-trial LightNav
ViT or vLLM image-embedding cache hit. A text-only prefix may still be reusable because vLLM
prefix caching is enabled, but cache-hit metrics were not enabled by this serving setup.

There was consequently no clear evidence of an identical-image full-request cache hit that
would require a second varied-image benchmark. The cache settings were not changed. This
qualification is part of the interpretation of the repeated-input result.

## Measurements

Model/engine build took `10206.579 ms`. Total timed build-plus-nine-request wall time was
`15447.836 ms`.

| Trial | Kind | Host `predict_waypoints` [ms] | LightNav reported [ms] |
|---:|---|---:|---:|
| 0 | first | 652.138 | 380.212 |
| 1 | warm | 556.915 | 304.444 |
| 2 | warm | 556.759 | 305.602 |
| 3 | warm | 557.342 | 305.118 |
| 4 | warm | 556.715 | 305.758 |
| 5 | warm | 556.838 | 304.459 |
| 6 | warm | 557.131 | 305.493 |
| 7 | warm | 556.582 | 305.472 |
| 8 | warm | 556.470 | 305.039 |

Warm host statistics (population standard deviation, NumPy linear percentiles):

- count: 8
- mean: `556.844 ms`
- median: `556.799 ms`
- standard deviation: `0.266 ms`
- minimum / maximum: `556.470 / 557.342 ms`
- p90 / p95: `557.194 / 557.268 ms`
- first / warm-median ratio: `1.1712`

With only eight warm observations, especially p90 and p95 have limited statistical meaning.
All nine calls returned identical deterministic float32 `(10, 3)` arrays, and all values were
finite. The first row was
`[0.15485105, 0.00006702, -0.00419930]`; the last was
`[0.71874577, -0.00634444, -0.00297201]`. Every action array and raw text response is stored
under its own immutable trial filename.

## Interpretation

Within this already initialized software installation, the first request after model build
was only about 17% slower than the warm median. The historical Stage 0-C one-shot process took
`49561.998 ms` host time, about 89 times this warm median. That earlier measurement therefore
does not characterize current steady-state serving latency. It likely included one-time
cross-process cold/JIT/cache work not reproduced here, but EXP-01A did not clear system or
compiler caches and cannot attribute the 49.6-second cost to one component. A deployment must
load and warm the model before starting the online OLD/NEW loop and must treat cold-start
latency separately.

The warm median divided by the validated Stage 0-C execution duration is `0.2421`; conversely,
that one 2.3-second execution was about `4.13x` longer than a warm prediction. Under the
question's three-case interpretation this is Case C: the observed warm inference is shorter
than the observed chunk execution, so an asynchronous successive-chunk prototype is
technically worth implementing.

This does **not** define future waypoint timing. Stage 0-C's `2.300000 s` is the closed-loop
controller duration for one approximately 0.72 m decoded chunk. LightNav's public action rows
have no intrinsic waypoint timestamps, so this comparison is not a generic model-horizon
ratio and does not use `configs/exp01.yaml`'s example `waypoint_dt_seconds`.

## Cannot claim

These measurements do not establish latency across diverse scenes or instructions, real
robot latency, navigation quality, a final online OLD/NEW timing policy, or the cold-start
latency of a clean machine. They also do not establish an intrinsic duration for a LightNav
trajectory chunk. The small warm sample limits tail-latency conclusions.

## Next decision

Proceed to EXP-01B only with a persistent preloaded and explicitly warmed LightNav process.
EXP-01B must independently measure observation time, ready time, asynchronous OLD execution,
control period, OLD exhaustion, and the first usable NEW action. Cold-start behavior and
actual cache-hit observability should remain separate engineering follow-ups rather than being
silently folded into the online timing policy.

## Reproduction

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/lightnav/run_exp01a_lightnav_latency.sh
```

Validate the immutable output directory printed by that command:

```bash
.venv/bin/python scripts/summarize_exp01a_latency.py \
  data/exp01a/lightnav_latency/<benchmark_id>
```
