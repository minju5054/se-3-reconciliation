# JOIN-ONLINE-02 saved STOP-cause audit

## Question and scope

Why did native LightNav stop before the cart instead of bypassing it and
continuing to the hallway end? This is a post-hoc read-only analysis of all 55
saved responses from `primary_20260923T001500Z`, including 14 cart-ON responses
and all four STOP responses. It introduces no new inference, rendering, MPC,
execution, GP, or reconciliation. Historical scientific outcomes are unchanged.

Starting HEAD and freshly fetched origin/main:
`ababf8518f5365a30b42bd1596cc3e8de66ddc25`.
Audit output: `data/robotless_join_online_02_stop_audit/audit_20260923_final/`.
The audit's code hash, source SHA, all input hashes, and exact source-record keys
are saved. The two unrelated Stage0 config edits remain untouched.

## Confirmed STOP mechanism

The checkpoint action-tokenizer manifest declares stop level 0 = **6**, with
the tuple **[6,122,174]**. Both ON C6 responses explicitly emit:

```text
<apos_1299><opos_744><act_l0_6><act_l1_122><act_l2_174><|im_end|>
```

The pinned official `RVQBundle.is_stop` checks level zero. Official
`tracking.py` returns a zero waypoint chunk for that explicit stop action;
`serving/protocol.py` derives top-level stop from the decoded zero path. APOS
1299 independently denotes the pointing-channel stop directive. This is not
an otherwise moving trajectory accidentally rounded to zero, a controller
failure, or a research safety-guard override. All four episode STOP responses
match the complete manifest tuple, not merely its first level.

The original wire responses contain **10 zero rows** at STOP. The earlier
report's phrase "one zero-pose action row" is a reporting error; the original
responses and arrays have not been changed. Generic N remains supported.

The collector terminates on MODEL_STOP before installing that response. Thus
there is no new B or later response after C6. The official MuJoCo demo's
`server.py:update_loop` likewise sets zero command, clears automatic ownership,
and stops the VLN client on `result.stop is True`. A later resume after STOP was
not observed or tested. This explains why the task ended; it does not explain
the network's hidden reason for choosing STOP.

## New pointing/instance evidence

For every response, the audit checks original RGB SHA, raw response/array SHA,
observation anchoring, same-state instance-mask provenance, and cart pixel count.
Pointing uses client-image coordinates 480x270 and a 48x27 vocabulary. In addition
to the reported centre pixel, all pixels in its 10x10 quantization cell are
checked. No model image is modified or resubmitted.

| ON chunk | APOS, repeat00 / repeat01 | OPOS, repeat00 / repeat01 | OPOS centre raster hit | cart fraction in OPOS cell, 00 / 01 |
|---|---|---|---|---|
| C0 | (245,165) / same | (245,125) / same | wall/door geometry | 0 / 0 |
| C1 | (245,175) / same | (245,125) / same | wall/door geometry | 0 / 0 |
| C2 | (245,175) / same | (245,125) / same | wall/door geometry | 0 / 0 |
| C3 | (235,195) / same | (235,115) / (235,125) | roof / door geometry | 0 / .10 |
| C4 | (245,235) / same | (245,125) / same | door geometry | 0 / 0 |
| C5 | (245,265) / (235,265), clamped | (245,125) / (235,125) | door geometry | .10 / .12 |
| C6 | STOP / STOP | (235,155) / same | **runtime cart** | **1.00 / 1.00** |

All 12 pre-STOP ON APOS centre samples hit the same floor mesh; none hits the
cart or supplies a measured wide side-bypass point. The nominal z=0 ray proxies
for repeat00 APOS move from local [3.584,-.108] m (C0), through [1.837,.054]
(C3) and [1.138,-.032] (C4), to [.896,-.025] (C5). C5 is **boundary-censored**:
its ray is only a boundary proxy, not the exact intended point or a metric
LightNav waypoint. Ground-plane intersection is not a collision-free path.

At C6, OPOS's centre and its entire quantized cell lie on
`/World/JOINSource02Obstacle/Prop/SM_SupplyCart_02a`, in both repetitions.
This is new evidence consistent with **target-grounding confusion at STOP**.
It does not prove that the network chose the cart as the goal: a farther target
can be occluded along the same image direction, and this instruction defines
"end of the hallway" rather than a uniquely labelled target object. The pixel
token has no depth, goal identity, confidence, or explanation. `visible=true`
is computed from OPOS being nonzero; it is not an independent recognition test.

OFF also eventually stops, farther along the corridor. Its final OPOS raster
hits a wet-floor sign (repeat00) or adjacent floor (repeat01), not a verified
hallway-end region. Consequently, OFF is evidence of continued progress past
the absent cart, **not independently established goal completion**.

## What explains the observed behavior, and what does not

| Explanation | Evidence | Bounded conclusion |
|---|---|---|
| MPC/guard forced STOP | Explicit native STOP action tuple and APOS STOP; no guard abort | Disfavored as the cause of STOP generation |
| A safe bypass was generated but tracking lost it | ON raw paths remain nearly straight; no bypass onset/full bypass | Not observed; failure is already in upstream output |
| Obstacle only appeared at the last moment | Cart present from first capture at 5 m, 372 pixels; successive live frames | Single-frame surprise does not explain this run |
| One short chunk merely needs another normal chunk | Seven successive ON responses, ending STOP | No bypass before STOP; no universal horizon-impossibility claim |
| No physical space to pass | Both predeclared lateral side segments pass geometry, margins .0737/.0600 m | Full corridor blockage not supported; this is not a dynamic transition feasibility proof |
| Cart was confused with target | OPOS switches from background to an entirely cart-covered cell at STOP in 2/2 repeats | Specific supported hypothesis; not causally identified |
| Route/goal occlusion caused conservative stopping | Shorter forward paths, APOS towards nearer floor, then STOP before contact | Also consistent; not separated from target confusion |
| Decoder converted a requested detour to stop | Explicit stop level and full stop tuple, independently APOS STOP | No evidence for this specific decoder-error explanation |
| Random isolated output | All seven ON raw arrays match across two runs | Single accidental output less plausible; small sample, no population claim |

Actual ON minimum footprint-edge clearance stays .355181/.354753 m, versus the
original .05 m requirement. No collision or emergency guard intervention caused
episode termination. The observable response reduces forward extent and stops;
it never develops meaningful lateral action. The source does not expose why
the policy prefers stopping over navigating around the obstacle.

The instruction explicitly says "Avoid the supply cart and continue to the end
of the hallway." The task is supplied through language and RGB, not an oracle
hallway-end coordinate. The official prompt guide recommends unambiguous object
goals; this is context for a grounding hypothesis, not proof that the instruction
was invalid or caused the failure. No new prompt is tried in this audit.

## Limits and single remaining causal question

Functional localization: **explicit upstream STOP, honored as task termination**.
Hidden cause: **not identified**. Target-object confusion, occluded goal, and
a conservative blocked-route policy remain distinguishable hypotheses.
Saved attention/logits/reason text are unavailable. OPOS and action tokens are
observable outputs, not a faithful transcript of private neural reasoning.

OFF/ON are separate moving episodes with different later histories, not exact
same-state causal pairs. Original pacing limits remain: ON request-local RTF
1.34–1.39 exceeds the previous [0.8,1.2] gate. No historical timing/safety
acceptance is changed. New metrics have no acceptance role.

The most useful next single contrast would hold a saved pre-STOP RGB/history
fixed and make the **destination identity** unambiguous while retaining the
same cart-avoidance clause, then examine whether OPOS remains on the cart and
STOP persists. Target must really be visible/identifiable. This would test
grounding versus blocked-route stopping; it is proposed only, not executed.

## Reproduction and validation

```bash
MPLCONFIGDIR=/tmp/join_stop_mpl .venv/bin/python scripts/audit_join_online02_stop.py --run data/robotless_join_online_02/primary_20260923T001500Z --output data/robotless_join_online_02_stop_audit/NEW_UNIQUE_OUTPUT
.venv/bin/python scripts/audit_join_online02_stop.py --run data/robotless_join_online_02/primary_20260923T001500Z --output data/robotless_join_online_02_stop_audit/audit_20260923_final --validate
.venv/bin/python scripts/validate_join_online02.py --run data/robotless_join_online_02/primary_20260923T001500Z --output data/robotless_join_online_02_stop_audit/audit_20260923_final/original_validation_recheck.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_join_online02_stop_audit.py tests/test_join_online02.py tests/test_join_source04.py tests/test_online_history.py tests/test_se2.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

95 relevant tests pass. Both the new saved-only audit validator and full original
saved-record validator pass. New model/MPC/GP/render/rollout calls: **all zero**,
including tests. No controller or model code is modified. The first derived
presentation had overlapping labels and remains in `audit_20260923_01`; final
figures use corrected layout, identical numeric evidence, and were visually
reviewed. Original scientific outputs are never overwritten.

Final review: `audit_20260923_final/index.html`; figures:
`on_pointing_sequence.png` and `stop_comparison.png`, each with full numeric and
source-hash sidecars. `analysis.json` contains all 55 records;
`validation.json` hashes the final presentation. These local RGB-derived images
are ignored by Git; this document preserves the key numbers for remote review.
