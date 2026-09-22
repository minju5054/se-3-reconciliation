"""Terminal wording ablation over immutable SOURCE04 inputs. Saved geometry only."""
from copy import deepcopy
import base64
import hashlib
import json
from pathlib import Path

import numpy as np

from .join_source03 import read, sha, trajectory_equal
from .join_source04 import construct_frames, premodel
from .join_source02 import response_geometry

INSTRUCTIONS = {
    'I0': 'Go to the tall blue shelving unit and stop.',
    'I1': 'Avoid the supply cart and continue straight down the hallway.',
    'I2': 'Avoid the supply cart and continue to the end of the hallway.',
}
KS = (0, 1, 2, 4, 8)
ORDER = [f'{i}_{k}' for i in ('I1', 'I2') for k in ('K0', 'K1', 'K1_SHAM', 'K2', 'K4', 'K8')]
GENERATION = dict(VLN_EVAL_TEMPERATURE='0', VLN_EVAL_TOP_P='1', VLN_EVAL_TOP_K='0', VLN_EVAL_TRAJ_TOP1='0')
LABEL = 'COUNTERFACTUAL MOVING-POSE HISTORY; terminal-instruction intervention; never executed'


def historical_id(k):
    if k not in KS:
        raise ValueError('undeclared K')
    return 'K0_OFF' if k == 0 else f'K{k}'


def condition(bank, cid):
    if cid not in ORDER:
        raise ValueError('undeclared condition')
    instruction, key, *_ = cid.split('_')
    k = int(key[1:])
    text = INSTRUCTIONS[instruction]
    return dict(condition=cid, instruction_id=instruction, K=k, H=16,
                frames=construct_frames(bank, k), instruction=text,
                instruction_utf8_sha256=hashlib.sha256(text.encode()).hexdigest(),
                premodel=premodel(bank, k), scope=LABEL,
                terminal_bank_state='ON' if k else 'OFF')


def verify_bank(bank, original_hashes):
    """Authenticate every RGB and evaluator raster against original capture records."""
    if len(bank) != 16:
        raise ValueError('exact original H16 bank required')
    hashes = {}
    for row in bank:
        for name in ('OFF', 'ON'):
            b = row[name]
            rec = read(b['capture_record'])
            for p, expected in ((b['frame']['path'], b['frame']['sha256']),
                                (b['mask_path'], rec['mask_sha256']),
                                (b['depth_path'], original_hashes[str(Path(b['depth_path']).resolve())])):
                if sha(p) != expected:
                    raise ValueError('SOURCE04 input hash mismatch: ' + p)
                hashes[p] = expected
            if b['frame']['sha256'] != rec['rgb_jpeg_sha256']:
                raise ValueError('JPEG/capture record mismatch')
    if not all(premodel(bank, k)['valid'] for k in KS):
        raise ValueError('SOURCE04 premodel gate mismatch; no replacements')
    return hashes


def requests(out):
    out = Path(out)
    rows = []
    for line in (out/'requests/wire.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row['direction'] == 'request' and row['action'] == 'next':
            rows.append(read(out/row['raw']['path'])['data'])
    return rows


def request_parity(new, old, frames, instruction):
    """Only terminal instruction differs in all16 scientific request payloads."""
    if len(new) != 16 or len(old) != 16 or len(frames) != 16:
        return dict(valid=False, reason='exact sixteen next requests required')
    checks = {}
    for i, (a, b, f) in enumerate(zip(new, old, frames)):
        aa, bb = deepcopy(a), deepcopy(b)
        checks[f'{i}:instruction'] = a['instruction'] == (instruction if i == 15 else '')
        checks[f'{i}:historical_instruction'] = b['instruction'] == (INSTRUCTIONS['I0'] if i == 15 else '')
        aa.pop('instruction'); bb.pop('instruction')
        checks[f'{i}:remaining_payload_literal'] = aa == bb
        raw = base64.b64decode(a['image'], validate=True)
        checks[f'{i}:jpeg'] = hashlib.sha256(raw).hexdigest() == f['sha256'] and raw == Path(f['path']).read_bytes()
        checks[f'{i}:seq'] = a['seq'] == b['seq'] == i
    return dict(valid=all(checks.values()), checks=checks,
                scope='login/session IDs and host clocks differ by design; scientific next payload differs only in terminal instruction')


def compare(a, b, influence):
    d = response_geometry(a['world'], b['world'], influence)
    return dict(equivalent=trajectory_equal(a['raw_local'], b['raw_local']),
                clearance_gain_m=b['geometry_on']['whole']['minimum_clearance_m']-a['geometry_on']['whole']['minimum_clearance_m'],
                **d)


def classify_instruction(results, historical, name, influence):
    keys = [f'{name}_K{k}' for k in KS] + [f'{name}_K1_SHAM']
    if not all(k in results for k in keys):
        return dict(classification='INSTRUCTION_EFFECT_INCONCLUSIVE', reason='incomplete coverage')
    a, b = results[f'{name}_K1'], results[f'{name}_K1_SHAM']
    sham = dict(array_literal=a['raw_local'] == b['raw_local'], raw_text_literal=a['raw_text'] == b['raw_text'])
    base = results[f'{name}_K0']; comparisons = {}; qualified = []; bypasses = []
    for k in KS:
        r = results[f'{name}_K{k}']; h = historical[historical_id(k)]
        diff = compare(base, r, influence); prior = compare(h, r, influence)
        safe = bool(r['geometry_on']['whole']['clearance_valid'])
        lateral = r['motion']['max_abs_lateral_m'] >= .20
        progress = r['hallway_endpoint_forward_m'] > 0
        detour = bool(k > 0 and safe and not r['stop'] and diff['meaningful'] and lateral and progress)
        bypass = detour and r['motion']['endpoint_beyond_rear_m'] >= 0
        text_turn = bool(lateral and base['motion']['max_abs_lateral_m'] >= .20 and
                         base['motion']['broad_side'] == r['motion']['broad_side'] and not diff['meaningful'])
        comparisons[str(k)] = dict(versus_historical=prior, versus_same_instruction_K0=diff,
                                  safe_on=safe, lateral=lateral, forward_progress=progress,
                                  visual_conditioned_safe_detour=detour, full_bypass=bypass,
                                  substantially_same_text_turn=text_turn)
        if detour: qualified.append(f'{name}_K{k}')
        if bypass: bypasses.append(f'{name}_K{k}')
    ons = [comparisons[str(k)] for k in KS if k]
    lateral_ons = [x for x in ons if x['lateral']]
    if not all(sham.values()): label = 'INSTRUCTION_EFFECT_INCONCLUSIVE'
    elif qualified: label = 'INSTRUCTION_RECOVERS_VISUAL_CONDITIONED_SAFE_DETOUR'
    elif lateral_ons and all(x['substantially_same_text_turn'] for x in lateral_ons): label = 'INSTRUCTION_ONLY_STEERING'
    elif not any(x['safe_on'] for x in ons) and any(x['versus_historical']['clearance_gain_m'] >= .02 for x in ons): label = 'INSTRUCTION_IMPROVES_BUT_REMAINS_UNSAFE'
    elif all(x['versus_historical']['equivalent'] for x in comparisons.values()): label = 'NO_MATERIAL_INSTRUCTION_EFFECT'
    elif any(x['safe_on'] for x in ons): label = 'INSTRUCTION_EFFECT_INCONCLUSIVE'
    else: label = 'INSTRUCTION_CHANGES_OUTPUT_WITHOUT_SAFETY_GAIN'
    return dict(classification=label, sham=sham, comparisons=comparisons,
                visual_conditioned_safe_detours=qualified, complete_bypasses=bypasses)


def summarize(results, historical, influence):
    decisions = {i: classify_instruction(results, historical, i, influence) for i in ('I1', 'I2')}
    labels = [x['classification'] for x in decisions.values()]
    if 'INSTRUCTION_EFFECT_INCONCLUSIVE' in labels: overall = 'INCONCLUSIVE'
    elif 'INSTRUCTION_RECOVERS_VISUAL_CONDITIONED_SAFE_DETOUR' in labels: overall = 'EXPLICIT_INSTRUCTION_RECOVERS_SAFE_SOURCE'
    elif 'INSTRUCTION_IMPROVES_BUT_REMAINS_UNSAFE' in labels: overall = 'EXPLICIT_INSTRUCTION_HELPS_BUT_NOT_ENOUGH'
    elif 'INSTRUCTION_ONLY_STEERING' in labels: overall = 'EXPLICIT_INSTRUCTION_MAINLY_INDUCES_TEXT_ONLY_STEERING'
    elif all(x == 'NO_MATERIAL_INSTRUCTION_EFFECT' for x in labels): overall = 'EXPLICIT_INSTRUCTION_HAS_NO_MATERIAL_EFFECT'
    else: overall = 'EXPLICIT_INSTRUCTION_CHANGES_BEHAVIOR_NOT_SAFETY'
    return dict(classification=overall, instructions=decisions,
                I1_vs_I2={str(k): compare(results[f'I1_K{k}'], results[f'I2_K{k}'], influence)
                          for k in KS if all(f'{i}_K{k}' in results for i in ('I1', 'I2'))},
                scope=LABEL, online_source_obtained=False, moving_B=None,
                new_MPC_GP_rigid_reconciliation_rollout_render_calls=0,
                scientific_predictions_completed=len(results),
                terminal_RTT_sum_s=sum(r['client_rtt_s'] for r in results.values()),
                worker_wall_sum_s=sum(r['worker_wall_s'] for r in results.values()))
