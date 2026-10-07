#!/usr/bin/env python3
"""Saved-only prerequisite report; deliberately has no scientific launch mode."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import yaml

from reconciliation.join_source03 import read, save, sha
from reconciliation.long_source_mpc01 import authenticate_speed
from reconciliation.successive_isaac_replay01 import (
    authenticate, authenticate_files, load_boundary, extract_schedule, jsonlines,
    require_raw_schedule,
)

CONFIG = ROOT/'configs/successive_isaac_reconciliation_replay_01.yaml'
OUT = ROOT/'results/successive_isaac_reconciliation_replay_01'
CODE = [
    'src/reconciliation/successive_isaac_replay01.py',
    'scripts/audit_successive_isaac_replay01.py',
    'tests/test_successive_isaac_replay01.py',
    'src/reconciliation/osa03_relative_ablation.py',
    'src/reconciliation/osa03_common_b.py',
    'scripts/osa03_common_b_mpc_worker.py',
    'src/reconciliation/online_mpc_adapter.py',
    'src/reconciliation/long_source_mpc01.py',
    'scripts/long_source_mpc_worker01.py',
    'scripts/online_mpc_worker.py',
    'scripts/run_continuous_obstacle_reveal_episode01b.py',
    'scripts/run_successive_native_source_acquisition02.py',
    'scripts/isaac/join_online02_collect.py',
    'scripts/isaac/robotless_online_handoffs.py',
    'src/reconciliation/robotless_online.py',
    'src/reconciliation/join_online02.py',
    'src/reconciliation/join_source02.py',
    'scripts/continuous_obstacle_reveal_exploratory_geometry02.py',
    'scripts/validate_successive_source02_accounting_addendum.py',
]


def build():
    cfg = yaml.safe_load(CONFIG.read_text())
    source, episode, auth = authenticate(ROOT, cfg)
    b, arrays, states, commands, events = load_boundary(source, episode, cfg)
    schedule = extract_schedule(states, commands, jsonlines(episode/'scheduler.jsonl'),
                                events, cfg['allowed_reference'])
    try:
        require_raw_schedule(schedule)
    except ValueError as exc:
        blocker = str(exc)
    else:
        raise RuntimeError('Expected blocker absent: review protocol; this auditor cannot launch science')
    mpc = read(source/'workers.json')['mpc']['provenance']
    speed = mpc['speed_intervention']
    assert authenticate_speed(Path(speed['prior_workers']).parent) == speed
    assert mpc['official_settings']['OBJNAV_V_MAX'] == mpc['effective_linear_velocity_limit_m_s'] == .4
    assert mpc['official_settings']['HORIZON'] == 5
    authenticate_files({mpc['mpc_source']: mpc['mpc_source_sha256']})
    reveal = read(episode/'obstacle_reveal.json')
    assert reveal['state_id'] < b['B_tick'] and reveal['oracle_present'] and reveal['rendering_present']
    chunks = read(source/'source_bundle/chunks.json')
    c1 = next(c for c in chunks if c['chunk_id'] == cfg['allowed_reference'])
    assert c1['cart_transform'] == reveal['cart_transform'] and c1['cart_present']
    guards = [g for g in jsonlines(episode/'guard.jsonl') if
              schedule['start_tick'] <= g['command']['application_tick'] < schedule['stop_before_application_tick']]
    assert len(guards) == schedule['integration_steps'] and all(g['safe'] and g['cart_present'] for g in guards)
    # Reuse the source's direct checker and abort-only guard, on saved inputs only.
    from continuous_obstacle_reveal_exploratory_geometry02 import environments
    from reconciliation.join_source02 import whole_raw_polyline_check
    from reconciliation.join_online02 import guard_check
    from validate_robotless_online_handoffs import equal_record
    _, env, _, _ = environments(source)
    reference = whole_raw_polyline_check(arrays['fresh_world'], env)
    assert reference['minimum_clearance_m'] == c1['geometry_on']['minimum_clearance_m']
    for g in guards:
        checked = guard_check(env, g['start_pose'], g['command'], b['integration_dt_s'])
        equal_record(checked['check'], g['check'])
        assert checked['safe'] == g['safe'] and not checked['command_modified']
    prior = yaml.safe_load((ROOT/cfg['parity_tolerance_source']).read_text())
    assert cfg['pose_atol'] == prior['historical_parity_pose_atol']
    assert cfg['command_atol'] == prior['historical_parity_command_atol']
    assert cfg['selection_atol'] == prior['selection_atol']
    calls = {name: 0 for name in ['Isaac_scientific_launches', 'scientific_rollouts', 'MPC_solves',
        'LightNav_calls', 'RGB_model_requests', 'optimizer_calls', 'B_ENTRY_Hermite_calls', 'retries']}
    auth.update(boundary=b, mpc=mpc, cart_transform=reveal['cart_transform'],
        reveal_state_id=reveal['state_id'], obstacle_present_at_saved_B=True,
        source_reference_clearance_m=reference['minimum_clearance_m'],
        source_C1_guard_minimum_clearance_lower_bound_m=min(g['check']['minimum_clearance_lower_bound_m'] for g in guards),
        source_C1_guard_checks=len(guards), source_guard_recomputation_pass=True,
        source_C1_guard_legacy_5cm_pass=all(g['check']['legacy_5cm_margin_pass'] for g in guards),
        live_Isaac_pose_cart_restoration_verified=False)
    scientific_fields = ['initial_B_pose_error', 'per_tick_pose_max_error', 'applied_command_max_error',
        'application_ticks_equal', 'controller_memory_equal', 'guard_decisions_equal',
        'minimum_swept_clearance_m', 'horizon_termination_equal']
    parity = {name: dict(value=None, status='NOT_RUN_SCHEDULE_PREREQUISITE_FAILED') for name in scientific_fields}
    summary = dict(experiment=cfg['experiment'], classification='TECHNICAL_BLOCKED',
        starting_sha=cfg['source_result_commit'], scientific_freeze_sha=None,
        source=str(source.relative_to(ROOT)), handoff=cfg['handoff'], method='RAW',
        prerequisite_failure=blocker, scientific_execution_occurred=False,
        source_schedule=schedule, parity=parity, safety_result='REPLAY_NOT_RUN',
        source_safety_only=dict(reference_clearance_m=auth['source_reference_clearance_m'],
            guard_minimum_clearance_lower_bound_m=auth['source_C1_guard_minimum_clearance_lower_bound_m']),
        calls=calls, next_B_ENTRY_scientifically_valid=False, figure=None,
        protocol_deviation='No scientific launch/freeze: followed the explicit schedule-prerequisite STOP rule. No schedule filtering, horizon shortening or C2 execution.')
    selected = [source/'source_bundle/handoffs/C0_to_C1/ready_record.json',
        source/'source_bundle/handoffs/C0_to_C1/context.json', source/'source_bundle/chunks.json',
        source/'source_bundle/manifest.json', source/'final_saved_artifact_seal.json',
        source/'workers.json', source/'scenario.json', source/'acquisition_scene.json',
        source/'protocol.json', source/'config_snapshot.yaml', source/'validation.json']
    selected += [episode/p for p in ['execution.csv', 'commands.csv', 'scheduler.jsonl',
        'controller/events.jsonl', 'guard.jsonl', 'metadata.json', 'obstacle_reveal.json']]
    selected += [Path(r['path']) for r in b['reference_files'].values()]
    manifest = dict(kind='BLOCKED_PREFLIGHT_SNAPSHOT_NOT_SCIENTIFIC_FREEZE',
        scientific_freeze_sha=None, scientific_launch_authorized=False,
        config_sha256=sha(CONFIG), source_authentication=auth,
        input_hashes={str(p.relative_to(ROOT)): sha(p) for p in selected},
        code_hashes={p: sha(ROOT/p) for p in CODE},
        logical_schedule=schedule, restoration_rules=cfg['restoration'],
        expected_budget=cfg['budget'], parity_tolerances={k: cfg[k] for k in
            ['pose_atol','command_atol','selection_atol','parity_rtol','byte_exact']})
    validation = dict(saved_only_audit_valid=True, source_authentication_valid=True,
        source_guard_recomputation_valid=True, schedule_prerequisite_pass=False,
        scientific_parity_pass=None, no_scientific_calls=True, no_final_PNG_expected=True,
        live_restoration_unverified=['Isaac B pose', 'actual cart transform', 'live MPC memory/generation'],
        classification='TECHNICAL_BLOCKED')
    return {'source_authentication.json': auth, 'schedule_audit.json': schedule,
        'freeze_manifest.json': manifest, 'result_summary.json': summary,
        'validation_summary.json': validation, 'call_accounting.json': calls,
        'parity.json': parity}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--write', action='store_true', help='create new tracked audit artifacts; never overwrite')
    args = p.parse_args()
    artifacts = build()
    for name, value in artifacts.items():
        if args.write:
            save(OUT/name, value)
        else:
            assert read(OUT/name) == value, name
    assert not list(OUT.rglob('*.png')), 'no scientific replay: do not fabricate a parity plot'
    print('TECHNICAL_BLOCKED; saved-only audit valid; new Isaac/MPC/LightNav/optimizer calls 0')


if __name__ == '__main__':
    main()
