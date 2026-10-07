"""Saved-data contracts and synthetic tests; zero scientific replay/solve calls."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts')]
from reconciliation.join_source03 import sha
from reconciliation.successive_isaac_replay01 import (
    authenticate, authenticate_files, load_boundary, extract_schedule, horizon,
    jsonlines, require_raw_schedule,
)
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.robotless_online import integrate_unicycle


@pytest.fixture(scope='module')
def saved():
    cfg = yaml.safe_load((ROOT/'configs/successive_isaac_reconciliation_replay_01.yaml').read_text())
    source, ep, auth = authenticate(ROOT, cfg)
    boundary, arrays, states, commands, events = load_boundary(source, ep, cfg)
    schedule = extract_schedule(states, commands, jsonlines(ep/'scheduler.jsonl'), events, cfg['allowed_reference'])
    return cfg, boundary, arrays, auth, schedule


def test_saved_authentication_and_exact_handoff(saved):
    _, b, arrays, auth, _ = saved
    assert auth['valid'] and auth['sealed_file_count'] == 401 and auth['episode_input_count'] == 297
    assert b['B_tick'] == 92 and b['reference_version'] == 1 and b['generation'] == 3
    assert b['saved_kinematic_max_component_error'] <= 1e-10
    assert b['saved_kinematic_check_is_replay'] is False
    assert all(a.shape == (10, 3) for a in arrays.values())


def test_hash_tampering_fails(tmp_path):
    p = tmp_path/'raw.npy'
    np.save(p, [[1., 2., 3.]])
    frozen = {p: sha(p)}
    assert authenticate_files(frozen) == 1
    p.write_bytes(p.read_bytes()+b'tampered')
    with pytest.raises(ValueError, match='hash mismatch'):
        authenticate_files(frozen)


@pytest.mark.parametrize('prefix,anchor', [('old','A_old'), ('fresh','A')])
def test_original_frames_no_B_reanchor_no_mutation(saved, prefix, anchor):
    _, b, arrays, _, _ = saved
    raw, world = arrays[prefix+'_raw_local'], arrays[prefix+'_world']
    before = raw.tobytes(), world.tobytes()
    np.testing.assert_array_equal(local_trajectory_to_world(b[anchor], raw), world)
    assert not np.allclose(local_trajectory_to_world(b['B'], raw), world)
    with pytest.raises(ValueError):
        world[0, 0] += 1
    assert before == (raw.tobytes(), world.tobytes())


def test_full_horizon_does_not_filter_C2_submission(saved):
    _, b, _, _, s = saved
    assert s['start_tick'] == 92 and s['stop_before_application_tick'] == 122
    assert s['integration_steps'] == 30
    assert s['attempted_submit_ticks'] == s['accepted_submit_ticks'] == [96,102,108,114,120]
    assert s['new_application_ticks'] == [97,103,109,115]
    assert [r['application_tick'] for r in s['submissions']] == [97,103,109,115,122]
    assert s['submissions'][-1]['chunk_id'] == 'chunk_002'
    assert s['installs'][0]['install_tick'] == 120
    assert s['installs'][0]['reply_seen_tick'] == 121
    assert b['generation'] == 3 and s['generation_changes'][0]['generation'] == 4
    with pytest.raises(ValueError, match='TECHNICAL_BLOCKED'):
        require_raw_schedule(s)


def fixture(foreign=True):
    states = [dict(tick=t, sim_time_s=t/10) for t in range(6)]
    commands = [dict(application_tick=t, chunk_id='raw' if t<5 else 'next',
        reason='new_solve' if t in [0,2,5] else 'hold',
        solve_id='initial' if t<2 else 'a' if t<5 else 'b') for t in range(6)]
    scheduler = [dict(start_state_id=t, mpc_submitted=t in ([1,4] if foreign else [1])) for t in range(6)]
    def solve(sid, generation, tick):
        return dict(type='solve_result', solve_id=sid, official_generation=generation,
                    result_generation=generation, status='command', seen_in_isaac={'sim_time_s':tick/10})
    events = [solve('initial',3,0),dict(status='submitted',solve_id='a',input_state_id=1,
              chunk_id='raw',reference_version=1,official_generation=3),solve('a',3,2)]
    if foreign:
        events += [dict(status='installed',chunk_id='next',reference_version=2,official_generation=4,
            install_context={'t_install':{'sim_time_s':.4}},seen_in_isaac={'sim_time_s':.4}),
            dict(status='submitted',solve_id='b',input_state_id=4,chunk_id='next',
                 reference_version=2,official_generation=4),solve('b',4,5)]
    return states, commands, scheduler, events


def test_synthetic_single_reference_gate_pass_is_not_science():
    s = extract_schedule(*fixture(False), 'raw')
    assert s['raw_only_eligible'] and s['integration_steps'] == 5
    require_raw_schedule(s)


def test_synthetic_next_result_at_horizon_still_has_in_window_events():
    s = extract_schedule(*fixture(), 'raw')
    assert s['submissions'][-1]['application_tick'] == s['stop_before_application_tick']
    assert len(s['foreign_reference_events']) == 2
    with pytest.raises(ValueError):
        require_raw_schedule(s)


def test_schedule_missing_submit_fails_closed():
    states, commands, scheduler, events = fixture()
    scheduler[4]['mpc_submitted'] = False
    with pytest.raises(AssertionError, match='missing submit'):
        extract_schedule(states, commands, scheduler, events, 'raw')


def test_horizon_requires_contiguous_actual_commands():
    _, commands, _, _ = fixture()
    commands[2]['chunk_id'] = 'other'
    with pytest.raises(ValueError, match='discontinuous'):
        horizon(commands, 'raw')


def test_existing_restoration_with_saved_boundary_mock_only(saved):
    from osa03_common_b_mpc_worker import initialize
    cfg, b, _, _, s = saved
    from reconciliation.join_source03 import read
    ready = read(ROOT/cfg['source']/'source_bundle/handoffs/C0_to_C1/ready_record.json')
    common = dict(B=b['B'], B_tick=b['B_tick'], B_sim_s=b['B_sim_s'], u_minus=b['u_minus'],
        u_B_plus=b['u_B_plus'], u_mem_B=b['controller_memory']['previous_control'],
        fresh_capture_pose=b['A'], fresh_chunk_id=cfg['allowed_reference'], fresh_version=b['reference_version'],
        original_generation=b['generation'], first_FRESH_solve=ready['first_solve'],
        next_submit_after_B=s['accepted_submit_ticks'][0], integration_dt_s=b['integration_dt_s'])
    refs = b['reference_files']
    ref = dict(local_path=refs['fresh_raw_local']['path'],local_sha256=refs['fresh_raw_local']['sha256'],
        world_path=refs['fresh_world']['path'],world_sha256=refs['fresh_world']['sha256'])
    class MockAdapter:
        def __init__(self):
            self.episode_id=None; self.used_solve_ids=set(); self.pending=None
            self.tracker=SimpleNamespace(_future=None)
        def reset(self, identity):
            self.episode_id=identity
        def install(self, **kw):
            assert kw['capture_pose'] == b['A'] and kw['capture_pose'] != b['B']
            self.world=local_trajectory_to_world(kw['capture_pose'],np.load(kw['raw_local_path']))
            self.installed={'official_generation':1}
            return kw
    a=MockAdapter(); restored=initialize(a,common,ref,'SYNTHETIC_RESTORE_ONLY')
    assert restored['previous_control'] == b['controller_memory']['previous_control']
    assert restored['held_command'] == b['u_B_plus'] != b['u_minus']
    assert restored['generation'] == 3 and restored['new_solve_calls'] == 0
    assert restored['restored_future_results'] == 0


@pytest.mark.parametrize('omega',[0.,.5,-.5])
def test_synthetic_se2_integration_composes_spatial_pose(omega):
    p=np.array([1.,2.,3.13]); dt=float(np.float32(1/60)); u=[.4,omega]
    x=p.copy()
    for _ in range(30):
        x=integrate_unicycle(x,u,dt)
    np.testing.assert_allclose(x,integrate_unicycle(p,u,30*dt),rtol=0,atol=1e-12)


def test_abort_only_guard_reused_without_command_mutation():
    from reconciliation.join_online02 import guard_check
    class Unsafe:
        def check_trajectory(self, *args, **kwargs):
            return dict(clearance_valid=False)
    c=dict(v_mps=.4,omega_radps=.1,reason='new_solve'); original=deepcopy(c)
    assert not guard_check(Unsafe(),[0,0,0],c,1/60)['safe']
    assert c == original


def test_saved_only_entry_points_have_no_science_calls_or_imports():
    for name in ['src/reconciliation/successive_isaac_replay01.py','scripts/audit_successive_isaac_replay01.py']:
        tree=ast.parse((ROOT/name).read_text())
        calls={n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id
            for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,(ast.Attribute,ast.Name))}
        assert not calls & {'Popen','SimulationApp','Worker','solve','solve_graph','minimize',
            'generate','capture_rgb','replay_reference','create_connection'}
        imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        assert not any('lightnav_worker' in m or 'graph_optimizer' in m or 'isaac.' in m for m in imports)


def test_blocked_report_has_null_parity_zero_calls_no_fake_figure():
    from audit_successive_isaac_replay01 import build, OUT
    a=build()
    r=a['result_summary.json']
    assert r['classification']=='TECHNICAL_BLOCKED' and not r['next_B_ENTRY_scientifically_valid']
    assert all(v['value'] is None for v in a['parity.json'].values())
    assert all(v==0 for v in a['call_accounting.json'].values())
    assert r['scientific_freeze_sha'] is None and not r['scientific_execution_occurred']
    assert r['figure'] is None and not list(OUT.rglob('*.png'))
