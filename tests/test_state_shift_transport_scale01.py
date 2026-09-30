"""Synthetic fixtures and saved-only authentication. No new scientific evidence."""
import ast,inspect,json,sys
from pathlib import Path
from copy import deepcopy
from dataclasses import asdict
from types import SimpleNamespace
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_state_shift_transport_scale01 as runner
import test_relative_factor_multisource01 as oldtests
import test_osa03_relative_factor_ablation01 as r00tests
from reconciliation.state_shift_transport_scale import *
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.join_source03 import read,save,sha
from reconciliation.graph_optimizer import SolverConfig
from validate_state_shift_transport_scale01 import validate_method
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
OLD=oldtests.PREP


def problem(n=7,alpha=.5):
    A=np.array([2.,-3.,.6]);B=np.array([2.4,-2.7,1.2])
    local=np.c_[np.linspace(0,1,n),np.linspace(0,.4,n)**2,np.linspace(.1,.7,n)]
    return TransportScaleProblem(A,B,local_trajectory_to_world(A,local),alpha=alpha)


@pytest.mark.parametrize('n',[2,3,7,14])
def test_alpha1_exact_historical_target_vector_cost(n):
    p=problem(n,1);old=LocalSE2Problem(p.A,p.B,p.fresh);x=p.fresh.copy();x[:,1]+=.02
    np.testing.assert_array_equal(p.target,old.target)
    np.testing.assert_array_equal(p.residual_vector(x),old.residual_vector(x));assert p.costs(x)==old.costs(x)


def test_alpha0_exact_original_and_zero_cost_without_solve():
    p=problem(alpha=0);np.testing.assert_array_equal(p.target,p.fresh)
    assert p.costs(p.fresh)['total']<1e-24


def test_half_lie_interpolation_not_componentwise():
    p=problem();expected=compose_poses(se2_exp(.5*se2_log(p.transport)),p.fresh)
    np.testing.assert_array_equal(p.target,expected)
    assert not np.allclose(p.target,compose_poses(.5*p.transport,p.fresh))
    np.testing.assert_allclose(relative_pose(p.A,p.target),compose_poses(se2_exp(.5*se2_log(relative_pose(p.A,p.B))),relative_pose(p.A,p.fresh)),atol=1e-14)


def test_default_unchanged_and_only_allowed_alpha():
    r00tests.test_pre_edit_full_golden_bit_exact()
    p=problem();q=TransportScaleProblem(p.A,p.B,p.fresh)
    assert q.alpha==1.;np.testing.assert_array_equal(q.target,LocalSE2Problem(p.A,p.B,p.fresh).target)
    with pytest.raises(ValueError):TransportScaleProblem(p.A,p.B,p.fresh,alpha=.25)


def test_immutable_A_B_F_no_reanchor_R_A_weights_scales():
    p=problem();raw=p.fresh.copy();A=p.A.copy();B=p.B.copy();x=p.target
    full=LocalSE2Problem(p.A,p.B,p.fresh)
    for k in ['R','A']:np.testing.assert_array_equal(p.residual_blocks(x)[k],full.residual_blocks(x)[k])
    np.testing.assert_array_equal(p.progress,full.progress);np.testing.assert_array_equal(p.scales,full.scales)
    np.testing.assert_array_equal(p.fresh,raw);np.testing.assert_array_equal(p.A,A);np.testing.assert_array_equal(p.B,B)
    with pytest.raises(ValueError):p.fresh[0,0]=1
    np.testing.assert_allclose(local_trajectory_to_world(A,p.original_observation_local(x)),x,atol=1e-14)
    assert not np.allclose(local_trajectory_to_world(B,p.original_observation_local(x)),x)
    np.testing.assert_allclose(relative_pose(B,full.target),relative_pose(A,raw),atol=1e-14)


def test_solver_config_and_single_original_initialization(tmp_path,monkeypatch):
    p=problem(3);(tmp_path/'references').mkdir();np.save(tmp_path/'references/M0_NATIVE_world.npy',p.fresh)
    save(tmp_path/'common_state.json',dict(fresh_capture_pose=p.A.tolist(),B=p.B.tolist()));save(tmp_path/'protocol.json',CFG)
    env={'on':SimpleNamespace(check_polyline=lambda x:{'clearance_valid':True})};monkeypatch.setattr(runner,'geometry',lambda _:env)
    x=runner.solve_half(tmp_path)
    from validate_osa03_relative_factor_replication01 import validate_planning
    assert validate_planning(tmp_path/'planning'/HALF,p,x,env,True)['valid']
    start=read(tmp_path/'planning'/HALF/'optimization_start.json')
    assert start['solver_config']==asdict(SolverConfig()) and start['alpha']==.5 and start['include_relative']
    np.testing.assert_array_equal(np.load(tmp_path/'references/M0_NATIVE_world.npy'),p.fresh)
    assert set(p.costs(x))=={'L','R','A','total'}
    with pytest.raises(FileExistsError):runner.solve_half(tmp_path)


def test_historical_authentication_and_all_three_old_validators():
    old,specs=runner.authenticate(CFG,deep=True);assert old==OLD and [s['id'] for s in specs]==SOURCE_IDS
    tampered=deepcopy(CFG);tampered['historical_hashes'][CFG['historical_result']]='0'*64
    with pytest.raises(AssertionError):runner.authenticate(tampered)


@pytest.mark.parametrize('sid',SOURCE_IDS)
def test_exact_frozen_source_schedule_reuse(sid):
    folder=OLD/'sources'/sid.replace('/','__');c=read(folder/'common_state.json');frozen=read(folder/'schedule.json')
    rs={NATIVE:read(folder/'methods'/NATIVE/'rollout.json'),FULL:read(folder/'methods'/FULL/'rollout.json')}
    rs[HALF]=deepcopy(rs[FULL]);assert schedule_gate(rs,frozen,c)['passed']
    rs[HALF]['phase']['u_mem_B'][0]+=.01;assert not schedule_gate(rs,frozen,c)['passed']


@pytest.mark.parametrize('bt',[92,997,1804,1524])
def test_half_install_restore_schedule_memory_primary_secondary(tmp_path,monkeypatch,bt):
    c,refs,manifest,env,scene=oldtests.setup(tmp_path,monkeypatch,bt)
    refs[HALF]=refs['NO_RELATIVE'];(tmp_path/'references.json').write_text(json.dumps(refs));rollouts={}
    for name in ORDER:
        runner.legacy.run_method(tmp_path,name)
        r=read(tmp_path/'methods'/name/'rollout.json');rollouts[name]=r
        _,_,audit=validate_method(tmp_path,name,refs[name],c,np.load(refs[NATIVE]['world_path']),env['on'],scene,manifest['mpc']['official_settings'])
        assert audit['valid'] and r['new_MPC_solved']==30 and r['termination']=='OBSERVATION_CAP'
        assert read(tmp_path/'methods'/name/'metrics.json')==read(tmp_path/'methods'/name/'own_reference_metrics.json')
    assert schedule_gate(rollouts,read(tmp_path/'schedule.json'),c)['passed']
    if bt==997:
        last=[e for e in rollouts[HALF]['events'] if e.get('type')=='solve_result'][-1]
        assert last['withheld_by_logical_scheduler'] and 'continuation_seen' not in last


def test_release_slow_wall_clock_stale_generation_abort_reference_gate(tmp_path,monkeypatch):
    oldtests.test_release_and_generation_existing_path_unchanged()
    oldtests.test_abort_guard_keeps_prefix_and_unreleased_counts(tmp_path,monkeypatch)


def test_unsafe_reference_gate(tmp_path,monkeypatch):
    oldtests.test_unsafe_reference_never_starts_controller(tmp_path,monkeypatch)


def test_primary_evaluator_null_and_own_reference_unchanged():
    oldtests.test_noR_old_behavior_and_attachment_unchanged()
    from reconciliation.osa03_native import evaluate
    assert runner.legacy.evaluate is evaluate


def fixture_summary():
    old=read(OLD/'summary.json');s=deepcopy(old);s['sources']={}
    for spec in s['selected_sources']:
        sid=spec['id'];q=deepcopy(old['sources'][sid]);q['primary_metrics'][HALF]=deepcopy(q['primary_metrics'][FULL])
        q['planning'][HALF]=deepcopy(q['planning'][FULL]);q['reference_safety'][HALF]=q['reference_safety'][FULL]
        q['primary_metrics']={n:q['primary_metrics'][n] for n in ORDER}
        q['termination']={n:'OBSERVATION_CAP' for n in ORDER};q['schedule_gate']={'passed':True}
        q['pairwise']={FULL:{'max_execution_XY_gap_m':0},NATIVE:{'max_execution_XY_gap_m':0}}
        c=read(Path(spec['folder'])/'common_state.json');f=np.load(Path(spec['folder'])/'references/M0_NATIVE_world.npy')
        q['transport']=transport_diagnostics(TransportScaleProblem(c['fresh_capture_pose'],c['B'],f,alpha=.5));s['sources'][sid]=q
    s['cross_source']=cross_source(s['sources']);return s


def test_classification_precedence_no_null_imputation():
    s=fixture_summary()['sources'];assert cross_source(s)['classification']=='TRANSPORT_MAGNITUDE_INSUFFICIENT'
    for q in s.values():q['primary_metrics'][HALF]['position_auc_09_m_s']=q['primary_metrics'][NATIVE]['position_auc_09_m_s']-.01
    assert cross_source(s)['classification']=='FULL_TRANSPORT_OVERCOMPENSATES'
    s[SOURCE_IDS[0]]['primary_metrics'][HALF]['position_auc_09_m_s']+=1
    assert cross_source(s)['classification']=='SOURCE_DEPENDENT_TRANSPORT'
    s[SOURCE_IDS[2]]['schedule_gate']['passed']=False
    assert cross_source(s)['classification']=='TECHNICAL_BLOCKED'
    assert cross_source(s)['deltas'][SOURCE_IDS[2]]['sustained_attachment_s'] is None


def test_selector_readonly_cooccurrence_and_nulls():
    folder=OLD/'sources'/SOURCE_IDS[3].replace('/','__');rs={n:read(folder/'methods'/n/'rollout.json') for n in [NATIVE,FULL]};rs[HALF]=deepcopy(rs[FULL])
    before=deepcopy(rs);p=fixture_summary()['sources'][SOURCE_IDS[3]]['primary_metrics'];z=selectors(rs,p)
    assert rs==before and z['comparisons'][FULL]['first_differing_submit_tick'] is None
    assert z['comparisons'][NATIVE]['first_differing_submit_tick'] is not None
    assert z['comparisons'][NATIVE]['attachment_changed'] is True


def test_exact_four_PNGs_numeric_sidecars(tmp_path,monkeypatch):
    import report_state_shift_transport_scale01 as report
    s=fixture_summary()
    # Historical Full is a plotting fixture for Half only; not new scientific evidence.
    for spec in s['selected_sources']:
        old=Path(spec['folder']);folder=tmp_path/old.name;folder.mkdir();spec['folder']=str(folder)
        for f in ['common_state.json','source_manifest.json','recorded_old_to_B.npy']:(folder/f).write_bytes((old/f).read_bytes())
        refs=read(old/'references.json');refs[HALF]=refs[FULL];save(folder/'references.json',refs)
        save(folder/'reuse.json',{'methods':{n:{'folder':str(old/'methods'/n)} for n in [NATIVE,FULL]}})
        (folder/'methods').mkdir();(folder/'methods'/HALF).symlink_to(old/'methods'/FULL,target_is_directory=True)
    records=report.compact_figures(s,tmp_path/'figures')
    assert sorted(p.name for p in (tmp_path/'figures').iterdir())==sorted(PNGS)
    assert all((tmp_path/'figures'/r['file']).read_bytes().startswith(b'\x89PNG') for r in records)
    assert records[1]['numeric_sidecar']['sustained_attachment_s'][HALF][2] is None


def test_no_extra_science_in_validator_report_and_runtime_reuse():
    assert runner.legacy.run_method is oldtests.runner.run_method
    for role in ['validate','report']:
        tree=ast.parse((ROOT/f'scripts/{role}_state_shift_transport_scale01.py').read_text())
        calls={n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not calls & {'solve_least_squares','solve_half','run_method','execute','Worker','submit','load_official'}
    source=inspect.getsource(runner.execute)
    assert source.count('solve_half(')==source.count('legacy.run_method(')==1
    assert 'periodic_schedule' not in inspect.getsource(runner.prepare)
