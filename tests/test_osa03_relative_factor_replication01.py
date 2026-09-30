"""Saved inputs and synthetic fixtures only; no new scientific solve in tests."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_osa03_relative_factor_replication01 as runner
import run_osa03_relative_factor_ablation01 as prior
import test_osa03_relative_factor_ablation01 as oldtests
from reconciliation.osa03_relative_replication import compatibility, selector_diagnostics, paired_effects, classify
from reconciliation.osa03_relative_ablation import ORDER, LogicalRelease, comparability, metric_row, gaps
from reconciliation.se2 import local_trajectory_to_world, relative_pose
from reconciliation.join_source03 import read, save, sha
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from validate_osa03_relative_factor_replication01 import validate_planning
PREP=ROOT/'data/osa03_relative_factor_replication_01/primary_20260930T010000Z'
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())


def test_exact_R01_authentication_anchor_state_and_no_R00_leakage():
    phase,fresh,raw,provenance=runner.source_phase(CFG)
    b=ROOT/CFG['source_run']/'source_bundle/REPEAT_01';source=read(b/'state_and_timing.json')
    assert phase['first_FRESH_solve']['solve_id']=='REPEAT_01_solve_000005'
    assert phase['original_generation']==6
    np.testing.assert_array_equal(phase['B'],source['B_state']['pose_world'])
    np.testing.assert_array_equal(phase['u_mem_B'],source['B_state']['memory']['previous_control'])
    np.testing.assert_array_equal(phase['u_minus'],source['B_state']['u_minus'])
    np.testing.assert_allclose(local_trajectory_to_world(phase['fresh_capture_pose'],raw),fresh,rtol=0,atol=1e-12)
    assert not np.allclose(local_trajectory_to_world(phase['B'],raw),fresh,rtol=0,atol=1e-3)
    r00=read(oldtests.PREP/'common_state.json')
    for k in ['B','fresh_capture_pose','u_minus','u_mem_B','u_B_plus']:
        assert phase[k]!=r00[k]
    assert read(PREP/'paired_source_facts.json')['raw_FRESH_identical']


def test_R01_source_hash_tamper_rejected():
    cfg=deepcopy(CFG);cfg['R01_bundle_hashes_sha256']='0'*64
    with pytest.raises(AssertionError):runner.source_phase(cfg)


def test_schedule_authenticated_and_R01_generation_restored_exactly():
    old,_,schedule,phase,_,_,provenance,gate=runner.authenticate(CFG)
    assert gate['compatible'] and gate['restored_generation']==6 and gate['R00_generation']==3
    assert schedule==read(old/'schedule.json')==read(PREP/'schedule.json')
    pre=read(PREP/'restoration_preflight.json')
    assert pre['numerical_MPC_calls']==0 and pre['passed']
    for r in pre['rows']:
        assert r['generation']==6 and r['B']==phase['B'] and r['previous_control']==phase['u_mem_B']
        assert r['capture_pose']==phase['fresh_capture_pose'] and r['reference_version']==phase['fresh_version']
        assert r['new_solve_calls']==0 and r['restored_future_results']==0


@pytest.mark.parametrize('key,value',[('B_tick',93),('integration_dt_s',.02),('next_submit_after_B',97),
    ('fresh_version',2),('original_generation',3),('u_mem_B',[0,0])])
def test_incompatible_clock_memory_or_generation_fails(key,value):
    common=read(PREP/'common_state.json');common[key]=value
    old=read(oldtests.PREP/'common_state.json');settings=read(PREP/'source_manifest.json')['mpc']['official_settings']
    assert not compatibility(common,read(PREP/'schedule.json'),old,settings,settings)['compatible']


@pytest.mark.parametrize('n',[2,10,17])
def test_factor_switch_full_parity_noR_absent_diagnostic_and_immutable(n):
    f=np.c_[np.linspace(0,1,n),np.linspace(0,.2,n),np.linspace(.1,.3,n)]
    p=LocalSE2Problem([0,0,0],[.2,.03,.1],f);before=f.copy();x=p.target
    np.testing.assert_array_equal(p.residual_vector(x),p.residual_vector(x,include_relative=True))
    blocks=p.residual_blocks(x);v=p.residual_vector(x,include_relative=False)
    np.testing.assert_array_equal(v,np.r_[blocks['L'].ravel(),blocks['A'].ravel()])
    assert 'R' in p.costs(x) and 'R' not in p.costs(x,include_relative=False)
    assert len(v)==6*n and p.raw_residuals(x)['R'].shape==(n-1,3)
    np.testing.assert_array_equal(f,before);assert not p.fresh.flags.writeable
    np.testing.assert_allclose(relative_pose(p.B,p.target),relative_pose(p.A,p.fresh),atol=1e-14)


def test_both_planning_conditions_same_config_init_safety_and_saved_validation(tmp_path,monkeypatch):
    f=np.array([[0.,0.,.1],[.2,.1,.3],[.8,.2,.4]])
    (tmp_path/'references').mkdir();np.save(tmp_path/'references/M0_NATIVE_world.npy',f)
    save(tmp_path/'common_state.json',dict(fresh_capture_pose=[0,0,0],B=[.2,.02,.1]))
    save(tmp_path/'protocol.json',CFG);save(tmp_path/'source_manifest.json',{'source':str(tmp_path)})
    env=SimpleNamespace(check_polyline=lambda x:{'clearance_valid':True})
    monkeypatch.setattr(prior,'geometry',lambda _:{'on':env})
    p=LocalSE2Problem([0,0,0],[.2,.02,.1],f)
    starts=[]
    for name in ORDER[2:]:
        x=runner.solve_condition(tmp_path,name);assert x is not None
        out=tmp_path/'planning'/name;starts.append(read(out/'optimization_start.json'))
        assert validate_planning(out,p,x,{'on':env},name=='FULL_LOCAL_SE2')['valid']
        checks=read(out/'feasibility_checks.json');np.testing.assert_array_equal(checks[0]['state'],f)
        np.testing.assert_array_equal(np.load(tmp_path/'references/M0_NATIVE_world.npy'),f)
    assert starts[0]['solver_config']==starts[1]['solver_config']==CFG['solver']
    assert starts[0]['initial_world_sha256']==starts[1]['initial_world_sha256']
    assert starts[0]['include_relative'] and not starts[1]['include_relative']
    r=read(tmp_path/'planning/NO_RELATIVE/planning_result.json')
    assert r['diagnostic_relative_edge_distortion_not_optimized_cost']>0 and 'R' not in r['final']
    with pytest.raises(FileExistsError):runner.solve_condition(tmp_path,'NO_RELATIVE')


def setup_R01_mock(tmp_path,monkeypatch):
    monkeypatch.setattr(oldtests,'PREP',PREP)
    common,refs,manifest=oldtests.setup_mock(tmp_path,monkeypatch)
    parent=prior.Worker;oldask=prior.ask
    class R01Worker(parent):
        def drain(self):
            rows=super().drain()
            for e in rows:
                if e.get('type')=='solve_result':e.update(official_generation=common['original_generation'],result_generation=common['original_generation'])
            return rows
    def ask(w,op,**kw):
        r=oldask(w,op,**kw);r['generation']=common['original_generation'];return r
    monkeypatch.setattr(prior,'Worker',R01Worker);monkeypatch.setattr(prior,'ask',ask)
    return common,refs,manifest


def test_four_R01_mock_rollouts_schedule_memory_selector_and_secondary(tmp_path,monkeypatch):
    from validate_osa03_common_b import validate_method
    common,refs,manifest=setup_R01_mock(tmp_path,monkeypatch);rs={};metrics={}
    for name in ORDER:
        prior.run_method(tmp_path,name);r=read(tmp_path/'methods'/name/'rollout.json');rs[name]=r
        assert r['error'] is None and r['new_MPC_solved']==30 and len(r['commands'])==180
        previous=common['u_mem_B']
        for e in r['events']:
            if e.get('status')=='submitted':assert e['previous_command']==previous
            if e.get('type')=='solve_result':previous=e['command'];assert e['result_generation']==6
        _,m,a=validate_method(tmp_path,name,refs[name],common,np.load(refs[name]['world_path']),
            prior.geometry(Path(manifest['source']))['on'],read(Path(manifest['source'])/'scenario.json'),manifest['mpc']['official_settings'])
        assert a['valid'];metrics[name]=m
        assert read(tmp_path/'methods'/name/'metrics.json')==read(tmp_path/'methods'/name/'own_reference_metrics.json')
    gate=comparability(rs,read(tmp_path/'schedule.json'),common);assert gate['comparable'] and gate['all_four_observed']
    before=deepcopy(rs);sel=selector_diagnostics(rs);assert before==rs
    assert len(sel['methods']['FULL_LOCAL_SE2'])==30 and sel['first_differing_submit_tick'] is None
    rs['NO_RELATIVE']['events'][1]['selection']['indices']=[9]*5
    assert selector_diagnostics(rs)['first_differing_submit_tick']==96
    rs['NO_RELATIVE']['phase']['B'][0]+=.1
    assert not comparability(rs,read(tmp_path/'schedule.json'),common)['comparable']
    # Plot synthetic records only. No report or validator ever invokes a controller.
    import validate_osa03_relative_factor_replication01 as validator
    import report_osa03_relative_factor_replication01 as reporter
    rows={n:metric_row(m,'OBSERVATION_CAP') for n,m in metrics.items()}
    summary=dict(classification='SYNTHETIC_TEST_ONLY',primary_metrics=rows,signed_method_minus_full=gaps(rows,'FULL_LOCAL_SE2'),
        secondary_own_reference_metrics={n:{'position_auc_03_m_s':v['position_auc_03_m_s']} for n,v in rows.items()},
        planning={n:refs[n]['descriptor'] for n in ORDER},pairwise={},replication_effects=[dict(metric='synthetic',R00_NoR_minus_Full=0.,R01_NoR_minus_Full=0.,sign_consistent=True)])
    save(tmp_path/'summary.json',summary);save(tmp_path/'validation.json',{'valid':True})
    monkeypatch.setattr(validator,'validate',lambda _:(summary,{'valid':True}))
    reporter.report(tmp_path);assert reporter.validate_figures(tmp_path)['valid']


def test_R01_guard_abort_after_wait_keeps_unreleased_result_and_prefix(tmp_path,monkeypatch):
    setup_R01_mock(tmp_path,monkeypatch);guard=prior.guard_check;calls=[]
    def blocked(*a):
        g=guard(*a);calls.append(1)
        if len(calls)==6:g['safe']=False
        return g
    monkeypatch.setattr(prior,'guard_check',blocked);prior.run_method(tmp_path,'NO_RELATIVE')
    r=read(tmp_path/'methods/NO_RELATIVE/rollout.json')
    assert r['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND' and len(r['commands'])==5
    assert not r['safety_abort']['command_applied'] and r['new_MPC_solved']==1
    assert r['events'][-1]['withheld_by_logical_scheduler']


def test_reference_unsafe_rejected_before_worker(tmp_path,monkeypatch):
    _,refs,_=setup_R01_mock(tmp_path,monkeypatch)
    refs['NO_RELATIVE']['safety']['clearance_valid']=False
    (tmp_path/'references.json').unlink();save(tmp_path/'references.json',refs)
    monkeypatch.setattr(prior,'Worker',lambda *a:pytest.fail('unsafe reference started a worker'))
    with pytest.raises(AssertionError):prior.run_method(tmp_path,'NO_RELATIVE')


def test_R01_slow_wait_and_no_early_release():
    w=oldtests.TinyWorker(dict(type='solve_result',status='command',official_generation=6,result_generation=6,command=[.8,1.]))
    q=LogicalRelease([dict(submit_tick=96,application_tick=99)],6);pose=np.array([1.,2.,.3]);ev=[]
    a=q.collect(w,96,'r01',pose,1.6,ev)
    assert a['sim_before']==a['sim_after'] and a['pose_before']==a['pose_after']
    assert w.calls>3 and q.release(98) is None and q.release(99)['result_generation']==6


@pytest.mark.parametrize('generation,status',[(3,'command'),(6,'stale_rejected'),(6,'controller_error')])
def test_R01_stale_and_generation_rejected(generation,status):
    q=LogicalRelease([dict(submit_tick=96,application_tick=99)],6)
    w=oldtests.TinyWorker(dict(type='solve_result',status=status,official_generation=6,result_generation=generation),delay=0)
    with pytest.raises(RuntimeError):q.collect(w,96,'r01',np.zeros(3),1.6,[])
    assert q.release(99) is None


def test_original_attachment_evaluator_and_secondary_parity():
    oldtests.test_primary_attachment_unchanged_own_reference_separate()
    assert prior.evaluate is runner.prior.evaluate


def test_R00_artifacts_and_validator_unchanged():
    from validate_osa03_relative_factor_ablation01 import validate
    old=ROOT/CFG['R00_tracked_result'];assert sha(old)==CFG['R00_tracked_result_sha256']
    record=read(old);run=Path(record['run']);summary,audit=validate(run)
    assert audit['valid'];assert summary==read(run/'summary.json')
    assert sha(run/'result_hashes.json')==record['result_hashes_sha256']


def test_validators_and_reports_contain_no_scientific_calls():
    for name in ['validate','report']:
        tree=ast.parse((ROOT/f'scripts/{name}_osa03_relative_factor_replication01.py').read_text())
        calls={n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not calls & {'solve_least_squares','solve_condition','run_method','execute','Worker','submit','load_official'}


def classification_fixture():
    s=deepcopy(read(oldtests.PREP/'summary.json'))
    return s


def test_predeclared_classification_and_signed_contrasts():
    s=classification_fixture();assert classify(s,True)=='REPLICATED_WITHIN_PAIRED_OSA03'
    effects=paired_effects(s,s);assert len(effects)==13 and all(r['sign_consistent'] for r in effects)
    s['primary_metrics']['NO_RELATIVE']['position_auc_09_m_s']=1.
    assert classify(s,True)=='PARTIAL_REPLICATION'
    s['primary_metrics']['NO_RELATIVE']['sustained_attachment_s']=3.
    assert classify(s,True)=='NOT_REPLICATED'
    s['schedule_gate']['comparable']=False
    assert classify(s,True)=='TECHNICAL_BLOCKED'


def test_missing_attachment_not_fabricated():
    s=classification_fixture();s['primary_metrics']['NO_RELATIVE']['sustained_attachment_s']=None
    assert classify(s,True)=='PARTIAL_REPLICATION'
    e=next(e for e in paired_effects(s,s) if e['metric']=='sustained_attachment_s')
    assert e['R01_NoR_minus_Full'] is None and e['sign_consistent'] is None
