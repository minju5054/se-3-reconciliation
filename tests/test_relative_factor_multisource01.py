"""Synthetic scheduler fixtures and authenticated saved reads; no scientific solves."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_relative_factor_multisource01 as runner
import test_osa03_relative_factor_ablation01 as historical
from reconciliation.relative_factor_multisource import *
from reconciliation.osa03_relative_ablation import comparability,metric_row
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from reconciliation.graph_optimizer import SolverConfig
from reconciliation.join_source03 import save,read,sha
from validate_relative_factor_multisource01 import validate_method
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
PREP=ROOT/'data/relative_factor_multisource_01/primary_20260930T033000Z'


def test_selected_source_uniqueness_and_duplicate_geometry_excluded():
    selected=read(PREP/'selected_sources.json');assert unique_sources(selected)
    rows=deepcopy(selected);rows[-1]['raw_sha256']=rows[0]['raw_sha256']
    with pytest.raises(ValueError,match='duplicate'):unique_sources(rows)
    rows=deepcopy(selected);rows[-1]['raw_value_sha256']=rows[0]['raw_value_sha256']
    with pytest.raises(ValueError,match='duplicate'):unique_sources(rows)
    with pytest.raises(ValueError,match='shortage'):unique_sources(selected[:2])
    inventory=candidate_inventory(CFG)
    assert len(inventory)==13 and len({x['raw_sha256'] for x in inventory})==10
    same=[r for r in inventory if r['raw_sha256']==selected[2]['raw_sha256']]
    assert len(same)==2 and sum(r['selected'] for r in same)==1


@pytest.mark.parametrize('spec',CFG['selected'],ids=[s['id'] for s in CFG['selected']])
def test_exact_source_provenance_frame_schedule_and_zero_solve_restoration(spec):
    d=load_source(CFG,spec);folder=PREP/'sources'/spec['id'].replace('/','__');c=d['common']
    assert c==read(folder/'common_state.json') and d['schedule']==read(folder/'schedule.json')
    np.testing.assert_array_equal(c['u_B_plus'],c['u_mem_B'])
    np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],d['raw']),d['fresh'],atol=1e-12,rtol=0)
    assert not np.allclose(local_trajectory_to_world(c['B'],d['raw']),d['fresh'])
    pre=read(folder/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
    for r in pre['rows']:
        assert r['generation']==c['original_generation'] and r['previous_control']==c['u_mem_B']
    pairs=d['schedule']['pairs'];assert pairs[0]['submit_tick']==c['next_submit_after_B']
    assert all(a['submit_tick']<a['application_tick']<b['submit_tick'] for a,b in zip(pairs,pairs[1:]))
    if spec['id'].endswith('handoff_020'):assert c['B_tick']==c['next_submit_after_B']==1524


def test_inventory_hash_tamper_fails():
    cfg=deepcopy(CFG);p=next(iter(cfg['inventory_hashes']));cfg['inventory_hashes'][p]='0'*64
    with pytest.raises(AssertionError):runner.checked(cfg)


@pytest.mark.parametrize('lag',[0,6,7,-1])
def test_unsupported_completion_lag_rejected(lag):
    c={'B_tick':100,'next_submit_after_B':102,'integration_dt_s':1/60}
    with pytest.raises(ValueError):periodic_schedule(c,{'submit_tick':102,'application_tick':102+lag},{})


def setup(tmp_path,monkeypatch,bt):
    c,refs,manifest=historical.setup_mock(tmp_path,monkeypatch)
    delta=bt-c['B_tick'];c['B_tick']=bt;c['next_submit_after_B']=((bt+5)//6)*6
    oldworker=historical.runner.Worker
    class FakeWorker(oldworker):
        def drain(self):
            events=super().drain()
            for e in events:
                if e.get('type')=='solve_result':e.update(official_generation=c['original_generation'],result_generation=c['original_generation'])
            return events
    def ask(w,op,**kw):
        w.world=np.load(kw['reference']['world_path'])
        return dict(B=c['B'],held_command=c['u_B_plus'],previous_control=c['u_mem_B'],generation=c['original_generation'],
            next_submit_tick=c['next_submit_after_B'],B_tick=bt,B_sim_s=c['B_sim_s'],integration_dt_s=c['integration_dt_s'],installed_world=w.world.tolist())
    (tmp_path/'common_state.json').write_text(json.dumps(c))
    (tmp_path/'protocol.json').write_text(json.dumps(CFG))
    nxt=c['next_submit_after_B'];sched=periodic_schedule(c,dict(submit_tick=nxt,application_tick=nxt+1),{})
    (tmp_path/'schedule.json').write_text(json.dumps(sched))
    scene=read(Path(manifest['source'])/'scenario.json');save(tmp_path/'scenario.json',scene)
    env=historical.runner.geometry(Path(manifest['source']))
    monkeypatch.setattr(runner,'Worker',FakeWorker);monkeypatch.setattr(runner,'ask',ask);monkeypatch.setattr(runner,'geometry',lambda _:env)
    return c,refs,manifest,env,scene


@pytest.mark.parametrize('bt',[997,1804,1524])
def test_variable_phase_four_method_parity_official_memory_and_primary_evaluator(tmp_path,monkeypatch,bt):
    c,refs,manifest,env,scene=setup(tmp_path,monkeypatch,bt);rollouts={}
    for name in ORDER:
        runner.run_method(tmp_path,name);r=read(tmp_path/'methods'/name/'rollout.json');rollouts[name]=r
        assert r['error'] is None and len(r['commands'])==180 and r['new_MPC_solved']==30
        _,_,v=validate_method(tmp_path,name,refs[name],c,np.load(refs[name]['world_path']),env['on'],scene,manifest['mpc']['official_settings']);assert v['valid']
        for wait in r['logical_waits']:assert wait['sim_before']==wait['sim_after'] and wait['pose_before']==wait['pose_after']
        assert read(tmp_path/'methods'/name/'metrics.json')==read(tmp_path/'methods'/name/'own_reference_metrics.json')
    gate=comparability(rollouts,read(tmp_path/'schedule.json'),c);assert gate['comparable'] and gate['all_four_observed']
    for r in rollouts.values():assert schedule(r,180)==read(tmp_path/'schedule.json')['full']
    rollouts['NO_RELATIVE']['phase']['u_mem_B'][0]+=.1
    assert not comparability(rollouts,read(tmp_path/'schedule.json'),c)['comparable']


def test_abort_guard_keeps_prefix_and_unreleased_counts(tmp_path,monkeypatch):
    setup(tmp_path,monkeypatch,997);old=runner.guard_check;calls=[]
    def fail(*args):
        g=old(*args);calls.append(1)
        if len(calls)==6:g['safe']=False
        return g
    monkeypatch.setattr(runner,'guard_check',fail);runner.run_method(tmp_path,'NO_RELATIVE')
    r=read(tmp_path/'methods/NO_RELATIVE/rollout.json')
    assert len(r['commands'])==5 and r['new_MPC_solved']==1 and not r['safety_abort']['command_applied']
    assert r['events'][-1]['withheld_by_logical_scheduler']


def test_unsafe_reference_never_starts_controller(tmp_path,monkeypatch):
    _,refs,_,_,_=setup(tmp_path,monkeypatch,997)
    refs['NO_RELATIVE']['safety']['clearance_valid']=False;(tmp_path/'references.json').write_text(json.dumps(refs))
    monkeypatch.setattr(runner,'Worker',lambda *a:pytest.fail('unsafe reference reached controller'))
    with pytest.raises(AssertionError):runner.run_method(tmp_path,'NO_RELATIVE')


def test_solver_settings_full_noR_and_posthoc_diagnostic(tmp_path,monkeypatch):
    from validate_osa03_relative_factor_replication01 import validate_planning
    f=np.array([[0.,0.,.1],[.2,.1,.3],[.8,.2,.4]])
    (tmp_path/'references').mkdir();np.save(tmp_path/'references/M0_NATIVE_world.npy',f)
    save(tmp_path/'common_state.json',dict(fresh_capture_pose=[0,0,0],B=[.2,.02,.1]));save(tmp_path/'protocol.json',CFG)
    env={'on':SimpleNamespace(check_polyline=lambda x:{'clearance_valid':True})};monkeypatch.setattr(runner,'geometry',lambda _:env)
    problem=LocalSE2Problem([0,0,0],[.2,.02,.1],f);starts=[]
    for name in ORDER[2:]:
        x=runner.solve_condition(tmp_path,name);out=tmp_path/'planning'/name;include=name=='FULL_LOCAL_SE2'
        assert validate_planning(out,problem,x,env,include)['valid'];starts.append(read(out/'optimization_start.json'))
        assert ('R' in problem.costs(x,include_relative=include))==include
    assert starts[0]['solver_config']==starts[1]['solver_config']==CFG['solver']
    assert starts[0]['initial_world_sha256']==starts[1]['initial_world_sha256']
    assert read(tmp_path/'planning/NO_RELATIVE/planning_result.json')['diagnostic_relative_edge_distortion_not_optimized_cost']>0
    np.testing.assert_array_equal(f,np.load(tmp_path/'references/M0_NATIVE_world.npy'))
    with pytest.raises(FileExistsError):runner.solve_condition(tmp_path,'NO_RELATIVE')


def test_noR_old_behavior_and_attachment_unchanged():
    historical.test_pre_edit_full_golden_bit_exact()
    historical.test_primary_attachment_unchanged_own_reference_separate()
    assert runner.evaluate is historical.runner.evaluate


def test_release_and_generation_existing_path_unchanged():
    historical.test_release_never_early_and_wall_wait_cannot_advance_simulation()
    historical.test_missed_tick_and_double_submit_fail_closed()
    for status,generation in [('stale_rejected',3),('command',4),('controller_error',3)]:historical.test_stale_generation_or_failure_never_released(status,generation)


def test_bounded_runtime_diff_only_source_geometry_phase_and_scene():
    import inspect
    before=inspect.getsource(historical.runner.run_method)
    after=inspect.getsource(runner.run_method)
    expected=before.replace("env=geometry(source)['on']","env=geometry(run)['on']").replace("restored['next_submit_tick']==96","restored['next_submit_tick']==common['next_submit_after_B']").replace("read(source/'scenario.json')","read(run/'scenario.json')")
    assert after==expected


def test_old_R00_R01_saved_validators_unchanged():
    from validate_osa03_relative_factor_ablation01 import validate as v00
    from validate_osa03_relative_factor_replication01 import validate as v01
    for p,v in zip(CFG['legacy_results'],[v00,v01]):
        assert sha(ROOT/p)==CFG['legacy_results'][p];r=Path(read(ROOT/p)['run']);s,a=v(r)
        assert a['valid'] and s==read(r/'summary.json')


def synthetic_summary():
    old=read(historical.PREP/'summary.json');q=deepcopy(old)
    q['termination']={n:'OBSERVATION_CAP' for n in ORDER}
    from reconciliation.osa03_relative_replication import selector_diagnostics
    q['selector']=selector_diagnostics({n:read(historical.PREP/'methods'/n/'rollout.json') for n in ORDER})
    return q


def test_predeclared_classification_reversals_nulls_and_tolerance():
    s={f'S{i}':synthetic_summary() for i in range(4)}
    assert cross_source(s)['classification']=='CONSISTENT_MULTISOURCE_PATTERN'
    assert all(not row['reversals_vs_R00_R01'] for row in cross_source(s)['sign_summary'])
    s['S0']['primary_metrics']['NO_RELATIVE']['position_auc_09_m_s']+=.1
    cross=cross_source(s);assert cross['classification']=='MIXED_EVIDENCE'
    assert 'S0' in next(x for x in cross['sign_summary'] if x['metric']=='position_auc_09_m_s')['reversals_vs_R00_R01']
    for q in s.values():
        q['primary_metrics']['NO_RELATIVE']=deepcopy(q['primary_metrics']['FULL_LOCAL_SE2'])
        q['primary_metrics']['NO_RELATIVE']['sustained_attachment_s']=None
    assert cross_source(s)['classification']=='NO_SUPPORT_FOR_PATTERN'
    s['S0']['schedule_gate']['comparable']=False;assert cross_source(s)['classification']=='TECHNICAL_BLOCKED'


def test_compact_required_PNGs_and_conditional_selector(tmp_path,monkeypatch):
    import report_relative_factor_multisource01 as report
    from reconciliation.osa03_relative_replication import selector_diagnostics
    # Saved R00 curves serve only as a plotting fixture; repeated fixture is not evidence.
    q=synthetic_summary();selected=[];sources={}
    for i in range(4):
        folder=tmp_path/f's{i}';folder.mkdir()
        for file in ['references.json','common_state.json']:(folder/file).write_bytes((historical.PREP/file).read_bytes())
        (folder/'methods').symlink_to(historical.PREP/'methods',target_is_directory=True)
        c=read(folder/'common_state.json');np.save(folder/'recorded_old_to_B.npy',[c['fresh_capture_pose'],c['B']])
        selected.append(dict(id=f'S{i}',folder=str(folder),role='SYNTHETIC PLOTTING FIXTURE'));sources[f'S{i}']=deepcopy(q)
    env=historical.runner.geometry(Path(read(historical.PREP/'source_manifest.json')['source']))
    monkeypatch.setattr(report,'geometry',lambda _:env)
    s=dict(selected_sources=selected,sources=sources,cross_source=cross_source(sources))
    records=report.compact_figures(tmp_path,s,tmp_path/'figures')
    assert set(p.name for p in (tmp_path/'figures').iterdir())==set(REQUIRED_PNG+[SELECTOR_PNG])
    assert all((tmp_path/'figures'/r['file']).read_bytes().startswith(b'\x89PNG') for r in records)
    for q in sources.values():q['selector']['differing_submit_ticks']=[]
    report.compact_figures(tmp_path,s,tmp_path/'no_selector')
    assert sorted(p.name for p in (tmp_path/'no_selector').iterdir())==sorted(REQUIRED_PNG)


def test_saved_validators_reports_never_call_scientific_tools():
    for role in ['validate','report']:
        tree=ast.parse((ROOT/f'scripts/{role}_relative_factor_multisource01.py').read_text())
        names={n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not names & {'solve_least_squares','solve_condition','run_method','execute','Worker','submit','load_official'}
