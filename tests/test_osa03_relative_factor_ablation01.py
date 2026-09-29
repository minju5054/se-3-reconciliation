"""Synthetic tests and saved authentication only; no scientific MPC/planning calls."""
import ast,json,sys,shutil
from pathlib import Path
from copy import deepcopy
import numpy as np
import pytest
ROOT=Path(__file__).parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares
from reconciliation.se2 import compose_poses,relative_pose
from reconciliation.osa03_relative_ablation import *
from reconciliation.osa03_common_b import may_execute
from reconciliation.join_source03 import save,read,sha
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.handoff_execution_loss import attachment_loss
import run_osa03_relative_factor_ablation01 as runner

PREP=ROOT/'data/osa03_relative_factor_ablation_01/primary_20260929T050000Z'


def test_pre_edit_full_golden_bit_exact():
    d=read(ROOT/'tests/fixtures/relative_ablation_full_golden.json');p=LocalSE2Problem(d['A'],d['B'],d['fresh'])
    np.testing.assert_array_equal(p.residual_vector(p.target),d['target_residual']);assert p.costs(p.target)==d['costs']
    r=solve_least_squares(p.fresh,p.residual_vector,SolverConfig())
    np.testing.assert_array_equal(r.optimized,d['optimized']);assert r.to_dict()==d['solver']


@pytest.mark.parametrize('n',[2,3,10,17])
def test_noR_vector_cost_diagnostic_immutable_frames(n):
    A=np.array([2.,-1.,3.1]);B=compose_poses(A,[.2,.03,.1]);t=np.linspace(0,1,n)**1.4
    raw=np.c_[t,.3*t*t,.2+.4*t];F=compose_poses(A,raw);before=F.copy();p=LocalSE2Problem(A,B,F)
    x=p.target.copy();x[-1,:2]+=.02
    blocks=p.residual_blocks(x);v=p.residual_vector(x,include_relative=False)
    np.testing.assert_array_equal(v,np.r_[blocks['L'].ravel(),blocks['A'].ravel()]);assert len(v)==6*n
    c=p.costs(x,include_relative=False);assert set(c)=={'L','A','total'}
    assert c['total']==c['L']+c['A'] and c['total']==pytest.approx(v@v)
    assert p.costs(x)['R']>0
    np.testing.assert_allclose(relative_pose(B,p.target),raw,atol=1e-14)
    np.testing.assert_allclose(compose_poses(A,p.original_observation_local(x)),x,atol=1e-14)
    assert not np.allclose(relative_pose(B,F),raw)
    np.testing.assert_array_equal(F,before);np.testing.assert_array_equal(p.fresh,before)
    assert not p.fresh.flags.writeable


def test_synthetic_noR_solve_diagnostic_and_safety_callback():
    F=np.array([[0.,0.,.2],[.2,.1,.4],[.8,.2,.5]])
    p=LocalSE2Problem([0,0,0],[.2,.02,.1],F);seen=[]
    def safe(x):seen.append(x.copy());return True
    r=solve_least_squares(F,lambda x:p.residual_vector(x,include_relative=False),SolverConfig(),candidate_feasibility_fn=safe)
    assert len(seen)>1 and 'R' not in p.costs(r.optimized,include_relative=False)
    assert p.costs(r.optimized)['R']>0
    np.testing.assert_array_equal(p.fresh,F)


def test_saved_schedule_authentication_and_tamper(tmp_path):
    cfg=runner.yaml.safe_load(runner.CONFIG.read_text());old=ROOT/cfg['common_B_run']
    s=load_schedule(old/'methods/M0_NATIVE/rollout.json',cfg['M0_rollout_sha256'],ROOT/cfg['tracked_common_result'],cfg['tracked_common_result_sha256'])
    assert s['pairs'][0]==dict(submit_tick=96,application_tick=99)
    assert s['primary']['applications'][-1]==dict(submit_tick=144,application_tick=145)
    q=tmp_path/'rollout.json';q.write_bytes((old/'methods/M0_NATIVE/rollout.json').read_bytes()+b' ')
    with pytest.raises(ValueError):load_schedule(q,cfg['M0_rollout_sha256'],ROOT/cfg['tracked_common_result'],cfg['tracked_common_result_sha256'])


class TinyWorker:
    def __init__(self,result,delay=3):self.result=result;self.delay=delay;self.calls=0
    def send(self,op,**kw):self.kw=kw
    def drain(self):
        self.calls+=1
        if self.calls<=self.delay:return []
        out=[dict(status='submitted',solve_id=self.kw['solve_id']),dict(self.result,solve_id=self.kw['solve_id'],input_state_id=self.kw['input_state_id'],input_pose=self.kw['pose'])]
        self.delay=10**6;return out


def test_release_never_early_and_wall_wait_cannot_advance_simulation():
    w=TinyWorker(dict(type='solve_result',status='command',official_generation=3,result_generation=3,command=[.8,1.]))
    q=LogicalRelease([dict(submit_tick=96,application_tick=99)],3);p=np.array([1.,2.,.3]);events=[]
    wait=q.collect(w,96,'solve',p,1.6,events)
    assert w.calls>=4 and wait['sim_before']==wait['sim_after']==1.6
    assert wait['pose_before']==wait['pose_after']==p.tolist()
    assert q.release(96) is None and q.release(98) is None
    assert q.release(99)['command']==[.8,1.]
    assert q.release(100) is None


@pytest.mark.parametrize('status,generation',[('stale_rejected',3),('command',4),('controller_error',3)])
def test_stale_generation_or_failure_never_released(status,generation):
    w=TinyWorker(dict(type='solve_result',status=status,official_generation=3,result_generation=generation),delay=0)
    q=LogicalRelease([dict(submit_tick=96,application_tick=99)],3)
    with pytest.raises((AssertionError,RuntimeError)):q.collect(w,96,'s',np.zeros(3),1.6,[])
    assert q.release(99) is None


def setup_mock(tmp_path,monkeypatch):
    if not PREP.exists():pytest.skip('saved R00 preflight absent')
    for name in ['protocol.json','common_state.json','source_manifest.json','metric_protocol.json','schedule.json']:
        shutil.copyfile(PREP/name,tmp_path/name)
    c=read(tmp_path/'common_state.json');base=read(PREP/'prepared_references.json')['M0_NATIVE'];refs={n:deepcopy(base) for n in ORDER}
    (tmp_path/'references').mkdir()
    shutil.copyfile(base['world_path'],tmp_path/'references/M0_NATIVE_world.npy')
    save(tmp_path/'references.json',refs);manifest=read(tmp_path/'source_manifest.json')
    class FakeWorker:
        def __init__(self,*args):self.queue=[];self.previous=c['u_mem_B'];self.world=None;self.delay=0
        def await_type(self,*args,**kw):return dict(provenance=manifest['mpc'])
        def drain(self):
            if self.delay:self.delay-=1;return []
            q=self.queue;self.queue=[];return q
        def send(self,op,**kw):
            assert op=='submit';p=kw['pose'];world=self.world
            residual=world-p;residual[:,2]=np.arctan2(np.sin(residual[:,2]),np.cos(residual[:,2]));near=np.argmin(np.sum(residual**2*np.array([10,10,1]),axis=1));idx=np.minimum(near+np.arange(1,6),len(world)-1)
            self.queue.append(dict(type='reply',status='submitted',previous_command=self.previous,**kw))
            e=deepcopy(c['first_FRESH_solve']);e.update(kw,input_pose=p,previous_command=self.previous,command=[0.,0.],official_solve_ms=0.,selection=selection_audit(world,p,world[idx],horizon=5,weights=[10,10,1]),official_generation=3,result_generation=3)
            e.pop('pose',None);self.previous=e['command'];self.queue.append(e);self.delay=2
        def shutdown(self):pass
    def ask(w,op,**kw):
        assert op=='initialize_common_B';w.world=np.load(kw['reference']['world_path'])
        return dict(B=c['B'],held_command=c['u_B_plus'],previous_control=c['u_mem_B'],generation=3,next_submit_tick=96,B_tick=92,B_sim_s=c['B_sim_s'],integration_dt_s=c['integration_dt_s'],installed_world=w.world.tolist())
    monkeypatch.setattr(runner,'Worker',FakeWorker);monkeypatch.setattr(runner,'ask',ask)
    return c,refs,manifest


def test_four_method_mock_runner_schedule_memory_validator_and_report(tmp_path,monkeypatch):
    c,refs,manifest=setup_mock(tmp_path,monkeypatch)
    from validate_osa03_common_b import validate_method
    rollouts={};metrics={};checks={}
    for name in ORDER:
        runner.run_method(tmp_path,name)
        r=read(tmp_path/'methods'/name/'rollout.json');assert r['error'] is None
        rollouts[name]=r;assert len(r['commands'])==180 and r['new_MPC_solved']==30
        previous=c['u_mem_B']
        for e in r['events']:
            if e.get('status')=='submitted':assert e['previous_command']==previous
            if e.get('type')=='solve_result':previous=e['command']
        _,m,a=validate_method(tmp_path,name,refs[name],c,np.load(refs[name]['world_path']),runner.geometry(Path(manifest['source']))['on'],read(Path(manifest['source'])/'scenario.json'),manifest['mpc']['official_settings'])
        assert a['valid'];metrics[name]=m
    gate=comparability(rollouts,read(tmp_path/'schedule.json'),c);assert gate['comparable'] and gate['all_four_observed']
    rollouts['NO_RELATIVE']['submit_requests'][0]['tick']+=1
    assert not comparability(rollouts,read(tmp_path/'schedule.json'),c)['comparable']
    # Saved-only plotting smoke test. These mock-controller curves are not evidence.
    import validate_osa03_relative_factor_ablation01 as validator
    import report_osa03_relative_factor_ablation01 as reporter
    rows={n:metric_row(m,'OBSERVATION_CAP') for n,m in metrics.items()}
    summary=dict(classification='SYNTHETIC_TEST_ONLY',primary_metrics=rows,signed_method_minus_native=gaps(rows,'M0_NATIVE'),factor_costs={},secondary_own_reference_metrics={n:{'position_auc_03_m_s':r['position_auc_03_m_s']} for n,r in rows.items()})
    save(tmp_path/'summary.json',summary);save(tmp_path/'validation.json',{'valid':True});save(tmp_path/'freeze.json',{'synthetic':True})
    monkeypatch.setattr(validator,'validate',lambda _: (summary,{'valid':True}))
    reporter.report(tmp_path);assert reporter.validate_figures(tmp_path)['valid']


def test_reference_gate_and_execution_abort_preserve_prefix(tmp_path,monkeypatch):
    setup_mock(tmp_path,monkeypatch)
    assert not may_execute(dict(clearance_valid=False))
    original=runner.guard_check;count=[]
    def guard(*args):
        g=original(*args);count.append(1)
        if len(count)==4:g['safe']=False
        return g
    monkeypatch.setattr(runner,'guard_check',guard);runner.run_method(tmp_path,'NO_RELATIVE')
    r=read(tmp_path/'methods/NO_RELATIVE/rollout.json')
    assert r['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND' and len(r['commands'])==3 and len(r['states'])==4
    assert not r['safety_abort']['command_applied']


def test_primary_attachment_unchanged_own_reference_separate():
    f=np.array([[0,0,np.pi-.01],[1,0,-np.pi+.01]]);t=np.arange(61)/60;p=np.c_[t,np.zeros(61),np.full(61,-np.pi)]
    primary=attachment_loss(t,p,f);own=f.copy();own[:,1]=.3
    secondary=attachment_loss(t,p,own)
    assert primary['join_time_s']==0 and secondary['join_time_s'] is None
    assert attachment_loss(t,p,f)['position_auc_m_s']==primary['position_auc_m_s']
    p[:,-1]=0;assert attachment_loss(t,p,f)['join_time_s'] is None


def test_validator_and_report_have_no_new_scientific_calls():
    for path in ['scripts/validate_osa03_relative_factor_ablation01.py','scripts/report_osa03_relative_factor_ablation01.py']:
        tree=ast.parse((ROOT/path).read_text())
        names=[n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)]
        assert not set(names)&{'solve_least_squares','run_method','execute','Worker','submit','load_official'}


def test_missed_tick_and_double_submit_fail_closed():
    q=LogicalRelease([dict(submit_tick=96,application_tick=99)],3)
    q.pending=(99,{'command':[.8,1.]})
    with pytest.raises(RuntimeError,match='missed'):q.release(100)
    with pytest.raises(RuntimeError,match='unreleased'):q.collect(None,96,'s',np.zeros(3),1.6,[])


def test_source_planning_descriptor_no_connector():
    f=np.array([[0.,0.,0.],[.1,.1,.2],[.7,.4,.5]])
    d=planning(f,f,[0.,-.2,0.])
    assert not d['self_intersection'] and d['first_node_displacement_m']==0
    assert d['B_to_first_node_position_m']==pytest.approx(.2)
    assert d['total_XY_arc_m']==pytest.approx(np.linalg.norm(np.diff(f[:,:2],axis=0),axis=1).sum())


def test_waited_solve_count_retained_when_abort_precedes_release(tmp_path,monkeypatch):
    setup_mock(tmp_path,monkeypatch);original=runner.guard_check;count=[]
    def guard(*args):
        g=original(*args);count.append(1)
        if len(count)==6:g['safe']=False  # tick97, between submit96 and release99
        return g
    monkeypatch.setattr(runner,'guard_check',guard);runner.run_method(tmp_path,'NO_RELATIVE')
    r=read(tmp_path/'methods/NO_RELATIVE/rollout.json')
    assert len(r['commands'])==5 and r['new_MPC_solved']==1
    assert r['events'][-1]['withheld_by_logical_scheduler']
    assert all(c['solve_id']==r['phase']['first_FRESH_solve']['solve_id'] for c in r['commands'])


def test_saved_planning_validator_on_synthetic_solve(tmp_path):
    from types import SimpleNamespace
    from validate_osa03_relative_factor_ablation01 import validate_planning
    f=np.array([[0.,0.,.1],[.2,.1,.3],[.8,.2,.4]])
    p=LocalSE2Problem([0,0,0],[.2,.02,.1],f);trace=[];checks=[]
    env=SimpleNamespace(check_polyline=lambda x:{'clearance_valid':True})
    def feasible(x):checks.append(dict(state=x.tolist(),check=env.check_polyline(x)));return True
    def observe(e):trace.append(runner.plain(dict(e,factor_costs=p.costs(e['state'],include_relative=False))))
    r=solve_least_squares(f,lambda x:p.residual_vector(x,include_relative=False),SolverConfig(),candidate_feasibility_fn=feasible,iteration_callback=observe)
    save(tmp_path/'solver_trace.json',trace);save(tmp_path/'feasibility_checks.json',checks)
    save(tmp_path/'planning_result.json',dict(final=p.costs(r.optimized,include_relative=False),solver=r.to_dict(),diagnostic_relative_edge_distortion_not_optimized_cost=p.costs(r.optimized)['R']))
    assert validate_planning(tmp_path,p,r.optimized,{'on':env})['valid']
