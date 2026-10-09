"""Synthetic severity/metric tests; source authentication never ranks real handoffs."""
from copy import deepcopy
import ast
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts')]
import audit_successive_handoff_severity01 as audit
from reconciliation.successive_handoff_severity01 import *
from reconciliation.join_source03 import read,sha
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.continuous_obstacle_reveal_episode01 import evolution
from reconciliation.continuous_obstacle_reveal_exploratory02 import extra_evolution
RUN=ROOT/'data/successive_post_reveal_handoff_severity_audit_01/primary_20261009'


@pytest.fixture(scope='module')
def inputs():return audit.load_inputs()


def test_exact_source_result_index_and_ten_post_reveal(inputs):
    cfg,source,auth,b=inputs
    assert auth['valid'] and auth['sealed_file_count']==401 and auth['episode_input_count']==297
    assert auth['exact_post_reveal_handoffs']==HANDOFFS and len(HANDOFFS)==10 and 'C0_to_C1' not in HANDOFFS
    assert auth['handoff_index_sha256']==sha(source/'source_bundle/handoff_index.json')
    for name in HANDOFFS:
        q=b[name];assert q['ready']['fresh_chunk_id']==f"chunk_{q['index']+1:03d}"
        assert q['times'][0]==0 and q['end_tick']>q['B_tick']


def test_exact_stable_controls(inputs):
    stable=[n for n,b in inputs[3].items() if b['evolution']['raw_hash_equal']]
    assert stable==CONTROLS
    for n in stable:
        a=inputs[3][n]['arrays'];assert a['old_raw_local'].tobytes()==a['fresh_raw_local'].tobytes()
        assert a['old_world'].tobytes()!=a['fresh_world'].tobytes()


def test_authentication_tamper_fails(tmp_path):
    p=tmp_path/'file';p.write_text('one');h=sha(p);p.write_text('two')
    with pytest.raises(ValueError):audit.authenticate_files({p:h})


def test_full_real_result_requires_post_push_marker(tmp_path):
    with pytest.raises(AssertionError,match='pushed freeze'):audit.compute(tmp_path)


def test_C0_sanity_exact_historical_C3_only(inputs):
    b=inputs[3]['C0_to_C1'];a=b['arrays'];r=b['ready']
    from reconciliation.spatial_entry_suffix import suffix_reference
    _,_,e=suffix_reference(a['fresh_world'],a['fresh_raw_local'],r['A_fresh'],r['B'])
    assert e==read(ROOT/'results/successive_isaac_four_method_comparison_01/freeze_manifest.json')['entry']
    assert e['correspondence']['arc_m']==0


class Safety:
    def __init__(self):self.paths=[]
    def check_polyline(self,p,**kw):
        assert kw==dict(radius=.20,required_clearance=.05)
        self.paths.append(np.asarray(p).copy())
        return dict(minimum_clearance_m=.12,physical_overlap=False,legacy_5cm_margin_pass=True,clearance_valid=True)


def fixture():
    A=np.array([3.,2.,.2]);raw=np.array([[0,0,0],[1,0,.2],[2,1,.6],[2,2,1.]])
    F=local_trajectory_to_world(A,raw);B=local_trajectory_to_world(A,[[.5,0,.1]])[0];P=B-[.1,0,0]
    return F,raw,A,B,P


def test_C3_progress_gap_suffix_identity_and_E_local_turn():
    F,raw,A,B,P=fixture();before=(F.tobytes(),raw.tobytes());env=Safety()
    row,d=geometry_descriptors(F,raw,A,B,P,env)
    c=SpatialCurve(F).correspondences(B)['C3_FORWARD_SE2']
    assert d['entry']['correspondence']==c
    assert row['E_star_arc_m']==c['arc_m'] and row['E_star_alpha']==c['alpha']
    assert row['removed_prefix_fraction']==c['arc_m']/c['total_arc_m']
    np.testing.assert_allclose(row['B_to_E_gap_m'],np.linalg.norm(B[:2]-np.array(row['E_star_world'])[:2]))
    S=np.array(d['suffix_world']);E=np.array(row['E_star_world']);local=relative_pose(E,S)
    np.testing.assert_array_equal(local,d['suffix_E_local'])
    assert row['fresh_suffix_final_lateral_m']==local[-1,1]
    assert row['fresh_suffix_abs_net_yaw_rad']==abs(wrap_angle(F[-1,2]-E[2]))
    for j,i in enumerate(d['entry']['row_original_identities']):
        if i is not None:assert S[j].tobytes()==F[i].tobytes()
    assert before==(F.tobytes(),raw.tobytes()) and len(env.paths)==3
    np.testing.assert_array_equal(env.paths[-1],np.vstack([B,E]))


def test_P_B_direction_wrap_and_invalid_fail_closed():
    angle=np.deg2rad(179);B=np.zeros(3);P=-np.r_[np.cos(angle),np.sin(angle),0]
    tangent,reason=incoming_tangent(P,B)
    assert reason is None and np.isclose(wrap_angle(np.deg2rad(-179)-tangent),np.deg2rad(2))
    assert incoming_tangent(B,B)[0] is None
    assert 'no B-yaw' in incoming_tangent(B,B)[1]


def test_existing_evolution_definitions_unchanged(inputs):
    # Authentication/parity of existing descriptors, not new severity or ranking.
    declaration=read(inputs[1]/'protocol.json')['declaration']
    for b in inputs[3].values():
        a=b['arrays'];e=b['evolution'];r=b['ready']
        p=evolution(a['old_raw_local'],a['fresh_raw_local'],declaration['evolution'])
        assert all(p[k]==e[k] for k in p)
        pair=[dict(A=r['A_'+kind],max_yaw_deg=float(np.degrees(abs(wrap_angle(a[kind+'_raw_local'][:,2])).max()))) for kind in ['old','fresh']]
        extra=extra_evolution(*pair);assert all(extra[k]==e[k] for k in extra)
    x=extra_evolution(dict(A=[0,0,0],max_yaw_deg=30),dict(A=[0,0,0],max_yaw_deg=10))
    assert x['max_absolute_local_yaw_difference_deg']==-20 # historical signed difference, not pairwise maximum


@pytest.mark.parametrize('sign',[1,-1])
def test_turn50_left_right_and_two_samples(sign):
    t=np.arange(5)*.1;y=sign*np.array([0,.4,.6,.7,.8])
    r=turn50(t,y,0,sign*1.,.4)
    assert r['status']=='OBSERVED' and r['time_s']==.2 and np.isclose(r['confirmation_time_s'],.3)


def test_turn50_wrap_across_pi():
    B=np.deg2rad(179);target=np.deg2rad(-177)
    yaw=wrap_angle(B+np.deg2rad([0,1,2.5,3]))
    r=turn50([0,.1,.2,.3],yaw,B,target,.3)
    assert r['status']=='OBSERVED' and r['time_s']==.2


@pytest.mark.parametrize('yaw',[ [0,.1,.2], [0,.6,.2], [0,.1,.6] ])
def test_turn50_not_reached_or_unconfirmed_is_null(yaw):
    r=turn50([0,.1,.2],yaw,0,1.,.2)
    assert r['time_s'] is None and r['status']=='CENSORED_NOT_REACHED'


def test_turn50_near_zero_and_incomplete():
    assert turn50([0,.1],[0,1],0,EPS,.1)['status']=='N/A_NEAR_ZERO_TURN'
    assert turn50([0,.1],[0,1],0,1.,.3)['status']=='INCOMPLETE_WINDOW'
    assert turn50([0,.1],[0,1],0,0.,.1)['time_s'] is None


def test_native_AUC_parity_execution_time_and_censor():
    F=np.array([[0.,0,0],[2,0,0]]);t=np.array([0.,.1,.2,.4]);p=np.c_[t,np.ones(4)*.1,np.ones(4)*.2]
    m,d=native_context(F,t,p,p[0],.6)
    assert np.isclose(m['native_position_auc_03_m_s'],.03) and np.isclose(m['native_yaw_auc_03_rad_s'],.06)
    from reconciliation.successive_isaac_four_method01 import auc as historical_auc
    assert m['native_position_auc_03_m_s']==historical_auc(t,np.ones(4)*.1,.3)
    q,_=native_context(F,t[:2],p[:2],p[0],.6)
    assert q['native_position_auc_03_m_s'] is None and q['native_T_turn50_s'] is None


def rows():
    out=[]
    for i in range(1,11):
        r=dict(handoff=HANDOFFS[i-1],handoff_index=i,evolution='STABLE' if HANDOFFS[i-1] in CONTROLS else 'EVOLVING')
        r.update({key:float(i) for key in DIMENSIONS.values()});out.append(r)
    return out


def test_native_and_method_outcomes_cannot_affect_selection():
    a=rows();expected=select_geometry(a);b=deepcopy(a)
    for r in b:r.update(native_position_auc_03_m_s=-1e200,native_T_turn50_s=999,Graph_result='better',safety_execution=False)
    assert select_geometry(b)==expected
    assert set(expected['selection_inputs'])==set(DIMENSIONS.values())|{'handoff_index'}
    assert 'C0_to_C1' not in str(expected) and not any(n in str(expected) for n in CONTROLS)


def test_deterministic_rank_tie_break_distinct_roles_pareto():
    a=rows();s=select_geometry(a)
    assert s['recommended']==dict(TURN='C9_to_C10',PROGRESS='C7_to_C8',REVISION='C6_to_C7')
    assert s['pareto_hard']==['C9_to_C10']
    for r in a:r.update({key:1. for key in DIMENSIONS.values()})
    tied=select_geometry(a);assert tied['top3']['turn']==HANDOFFS[:3]
    a[1]['B_to_E_gap_m']=2.;assert select_geometry(a)['recommended']['TURN']==HANDOFFS[1]
    a[0]['B_to_E_gap_m']=2.;a[0]['fresh_suffix_abs_net_yaw_rad']=3.
    assert select_geometry(a)['recommended']['TURN']==HANDOFFS[0]
    a[0]['abs_transition_turn_demand_rad']=None
    with pytest.raises(ValueError,match='TECHNICAL_BLOCKED'):select_geometry(a)


@pytest.mark.parametrize('name',['solve_least_squares','hermite_bridge','staged_reference','SimulationApp','load_official','capture_rgb','generate','integrate_unicycle'])
def test_no_scientific_calls_guard(name):
    ns={};exec('def '+name+'():\n    raise AssertionError("body must not execute")',ns)
    with pytest.raises(RuntimeError):
        with saved_only_guard():ns[name]()


def test_no_solver_runtime_calls_or_scalar_score_in_audit_AST():
    tree=ast.parse((ROOT/'scripts/audit_successive_handoff_severity01.py').read_text())
    names={x.func.id for x in ast.walk(tree) if isinstance(x,ast.Call) and isinstance(x.func,ast.Name)}
    assert not names&{'solve_least_squares','hermite_bridge','staged_reference','SimulationApp','Worker','integrate_unicycle','load_official'}
    import inspect
    text=inspect.getsource(select_geometry)
    assert 'sum(' not in text and 'mean(' not in text
    accessed=[n.slice.value for n in ast.walk(ast.parse(text)) if isinstance(n,ast.Subscript) and isinstance(n.slice,ast.Constant)]
    assert not any(str(k).startswith('native') for k in accessed)


def test_two_geometry_only_PNGs(tmp_path):
    from shapely.geometry import box
    from report_successive_handoff_severity01 import draw
    F,raw,A,B,P=fixture();geom,d=geometry_descriptors(F,raw,A,B,P,Safety());a=rows();selection=select_geometry(a);details={}
    for row in a:
        row.update({k:geom[k] for k in ['E_star_world','B_to_E_gap_m','abs_transition_turn_demand_rad','removed_prefix_fraction','fresh_suffix_abs_net_yaw_rad']})
        row['candidate_role']=next((k for k,n in selection['recommended'].items() if n==row['handoff']),'STABLE_CONTROL' if row['handoff'] in CONTROLS else None)
        details[row['handoff']]=dict(FRESH_world=F.tolist(),B=B.tolist(),P=P.tolist())
    s=dict(rows=a,selection=selection,stable_controls=CONTROLS)
    m=draw(s,details,box(1,0,2,1),tmp_path/'figures')
    assert m['geometry_only'] and m['figure_count']==2
    assert sorted(p.name for p in (tmp_path/'figures').glob('*.png'))==sorted(PNGS)


def test_failed_saved_audit_records_blocker_and_cannot_retry(tmp_path,monkeypatch):
    from reconciliation.join_source03 import save
    save(tmp_path/'protocol.json',{'budget':{'Isaac':0,'Graph':0,'MPC':0}})
    monkeypatch.setattr(audit,'OUT',tmp_path/'out')
    monkeypatch.setattr(audit,'verify',lambda *a:None)
    monkeypatch.setattr(audit,'git',lambda *a:'SYNTHETIC_TEST')
    def fail(*a):raise ValueError('synthetic invalid descriptor')
    monkeypatch.setattr(audit,'compute',fail)
    with pytest.raises(ValueError):audit.run_audit(tmp_path)
    assert read(tmp_path/'out/technical_blocker.json')['classification']=='TECHNICAL_BLOCKED'
    with pytest.raises(FileExistsError):audit.run_audit(tmp_path)
