"""Synthetic fixtures test audit logic only; never scientific evidence."""
from copy import deepcopy
import numpy as np
import pytest

from reconciliation.project_audit import (shape_features, saved_window, match_controls,
    execution_metrics, summarize_events)


def test_shape_yaw_wrap_and_rotation_only_rows():
    fresh=np.array([[0.,0.,np.pi-.1],[0.,0.,-np.pi+.1],[1.,0.,-np.pi+.1]])
    before=fresh.copy();r=shape_features(fresh)
    assert r['raw_yaw_variation_deg']==pytest.approx(np.degrees(.2))
    assert r['raw_arc_m']==1 and r['tangent_segments']==1
    assert r['raw_tangent_variation_deg']==0
    np.testing.assert_array_equal(fresh,before)


def test_shape_intrinsic_turn_not_boundary_orientation():
    f=np.array([[0.,0.,0.],[1.,0.,0.],[1.,1.,np.pi/2]])
    assert shape_features(f)['raw_tangent_variation_deg']==pytest.approx(90)
    theta=.7;R=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    g=f.copy();g[:,:2]=f[:,:2]@R.T+[8,-3];g[:,2]+=theta
    for k in ('raw_arc_m','raw_yaw_variation_deg','raw_tangent_variation_deg'):
        assert shape_features(f)[k]==pytest.approx(shape_features(g)[k])


@pytest.mark.parametrize('bad',[[],[[1,2]],[[1,2,float('nan')]]])
def test_bad_reference(bad):
    with pytest.raises(ValueError):shape_features(bad)


def streams():
    states=[dict(state_id=i,sim_time_s=i*.1,x=i*.01,y=0.,yaw=0.,incoming_command_id=i-1) for i in range(5)]
    commands=[dict(command_id=i,application_state_id=i,chunk_id='new' if 1<=i<3 else 'other',v_mps=.1,omega_radps=0.) for i in range(4)]
    context=dict(switch_state_id=1,fresh_chunk_id='new',B=[.01,0.,0.],pre_switch_command=dict(v_mps=.1,omega_radps=0.))
    return context,states,commands


def test_next_command_excluded_endpoint_state_included():
    c,s,u=streams();w=saved_window(c,s,u)
    assert [x['command_id'] for x in w['commands']]==[1,2]
    assert w['end_reason']=='NEXT_REFERENCE_APPLICATION'
    assert w['times'][-1]==pytest.approx(.2)
    assert w['poses'][-1,0]==.03


def test_terminal_censoring_not_invented_next_switch():
    c,s,u=streams();u[3]['chunk_id']='new'
    w=saved_window(c,s,u)
    assert w['end_reason']=='EPISODE_END' and w['next_chunk_id'] is None


def test_missing_or_wrong_incoming_command_rejected():
    c,s,u=streams()
    with pytest.raises(ValueError):saved_window(c,s,u[:2])
    s[2]['incoming_command_id']=99
    with pytest.raises(ValueError):saved_window(c,s,u)


def matching_fixture():
    import yaml
    from pathlib import Path
    protocol=yaml.safe_load((Path(__file__).parents[1]/'configs/project_audit_native_lightnav.yaml').read_text())['matching']
    base=dict(case_id='hard',episode_id='ep0',category='turn',source_safe=True,lateral_m=.2,
        pose_yaw_deg=30.,direction_deg=30.,raw_arc_m=1.,raw_yaw_variation_deg=60.,
        raw_tangent_variation_deg=50.,v_minus_mps=.6,lifetime_s=1.2)
    control=dict(base,case_id='control',episode_id='ep1',lateral_m=.01,pose_yaw_deg=1.,direction_deg=1.)
    return protocol,base,control


def test_matching_outcome_independent_and_ties_stable():
    protocol,h,c=matching_fixture();d=dict(c,case_id='z')
    a=match_controls([h,d,c],['hard'],protocol)
    c['position_auc']=999;d['position_auc']=0
    assert match_controls([c,h,d],['hard'],protocol)==a
    assert a[0]['control_id']=='control'


@pytest.mark.parametrize('change',[dict(episode_id='ep0'),dict(category='straight'),dict(direction_deg=None),dict(lateral_m=.050001),dict(raw_yaw_variation_deg=81.),dict(v_minus_mps=.701),dict(lifetime_s=1.451),dict(source_safe=False)])
def test_no_matching_relaxation(change):
    protocol,h,c=matching_fixture();c.update(change)
    assert match_controls([h,c],['hard'],protocol)[0]['control_id'] is None


def test_short_reference_lifetime_not_padded_or_called_three_second_failure():
    from pathlib import Path
    import yaml
    root=Path(__file__).parents[1];cfg=yaml.safe_load((root/'configs/saved_handoff_execution_loss.yaml').read_text())
    acfg=yaml.safe_load((root/'configs/project_audit_native_lightnav.yaml').read_text())['attachment']
    c,s,u=streams();w=saved_window(c,s,u);fresh=np.array([[0,0,0],[1,0,0]],float)
    r=execution_metrics(w,fresh,fresh,c,cfg,acfg)
    assert r['common'] is None
    assert r['full']['observation_status']=='TUBE_ENTERED_DWELL_RIGHT_CENSORED'
    assert r['full']['join_time_s'] is None
    assert r['boundary_reference_distance_jump_m']==0
    assert r['command']['reconstruction_max_error']<1e-12


def test_episode_weighting_not_IID_event_pool():
    def record(ep,auc):
        return dict(episode_id=ep,common=dict(position_auc_m_s=auc),B_inside_joint_tube=True,growth_observed=False,
            full=dict(observation_status='OBSERVED_SAMPLED_JOIN',join_success=True))
    result=summarize_events([record('a',1.),record('a',1.),record('b',4.)])
    assert result['event_mean_position_auc_m_s']==2.
    assert result['episode_mean_position_auc_m_s']==2.5


def test_frozen_tube_parameters_match_unchanged_implementation():
    import yaml
    from pathlib import Path
    from reconciliation.gp_se2_join01 import POSITION_M, YAW_RAD, DWELL_S
    cfg=yaml.safe_load((Path(__file__).parents[1]/'configs/project_audit_native_lightnav.yaml').read_text())['attachment']
    assert cfg['position_m']==POSITION_M
    assert np.radians(cfg['yaw_deg'])==YAW_RAD
    assert cfg['dwell_s']==DWELL_S


def test_report_tables_and_exclusive_output(tmp_path):
    import importlib.util
    from pathlib import Path
    path=Path(__file__).parents[1]/'scripts/report_project_native_lightnav.py'
    spec=importlib.util.spec_from_file_location('audit_report_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    tables=module.document_tables(module.DOCUMENT.read_text())
    assert len(tables['q1_root_cause_matrix'])==17
    assert len(tables['q2_contribution_matrix'])==8
    (tmp_path/'index.html').write_text('preserved')
    with pytest.raises(FileExistsError):module.Report(tmp_path)
    assert (tmp_path/'index.html').read_text()=='preserved'


def test_audit_scripts_have_no_scientific_call_sites():
    import ast
    from pathlib import Path
    forbidden={'minimize','solve_gp','solve_rigid','counterfactual_rollout','historical_solve_audit',
               'execute_reference','load_official','create_subprocess_exec','Popen','SimulationApp'}
    root=Path(__file__).parents[1]
    for name in ('audit_project_native_lightnav.py','report_project_native_lightnav.py',
                 'validate_project_native_lightnav.py'):
        tree=ast.parse((root/'scripts'/name).read_text())
        calls={node.func.id if isinstance(node.func,ast.Name) else node.func.attr
               for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,(ast.Name,ast.Attribute))}
        assert not calls.intersection(forbidden)
