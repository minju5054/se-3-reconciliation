"""Synthetic implementation checks only; no actual model/controller calls."""
import ast
from pathlib import Path

import numpy as np
import pytest
import yaml
from reconciliation.genuine_source_scan import mismatch, motion_record, subsets, order_key

ROOT=Path(__file__).resolve().parents[1]
T=yaml.safe_load((ROOT/'configs/genuine_source_moving_mismatch_scan.yaml').read_text())['thresholds']
IN=dict(available=True,chord_m=.1,phi_rad=0.)


def test_endpoint_longitudinal_distance_does_not_masquerade_as_lateral():
    r=mismatch([-.3,0,0],[[0,0,0],[1,0,0]],IN,T)
    assert r['original_distance_mismatch'] and not r['strict_mismatch']
    assert r['signed_lateral_gap_m']==0 and r['along_tangent_gap_m']==-.3


def test_lateral_and_periodic_yaw():
    r=mismatch([.3,.2,2*np.pi],[[0,0,0],[1,0,0]],IN,T)
    assert r['lateral_mismatch'] and r['strict_mismatch'] and not r['pose_yaw_mismatch']
    assert r['projection']['projection_location']=='interior'


def test_direction_needs_actual_translation_chord():
    r=mismatch([.3,0,0],[[0,0,0],[1,0,0]],dict(available=True,chord_m=.001,phi_rad=np.pi/2),T)
    assert r['reliable_window_direction_deg'] is None and not r['direction_mismatch']


def test_observation_boundary_and_physical_command_reconstruction():
    poses=[[0,0,0],[.05,0,0],[.1,0,0]]
    states=[dict(state_id=str(i),sim_time_s=str(i*.1),x=str(p[0]),y='0',yaw='0',incoming_command_id=str(i)) for i,p in enumerate(poses)]
    commands={i:dict(command_id=str(i),v_mps='.5',omega_radps='0') for i in range(3)}
    c=dict(obs_state_id=0,switch_state_id=2,R_obs=poses[0],B=poses[2],pre_switch_command=dict(command_id=2,v_mps=.5,omega_radps=0),first_fresh_solve=dict(previous_command=[.4,.1]))
    r=motion_record(c,states,commands)
    assert r['observation_to_B_arc_m']==pytest.approx(.1) and r['v_minus_mps']==.5
    assert r['previous_control']==[.4,.1] and r['reconstruction_max_abs_error']<1e-12
    c['B']=[.2,0,0]
    with pytest.raises(ValueError,match='pose mismatch'):motion_record(c,states,commands)


def row():
    return dict(source_predicates={k:True for k in ('original_source_eligibility','boundary_clearance','enough_future','unambiguous_projection','exact_recorded_timing','physical_state_provenance','raw_future_clearance','common_future_clearance','no_required_gate_and_known_route')},
        motion=dict(v_minus_mps=.5,observation_to_B_arc_m=.1),entire_raw_environment=dict(clearance_valid=True),past_environment=dict(clearance_valid=True),
        mismatch=dict(strict_mismatch=True,lateral_mismatch=True,position_AND_orientation_mismatch=False,original_distance_mismatch=True),raw_suffix_clearance_m=.1)


def test_moving_label_cannot_replace_physical_speed():
    r=row();r['motion']['v_minus_mps']=1e-9
    assert not subsets(r,T)['candidate']
    r['motion']['v_minus_mps']=.20
    assert not subsets(r,T)['actual_moving']


def test_full_raw_unsafe_is_not_promoted_by_suffix():
    r=row();r['entire_raw_environment']['clearance_valid']=False
    flags=subsets(r,T)
    assert not flags['candidate'] and flags['future_only_candidate']


def test_obstacle_near_and_gate_are_separate_subsets():
    r=row();r['raw_suffix_clearance_m']=1
    flags=subsets(r,T)
    assert flags['candidate'] and not flags['candidate_obstacle_sensitive']
    r['raw_suffix_clearance_m']=.1;r['source_predicates']['no_required_gate_and_known_route']=False
    assert subsets(r,T)['candidate_obstacle_sensitive'] and not subsets(r,T)['candidate_obstacle_sensitive_no_gate']


def test_order_does_not_read_performance():
    a={'case_id':'episode_1/handoff_2'}
    assert order_key(a)==order_key(dict(a,optimizer_success=False,MPC_outcome=True))


def test_no_new_solve_or_rollout_in_audit():
    tree=ast.parse((ROOT/'src/reconciliation/genuine_source_scan.py').read_text())
    called={n.func.id for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
    assert not called.intersection({'MpcTracker','GPProblem','solve_gp','minimize','counterfactual_rollout','Session'})


def test_source_array_is_not_modified_and_singleton_is_explicit():
    raw=np.array([[1.,0.,0.]])
    original=raw.copy()
    result=mismatch([0.,0.,0.],raw,IN,T)
    assert np.array_equal(raw,original)
    assert result['signed_lateral_gap_m'] is None
    assert result['projection']['projection_location']=='singleton'


def test_runner_refuses_existing_output_before_reading_source(tmp_path):
    import subprocess
    import sys
    out=tmp_path/'existing';out.mkdir();marker=out/'raw';marker.write_text('unchanged')
    result=subprocess.run([sys.executable,str(ROOT/'scripts/scan_genuine_moving_sources.py'),'--output',str(out)],capture_output=True,text=True)
    assert result.returncode!=0 and 'FileExistsError' in result.stderr
    assert marker.read_text()=='unchanged' and list(out.iterdir())==[marker]
