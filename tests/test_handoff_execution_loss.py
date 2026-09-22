"""Synthetic checker tests only. No new model/controller/optimizer execution."""
import ast
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.handoff_execution_loss import attachment_loss,command_loss
from reconciliation.robotless_online import integrate_unicycle

ROOT=Path(__file__).resolve().parents[1]


def test_transient_crossing_is_not_sustained_and_NA_is_not_horizon():
    t=np.arange(11)*.1;poses=np.column_stack([t,np.full(11,.2),np.zeros(11)])
    poses[3,1]=0
    result=attachment_loss(t,poses,[[0,0,0],[2,0,0]])
    assert not result['join_success'] and result['join_time_s'] is None
    assert result['pre_join_position_auc_m_s'] is None
    assert result['position_auc_m_s']>0


def test_fixed_B_gap_and_post_boundary_cost_are_distinct():
    t=np.arange(11)*.1;poses=np.column_stack([t,np.r_[.2,.15,np.zeros(9)],np.zeros(11)])
    result=attachment_loss(t,poses,[[0,0,0],[2,0,0]])
    assert result['B_distance_m']==.2
    assert result['join_time_s']==pytest.approx(.2)
    assert result['position_auc_m_s']==pytest.approx(.025)
    assert result['position_excess_auc_m_s']==pytest.approx(.01)


def test_late_entry_without_complete_dwell_is_censored():
    t=np.arange(6)*.1;poses=np.column_stack([t,[.2,.2,.2,0,0,0],np.zeros(6)])
    result=attachment_loss(t,poses,[[0,0,0],[1,0,0]])
    assert result['join_time_s'] is None
    assert result['latest_testable_join_start_s']==pytest.approx(.2)
    assert result['observation_status']=='TUBE_ENTERED_DWELL_RIGHT_CENSORED'
    assert result['longest_observed_inside_span_s']==pytest.approx(.2)


def test_exit_after_observed_dwell_is_retained():
    t=np.arange(8)*.1;poses=np.column_stack([t,[.2,0,0,0,0,.2,.2,.2],np.zeros(8)])
    result=attachment_loss(t,poses,[[0,0,0],[2,0,0]])
    assert result['join_time_s']==pytest.approx(.1)
    assert result['exits_after_first_join'] is True


def test_transient_entry_and_never_entry_are_separate():
    t=np.arange(6)*.1;poses=np.column_stack([t,[.2,.2,0,.2,.2,.2],np.zeros(6)])
    assert attachment_loss(t,poses,[[0,0,0],[1,0,0]])['observation_status']=='TRANSIENT_ENTRY_THEN_EXIT'
    poses[:,1]=.2
    assert attachment_loss(t,poses,[[0,0,0],[1,0,0]])['observation_status']=='NO_TUBE_ENTRY_OBSERVED'


@pytest.mark.parametrize('n',[2,10,17])
def test_original_fresh_unchanged_generic_N_and_forward_progress(n):
    fresh=np.column_stack([np.linspace(0,1,n),np.zeros(n),np.full(n,np.pi-.01)])
    initial=fresh.copy();t=np.arange(7)*.1
    poses=np.column_stack([[.2,.3,.4,.35,.3,.5,.6],np.zeros(7),np.full(7,-np.pi+.01)])
    result=attachment_loss(t,poses,fresh)
    np.testing.assert_array_equal(fresh,initial)
    assert result['progress_monotonic']
    assert result['B_yaw_error_deg']==pytest.approx(np.degrees(.02))


def test_recorded_command_reconstruction_TV_and_nominal_grid_semantics():
    config=yaml.safe_load((ROOT/'configs/saved_handoff_execution_loss.yaml').read_text())
    t=np.arange(13)/60;u=np.array([[.4,.5]]*6+[[.6,1.]]*6)
    poses=[[0.,0.,0.]]
    for command,dt in zip(u,np.diff(t)):poses.append(integrate_unicycle(poses[-1],command,dt))
    r=command_loss(t,poses,u,[.2,0.],config)
    assert r['linear_TV_mps']==pytest.approx(.4)
    assert r['angular_TV_radps']==pytest.approx(1.)
    assert r['nominal_10Hz_max_delta_v_over_dt_mps2']==pytest.approx(2.)
    assert r['nominal_10Hz_max_delta_omega_over_dt_radps2']==pytest.approx(5.)
    assert r['reconstruction_max_error']<1e-12
    assert r['nominal_command_grid_valid']
    poses[-1][0]+=.01
    with pytest.raises(ValueError,match='reconstruct'):command_loss(t,poses,u,[.2,0.],config)


def test_no_solver_or_new_rollout_calls():
    names=set()
    for p in ['src/reconciliation/handoff_execution_loss.py','scripts/audit_saved_handoff_losses.py']:
        for node in ast.walk(ast.parse((ROOT/p).read_text())):
            if isinstance(node,ast.Call):
                if isinstance(node.func,ast.Name):names.add(node.func.id)
                elif isinstance(node.func,ast.Attribute):names.add(node.func.attr)
    assert not names.intersection({'minimize','solve_gp','solve_rigid','counterfactual_rollout','MpcTracker','Session','SimulationApp'})
