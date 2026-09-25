"""Saved-only interfaces and geometry fixtures; never calls a scientific solve."""
import ast
from pathlib import Path
import sys
import numpy as np
import pytest
from shapely.geometry import box
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_local_se2_reconciliation_formulation01 import save,sha,verify_hashes,config_contract,independent_clearance,CONFIG,read
from validate_local_se2_reconciliation_formulation01 import independent_costs,parity
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.local_se2_reconciliation import LocalSE2Problem
import yaml


def test_hash_corruption_and_exclusive_write(tmp_path):
    p=tmp_path/'raw.json'; save(p,{'raw':[1,2]}); h={str(p):sha(p)}; verify_hashes(h)
    with pytest.raises(FileExistsError):save(p,{'raw':[3]})
    p.write_text('corrupt')
    with pytest.raises(ValueError,match='hash mismatch'):verify_hashes(h)


def test_full_segment_safety_no_connector_and_radius_once():
    cart=box(.45,-.1,.55,.1); hospital=box(5,5,6,6); workspace=box(-10,-10,10,10)
    base=HospitalEnvironment(hospital,workspace); on=HospitalEnvironment(hospital.union(cart),workspace)
    loaded=dict(base=base,on=on,cart=cart)
    unsafe=np.array([[0,0,0],[1,0,0.]])
    r=independent_clearance(unsafe,loaded)
    assert not r['clearance_valid'] and r['minimum_clearance_m']==pytest.approx(-.2)
    assert r['first_minimum_segment']==0 and 0<r['minimum_segment_fraction']<1
    safe=np.array([[0,.36,0],[1,.36,0]])
    r=independent_clearance(safe,loaded)
    assert r['minimum_clearance_m']==pytest.approx(.06) and r['clearance_valid']
    # B is intentionally not an argument: no inferred connector may be tested.
    assert 'B' not in independent_clearance.__code__.co_varnames


def test_config_no_search_and_independent_equations():
    cfg=yaml.safe_load(CONFIG.read_text()); config_contract(cfg)
    cfg['representative']='REPEAT_01'
    with pytest.raises(AssertionError):config_contract(cfg)
    p=LocalSE2Problem([0,0,0],[.2,0,.1],[[0,0,.2],[.4,.2,.3],[1,.3,.4]])
    parity(p.costs(p.target),independent_costs(p.fresh,p.target,p.target))


def test_new_scientific_entrypoint_has_one_solver_and_no_execution_calls():
    root=Path(__file__).parents[1]
    paths=[root/'src/reconciliation/local_se2_reconciliation.py', root/'scripts/run_local_se2_reconciliation_formulation01.py']
    calls=[]
    for p in paths:
        calls += [n.func.id if isinstance(n.func,ast.Name) else n.func.attr for n in ast.walk(ast.parse(p.read_text()))
                  if isinstance(n,ast.Call) and isinstance(n.func,(ast.Name,ast.Attribute))]
    assert calls.count('solve_least_squares')==1
    assert not set(calls)&{'solve_graph','MpcTracker','solve_mpc','execute_native','run_rollout','run_gp','replay_prefix','OracleCorrespondence'}
    text=paths[1].read_text()
    assert "head==remote" in text and "open('x')" in text
    assert "'REPEAT_01'" not in text


def test_exact_sealed_R00_saved_loader_when_available():
    from run_local_se2_reconciliation_formulation01 import ROOT,load_source
    cfg=yaml.safe_load(CONFIG.read_text())
    if not (ROOT/cfg['source_run']).exists():pytest.skip('sealed local source not available')
    source=load_source(cfg); p=source['problem']
    assert sha(source['bundle']/'raw/fresh_lightnav.npy')==cfg['fresh_sha256']
    wire=read(source['bundle']/'raw/fresh_response.json')
    np.testing.assert_array_equal(source['raw'],wire['data']['actions']['actions'])
    np.testing.assert_array_equal(p.A,source['context']['R_obs'])
    np.testing.assert_array_equal(p.B,source['sealed']['B_state']['pose_world'])
    assert not np.array_equal(source['sealed']['B_state']['u_minus'],source['sealed']['B_state']['memory']['previous_control'])
