"""Import-isolation correction tests; no real MPC/GP call."""
import subprocess
import sys
from pathlib import Path
import pytest
from reconciliation import gp_se2_diag08_endpoint_margin as original
from reconciliation import gp_se2_diag08_execution as isolated
from test_gp_se2_diag08 import full
from test_gp_se2_diag07 import fixture

@pytest.mark.parametrize('hard,valid,reserve',[(h,v,r) for h in [True,False] for v in [True,False] for r in [True,False]])
def test_exact_admission_parity(hard,valid,reserve):
    f=full(not valid);r={'planning_endpoint_reserve_pass':reserve}
    assert isolated.execution_admission(f,r,hard=hard)==original.execution_admission(f,r,hard=hard)

@pytest.mark.parametrize('hard',[True,False])
def test_exact_transport_parity(hard):
    m,c,r=fixture();ad=original.execution_admission(full(hard),{'planning_endpoint_reserve_pass':True},hard=hard)
    a=original.execute_reference(m,c,r,ad);b=isolated.execute_reference(m,c,r,ad)
    for key in ['states','commands','reference_world','reference_capture_local','plan_valid','deployment_candidate']:assert a[key]==b[key]
    assert isolated.validate_execution_records(b,c,r,ad)==original.validate_execution_records(b,c,r,ad)==[]


def test_import_does_not_load_scipy_jax_or_gp_solver():
    root=Path(__file__).resolve().parents[1]
    code="import sys;sys.path.insert(0,'src');import reconciliation.gp_se2_diag08_execution;assert not any(k.startswith(('scipy','jax','reconciliation.gp_se2_formulation')) for k in sys.modules)"
    subprocess.run([sys.executable,'-c',code],cwd=root,check=True)
