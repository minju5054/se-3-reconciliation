"""Reporting-only guard-schema regression; no model or controller calls."""
import sys
from pathlib import Path
from copy import deepcopy
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from validate_osa03_native_continuation import guard_parity


def test_only_post_guard_journal_metadata_is_optional():
    saved=dict(safe=True,command=dict(v_mps=.8,omega_radps=.5,solve_id='frozen',application_tick=102),check=dict(clearance=.2))
    new=deepcopy(saved);new['command'].update(command_id=102,relative_time_s=.1,origin='NEW_NATIVE_CONTINUATION')
    guard_parity(new,saved)
    new['command']['omega_radps']=.6
    with pytest.raises(ValueError):guard_parity(new,saved)
    new=deepcopy(saved);new['command']['unknown']='cannot hide extra fields'
    with pytest.raises(AssertionError):guard_parity(new,saved)
    new=deepcopy(saved);new['safe']=False
    with pytest.raises(ValueError):guard_parity(new,saved)
