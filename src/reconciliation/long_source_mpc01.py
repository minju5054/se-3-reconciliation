"""Authenticate and set one native MPC bound before constructing its tracker.

No controller instance, solve, command transformation or external source edit.
The pinned tracker reads OBJNAV_V_MAX at solve time and passes it to the existing
CasADi parameter. Its preexisting extraction remains part of official behavior.
"""
from copy import deepcopy
from pathlib import Path
import ast
import json
from .join_source03 import read, sha
from .online_mpc_adapter import PINNED_MPC_SHA256, PINNED_LIGHTNAV_SHA, SETTING_NAMES


def authenticate_speed(prior_run):
    path = Path(prior_run)/'workers.json'
    p = read(path)['mpc']['provenance']
    assert p['lightnav_sha'] == PINNED_LIGHTNAV_SHA and not p['external_git_status']
    assert sha(p['mpc_source']) == p['mpc_source_sha256'] == PINNED_MPC_SHA256
    source = Path(p['mpc_source']).read_text()
    tree = ast.parse(source)
    tracker = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MpcTracker')
    solve = next(n for n in tracker.body if isinstance(n, ast.FunctionDef) and n.name == '_solve')
    calls = [n for n in ast.walk(solve) if isinstance(n, ast.Call) and
             isinstance(n.func, ast.Attribute) and n.func.attr == 'solve']
    assert len(calls) == 1 and ast.unparse(calls[0].args[-1]) == 'OBJNAV_V_MAX'
    assert 'self.opti.set_value(self.v_max, float(v_max))' in source
    assert 'opti.subject_to(controls[:, 0] <= v_max)' in source
    base = p['effective_linear_velocity_limit_m_s']
    assert base == p['official_settings']['OBJNAV_V_MAX'] and base > 0
    return dict(prior_workers=str(path.resolve()), prior_workers_sha256=sha(path),
        external_source=p['mpc_source'], external_source_sha256=p['mpc_source_sha256'],
        external_git_sha=p['lightnav_sha'], base_effective_linear_limit_m_s=base,
        new_effective_linear_limit_m_s=.5*base, ratio=.5,
        prior_settings=p['official_settings'], changed_setting='OBJNAV_V_MAX',
        mechanism='module OBJNAV_V_MAX assigned before tracker construction; native solve sets existing CasADi v_max parameter',
        added_output_postprocessing=False, official_extraction_unchanged=True,
        intrinsic_model_waypoint_dt_s=None)


def configure_before_tracker(module, provenance, frozen):
    """Called once by the isolated worker loader, before original worker main builds tracker."""
    assert sha(frozen['prior_workers']) == frozen['prior_workers_sha256']
    assert authenticate_speed(Path(frozen['prior_workers']).parent) == frozen
    assert provenance['mpc_source_sha256'] == frozen['external_source_sha256']
    before = {k:getattr(module,k) for k in SETTING_NAMES}
    # JSON-normalize the tuple-valued official weights against saved provenance.
    assert json.loads(json.dumps(before)) == frozen['prior_settings']
    assert module.OBJNAV_V_MAX == frozen['base_effective_linear_limit_m_s']
    module.OBJNAV_V_MAX = frozen['new_effective_linear_limit_m_s']
    after = {k:getattr(module,k) for k in SETTING_NAMES}
    assert [k for k in before if before[k] != after[k]] == ['OBJNAV_V_MAX']
    result = deepcopy(provenance)
    result.update(official_settings=after, effective_linear_velocity_limit_m_s=module.OBJNAV_V_MAX,
        speed_intervention=deepcopy(frozen), configured_before_tracker_construction=True,
        tracker_instrumentation='unchanged timing subclass; isolated loader sets only native OBJNAV_V_MAX before construction')
    return module, result
