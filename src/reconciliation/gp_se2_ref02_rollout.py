"""REF-02 per-instance selector injection with unchanged official MPC computation.

No external code is copied or edited. A new function object uses the *same* pinned
``MpcTracker.submit.__code__`` and private globals differing in only its selector
binding. Original ``_solve``, ``poll``, installation, futures and controller solve
remain in use. The controller solve recorder copies arguments and delegates to
that instance's original bound method. The existing counterfactual integration
loop similarly keeps its code object and substitutes only its instrumented solve
entry point. This changes the reference selector, not the MPC computation.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import time
from types import FunctionType, MethodType

import numpy as np

from reconciliation import gp_se2_rollout as original
from reconciliation.gp_se2_ref02_reference import (
    METHODS, enrich_selector_result, select_source_progress, validate_lineage,
)
from reconciliation.online_mpc_adapter import finite_pose
from reconciliation.se2 import wrap_angle


def array_sha256(value):
    return hashlib.sha256(np.asarray(value, dtype='<f8').tobytes(order='C')).hexdigest()


def _function_with_private_binding(function, name, replacement):
    """Preserve code/defaults/closure and change one copied global binding only."""
    if name not in function.__globals__ or name not in function.__code__.co_names:
        raise ValueError('pinned function does not contain expected injection binding: '+name)
    namespace = dict(function.__globals__)
    namespace[name] = replacement
    clone = FunctionType(function.__code__, namespace, function.__name__,
                         function.__defaults__, function.__closure__)
    clone.__kwdefaults__ = function.__kwdefaults__
    clone.__annotations__ = dict(function.__annotations__)
    return clone


def selector_result(module, method, reference, pose, lineage, original_goal_index,
                    *, constant_reference_metadata=None):
    """One selector invocation, then an independent method-specific audit."""
    if method not in METHODS:
        raise ValueError('unknown REF-02 method; no fallback')
    progress = validate_lineage(reference, lineage,
        constant_reference_metadata=constant_reference_metadata)
    details = None
    if method == 'C_DENSE_SOURCE_PROGRESS':
        selected, details = select_source_progress(reference, pose, progress,
            horizon=module.HORIZON, weights=module.Q_WEIGHTS,
            constant_reference=bool(len(progress) == 1 or np.all(progress == progress[0])))
    else:
        selected = module.build_pose_aligned_reference(reference, pose,
            horizon=module.HORIZON, weights=module.Q_WEIGHTS)
    audit = enrich_selector_result(reference, pose, selected, lineage, method=method,
        weights=module.Q_WEIGHTS, horizon=module.HORIZON,
        original_goal_row_index=original_goal_index, selection_details=details,
        constant_reference_metadata=constant_reference_metadata)
    return selected, audit


def make_diagnostic_tracker(module, method, reference, lineage, original_goal_index,
                            *, constant_reference_metadata=None):
    """Construct one official tracker and inject an instance-local selector.

    No tracker class, official module global, solver formula or shared controller
    function is patched. Audits run in the selector call whose actual returned
    array is passed by the original submit bytecode to the original ``_solve``.
    """
    if method not in METHODS:
        raise ValueError('unknown REF-02 method; no fallback')
    validate_lineage(reference, lineage, constant_reference_metadata=constant_reference_metadata)
    tracker = module.MpcTracker()
    capture = dict(selector_calls=[], controller_calls=[], method=method)
    tracker._ref02_capture = capture
    original_submit = module.MpcTracker.submit
    original_controller_solve = tracker.controller.solve

    def selected_reference(installed, pose, *, horizon, weights):
        if horizon != module.HORIZON or tuple(weights) != tuple(module.Q_WEIGHTS):
            raise ValueError('official submit changed the frozen selector settings')
        if not np.allclose(np.asarray(installed)[:, :2], np.asarray(reference)[:, :2], atol=1e-11, rtol=0) or not np.allclose(wrap_angle(np.asarray(installed)[:, 2]-np.asarray(reference)[:, 2]), 0., atol=1e-11, rtol=0):
            raise ValueError('installed path differs from unchanged source reference')
        actual, diagnostic = selector_result(module, method, installed, pose, lineage,
            original_goal_index, constant_reference_metadata=constant_reference_metadata)
        capture['selector_calls'].append(dict(input_pose=np.asarray(pose).copy(),
            installed_path=np.asarray(installed).copy(), reference=actual.copy(), diagnostic=diagnostic))
        return actual

    def recorded_controller_solve(pose, selected, previous, v_max):
        # These are the actual arguments inside the unchanged MpcTracker._solve.
        record = dict(local_pose=np.asarray(pose).copy(), local_reference=np.asarray(selected).copy(),
                      previous_control=tuple(previous), v_max=float(v_max))
        capture['controller_calls'].append(record)
        try:
            result = original_controller_solve(pose, selected, previous, v_max)
            record['returned'] = True
            return result
        except Exception as exc:
            record['returned'] = False
            record['exception'] = f'{type(exc).__name__}: {exc}'
            raise

    injected = _function_with_private_binding(original_submit,
        'build_pose_aligned_reference', selected_reference)
    tracker.submit = MethodType(injected, tracker)
    tracker.controller.solve = recorded_controller_solve
    capture['identity'] = {
        'official_submit_code_object_preserved': tracker.submit.__func__.__code__ is original_submit.__code__,
        'official_submit_globals_unmodified': original_submit.__globals__['build_pose_aligned_reference'] is module.build_pose_aligned_reference,
        'official_tracker_solve_unmodified': tracker._solve.__func__ is module.MpcTracker._solve,
        'official_poll_unmodified': tracker.poll.__func__ is module.MpcTracker.poll,
        'official_installation_unmodified': tracker.set_body_path.__func__ is module.MpcTracker.set_body_path,
        'official_controller_solve_delegated': original_controller_solve.__func__ is module.MPCController.solve,
        'private_submit_globals': injected.__globals__ is not original_submit.__globals__,
        'selector_binding_only_global_change': all(value is original_submit.__globals__[key]
            for key, value in injected.__globals__.items() if key != 'build_pose_aligned_reference'),
    }
    if not all(capture['identity'].values()):
        tracker.close()
        raise ValueError('official runtime method identity mismatch')
    return tracker


def diagnostic_synchronous_solve(module, tracker, pose):
    """Official submit/future/poll, auditing the actual solver input reference."""
    pose = finite_pose(pose, 'solve input')
    previous = original._command(tracker.previous_command)
    capture = tracker._ref02_capture
    selector_before, controller_before = len(capture['selector_calls']), len(capture['controller_calls'])
    start = time.perf_counter()
    tracker.submit(pose.copy())
    future = tracker._future
    if future is None:
        raise RuntimeError('official tracker did not submit a solve: '+tracker.error)
    result, error = None, None
    try:
        result = future.result()
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    command = tracker.poll()
    if command is None:
        raise RuntimeError('official synchronous solve produced no current-generation command')
    applied = original._command(command)
    if len(capture['selector_calls']) != selector_before+1 or len(capture['controller_calls']) != controller_before+1:
        raise ValueError('expected exactly one actual selector and controller call')
    selection, controller = capture['selector_calls'][-1], capture['controller_calls'][-1]
    actual_reference = selection['reference']
    if not np.array_equal(selection['input_pose'], pose):
        raise ValueError('selector input state changed')
    expected_local = module.project_world_to_local(actual_reference, pose)
    if not np.array_equal(expected_local, controller['local_reference']) or not np.array_equal(controller['local_pose'], np.zeros(3)):
        raise ValueError('actual MPCController.solve reference differs from selected rows')
    if controller['previous_control'] != tuple(previous) or controller['v_max'] != module.OBJNAV_V_MAX:
        raise ValueError('actual controller memory/limit changed')
    if result is not None and not np.array_equal(result.reference, actual_reference):
        raise ValueError('actual official result reference differs from selector output')
    audit = selection['diagnostic']
    # Preserve baseline selection schema/values for the original reproduction check.
    original_keys = ('nearest_index', 'indices', 'reference_world', 'endpoint_repeated',
                     'nearest_is_final_row', 'selection_semantics', 'index_validation')
    basic = {key: audit[key] for key in original_keys}
    return dict(input_pose_world=pose.tolist(), previous_control=previous.tolist(),
        command=applied.tolist(), previous_control_after=list(tracker.previous_command),
        success=error is None and not tracker.error, error=error or tracker.error or None,
        selection=basic, selection_diagnostic=deepcopy(audit),
        selection_source='captured actual instance selector input to official _solve; actual MPCController.solve local arguments independently matched',
        actual_submitted_reference_world=actual_reference.tolist(),
        actual_controller_reference_local=controller['local_reference'].tolist(),
        actual_controller_input_pose_local=controller['local_pose'].tolist(),
        actual_controller_previous_control=list(controller['previous_control']),
        actual_controller_v_max=controller['v_max'],
        actual_reference_capture_complete=True,
        result_reference_world=None if result is None else np.asarray(result.reference).tolist(),
        prediction_world=None if result is None else np.asarray(result.prediction).tolist(),
        official_solve_wall_s=None if result is None else float(result.solve_ms)/1000,
        submit_wait_poll_wall_s=time.perf_counter()-start,
        simulation_time_advanced_during_solve_s=0.,
        installed_path_sha256=array_sha256(selection['installed_path']),
        method=capture['method'])


class _ModuleFacade:
    def __init__(self, original_module, factory):
        self._original_module = original_module
        self.MpcTracker = factory

    def __getattr__(self, name):
        return getattr(self._original_module, name)


def counterfactual_rollout(module, frozen, reference, lineage, method,
                           *, original_goal_row_index, constant_reference_metadata=None):
    """Run the exact historical 30-solve / 180-tick loop with this selector."""
    instances = []

    def factory():
        tracker = make_diagnostic_tracker(module, method, reference, lineage,
            original_goal_row_index, constant_reference_metadata=constant_reference_metadata)
        instances.append(tracker)
        return tracker

    facade = _ModuleFacade(module, factory)
    loop = _function_with_private_binding(original.counterfactual_rollout,
        '_synchronous_solve', diagnostic_synchronous_solve)
    rollout = loop(facade, frozen, reference, horizon_s=3., control_hz=10., integration_hz=60.)
    if len(instances) != 1:
        raise ValueError('each primary rollout requires one independent tracker')
    capture = instances[0]._ref02_capture
    installed = np.asarray(instances[0]._trajectory)
    rollout.update(method=method, diagnostic_label='OFFLINE LOOKAHEAD-SELECTION DIAGNOSTIC',
        installed_reference_world=installed.tolist(), installed_path_sha256=array_sha256(installed),
        installed_path_hash_encoding='C-contiguous little-endian float64 Nx3 bytes',
        source_reference_array_sha256=array_sha256(reference),
        selection_instrumentation='per-instance submit selector binding plus actual controller argument capture',
        wrapper_identity={**capture['identity'], 'counterfactual_loop_code_object_preserved': loop.__code__ is original.counterfactual_rollout.__code__},
        actual_selector_calls=len(capture['selector_calls']), actual_controller_calls=len(capture['controller_calls']),
        GP_or_rigid_optimization_performed=False, independent_tracker_instance=True)
    return rollout
