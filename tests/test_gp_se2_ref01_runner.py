"""Frozen scope and exclusive orchestration tests, not experimental evidence."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('ref01_runner', ROOT/'scripts/run_gp_se2_ref01.py')
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


def configuration():
    return yaml.safe_load((ROOT/'configs/gp_se2_ref01.yaml').read_text())


def test_fixed_design_has_two_events_four_real_variant_names_and_exact_schedule():
    config = runner.validate_config(configuration())
    assert [r['case_id'] for r in config['cases']] == [
        'episode_014_repeat_01/handoff_026', 'episode_001_repeat_01/handoff_002']
    assert config['variants'] == ['R00_NATIVE', 'R10_SUFFIX_ONLY', 'R01_RESAMPLE_ONLY', 'R11_CURRENT_ADAPTER']
    assert config['rollout'] == dict(horizon_s=3., control_hz=10., integration_hz=60.)
    assert all(not k.startswith(('M0', 'M1', 'M2', 'M3')) for k in config['variants'])


@pytest.mark.parametrize('section,key,value', [
    ('rollout', 'horizon_s', 4.), ('rollout', 'control_hz', 20.),
    ('rollout', 'integration_hz', 120.), ('reference', 'output_dt_s', .2),
    ('mpc', 'HORIZON', 6), ('mpc', 'MPC_DT_S', .2),
    ('reproduction', 'command_atol', 1e-3), ('reproduction', 'state_atol', 1e-3),
    ('reproduction', 'selected_pose_atol', 1e-5), ('reproduction', 'rtol', 1e-3)])
def test_fixed_design_refuses_posthoc_timing_or_reproduction_tolerance_change(section, key, value):
    config = configuration(); config[section][key] = value
    with pytest.raises(ValueError):
        runner.validate_config(config)


@pytest.mark.parametrize('key', ['new_vla_inference', 'new_data_collection', 'gp_or_rigid_optimization', 'controller_modified', 'retry'])
def test_fixed_design_rejects_unrequested_interventions(key):
    config = configuration(); config[key] = True
    with pytest.raises(ValueError):
        runner.validate_config(config)


def test_fixed_design_rejects_case_or_variant_replacement():
    config = configuration(); config['cases'].reverse()
    with pytest.raises(ValueError):
        runner.validate_config(config)
    config = configuration(); config['variants'][1] = 'M0_ADAPTER'
    with pytest.raises(ValueError):
        runner.validate_config(config)


def test_exclusive_data_writes_preserve_existing_evidence(tmp_path):
    target = tmp_path/'record.json'; runner.write(target, {'first': True})
    before = runner.digest(target)
    with pytest.raises(FileExistsError):
        runner.write(target, {'first': False})
    assert runner.digest(target) == before
    source = tmp_path/'source'; source.write_text('original')
    with pytest.raises(FileExistsError):
        runner.copy_new(source, target)
    assert runner.digest(target) == before


def test_nonfinite_evidence_is_rejected_not_silently_sanitized(tmp_path):
    with pytest.raises(ValueError):
        runner.write(tmp_path/'invalid.json', {'numeric': float('nan')})


def test_no_execution_without_freeze_or_after_first_started_record(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'verify', lambda _: (configuration(), {}))
    with pytest.raises(ValueError, match='freeze'):
        runner.execute(tmp_path)
    runner.write(tmp_path/'experiment_freeze.json', {})
    runner.write(tmp_path/'execution_started.json', {'partial_execution_is_evidence': True})
    with pytest.raises(FileExistsError, match='no hidden retry'):
        runner.execute(tmp_path)


def test_input_and_core_drift_block_all_later_phases(tmp_path, monkeypatch):
    config = configuration(); core = tmp_path/'immutable_core.py'; core.write_text('original')
    original = tmp_path/'original.npy'; original.write_text('bytes')
    runner.write(tmp_path/'source.json', dict(original_config_sha256=runner.digest(original),
        experiment_config_sha256='pending', copied_source_sha256={str(original): runner.digest(original)},
        preserved_core_sha256={str(core): runner.digest(core)}))
    runner.copy_new(original, tmp_path/'config_snapshot.yaml')
    (tmp_path/'experiment_config.yaml').write_text(yaml.safe_dump(config))
    source = runner.read(tmp_path/'source.json'); source['experiment_config_sha256'] = runner.digest(tmp_path/'experiment_config.yaml')
    monkeypatch.setattr(runner, 'read', lambda _: deepcopy(source))
    runner.verify(tmp_path)
    core.write_text('changed')
    with pytest.raises(ValueError, match='original source changed'):
        runner.verify(tmp_path)


def test_matched_probe_compares_world_and_source_progress_not_local_indices():
    import numpy as np
    diagnostic = dict(reference_world=np.zeros((5, 3)).tolist(),
        selected_original_fractional_row_coordinates=[1., 2., 3., 4., 5.],
        final_goal_row_in_horizon=False, indices=[1, 2, 3, 4, 5])
    probes = [dict(time_s=i/10, variants={v: deepcopy(diagnostic) for v in runner.VARIANTS}) for i in range(30)]
    probes[0]['variants']['R01_RESAMPLE_ONLY']['indices'] = [10, 11, 12, 13, 14]
    values = runner.selection_pairwise('synthetic/not_evidence', probes, matched=True)
    assert len(values) == 120 and not any(v['selection_changed'] for v in values)
    probes[0]['variants']['R01_RESAMPLE_ONLY']['selected_original_fractional_row_coordinates'][0] = 1.5
    values = runner.selection_pairwise('synthetic/not_evidence', probes, matched=True)
    changed = [v for v in values if v['selection_changed']]
    assert len(changed) == 2
    assert all(v['same_input_pose'] and v['command_difference'] is None for v in changed)


def test_diagnosis_refuses_regression_confirmation_when_original_replay_disagrees():
    from reconciliation.gp_se2_ref01_evaluation import factor_contrasts
    rows, reproductions = [], {}
    for cid, _ in runner.CASES:
        for v in runner.VARIANTS:
            rows.append(dict(case_id=cid, variant=v, primary_success=v == 'R00_NATIVE', terminal_yaw_error_rad=0. if v == 'R00_NATIVE' else .5))
        reproductions[cid] = dict(variants={v: dict(passed=False) for v in ('R00_NATIVE', 'R11_CURRENT_ADAPTER')},
            historical_success=dict(R00_NATIVE=True, R11_CURRENT_ADAPTER=False))
    result = runner.diagnosis(rows, factor_contrasts(rows), reproductions, [], configuration())
    assert not result['native_adapter_regression_reproduced']
    assert result['mechanism_localization_level'] == 'REPRODUCTION_NOT_CONFIRMED'
    assert result['suffix_effect_observed'] and result['resampling_effect_observed']
