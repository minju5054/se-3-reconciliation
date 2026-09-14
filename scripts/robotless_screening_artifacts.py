"""Immutable bank and file provenance shared by screening phases."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from reconciliation.robotless_handoff_screening import validate_bank
from reconciliation.robotless_single_chunk import load_config, make_observation_time, save_json_exclusive, sha256_file, validate_observation_time
from robotless_controlled_staleness_artifacts import file_record, git, read_json


def verify_file(root: Path, record: dict) -> None:
    path = (root / record['path']).resolve()
    if not path.is_relative_to(root.resolve()) or sha256_file(path) != record['sha256'] or path.stat().st_size != record['bytes']:
        raise ValueError(f'input changed: {record["path"]}')


def freeze_bank(config_path: Path, run: Path) -> dict:
    config = load_config(config_path)
    validate_bank(config)
    if any((run / name).exists() for name in ('config_snapshot.yaml', 'manifest_frozen.json', 'manifest_freeze.json', 'episodes')):
        raise FileExistsError('bank already frozen or episodes already exist')
    review = read_json(run / 'preparation/geometry_review.json')
    if review.get('reviewed_before_inference') is not True or review.get('lightnav_inference_count') != 0:
        raise ValueError('actual geometry review must precede bank freeze and inference')
    if review.get('bank_config_sha256') != sha256_file(config_path):
        raise ValueError('geometry review does not match the proposed bank configuration')
    run.mkdir(parents=True, exist_ok=True)
    with (run / 'config_snapshot.yaml').open('xb') as stream:
        stream.write(config_path.read_bytes())
    manifest = {'schema_version':1, 'stage':config['stage'], 'frozen_time':make_observation_time(0),
        'episodes':config['episodes'], 'motion':config['motion'], 'local_displacement':config['local_displacement'],
        'configuration':file_record(run/'config_snapshot.yaml',run),
        'geometry_review':file_record(run/'preparation/geometry_review.json',run),
        'starting_git_sha':git('rev-parse','HEAD'), 'episode_replacement_allowed':False,
        'selection_basis':'current Hospital geometry/RGB only; no historical predictions or metrics',
        'session_convention':'new connection -> login -> reset -> next(seq=0) -> next(seq=1) -> disconnect per episode; protocol failure preserves the completed prefix, without retry',
        'motion_convention':'controlled body-forward boundary-motion surrogate; not actual OLD execution',
        'representative_rules':['max_e_perp','max_abs_e_dir','max_abs_e_yaw','nearest_median_e_perp'],
        'representative_tau_s':1., 'representative_tie_rule':'lowest episode_id'}
    save_json_exclusive(run/'manifest_frozen.json',manifest)
    save_json_exclusive(run/'manifest_freeze.json', {'manifest':file_record(run/'manifest_frozen.json',run),
        'freeze_recorded_time':make_observation_time(0), 'episode_count':30, 'inference_started':False})
    return manifest


def load_frozen(run: Path) -> tuple[dict, dict, str]:
    freeze = read_json(run/'manifest_freeze.json')
    verify_file(run,freeze['manifest'])
    manifest = read_json(run/'manifest_frozen.json')
    verify_file(run,manifest['configuration'])
    verify_file(run,manifest['geometry_review'])
    config = load_config(run/'config_snapshot.yaml')
    validate_bank(config)
    if manifest['episodes'] != config['episodes'] or manifest['motion'] != config['motion'] or manifest['local_displacement'] != config['local_displacement']:
        raise ValueError('frozen manifest differs from configuration')
    validate_observation_time(manifest['frozen_time'])
    validate_observation_time(freeze['freeze_recorded_time'])
    if freeze['freeze_recorded_time']['monotonic_ns'] < manifest['frozen_time']['monotonic_ns'] or freeze['episode_count'] != 30:
        raise ValueError('invalid freeze ordering/count')
    return config, manifest, freeze['manifest']['sha256']


def phase_sources(extra=()) -> dict:
    files = ['src/reconciliation/robotless_handoff_screening.py','src/reconciliation/robotless_projection_handoff.py',
        'src/reconciliation/robotless_controlled_staleness.py','src/reconciliation/robotless_successive_chunks.py',
        'src/reconciliation/robotless_single_chunk.py','src/reconciliation/se2.py','scripts/robotless_screening_artifacts.py', *extra]
    return {name:sha256_file(ROOT/name) for name in files}
