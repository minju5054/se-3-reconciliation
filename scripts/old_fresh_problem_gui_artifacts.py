"""Read-only source gate and append-only evidence support for the Isaac GUI."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import numpy as np
from reconciliation.old_fresh_problem_gui import CASE_IDS, reconstruct
from reconciliation.robotless_single_chunk import sha256_file, save_json_exclusive
from robotless_old_consistent_artifacts import inventory

SOURCE = ROOT/'data/robotless_old_consistent_observation/20260915T043415Z'
SOURCE_VALIDATED = 'ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT_VALIDATED'
VALIDATED = 'ROBOTLESS_OLD_FRESH_PROBLEM_GUI_VALIDATED'
FAILED = 'ROBOTLESS_OLD_FRESH_PROBLEM_GUI_FAILED'
RUNTIME_NOT_VALIDATED = 'ROBOTLESS_OLD_FRESH_PROBLEM_GUI_RUNTIME_NOT_VALIDATED'


def read_json(path):
    return json.loads(Path(path).read_text())


def verify_source(source, expected=None):
    source = Path(source).resolve()
    if not source.is_dir():
        raise ValueError('frozen pilot is missing; no replacement is allowed')
    before = inventory(source)
    if expected is not None and before != expected:
        raise ValueError('source inventory/hash mismatch')
    from validate_robotless_old_consistent import validate_run
    result = validate_run(source)
    if result['status'] != SOURCE_VALIDATED:
        raise ValueError(f'frozen pilot validation failed: {result}')
    if before != inventory(source):
        raise ValueError('source changed during validation')
    return result, before


def load_cases(source):
    cases, records = {}, {}
    for case_id in CASE_IDS:
        location = Path(source)/'episodes'/case_id
        paths = ['raw/chunk_000.npy', 'raw/chunk_001.npy', 'derived/chunk_000_world.npy',
                 'derived/chunk_001_world.npy', 'derived/metrics.json', 'metadata.json', 'raw/observation_001.jpg']
        hashes = {p: sha256_file(location/p) for p in paths}
        arrays = [np.load(location/p, allow_pickle=False) for p in paths[:4]]
        saved, metadata = read_json(location/paths[4]), read_json(location/paths[5])
        if hashes[paths[0]] != saved['source_OLD_hash'] or hashes[paths[1]] != saved['new_FRESH_hash']:
            raise ValueError('source OLD/FRESH hash mismatch')
        cases[case_id] = reconstruct(case_id, metadata, *arrays, saved)
        inference = read_json(location/'inference_metadata.json')
        records[case_id] = {'source_hashes': hashes, 'RGB1_path': str(location/paths[-1]),
                           'observation_times': [o['observation_time'] for o in metadata['observations']],
                           'saved_request_times': [c['request_send_time'] for c in inference['chunks']],
                           'saved_readiness_times': [c['response_receive_time'] for c in inference['chunks']],
                           'inference_metadata_path': str(location/'inference_metadata.json'),
                           'execution_time': None}
    return cases, records


def guard_output(output, source):
    output, source = Path(output).resolve(), Path(source).resolve()
    if output == source or output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError('GUI output must be separate from frozen source')
    if output.exists():
        raise FileExistsError('GUI run must be new; no evidence overwrite')
    return output


def verify_hashes(source, expected):
    if inventory(Path(source)) != expected:
        raise ValueError('source inventory/hash mismatch')


def case_audit(case):
    return {'reconstructed': case.reconstructed, 'saved_metrics': case.saved_metrics,
            'OLD_world_drawn': case.old.tolist(), 'FRESH_world_drawn': case.fresh.tolist(),
            'augmented_OLD_world': case.augmented.tolist(), 'B_world': case.b.tolist(),
            'marker_start': case.marker(0.).tolist(), 'marker_end': case.marker(.30).tolist(),
            'geometry_from_source_arrays': True, 'saved_metrics_agree': True}


if __name__ == '__main__':
    result, files = verify_source(Path(sys.argv[1]))
    print(json.dumps({'validation': result, 'files': files}))
