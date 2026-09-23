"""Synthetic presentation corruption tests; no model or controller calls."""
import importlib.util
from pathlib import Path
import zipfile
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('osa02_report_validator', ROOT / 'scripts/validate_obstacle_source02_report.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_csv_rejects_missing_or_changed_result(tmp_path):
    p = tmp_path / 'episodes.csv'
    p.write_text('episode_id,qualified,failure_reasons\nsynthetic,False,obstruction\n')
    rows = [dict(episode_id='synthetic', qualified=False, raw_failure_reasons=['obstruction'])]
    mod.csv_parity(p, rows)
    with pytest.raises(AssertionError):
        mod.csv_parity(p, rows + rows)
    rows[0]['qualified'] = True
    with pytest.raises(AssertionError):
        mod.csv_parity(p, rows)


def test_review_zip_rejects_changed_sidecar(tmp_path):
    (tmp_path / 'numeric.json').write_text('{"clearance":0.2}')
    with zipfile.ZipFile(tmp_path / 'review_bundle.zip', 'x') as z:
        z.write(tmp_path / 'numeric.json', 'numeric.json')
    mod.zip_parity(tmp_path)
    (tmp_path / 'numeric.json').write_text('{"clearance":0.3}')
    with pytest.raises(AssertionError):
        mod.zip_parity(tmp_path)
