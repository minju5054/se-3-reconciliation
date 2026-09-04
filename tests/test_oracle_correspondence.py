from pathlib import Path

import numpy as np
import pytest

from reconciliation.exp02 import save_csv_exclusive
from reconciliation.online_switch import load_strict_json
from reconciliation.oracle_correspondence import (
    MEASUREMENT_CONVENTION,
    load_oracle_annotation,
    sha256_file,
    validate_source_hashes,
)


def write(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def test_oracle_io_validation_and_source_hashes(tmp_path: Path) -> None:
    trial = tmp_path / "trial"
    (trial / "raw").mkdir(parents=True)
    np.save(trial / "raw/old_actions.npy", np.zeros((2, 3)))
    np.save(trial / "raw/new_actions.npy", np.ones((2, 3)))
    annotation = tmp_path / "oracle.yaml"
    write(
        annotation,
        f"""schema_version: 1
source_experiment:
  raw_old_sha256: {sha256_file(trial / 'raw/old_actions.npy')}
  raw_new_sha256: {sha256_file(trial / 'raw/new_actions.npy')}
measurement_convention:
  Z_ij: "{MEASUREMENT_CONVENTION}"
correspondences:
  - old_index: 0
    new_index: 0
    relative_transform: [0.0, 0.0, 0.0]
    rationale: fixture
""",
    )
    document, correspondences = load_oracle_annotation(annotation, old_count=2, new_count=2)
    assert correspondences[0].relative_transform.tolist() == [0.0, 0.0, 0.0]
    validate_source_hashes(document, trial)


def test_oracle_invalid_transform_and_hash_rejected(tmp_path: Path) -> None:
    annotation = tmp_path / "bad.yaml"
    write(
        annotation,
        f"""schema_version: 1
measurement_convention:
  Z_ij: "{MEASUREMENT_CONVENTION}"
correspondences:
  - old_index: 0
    new_index: 0
    relative_transform: [.nan, 0.0, 0.0]
""",
    )
    with pytest.raises(ValueError, match="finite"):
        load_oracle_annotation(annotation, old_count=1, new_count=1)

    trial = tmp_path / "trial"
    (trial / "raw").mkdir(parents=True)
    np.save(trial / "raw/old_actions.npy", np.zeros((1, 3)))
    np.save(trial / "raw/new_actions.npy", np.zeros((1, 3)))
    document = {
        "source_experiment": {"raw_old_sha256": "wrong", "raw_new_sha256": "wrong"}
    }
    with pytest.raises(ValueError, match="does not match"):
        validate_source_hashes(document, trial)


def test_exp02_strict_json_and_immutable_csv(tmp_path: Path) -> None:
    csv_path = tmp_path / "profile.csv"
    save_csv_exclusive(csv_path, [{"new_index": 0, "translation": 0.1}])
    with pytest.raises(FileExistsError):
        save_csv_exclusive(csv_path, [{"new_index": 1, "translation": 0.2}])
    bad_json = tmp_path / "bad.json"
    bad_json.write_text('{"cost": NaN}', encoding="utf-8")
    with pytest.raises(ValueError):
        load_strict_json(bad_json)
