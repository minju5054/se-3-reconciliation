from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import zipfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
INSTALLER_PATH = REPOSITORY_ROOT / "scripts" / "install_vscode_trajectory_graph_viewer.py"


def _load_installer():
    spec = importlib.util.spec_from_file_location("vscode_trajectory_installer", INSTALLER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_vsix_contains_dependency_free_custom_editor(tmp_path: Path) -> None:
    installer = _load_installer()
    destination = installer.build_vsix(tmp_path / "trajectory-viewer.vsix")

    with zipfile.ZipFile(destination) as archive:
        names = set(archive.namelist())
        package = json.loads(archive.read("extension/package.json"))
        manifest = archive.read("extension.vsixmanifest").decode("utf-8")

    assert names == {
        "[Content_Types].xml",
        "extension.vsixmanifest",
        "extension/package.json",
        "extension/extension.js",
        "extension/README.md",
    }
    editor = package["contributes"]["customEditors"][0]
    assert editor["viewType"] == "reconciliation.trajectoryNpyGraph"
    assert editor["selector"] == [{"filenamePattern": "*.npy"}]
    assert editor["priority"] == "default"
    assert "trajectory-npy-graph-viewer" in manifest


def test_extension_invokes_existing_plotter_without_shell() -> None:
    source = (
        REPOSITORY_ROOT / "tools" / "vscode-trajectory-npy-viewer" / "extension.js"
    ).read_text(encoding="utf-8")

    assert '"scripts", "view_trajectory_npy.py"' in source
    assert '"--no-show"' in source
    assert '"--save"' in source
    assert "execFile(executable, args" in source
    assert "shell:" not in source
