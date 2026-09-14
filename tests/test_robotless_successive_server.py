"""Pure launcher checks; no model process or runtime evidence is created."""

import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/lightnav"))
SPEC = importlib.util.spec_from_file_location("successive_server", ROOT / "scripts/lightnav/robotless_successive_server.py")
server = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(server)


def test_launcher_uses_isolated_pinned_server_and_local_endpoint(tmp_path):
    config = {"paths": {"lightnav_checkout": "/external/official", "checkpoint_path": "/external/checkpoint"},
              "lightnav": {"server_url": "ws://127.0.0.1:8050"}}
    argv = server.server_arguments(config, tmp_path)
    assert argv[0] == "/external/official/.venv/bin/lightnav-serve"
    assert argv[argv.index("--model_path") + 1] == "/external/checkpoint"
    assert argv[argv.index("--port") + 1] == "8050"
    assert argv[-1] == str(tmp_path / "server.ready")


@pytest.mark.parametrize("url", ["ws://0.0.0.0:8050", "ws://127.0.0.1", "wss://127.0.0.1:8050", "ws://127.0.0.1:8050/path"])
def test_launcher_rejects_endpoint_it_cannot_match(url, tmp_path):
    with pytest.raises(ValueError, match="local server launcher"):
        server.server_arguments({"lightnav": {"server_url": url}}, tmp_path)
