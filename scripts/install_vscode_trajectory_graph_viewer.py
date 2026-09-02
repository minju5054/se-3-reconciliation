#!/usr/bin/env python3
"""Package and install the repository's dependency-free VS Code custom editor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
EXTENSION_SOURCE = REPOSITORY_ROOT / "tools" / "vscode-trajectory-npy-viewer"
CONFLICTING_EXTENSIONS = (
    "subh-tools.npy-viewer",
    "kiameow.npy-image-preview",
)

CONTENT_TYPES = """<?xml version="1.0" encoding="utf-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension=".json" ContentType="application/json"/>
  <Default Extension=".vsixmanifest" ContentType="text/xml"/>
  <Default Extension=".js" ContentType="application/javascript"/>
  <Default Extension=".md" ContentType="text/markdown"/>
</Types>
"""


def _manifest(package: dict[str, object]) -> str:
    return f"""<?xml version="1.0" encoding="utf-8"?>
<PackageManifest Version="2.0.0"
  xmlns="http://schemas.microsoft.com/developer/vsx-schema/2011">
  <Metadata>
    <Identity Language="en-US" Id="{package['name']}" Version="{package['version']}"
      Publisher="{package['publisher']}" />
    <DisplayName>{package['displayName']}</DisplayName>
    <Description xml:space="preserve">{package['description']}</Description>
    <Categories>Visualization</Categories>
    <Properties>
      <Property Id="Microsoft.VisualStudio.Code.Engine" Value="{package['engines']['vscode']}" />
      <Property Id="Microsoft.VisualStudio.Code.ExtensionKind" Value="workspace" />
      <Property Id="Microsoft.VisualStudio.Code.ExecutesCode" Value="true" />
    </Properties>
  </Metadata>
  <Installation><InstallationTarget Id="Microsoft.VisualStudio.Code"/></Installation>
  <Dependencies/>
  <Assets>
    <Asset Type="Microsoft.VisualStudio.Code.Manifest" Path="extension/package.json"
      Addressable="true" />
    <Asset Type="Microsoft.VisualStudio.Services.Content.Details" Path="extension/README.md"
      Addressable="true" />
  </Assets>
</PackageManifest>
"""


def build_vsix(destination: Path) -> Path:
    """Build a minimal VSIX without requiring Node.js or vendored dependencies."""

    package = json.loads((EXTENSION_SOURCE / "package.json").read_text(encoding="utf-8"))
    destination = destination.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", CONTENT_TYPES)
        archive.writestr("extension.vsixmanifest", _manifest(package))
        for name in ("package.json", "extension.js", "README.md"):
            archive.write(EXTENSION_SOURCE / name, f"extension/{name}")
    return destination


def _installed_extensions(code_command: str) -> set[str]:
    result = subprocess.run(
        [code_command, "--list-extensions"],
        check=True,
        capture_output=True,
        text=True,
    )
    return {line.strip().lower() for line in result.stdout.splitlines() if line.strip()}


def install_vsix(code_command: str, vsix: Path) -> None:
    """Remove conflicting NPY custom editors and install this repository's editor."""

    installed = _installed_extensions(code_command)
    for extension in CONFLICTING_EXTENSIONS:
        if extension in installed:
            subprocess.run(
                [code_command, "--uninstall-extension", extension],
                check=True,
            )
    subprocess.run(
        [code_command, "--install-extension", str(vsix), "--force"],
        check=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Install the VS Code SE(2) trajectory NPY graph custom editor."
    )
    parser.add_argument("--code", default="code", help="VS Code CLI command (default: code)")
    parser.add_argument(
        "--build-only",
        type=Path,
        metavar="OUTPUT.vsix",
        help="build a VSIX at this path without installing it",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.build_only is not None:
        output = build_vsix(args.build_only)
        print(f"built: {output}")
        return 0

    if shutil.which(args.code) is None:
        raise FileNotFoundError(f"VS Code CLI was not found: {args.code}")
    with tempfile.TemporaryDirectory(prefix="trajectory-npy-vsix-") as directory:
        output = build_vsix(Path(directory) / "trajectory-npy-graph-viewer.vsix")
        install_vsix(args.code, output)
    print("installed: se3-reconciliation.trajectory-npy-graph-viewer")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
