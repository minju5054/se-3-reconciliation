#!/usr/bin/env python3
"""Append compact DIAG-02 review evidence through an exclusive bundle revision.

No figures, numeric traces, solver outputs, acceptance, or source data change.
The original minimal bundle and ZIP are retained with an inventory before the
new allowlisted package is written. Repeated invocation refuses overwrite.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import zipfile


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def package(run):
    run = Path(run).resolve()
    bundle, archive = run / "review_bundle", run / "bundle_packaging_attempt01"
    if archive.exists() or (run / "bundle_packaging.json").exists():
        raise FileExistsError(archive)
    mapping = {
        "aggregate/all_solves.csv":"aggregate/all_solves.csv",
        "aggregate/paired_comparison.csv":"aggregate/paired_comparison.csv",
        "aggregate/candidate_acceptance.csv":"aggregate/candidate_acceptance.csv",
        "aggregate/derivative_coverage.csv":"aggregate/derivative_coverage.csv",
        "derivative_design.json":"derivative_design.json",
        "derivative_checks/authoritative_validation.json":"authoritative_validation.json",
        "summary.json":"experiment_summary.json",
    }
    for source in mapping:
        if not (run / source).is_file():
            raise FileNotFoundError(run / source)
    previous = read(bundle / "manifest.json")
    inventory = previous["allowlisted_files"]
    expected = {row["path"] for row in inventory} | {"manifest.json"}
    actual = {str(path.relative_to(bundle)) for path in bundle.rglob("*") if path.is_file()}
    if actual != expected:
        raise ValueError("minimal bundle inventory differs from its allowlist")
    for row in inventory:
        relative = Path(row["path"])
        if relative.is_absolute() or ".." in relative.parts or digest(bundle / relative) != row["sha256"]:
            raise ValueError("unsafe or changed minimal bundle entry")
    gate, summary = read(run / "derivative_checks/authoritative_validation.json"), read(run / "summary.json")
    if not gate["valid"]:
        raise ValueError("authoritative derivative gate must be valid")
    supplied = [row for row in summary["all_solves"] if row["derivative_mode"] == "SUPPLIED_JAC"]
    converged = sum(row["termination"] == "CONVERGED" for row in supplied)
    archive.mkdir()
    bundle.rename(archive / "review_bundle")
    (run / "review_bundle.zip").rename(archive / "review_bundle.zip")
    old_files = [dict(path=str(p.relative_to(archive)), bytes=p.stat().st_size, sha256=digest(p))
                 for p in sorted(archive.rglob("*")) if p.is_file()]
    write(archive / "inventory.json", dict(files=old_files, self_excluded="inventory.json",
          scope="minimal final-render bundle retained before compact evidence packaging"))
    bundle.mkdir()
    for row in inventory:
        relative = Path(row["path"])
        if str(relative) in ("README.md", "index.html"):
            continue
        target = bundle / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(archive / "review_bundle" / relative, target)
    extras = []
    for source, destination in mapping.items():
        target = bundle / destination
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run / source, target)
        extras.append(dict(source=source, bundled_as=destination, sha256=digest(target), bytes=target.stat().st_size))
    index = (archive / "review_bundle/index.html").read_text()
    index += "<h2>Authoritative derivative gate and compact experiment tables</h2><ul>"
    for row in extras:
        index += f'<li><a href="{row["bundled_as"]}">{row["bundled_as"]}</a></li>'
    (bundle / "index.html").write_text(index + "</ul>\n")
    supported = gate["supported_points_validated"]
    unsupported = gate["unsupported_points_correctly_rejected"]
    (bundle / "README.md").write_text(
        "# GP-SE2-DIAG-02 review bundle\n\nOpen index.html. All eight fixed comparison cells, including failures, are present.\n\n"
        f"Authoritative derivative validation: {supported} supported points validated and {unsupported} logarithm-cut point correctly rejected; no finite-difference fallback. See authoritative_validation.json and derivative_design.json.\n\n"
        f"Actual supplied-Jacobian solves: {converged}/{len(supplied)} converged. This is one fixed-event numerical optimization diagnostic. No new controller rollout, inference, navigation improvement or physical safety claim.\n\n"
        "experiment_summary.json contains the experiment-level gate, outcome flags and limitations. aggregate/all_solves.csv, paired_comparison.csv, candidate_acceptance.csv and derivative_coverage.csv retain compact complete comparisons. summary.json is the earlier eight-cell plot summary.\n\n"
        "18 PNGs have adjacent plotted numeric data and input/source hashes. References are GP interpolation of saved seeds/iterates/candidates, never executed trajectories. Full-feasibility histories are certified after solving and displayed at recorded discovery times. Cold setup, prepared solve and validation costs are disjoint; shared environment load and optional optimality diagnostics remain separate.\n\n"
        "Only allowlisted figures, numeric sidecars, compact tables, design/gate summaries and review text are included. Raw source datasets, environment geometry/grids, checkpoints, solver vectors, external source and archived presentation attempts are excluded. Source hashes refer to original local inputs that are not included.\n")
    metadata = dict(kind="REVIEW_BUNDLE_PACKAGING_ONLY", added=extras, source_script_sha256=digest(__file__),
                    original_bundle_archive=str(archive.relative_to(run)),
                    numerical_trace_change=False, solver_rerun=False, figure_regeneration=False)
    write(bundle / "packaging_provenance.json", metadata)
    final_files = [dict(path=str(p.relative_to(bundle)), bytes=p.stat().st_size, sha256=digest(p))
                   for p in sorted(bundle.rglob("*")) if p.is_file()]
    write(bundle / "manifest.json", dict(allowlisted_files=final_files, file_count=len(final_files),
          self_excluded="manifest.json", excluded=["raw datasets", "environment", "checkpoints", "solver vectors", "external source", "presentation archives"]))
    with zipfile.ZipFile(run / "review_bundle.zip", "x", compression=zipfile.ZIP_DEFLATED) as zipped:
        for path in sorted(bundle.rglob("*")):
            if path.is_file():
                zipped.write(path, arcname=str(path.relative_to(bundle)))
    metadata.update(bundle_sha256=digest(run / "review_bundle.zip"),
                    bundle_bytes=(run / "review_bundle.zip").stat().st_size,
                    final_bundle_file_count=len(final_files) + 1)
    write(run / "bundle_packaging.json", metadata)
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(package(args.run), indent=2))
