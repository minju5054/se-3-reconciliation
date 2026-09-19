#!/usr/bin/env python3
"""Add final scientific/operational reporting to the frozen base review package.

The original packager is invoked unchanged. Its complete first bundle, ZIP and
publication record are preserved in review_base/. A new final bundle adds the
explicit final report and exact narrative source; no numerical artifact changes.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import zipfile

from package_gp_se2_02_review import package, digest, read, write


def complete(run, narrative, gui=None):
    run=Path(run).resolve();narrative=Path(narrative).resolve()
    report=read(run/'final_report.json')
    if report['experiment']!='GP-SE2-02' or not narrative.is_file():
        raise ValueError('final GP-SE2-02 report and scientific narrative required')
    if any((run/name).exists() for name in ('review_bundle','review_bundle.zip','review_base','review_completion.json')):
        raise FileExistsError('new review package required; no overwriting existing evidence')
    initial=package(run,gui)
    base=run/'review_base';base.mkdir()
    for name in ('review_bundle','review_bundle.zip','review_bundle_manifest.json'):
        (run/name).rename(base/name)
    bundle=run/'review_bundle';bundle.mkdir()
    old_manifest=read(base/'review_bundle/manifest.json');records=[]
    for row in old_manifest['allowlisted_files']:
        if row['path'] in ('README.md','index.html'):
            continue
        target=bundle/row['path'];target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(base/'review_bundle'/row['path'],target);records.append(row)
    for source,name in ((run/'final_report.json','final_report.json'),(narrative,'RESULTS.md'),
                        (run/'aggregate/outcome_matrix.csv','aggregate/outcome_matrix.csv'),
                        (run/'aggregate/compute_summary.json','aggregate/compute_summary.json')):
        (bundle/name).parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,bundle/name)
        records.append(dict(path=name,source=str(source),sha256=digest(bundle/name),bytes=(bundle/name).stat().st_size))
    supplement=read(run/'history_supplement_manifest.json')
    supplemental_paths=['history_supplement_manifest.json']
    supplemental_paths.extend(name for name in ('history_supplement_visual_review.json','plot_visual_review.json') if (run/name).exists())
    for image in supplement['images']:
        supplemental_paths.extend([image['path'],str(Path(image['path']).with_suffix('.json'))])
    for name in supplemental_paths:
        source=run/name;target=bundle/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
        records.append(dict(path=name,source=str(source),sha256=digest(target),bytes=target.stat().st_size))
    completion=(f"\n## Final status\n\nOperational: {report['operational_status']}.\n"
        f"Execution finding: {report['execution_interpretation']}.\n\n"
        'Read RESULTS.md and final_report.json for the final interpretation. '
        'The original aggregate/summary.json retains its pre-GUI interim operational status; '
        'its numerical outcomes are unchanged. Operational completion requires the authoritative '
        'validation.json in the original run to pass. The preserved review_base package predates '
        'this final reporting addition. No optimization, controller execution, or scientific result was rerun.\n')
    (bundle/'README.md').write_text((base/'review_bundle/README.md').read_text()+completion)
    original=(base/'review_bundle/index.html').read_text()
    import html
    inserted=(f'<section><h2>Final result</h2><p>Operational: {html.escape(report["operational_status"])}. '
        f'Execution: {html.escape(report["execution_interpretation"])}.</p>'
        '<p><a href="RESULTS.md">Scientific report</a> · <a href="final_report.json">Final status and source hashes</a></p>'
        '<p>The original aggregate summary retains its interim operational field; numerical results are unchanged. '
        'Completion requires the original run’s authoritative validation.json to pass.</p></section>')
    inserted+='<section><h2>Full-time feasible-objective history supplements</h2><p>Original figures and numeric histories are preserved. These common axes expose initial N/A intervals.</p>'
    for image in supplement['images']:
        relative=html.escape(image['path'])
        inserted+=f'<a href="{relative}"><img src="{relative}" alt="full-time feasible-objective history"></a>'
    inserted+='</section>'
    (bundle/'index.html').write_text(original.replace('</body>',inserted+'</body>'))
    for name in ('README.md','index.html'):
        records.append(dict(path=name,source='generated review text',sha256=digest(bundle/name),bytes=(bundle/name).stat().st_size))
    write(bundle/'manifest.json',dict(old_manifest,allowlisted_files=records,final_reporting_added=True,
        final_report_sha256=digest(run/'final_report.json'),narrative_source=str(narrative),narrative_sha256=digest(narrative)))
    archive=run/'review_bundle.zip'
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as target:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():target.write(path,str(path.relative_to(bundle)))
    result=dict(bundle_sha256=digest(archive),bundle_bytes=archive.stat().st_size,file_count=len(records)+1,
        gui_runtime_validated=old_manifest['gui_runtime_validated'],packager_sha256=digest(__file__),
        original_packager_sha256=digest(Path(__file__).with_name('package_gp_se2_02_review.py')))
    write(run/'review_bundle_manifest.json',result)
    write(run/'review_completion.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),
        base_package_preserved=True,base_package_sha256=digest(base/'review_bundle.zip'),
        original_publication=initial,final_publication=result,final_report_sha256=digest(run/'final_report.json'),
        narrative_sha256=digest(narrative),numerical_artifacts_changed=False,new_optimization=False,new_rollout=False))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,type=Path);parser.add_argument('--narrative',required=True,type=Path)
    parser.add_argument('--gui',type=Path);args=parser.parse_args()
    print(json.dumps(complete(args.run,args.narrative,args.gui),indent=2))
