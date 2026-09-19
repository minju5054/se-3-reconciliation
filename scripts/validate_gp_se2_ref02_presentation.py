#!/usr/bin/env python3
"""Independent saved-file audit of REF-02 presentation-only corrections."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import zipfile


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(output, *, write_output=True):
    output=Path(output).resolve();target=output/'independent_validation.json'
    if write_output and target.exists():
        raise FileExistsError('refusing independent presentation validation overwrite')
    begin=time.perf_counter();errors=[];checks=0
    def check(value,label):
        nonlocal checks
        checks+=1
        if not value:errors.append(label)
    try:
        provenance=read(output/'presentation_provenance.json');primary=Path(provenance['primary_run'])
        case_rows=read(primary/'case_manifest.json')['selected']
        names=tuple(Path(r['path']).stem for r in read(primary/'plot_manifest.json')['images'] if r['path'].startswith('cases/'+case_rows[0]['case_directory']+'/'))
        check(len(names)==11 and len(set(names))==11,'primary 11-figure coverage')
        primary_manifest=read(primary/'review_bundle/manifest.json')
        expected_files={r['path'] for r in primary_manifest['allowlisted_files']}|{'presentation_provenance.json'}
        manifest=read(output/'manifest.json')
        records={r['path']:r for r in manifest['files']}
        check(set(records)==expected_files,'presentation strict source-derived report allowlist')
        for name,record in records.items():
            path=Path(name)
            check(not path.is_absolute() and '..' not in path.parts,'safe relative path '+name)
            check(path.suffix in ('.png','.json','.csv','.yaml','.html','.md'),'report-only extension '+name)
            check(digest(output/path)==record['sha256'],'presentation record hash '+name)
            check((output/path).stat().st_size==record['bytes'],'presentation record byte count '+name)
        for field in ('primary_files_preserved','primary_review_files_preserved'):
            for path,expected in provenance[field].items():
                check(digest(path)==expected,'original preserved '+path)
        for field in ('new_controller_solves','new_optimizer_solves','new_inference'):
            check(provenance[field]==0,'no new compute '+field)
        check(not provenance['numerical_results_changed'] and not provenance['primary_artifacts_overwritten'],'render-only scope')
        check(not provenance['gui_runtime_validated'],'no synthetic GUI claim')
        renderer=Path(__file__).resolve().with_name('present_gp_se2_ref02.py')
        check(digest(renderer)==provenance['renderer_sha256'],'renderer source hash')
        corrected=unchanged=0
        for case in case_rows:
            for name in names:
                relative=Path('cases')/case['case_directory']/'plots'/f'{name}.png'
                source,figure=primary/relative,output/relative
                old,new=read(source.with_suffix('.json')),read(figure.with_suffix('.json'))
                check(digest(figure)==new['image_sha256'],'actual image hash '+str(relative))
                check(new['numeric_data']==old['numeric_data'],'exact numeric_data '+str(relative))
                check(new['input_identity']==old['input_identity'],'exact fixed B/C identity '+str(relative))
                check(new['source_hashes']==old['source_hashes'],'exact source links '+str(relative))
                for path,h in new['source_hashes'].items():check(digest(path)==h,'referenced original hash '+path)
                if name=='fixed_input_world':
                    corrected+=1
                    check(new['primary_image_sha256']==digest(source),'corrected original image link')
                    check(new['primary_sidecar_sha256']==digest(source.with_suffix('.json')),'corrected original sidecar link')
                    check(new['renderer_sha256']==digest(renderer),'corrected renderer hash')
                    check(new['new_controller_solves']==0 and not new['numeric_values_changed'] and not new['primary_artifacts_overwritten'],'corrected declared scope')
                else:
                    unchanged+=1
                    check(digest(figure)==digest(source),'uncorrected image bytes '+str(relative))
                    check(digest(figure.with_suffix('.json'))==digest(source.with_suffix('.json')),'uncorrected sidecar bytes '+str(relative))
        check(corrected==2 and unchanged==20,'exact two corrected and twenty preserved figures')
        with zipfile.ZipFile(output/'review_bundle.zip') as archive:
            expected=set(records)|{'manifest.json'}
            check(set(archive.namelist())==expected and len(archive.namelist())==len(expected),'strict ZIP contents')
            for name in archive.namelist():check(hashlib.sha256(archive.read(name)).hexdigest()==digest(output/name),'ZIP member exact '+name)
        original=read(output/'validation.json')
        check(original['review_bundle_sha256']==digest(output/'review_bundle.zip'),'ZIP published hash')
        check(original['numeric_data_unchanged'] and original['primary_unchanged'],'producer identity report corroborated')
    except Exception as exc:
        check(False,f'{type(exc).__name__}: {exc}')
    report=dict(valid=not errors,checks=checks,errors=errors,validation_wall_s=time.perf_counter()-begin,
        checked_utc=datetime.now(timezone.utc).isoformat(),validator_sha256=digest(__file__),
        independent_saved_artifacts_only=True,new_controller_solves=0,new_optimizer_solves=0,
        validation_scope='presentation source/numeric/PNG/ZIP identity; original primary numerical validation remains authoritative')
    if write_output:
        with target.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--presentation',required=True,type=Path)
    result=validate(parser.parse_args().presentation);print(json.dumps(result,indent=2));raise SystemExit(0 if result['valid'] else 1)
