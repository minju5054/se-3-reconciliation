#!/usr/bin/env python3
"""Saved-only presentation correction; preserve the frozen renderer and run ledger."""
import argparse
import csv
import io
import json
from pathlib import Path
import shutil
from unittest.mock import patch
from matplotlib.axes import Axes
from matplotlib.ticker import MaxNLocator
from run_b_to_entry_graph_formulation_diag02 import ROOT,RESULTS,read,save,sha
from report_b_to_entry_graph_formulation_diag02 import compact_figures
from validate_b_to_entry_graph_formulation_diag02 import validate


def normalize_tables(archive):
    """Keep original CSV bytes; change CRLF serialization only, never cell values."""
    (archive/'tables').mkdir(exist_ok=False)
    hashes={}
    for p in RESULTS.glob('*.csv'):
        before=p.read_bytes();after=before.replace(b'\r\n',b'\n')
        assert list(csv.reader(io.StringIO(before.decode())))==list(csv.reader(io.StringIO(after.decode())))
        (archive/'tables'/p.name).write_bytes(before)
        p.write_bytes(after);hashes[p.name]=sha(p)
    return hashes


def report_layout(run,archive):
    summary,validation=validate(run,deep=False)
    assert validation['valid']
    archive.mkdir(parents=True,exist_ok=False)
    names=['result_summary.json','figure_manifest.json']
    for name in names:shutil.copyfile(RESULTS/name,archive/name)
    (archive/'figures').mkdir()
    for p in (RESULTS/'figures').glob('*.png'):shutil.copyfile(p,archive/'figures'/p.name)
    original=Axes.inset_axes
    def inset_axes(self,*args,**kwargs):
        kwargs['zorder']=40
        ax=original(self,*args,**kwargs)
        ax.xaxis.set_major_locator(MaxNLocator(3))
        ax.yaxis.set_major_locator(MaxNLocator(3))
        return ax
    with patch.object(Axes,'inset_axes',inset_axes):
        figures=compact_figures(summary,RESULTS/'figures')
    previous=read(archive/'figure_manifest.json')
    assert [f['numeric_sidecar'] for f in figures]==[f['numeric_sidecar'] for f in previous['figures']]
    table_hashes=normalize_tables(archive)
    audit=dict(label='presentation only: inset z-order, tick density and CSV LF line endings',new_scientific_solves=0,
        scientific_freeze_sha=summary['scientific_freeze_sha'],source_result_ledger_sha256=sha(run/'result_hashes.json'),
        frozen_renderer_sha256=sha(ROOT/'scripts/report_b_to_entry_graph_formulation_diag02.py'),
        layout_renderer_sha256=sha(__file__),archive=str(archive),
        archived_hashes={str(p.relative_to(archive)):sha(p) for p in archive.rglob('*') if p.is_file()},
        original_PNG_hashes={f['file']:f['sha256'] for f in previous['figures']},
        final_PNG_hashes={f['file']:f['sha256'] for f in figures},numeric_sidecars_unchanged=True,
        final_table_hashes=table_hashes,CSV_cells_unchanged=True)
    save(RESULTS/'layout_audit.json',audit)
    manifest=dict(previous,figures=figures,layout_audit_sha256=sha(RESULTS/'layout_audit.json'))
    (RESULTS/'figure_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    result=read(archive/'result_summary.json')
    result['figures']=[dict(path=str(RESULTS/'figures'/f['file']),sha256=f['sha256']) for f in figures]
    result['layout_audit_sha256']=sha(RESULTS/'layout_audit.json')
    result['table_hashes']=table_hashes
    (RESULTS/'result_summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(final_PNGs=len(figures),numeric_sidecars_unchanged=True,new_scientific_solves=0)))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--archive',required=True,type=Path);args=parser.parse_args()
    report_layout(args.run.resolve(),args.archive.resolve())
