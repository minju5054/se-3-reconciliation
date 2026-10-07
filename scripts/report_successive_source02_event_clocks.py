#!/usr/bin/env python3
"""Saved-only explicit clock table, preserving native nanosecond stamp schema."""
import argparse
import csv
import io
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from run_successive_native_source_acquisition02 import OUT,namespace


def event_rows(v):
    rows=[]
    for r in v['chunks']:
        if not r['generated']:continue
        sent=r['t_request_host'];receipt=r['t_receipt_host']
        rows.append(dict(chunk=r['chunk_id'],A=r['A'],B=r.get('B'),
            observation_sim_s=r['observation']['capture_sim_time_s'],
            observation_host_monotonic_ns=r['observation']['capture_monotonic_ns'],
            request_host_monotonic_ns=sent['monotonic_ns'],request_host_utc=sent['utc'],
            receipt_host_monotonic_ns=receipt['monotonic_ns'],receipt_host_utc=receipt['utc'],
            ready_seen_sim_s=(r.get('t_ready_seen_sim') or {}).get('sim_time_s'),
            install_sim_s=(r.get('t_install') or {}).get('sim_time_s'),
            application_sim_s=(r.get('t_application') or {}).get('sim_time_s'),
            inference_active_chunk=r.get('inference_active_chunk'),
            raw_sha256=(r.get('raw_local_ref') or {}).get('sha256')))
    return rows


def rendered(rows):
    s=io.StringIO();w=csv.DictWriter(s,fieldnames=list(rows[0]),lineterminator='\n')
    w.writeheader();w.writerows(rows);return s.getvalue()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args()
    run=namespace(a.run);text=rendered(event_rows(read(run/'validation.json')))
    out=OUT/'event_clocks.csv'
    if a.check:assert out.read_text()==text
    else:
        with out.open('x') as f:f.write(text)
        save(OUT/'event_clocks_manifest.json',dict(validation_sha256=sha(run/'validation.json'),
            output_sha256=sha(out),script_sha256=sha(__file__),new_scientific_calls=0,
            note='Exact native monotonic_ns and UTC stamps. Frozen plotting CSV optional host-second columns expect another stamp schema and remain blank; use this explicit table or original JSON. No timestamp or source alteration.'))
    print('Exact event clocks saved-only PASS',len(event_rows(read(run/'validation.json'))))
