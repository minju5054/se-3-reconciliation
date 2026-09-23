#!/usr/bin/env python3
"""Independent saved-record entry point; refuses drift and recomputes all tables."""
import argparse
import csv
from pathlib import Path
import time

import numpy as np
from PIL import Image

from audit_join_online03_turnback import (
    ROOT, read, sha, inventory, collect, summarize, table_rows, csv_value,
    write_new, PROTOCOL, EPISODES,
)
from reconciliation.join_online03_turnback import mask_statistics
from validate_robotless_online_handoffs import equal_record


def validate(out):
    start=time.monotonic(); source=read(out/'source_manifest.json');run=Path(source['primary_run'])
    assert read(out/'protocol.json') == PROTOCOL
    assert inventory(run) == source['entire_primary_before_sha256']
    for group in ['input_sha256','code_sha256','unrelated_edits_sha256']:
        for path,h in source[group].items():assert sha(path)==h,path
    # Call the original validator as a function: DO NOT rewrite primary validation.json.
    from run_join_online03 import validate as original_validate
    original=original_validate(run)
    assert original['valid']
    saved=read(out/'records.json');fresh=collect(run)
    equal_record(saved,fresh,'full saved-only audit recomputation',atol=0)
    equal_record(read(out/'summary.json'),summarize(fresh),'summary',atol=0)
    equal_record(read(out/'turnback_metrics.json'),fresh['transitions'],'transitions',atol=0)
    for name,rows in table_rows(fresh).items():
        with (out/(name+'.csv')).open() as f:actual=list(csv.DictReader(f))
        fields=list(dict.fromkeys(k for row in rows for k in row))
        assert actual==[{k:csv_value(r.get(k)) for k in fields} for r in rows],name
    # Direct independent raw raster count/bounds; do not rely only on serialized metrics.
    for f in saved['frames']:
        mask=np.load(f['mask_path'])['mask'];cart=np.isin(mask,f['cart_ids'])
        ys,xs=np.where(cart)
        assert len(xs)==f['visible_pixels']
        expected=None if not len(xs) else [int(min(xs)),int(min(ys)),int(max(xs)),int(max(ys))]
        assert expected==f['bbox_xyxy_inclusive']
        assert mask_statistics(cart)['visible_image_fraction']==f['visible_image_fraction']
    for r in saved['rows']:
        times=[h['capture_sim_time_s'] for h in r['sampled_history']]
        assert times==sorted(times) and max(times)==r['observation_time_s']
        assert all(h['age_sim_s']==r['observation_time_s']-h['capture_sim_time_s'] for h in r['sampled_history'])
        assert r['raw_pointing']==read(r['raw_response_path'])['data']['pointing']
        np.testing.assert_array_equal(np.load(r['raw_local_ref']['path']),r['raw_local'])
        np.testing.assert_array_equal(np.load(r['world_ref']['path']),r['world'])
    manifest=read(out/'plots/manifest.json')
    expected_data={'current_RGB_sequence':[r for r in saved['rows'] if int(r['chunk'][-3:])>=3],
                   'world_and_visual_evidence':[r for r in saved['rows'] if int(r['chunk'][-3:])>=3],
                   'sequence_metrics':saved['rows']}
    for r in saved['rows']:
        if r['chunk'] in ['chunk_004','chunk_005']:
            expected_data[r['episode']+'_'+r['chunk']+'_history']=r['sampled_history']
    assert set(expected_data)=={e['name'] for e in manifest['figures']}
    for e in manifest['figures']:
        name=e['name'];image=out/'plots'/(name+'.png');side=out/'plots'/(name+'.json')
        assert sha(image)==e['png_sha256'] and sha(side)==e['sidecar_sha256']
        data=read(side);assert data['image_sha256']==sha(image)
        for path,h in data['source_hashes'].items():assert sha(path)==h
        equal_record(data['data'],expected_data[name],'plotted numbers',atol=0)
        with Image.open(image) as im:im.verify()
    assert inventory(run)==source['entire_primary_before_sha256']
    return dict(valid=True,original_JOIN_ONLINE_03_validator=original,
        records=len(saved['rows']),unique_frames=len(saved['frames']),
        source_files_before_after_unchanged=len(source['entire_primary_before_sha256']),
        source_and_code_hashes_preserved=True,table_and_plot_parity=True,
        new_scientific_model_calls=0,new_MPC_calls=0,new_GP_calls=0,new_rollouts=0,new_scientific_renders=0,
        internal_sampler_telemetry=False,wall_s=time.monotonic()-start)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    out=p.parse_args().output.resolve();result=validate(out);write_new(out/'validation.json',result)
    print('VALID',result['records'],'chunks;',result['unique_frames'],'unique captures; source unchanged; no new scientific calls')
