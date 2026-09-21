"""Reporting compatibility only; synthetic shapes, no inference/geometry change."""
import importlib.util
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import GeometryCollection, LineString, box

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('source03_presentation_test',ROOT/'scripts/join_source03_presentation.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_empty_and_mixed_clipped_geometry_render_without_changing_input():
    for g in [GeometryCollection(),GeometryCollection([box(0,0,1,1),LineString([(1,1),(2,2)])])]:
        before=g.wkb
        fig,ax=plt.subplots()
        module.collection_plot(g,ax=ax,color='gray',add_points=False)
        fig.canvas.draw();plt.close(fig)
        assert g.wkb==before


def test_output_adapter_does_not_change_numeric_literals(tmp_path):
    p=tmp_path/'fixture.py';p.write_text("output='review'\nlimit=.05\n")
    m=module.load_frozen(p,{'review':'review_v2'})
    assert m.output=='review_v2' and m.limit==.05


def test_saved_review_validator_preserves_scientific_failure_and_detects_tampering(tmp_path):
    """Synthetic record is a validator fixture, not model evidence."""
    import csv
    import json
    import zipfile
    spec=importlib.util.spec_from_file_location('review_validator',ROOT/'scripts/validate_join_source03_review.py')
    validator=importlib.util.module_from_spec(spec);spec.loader.exec_module(validator)
    def write(name,value):
        p=tmp_path/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(json.dumps(value))
    write('validation_v2.json',{'valid':True})
    write('review_comparisons/figure_manifest.json',[])
    write('aggregate/ledger.json',[{'condition_id':'fixture','status':'COMPLETED'}])
    write('paired_diagnostics/fixture/evaluation.json',{'success':False,'details':{'B':{
        'raw_local':[[0,0,0]],'world':[[1,2,0]],'world_sha256':'fixture_hash',
        'geometry_on':{'node_clearance_m':[-.1]}}}})
    row=dict(condition='fixture',branch='B',row_index=0,local_forward_m=0,local_left_m=0,
             local_yaw_rad=0,world_x_m=1,world_y_m=2,world_yaw_rad=0,edge_clearance_m=-.1,
             source_world_sha256='fixture_hash')
    path=tmp_path/'aggregate/row_geometry.csv'
    def table():
        with path.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(row));w.writeheader();w.writerow(row)
    table()
    with zipfile.ZipFile(tmp_path/'review_bundle_final.zip','x'):
        pass
    assert validator.validate(tmp_path)['valid']
    row['edge_clearance_m']=0.0;table()
    assert not validator.validate(tmp_path)['valid']
