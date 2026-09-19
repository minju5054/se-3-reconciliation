"""Synthetic reporting-only checks; no trajectory optimization or evidence."""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('history_supplement_test',ROOT/'scripts/supplement_gp_se2_02_history.py')
report=importlib.util.module_from_spec(spec);spec.loader.exec_module(report)


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))


@pytest.fixture
def case(tmp_path):
    folder=tmp_path/'fixture';image=folder/'plots/feasible_objective_history.png';image.parent.mkdir(parents=True);image.write_bytes(b'synthetic image placeholder; not experimental evidence')
    history={};source=folder/'frozen_source.json';write(source,{'synthetic':True})
    for i,method in enumerate(report.GP_METHODS):
        for j,seed in enumerate(report.SEEDS):
            key=method+'/'+seed;valid=i==0 and j==1
            history[key]={'seed_full_feasible':False,'seed_objective':20.,'points':[
                {'discovery_time_s':0.,'best_full_objective':None,'full_feasible':False},
                {'discovery_time_s':2.8,'best_full_objective':6. if valid else None,'full_feasible':valid},
                {'discovery_time_s':3.,'best_full_objective':6. if valid else None,'full_feasible':valid}]}
            write(folder/'methods'/method/'starts'/seed/'solver_result.json',{'solve_wall_time_s':3.+i+j})
    write(image.with_suffix('.json'),{'image_sha256':report.file_sha256(image),'source_hashes':{str(source):report.file_sha256(source)},'numeric_data':history})
    return folder


def test_full_axes_keep_early_nulls_identical_and_do_not_modify_original(case):
    before={str(p):report.file_sha256(p) for p in case.rglob('*') if p.is_file()}
    data=report.payload(case)
    assert data['axes']['xlim_s']==[0.,5.]
    assert data['axes']['ylim_original_objective']==pytest.approx([0.,6.3])
    assert data['numeric_data']['M2_GP_NO_OBSTACLE/I1_DECEL']['points'][0]['best_full_objective'] is None
    result=report.render(case)
    assert all(report.file_sha256(p)==digest for p,digest in before.items())
    side=report.read(Path(result['path']).with_suffix('.json'))
    assert side['numeric_data']==report.read(case/'plots/feasible_objective_history.json')['numeric_data']
    assert not side['numerical_values_changed'] and not side['solver_rerun']
    with pytest.raises(FileExistsError):report.render(case)


def test_known_feasible_seed_uses_same_y_range_without_rescaling(case):
    path=case/'plots/feasible_objective_history.json';data=report.read(path)
    data['numeric_data']['M3_GP_CONSTRAINED/I1_DECEL'].update(seed_full_feasible=True,seed_objective=100.)
    write(path,data)
    assert report.payload(case)['axes']['ylim_original_objective']==[0.,105.]


def test_source_mutation_and_discovery_past_solve_fail(case):
    path=case/'plots/feasible_objective_history.json';data=report.read(path)
    data['numeric_data']['M2_GP_NO_OBSTACLE/I0_FRESH']['points'][-1]['discovery_time_s']=8.
    write(path,data)
    with pytest.raises(ValueError,match='outside prepared solve'):report.payload(case)
    write(case/'frozen_source.json',{'synthetic':'changed'})
    with pytest.raises(ValueError,match='source changed'):report.payload(case)
