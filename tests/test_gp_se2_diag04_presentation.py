"""Synthetic presentation QA only; no actual event performance expectations."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

p=load('diag04_presentation',ROOT/'scripts/present_gp_se2_diag04.py')
f=load('diag04_plot_fixtures',ROOT/'tests/test_gp_se2_diag04_plots.py')


def test_nonnegative_constraint_labels_sparse_ticks_outside_legend_and_time(tmp_path):
    audit,_=f.fixture(tmp_path)
    data=p.base.expected_numeric(audit)['constraint_violation_history']
    fig=p.base.constraint_plot(data)
    original=[(line.get_xdata().copy(),line.get_ydata().copy()) for ax in fig.axes for line in ax.lines]
    p.adjust_figure(fig,'constraint_violation_history',data)
    assert all(ax.get_ylim()[0]==0 for ax in fig.axes)
    assert all(ax.get_xlim()==(0.,.5) for ax in fig.axes)
    assert all(ax.get_legend() is None for ax in fig.axes)
    assert len(fig.legends)==1
    assert all(ax.get_ylabel()=='max inequality violation' for ax in fig.axes[1::2])
    modified=[(line.get_xdata(),line.get_ydata()) for ax in fig.axes for line in ax.lines]
    assert all(np.array_equal(a,c) and np.array_equal(b,d) for (a,b),(c,d) in zip(original,modified))
    p.plt.close(fig)


def test_na_history_keeps_real_time_and_large_range_preserves_small_values(tmp_path):
    audit,_=f.fixture(tmp_path)
    samples=audit['solves'][1]['latest']['samples']
    samples['body_twists'][1][1]=5.
    numeric=p.base.expected_numeric(audit)
    history=numeric['feasible_objective_history']
    fig=p.adjust_figure(p.base.objective_plot(history),'feasible_objective_history',history)
    assert all(ax.get_xlim()==(0.,.5) for ax in fig.axes)
    assert 'N/A' in fig.axes[1].texts[0].get_text()
    p.plt.close(fig)
    lateral=numeric['lateral_velocity_vs_time']
    fig=p.base.motion_plot(lateral,'v_y')
    curves=[line.get_ydata().copy() for line in fig.axes[0].lines]
    p.adjust_figure(fig,'lateral_velocity_vs_time',lateral)
    assert fig.axes[0].get_yscale()=='symlog'
    assert 'symlog' in fig.axes[0].get_ylabel()
    assert all(np.array_equal(before,line.get_ydata()) for before,line in zip(curves,fig.axes[0].lines))
    assert any(np.any((np.abs(v)>1e-5)&(np.abs(v)<1e-4)) for v in curves)
    p.plt.close(fig)


def test_separate_output_exact_payload_no_primary_change_and_no_queries(tmp_path,monkeypatch):
    primary=tmp_path/'synthetic_primary'
    primary.mkdir()
    audit,environment=f.fixture(primary)
    p.base.generate(primary,audit=audit,environment=environment)
    before=p.primary_inventory(primary)
    def forbidden(*args,**kwargs):
        raise AssertionError('presentation cannot query or solve a trajectory')
    import reconciliation.gp_se2 as gp
    import reconciliation.gp_se2_formulation as formulation
    monkeypatch.setattr(gp,'sample_gp',forbidden)
    monkeypatch.setattr(gp,'interpolate_interval',forbidden)
    monkeypatch.setattr(formulation,'minimize',forbidden)
    output=tmp_path/'synthetic_presentation'
    result=p.generate(primary,output,environment=environment)
    assert result['plot_count']==10 and result['presentation_only']
    assert result['new_queries']==result['new_solves']==result['new_execution']==0
    assert p.primary_inventory(primary)==before
    assert p.validate(primary,output)['valid']
    assert p.base.read(output/'source.json')['presentation']['primary_artifact_sha256']==before
    assert 'Presentation-only rendering correction' in (output/'index.html').read_text()
    for name in p.base.PLOT_NAMES:
        original=p.base.read(primary/'plots'/(name+'.json'))
        presented=p.base.read(output/'plots'/(name+'.json'))
        assert original['numeric_data']==presented['numeric_data']
        assert presented['presentation_script_sha256']==p.base.digest(p.__file__)
    with pytest.raises(FileExistsError):
        p.generate(primary,output,environment=environment)
    with pytest.raises(ValueError,match='160 dpi'):
        p.generate(primary,tmp_path/'too_small',environment=environment,dpi=100)
