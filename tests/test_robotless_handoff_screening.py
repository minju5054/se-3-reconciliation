"""Synthetic screening arithmetic and bank tests, not experimental evidence."""
import copy
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation import robotless_handoff_screening as screening

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def bank():
    return yaml.safe_load((ROOT/'configs/robotless_handoff_screening.yaml').read_text())


def test_thirty_explicit_ids_and_balanced_categories(bank):
    screening.validate_bank(bank)
    assert len(bank['episodes'])==30
    assert all(e['local_displacement']==[.3,0,0] for e in bank['episodes'])


@pytest.mark.parametrize('change', ['count','duplicate','id','category','R1','instruction','displacement','tau'])
def test_invalid_manifest_rejected(bank,change):
    if change=='count': bank['episodes'].pop()
    elif change=='duplicate': bank['episodes'][1]['episode_id']=bank['episodes'][0]['episode_id']
    elif change=='id': bank['episodes'][0]['episode_id']='other'
    elif change=='category': bank['episodes'][0]['category']='doorway'
    elif change=='R1': bank['episodes'][0]['R1'][0]+=.2
    elif change=='instruction': bank['episodes'][0]['instruction']=''
    elif change=='displacement': bank['episodes'][0]['local_displacement']=[.5,0,0]
    else: bank['motion']['tau_s'][-1]=2
    with pytest.raises(ValueError): screening.validate_bank(bank)


def test_ordered_pair_hash_deterministic_and_order_sensitive():
    assert screening.ordered_pair_hash('a'*64,'b'*64)==screening.ordered_pair_hash('a'*64,'b'*64)
    assert screening.ordered_pair_hash('a'*64,'b'*64)!=screening.ordered_pair_hash('b'*64,'a'*64)
    with pytest.raises(ValueError): screening.ordered_pair_hash('bad','b'*64)


@pytest.mark.parametrize('n',[2,7,10,53])
def test_projection_reuse_arbitrary_n_and_immutability(n,monkeypatch):
    fresh=np.column_stack((np.linspace(.15,1,n),np.zeros(n),np.linspace(0,.2,n)))
    before=fresh.copy(); fresh.setflags(write=False)
    original=screening.characterize_projection; calls=[]
    def spy(*args,**kwargs):
        calls.append((args,kwargs)); return original(*args,**kwargs)
    monkeypatch.setattr(screening,'characterize_projection',spy)
    rows=screening.transition_metrics({'episode_id':'test','category':'straight'},[0,0,0],fresh,'a'*64,'b'*64)
    assert len(calls)==1 and len(rows)==4
    assert rows[-1]['projection_is_interior'] is True
    assert all(r['e_perp_minus_previous_d_poly_m']==0 for r in rows)
    assert np.array_equal(fresh,before)


@pytest.mark.parametrize('value',[np.nan,np.inf,-np.inf])
def test_nonfinite_projection_inputs_rejected(value):
    with pytest.raises(ValueError):
        screening.transition_metrics({'episode_id':'x','category':'straight'},[0,0,0],[[.1,0,0],[value,0,0]],'a'*64,'b'*64)


def test_percentile_linear_definition_and_missing_counts():
    result=screening.percentiles([0,1,2,3,None])
    assert [result[k] for k in screening.STAT_NAMES]==pytest.approx([0,1.5,2.25,2.7,3])
    assert result['n_available']==4 and result['n_unavailable']==1
    assert screening.percentiles([None])['median'] is None
    with pytest.raises(ValueError): screening.percentiles([np.nan])


def rows_and_statuses():
    rows=[];statuses=[]
    for i,value in enumerate([1.,3.,9.]):
        old='a'*64; fresh=('b' if i<2 else 'c')*64
        for tau in screening.MOTION['tau_s']:
            rows.append({'episode_id':f'episode_{i:03d}','category':'straight','tau_s':tau,
                'OLD_raw_sha256':old,'FRESH_raw_sha256':fresh,'pair_sha256':screening.ordered_pair_hash(old,fresh),
                'e_perp_m':value,'abs_e_dir_deg':value*2,'abs_e_yaw_deg':value*3,'normalized_progress':value/10})
        statuses.append({'episode_id':f'episode_{i:03d}','category':'straight','status':'VALID_PAIR'})
    statuses.append({'episode_id':'episode_003','category':'doorway','status':'OLD_STOP','reason':'synthetic stop'})
    return rows,statuses


def test_duplicate_pair_group_medians_and_equal_pair_distribution():
    rows,statuses=rows_and_statuses(); result=screening.summarize(rows,statuses)
    d=result['diversity']
    assert (d['attempted'],d['valid'],d['invalid'])==(4,3,1)
    assert (d['unique_OLD_count'],d['unique_FRESH_count'],d['unique_ordered_pair_count'])==(1,2,2)
    assert d['most_frequent_pair_count']==2 and d['most_frequent_pair_fraction']==pytest.approx(2/3)
    group=next(r for r in result['unique_pairs'] if r['count']==2 and r['tau_s']==1)
    assert group['e_perp_m']==2 and group['episode_ids']==['episode_000','episode_001']
    assert result['statistics']['episode_weighted'][3]['metrics']['e_perp_m']['median']==3
    assert result['statistics']['unique_pair_weighted'][3]['metrics']['e_perp_m']['median']==5.5


def test_missing_metric_excluded_individually():
    rows,statuses=rows_and_statuses()
    for row in rows:
        if row['episode_id']=='episode_000': row['abs_e_dir_deg']=None
    result=screening.summarize(rows,statuses)
    assert result['statistics']['episode_weighted'][0]['metrics']['abs_e_dir_deg']['n_unavailable']==1
    assert result['statistics']['unique_pair_weighted'][0]['metrics']['abs_e_dir_deg']['n_available']==2


def test_representatives_deterministic_with_id_ties_and_missing_direction():
    rows,statuses=rows_and_statuses()
    selected=screening.select_representatives(rows)
    assert [s['episode_id'] for s in selected]==['episode_002']*3+['episode_001']
    assert selected[-1]['median_target']==3
    assert screening.select_representatives(list(reversed(rows)))==selected
    for row in rows: row['abs_e_dir_deg']=None
    assert screening.select_representatives(rows)[1]['episode_id'] is None
    tau_rows=[dict(rows[0],episode_id=f'episode_{i:03d}',tau_s=1.,e_perp_m=v) for i,v in enumerate([1,3,5,7])]
    assert screening.select_representatives(tau_rows)[-1]['episode_id']=='episode_001'


@pytest.mark.parametrize('change',['missing_tau','duplicate_status','changed_hash','unknown_episode'])
def test_summary_rejects_inconsistent_episode_table(change):
    rows,statuses=rows_and_statuses()
    if change=='missing_tau': rows.pop()
    elif change=='duplicate_status': statuses.append(statuses[0])
    elif change=='changed_hash': rows[0]['pair_sha256']='d'*64
    else: rows[0]['episode_id']='unknown'
    with pytest.raises(ValueError): screening.summarize(rows,statuses)
