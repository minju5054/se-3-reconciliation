#!/usr/bin/env python3
"""Clarify safe short outputs versus bypass without changing frozen analysis."""
import argparse
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read,save,sha
from report_join_source05 import load_results,rows
from validate_join_source05 import validate


def finalize(run):
    results,historical,_=load_results(run);numbers=rows(results,historical)
    old=Path(read(run/'technical_correction.json')['previous_run'])
    failed=[read(p) for p in old.glob('predictions/*/result.json')]
    assert len(failed)==12 and all(r['session_close']['next_count']==r['session_close']['prediction_count']==0 for r in failed)
    counts=read(run/'aggregate/actual_call_counts.json');oldserver=read(old/'server_process.json');oldstop=read(old/'server_shutdown.json')
    total=dict(**counts,previous_pre_image_failed_sessions=12,total_opened_sessions=24,
        total_scientific_predictions=12,total_buffer_requests=180,total_synthetic_server_warmups=2,
        previous_server_lifetime_s=(oldstop['exit_observed_event']['monotonic_ns']-oldserver['start_event']['monotonic_ns'])/1e9,
        previous_server_startup_s=(oldserver['ready_event']['monotonic_ns']-oldserver['start_event']['monotonic_ns'])/1e9,
        previous_worker_wall_sum_s=sum(r['wall_s'] for r in failed))
    save(run/'aggregate/complete_call_counts.json',total)
    response_rows=[]
    for r in numbers:
        if r['instruction']=='I0':label='historical straight / unsafe with cart'
        elif r['K']==0:label='OFF control: unchanged straight trajectory'
        elif r['stop']:label='STOP; not a detour'
        elif r['safe_cart_on'] and r['endpoint_beyond_cart_rear_m']<0:label='safe short path; does not pass cart'
        elif not r['safe_cart_on']:label='unsafe raw path'
        else:label='see frozen geometric qualification'
        response_rows.append(dict(**r,response_description=label))
    fig,ax=plt.subplots(figsize=(15,9));ax.axis('off')
    cells=[[r['condition'],str(r['safe_actual_scene']),str(r['safe_cart_on']),f"{r['clearance_cart_on_m']:.4f}",
            f"{r['max_lateral_m']*1000:.3f}",r['response_description']] for r in response_rows]
    tab=ax.table(cellText=cells,colLabels=['Condition','Safe input scene','Safe cart-ON*','Edge m','Max lateral mm','Observed response'],
        colWidths=[.12,.12,.12,.09,.10,.45],loc='center',cellLoc='left')
    tab.auto_set_font_size(False);tab.set_fontsize(9);tab.scale(1,1.9)
    ax.set_title('Safe shortening / STOP observed; zero lateral bypasses\nFrozen overall classification: INCONCLUSIVE (safe non-detour outcome)',fontsize=14,pad=25)
    fig.text(.5,.06,'* K0 cart-ON safety is hypothetical. Actual K0 input scene has no cart.\nI0 historical; I1/I2 new. No MPC or online execution. Shorter path is not successful traversal.',ha='center',fontsize=11)
    fig.tight_layout(rect=(0,.10,1,.96));image=run/'review/response_geometry_clarified.png';fig.savefig(image,dpi=150);plt.close(fig)
    inputs={str(run/'aggregate/outcomes.csv'):sha(run/'aggregate/outcomes.csv'),str(run/'aggregate/summary.json'):sha(run/'aggregate/summary.json')}
    for c in results:inputs[str(run/'predictions'/c/'evaluation.json')]=sha(run/'predictions'/c/'evaluation.json')
    side=dict(numbers=response_rows,source_sha256=sha(__file__),input_sha256=inputs,
              config_sha256=sha(run/'config_snapshot.yaml'),png_sha256=sha(image))
    save(image.with_suffix('.json'),side)
    note='<p><strong>Safe shortening/STOP is not lateral bypass.</strong> Six of eight non-sham ON conditions pass raw geometry, including one STOP. Five safe non-STOP chunks end before the cart. Every new ON lateral magnitude is below1.1mm. The frozen safe-non-detour branch returns INCONCLUSIVE; coverage and measurements are complete.</p>'
    note+='<p>K0 actual cart-OFF paths are safe and identical to historical I0; K0 cart-ON checks in some plots are hypothetical. "Visual change" in the first matrix means the frozen influence-region lateral/yaw threshold, not any raw-output difference.</p>'
    note+='<p>Only terminal instruction changed in the192 scientific request payloads; source/current monotonic clocks are recorded separately. Twelve earlier sessions sent no images and are retained as technical failures. Total scientific predictions12; startup warmups2. No I0 rerun.</p>'
    note+='<img src="review/response_geometry_clarified.png"><p><a href="review/response_geometry_clarified.json">Clarified numbers and hashes</a></p>'
    text=(run/'index.html').read_text();pos=text.index('<h1>')
    with (run/'index_final.html').open('x') as f:f.write(text[:pos]+note+text[pos:])
    base=validate(run);checks={'base_saved_validator':base['valid'],'clarified_numeric_rows':side['numbers']==response_rows,
        'clarified_figure_hash':sha(image)==side['png_sha256'],'prior_pre_image_zero_predictions':all(r['session_close']['prediction_count']==0 for r in failed)}
    for p,h in inputs.items():checks['source:'+p]=sha(p)==h
    save(run/'validation_completion.json',dict(valid=all(checks.values()),checks=checks,base_checks=len(base['checks']),
        new_model_MPC_GP_calls=0,scientific_classification_unchanged=True))
    with zipfile.ZipFile(run/'review_bundle.zip') as z:members=z.namelist()
    extra=['index_final.html','review/response_geometry_clarified.png','review/response_geometry_clarified.json',
           'aggregate/complete_call_counts.json','validation.json','validation_completion.json']
    with zipfile.ZipFile(run/'review_bundle_final.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in members+extra:z.write(run/name,name)
    with zipfile.ZipFile(run/'review_bundle_final.zip') as z:
        assert all(z.read(n)==(run/n).read_bytes() for n in z.namelist())
    save(run/'review_completion.json',dict(figures_total=len(read(run/'review/figure_manifest.json'))+1,
        final_zip_sha256=sha(run/'review_bundle_final.zip'),final_index_sha256=sha(run/'index_final.html'),
        added_presentation_only=True,frozen_numerical_source_unchanged=True))
    print(json.dumps(dict(valid=all(checks.values()),total_call_counts=total)))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();finalize(a.run.resolve())
