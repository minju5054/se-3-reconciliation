#!/usr/bin/env python3
"""Reuse frozen four-figure report for data; one explicit status PNG if blocked."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.continuous_obstacle_reveal_episode01 import CHUNKS
from run_continuous_obstacle_reveal_episode01b import OUT,namespace
from report_continuous_obstacle_reveal_episode01 import report as report01, validate_report
from validate_robotless_online_handoffs import equal_record


def report(run):
    v=read(run/'attempt_validation.json')
    if v['scientific']:
        report01(run,OUT)
        save(OUT/'report_validation.json',validate_report(run,OUT))
        return
    summary={k:value for k,value in v.items() if k!='scientific'}
    summary['chunks']=[dict(chunk_id=c,generated=None,applied=None,A=None,B=None,
        observation_time=None,request_time=None,ready_time=None,application_time=None,
        raw_local_sha256=None,cart_visible_pixels=None,safety=None) for c in CHUNKS]
    if v['calls']['terminal_LightNav_predictions']==0:
        for c in summary['chunks']:c.update(generated=False,applied=False)
    summary['termination_reason']=v['scientific_validator_error']
    save(OUT/'result_summary.json',summary)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figures=OUT/'figures';figures.mkdir(exist_ok=False)
    fig,ax=plt.subplots(figsize=(9,3));ax.axis('off')
    text=('01B — TECHNICAL_EXECUTION_BLOCKED\n\n'
          f"SimulationApp launch attempts: {v['calls']['full_SimulationApp_launch_attempts']}  |  Retries: 0\n"
          f"Episodes initialized: {v['calls']['scientific_episodes_initialized']}  |  Terminal predictions: {v['calls']['terminal_LightNav_predictions']}\n"
          'No validated continuous C0–C3 source. Scientific geometry/timing: N/A.\n'
          'See execution_audit.json and preserved logs for the exact failure.')
    ax.text(.03,.92,text,va='top',fontsize=12,linespacing=1.7)
    path=figures/'technical_status.png';fig.savefig(path,dpi=160,bbox_inches='tight');plt.close(fig)
    save(OUT/'figure_manifest.json',dict(PNG_sha256={path.name:sha(path)},
        technical_only=True,result_summary_sha256=sha(OUT/'result_summary.json'),report_script_sha256=sha(__file__)))
    save(OUT/'report_validation.json',dict(valid=True,scientific_data=False,technical_status_only=True,
        new_model_MPC_optimizer_Isaac_calls=0))


def check(run):
    v=read(run/'attempt_validation.json')
    if v['scientific']:
        return validate_report(run,OUT)
    summary=read(OUT/'result_summary.json')
    for k,value in v.items():
        if k!='scientific': equal_record(value,summary[k])
    m=read(OUT/'figure_manifest.json')
    assert sha(OUT/'result_summary.json')==m['result_summary_sha256']
    assert sha(__file__)==m['report_script_sha256']
    assert sorted(p.name for p in (OUT/'figures').glob('*.png'))==['technical_status.png']
    assert sha(OUT/'figures/technical_status.png')==m['PNG_sha256']['technical_status.png']
    from PIL import Image
    with Image.open(OUT/'figures/technical_status.png') as image:image.verify()
    return dict(valid=True,technical_only=True,new_scientific_calls=0)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--validate',action='store_true');a=p.parse_args();run=namespace(a.run)
    if a.validate:print(check(run))
    else:report(run)
