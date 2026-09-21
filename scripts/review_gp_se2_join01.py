#!/usr/bin/env python3
"""Compact static review of preserved nonqualification evidence, no new runtime."""
import argparse,html,json,os,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
os.environ.setdefault('MPLCONFIGDIR','/tmp/join01-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run_gp_se2_join01 import read,write,digest,verify


def review(run):
    verify(run);s=read(run/'summary.json');v=read(run/'verification/validation.json')
    if not v['valid'] or v['report_source_sha256']!=digest(ROOT/'scripts/report_gp_se2_join01.py'):raise ValueError('authoritative report invalid or changed')
    out=run/'review';out.mkdir(exist_ok=False)
    rows=s['placements'];fig,axes=plt.subplots(1,3,figsize=(13,4.8))
    labels=[r['placement'].replace('placement_','P') for r in rows]
    for ax,key,title,threshold in [(axes[0],'B_clearance_m','B footprint-edge clearance [m]',.05),(axes[1],'FRESH_suffix_clearance_m','Original FRESH suffix clearance [m]',.05),(axes[2],'maximum_cross_track_m','Maximum cross-track disagreement [m]',.20)]:
        ax.bar(labels,[r[key] for r in rows],color='#298787' if key=='B_clearance_m' else '#c96040')
        ax.axhline(threshold,color='black',ls='--',label=f'frozen threshold {threshold:g} m')
        for i,r in enumerate(rows):ax.annotate(f'{r[key]:.6f}',(i,r[key]),xytext=(0,8 if r[key]>=0 else -15),textcoords='offset points',ha='center',fontsize=9)
        ax.set_title(title,fontsize=10);ax.legend(fontsize=8);ax.grid(axis='y',alpha=.2)
    axes[0].set_ylim(0,.82);axes[1].set_ylim(-.27,.14);axes[2].set_ylim(0,.24)
    fig.suptitle('JOIN-01: 0 / 3 qualifying source handoffs — no method comparison')
    fig.text(.5,.015,'Negative FRESH clearance denotes overlap of the reference footprint, not an executed collision. No GP/rigid/common-B rollout.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.05,1,.92));fig.savefig(out/'qualification_summary.png',dpi=170);plt.close(fig)
    write(out/'qualification_summary.json',dict(numeric=rows,source_sha256=digest(run/'source.json'),protocol_sha256=digest(run/'protocol.json'),
        summary_sha256=digest(run/'summary.json'),image_sha256=digest(out/'qualification_summary.png'),script_sha256=digest(Path(__file__))))
    content=['<!doctype html><meta charset="utf-8"><title>JOIN-01 source qualification</title>',
        '<style>body{font:16px system-ui;max-width:1200px;margin:32px auto;line-height:1.55}td,th{padding:8px;border:1px solid #bbb}table{border-collapse:collapse}img{max-width:100%}.rgb{width:480px}</style>',
        '<h1>GP-SE2-JOIN-01 · 장애물 reveal source qualification</h1>',
        '<p><strong>NO_QUALIFYING_OBSTACLE_REVEAL_HANDOFF</strong> — 사전 선언한 세 배치 모두 FRESH 우회 경로 조건을 충족하지 못했습니다. GP/rigid 최적화와 공통 B에서의 방법별 실행은 0회입니다.</p>',
        '<p>공식 LightNav OLD/FRESH 6개, 실제 Isaac RGB, OLD 실행 중 FRESH 요청·수신·적용 기록입니다. 아래 경로 그림은 source 수집 결과이며 GP 성능 비교가 아닙니다.</p>',
        '<p><strong>계측 한계:</strong> 저장 semantic mask는 상자 label을 내지 못했습니다. 실제 FRESH RGB에는 상자가 보입니다. 0 labelled pixels를 장애물 미관측의 증거로 해석하지 않습니다. 모든 사례는 이 항목과 독립적으로 FRESH clearance와 mismatch 조건도 실패했습니다. P01은 inference 구간 RTF도 실패했습니다.</p>',
        '<p><a href="../verification/validation.json">최종 authoritative validation</a> · <a href="../summary.json">전체 요약</a> · <a href="../aggregate/qualification.csv">수치 CSV</a> · <a href="../aggregate/method_ledger.csv">모든 방법 N/A ledger</a> · <a href="../protocol.json">동결 protocol</a></p>',
        '<img src="qualification_summary.png"><p><a href="qualification_summary.json">그림 수치 / hashes</a></p>',
        '<table><tr><th>배치</th><th>OLD 관측</th><th>Reveal/FRESH 관측</th><th>Ready seen</th><th>B 적용</th><th>요청→수신</th><th>관측→B 이동</th></tr>']
    for r in rows:
        values=[r['placement']]+[f"{r[k]:.3f} s" for k in ['old_observation_sim_s','fresh_observation_sim_s','ready_seen_sim_s','application_B_sim_s','request_to_receipt_s']]+[f"{r['observation_to_B_translation_m']:.3f} m"]
        content.append('<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in values)+'</tr>')
    content+=['</table><p>관측·ready seen·B는 simulation time, 요청→수신은 monotonic wall time입니다. 서로 대체하지 않습니다. FRESH는 원래 관측 pose에 고정되며 B로 재anchor하지 않았습니다.</p>',
        '<p>M0_NATIVE / M0_ADAPTER / M1_RIGID / M3_CURRENT_GP / M4_JOIN_GP: 모두 NOT_RUN_UPSTREAM_QUALIFICATION_FAILED. Planned/executed join time, 안전·운동·목표 비교 및 optimizer/MPC 비교 시간은 N/A입니다. 이를 GP 성공/실패로 세지 않습니다.</p>']
    for r in rows:
        ep=r['placement'];folder=run/'source_event/episodes'/ep;raw=read(folder/'handoffs/handoff_000/context.json')
        captures=[json.loads(line) for line in (folder/'capture.jsonl').read_text().splitlines()]
        old=next(c for c in captures if c['capture_sim_time_s']==raw['old_observation_pose_time']['time'])
        fresh=raw['fresh_observation_pose_time']['frame_id']
        content += [f'<h2>{ep}</h2>',f'<p>{html.escape(r["failure_reasons"])}</p>',f'<img src="../plots/{ep}_source_handoff.png">',
            f'<p><a href="../plots/{ep}_source_handoff.json">기하 수치 / hashes</a></p>',
            '<p>실제 OLD observation (왼쪽) / FRESH observation (오른쪽). 원본 RGB는 ZIP에서 제외, 전체 local run에서 열립니다.</p>',
            f'<img class="rgb" src="../source_event/episodes/{ep}/rgb/{old["frame_id"]}.jpg">',f'<img class="rgb" src="../source_event/episodes/{ep}/rgb/{fresh}.jpg">']
    content += ['<p>다음 단일 uncertainty: 공식 upstream이 새 장애물에 반응하는 안전한 원본 FRESH를 실제로 생성할 수 있는가? 현재 자료로 M4의 합류 속도나 navigation 효과를 평가할 수 없습니다.</p>']
    with (out/'index.html').open('x') as f:f.write('\n'.join(content))
    paths=[run/p for p in ['source.json','protocol.json','planning_config.yaml','config_snapshot.yaml','collection_result.json','summary.json','validation.json','index.html']]
    for sub in ['plots','qualification','aggregate','verification','review']:
        paths += [p for p in (run/sub).rglob('*') if p.is_file()]
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in paths:z.write(p,p.relative_to(run))
        z.writestr('README.txt','Open review/index.html. No qualifying source handoff; no GP/rigid/common-B execution. Read verification/validation.json; root validation preserves the development bootstrap-scope error. Raw RGB is excluded from this packet. Local full-run HTML links retain the actual RGB. No navigation or GP performance conclusion.')
    with zipfile.ZipFile(run/'review_bundle.zip') as z:
        members_ok=all(z.read(str(p.relative_to(run)))==p.read_bytes() for p in paths)
    side=read(out/'qualification_summary.json')
    write(run/'review_validation.json',dict(valid=members_ok and side['numeric']==s['placements'],
        archive_sha256=digest(run/'review_bundle.zip'),archive_bytes=(run/'review_bundle.zip').stat().st_size,
        members=len(paths)+1,all_file_bytes_match=members_ok,plot_numbers_equal_summary=side['numeric']==s['placements'],
        excluded=['raw RGB','full dataset','environment export','checkpoint'],source_sha256=digest(run/'source.json')))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();review(a.run.resolve())
