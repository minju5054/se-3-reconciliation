#!/usr/bin/env python3
"""Export a Korean summary PNG from authenticated saved R01 results; no new solves."""
import argparse
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np
from shapely import wkb

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT/'data/osa03_relative_factor_replication_01/primary_20260930T010000Z'
TRACKED = ROOT/'results/osa03_relative_factor_replication_01/result_summary.json'
ORDER = ['M0_NATIVE', 'M1_TAPER', 'FULL_LOCAL_SE2', 'NO_RELATIVE']
NAMES = ['Native', 'Taper', 'Full', 'No-relative']
COLORS = ['#187f86', '#d88b18', '#7654a5', '#c33268']
STYLES = ['-', '--', '-.', ':']
MARKERS = ['o', 's', '^', 'x']


def read(p):
    return json.loads(p.read_text())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def export(out):
    tracked = read(TRACKED)
    assert sha(RUN/'result_hashes.json') == tracked['result_hashes_sha256']
    ledger = read(RUN/'result_hashes.json')
    for path, digest in ledger.items():
        assert sha(RUN/path) == digest, path
    summary = read(RUN/'summary.json')
    assert summary == tracked['summary']
    assert read(RUN/'validation.json')['valid'] and summary['schedule_gate']['comparable']
    side = read(RUN/'review/world_execution.json')['data']
    rows = summary['primary_metrics']
    effects = {e['metric']: e for e in summary['replication_effects']}
    out.mkdir(parents=True, exist_ok=False)
    plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False,
                         'font.size': 12, 'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(16, 13), facecolor='#f7f9fc')
    fig.text(.06, .951, '상대운동 보존 항 제거: REPEAT_01 결과', fontsize=27, weight='bold', color='#172d42')
    fig.text(.06, .917, 'OSA03  |  같은 논리 일정에서 비교한 오프라인 복제 실험  |  평가 대상: 원래 FRESH 경로', fontsize=13, color='#526376')
    auc = effects['position_auc_09_m_s']['R01_NoR_minus_Full']
    attach = effects['sustained_attachment_s']['R01_NoR_minus_Full']
    tv = effects['linear_command_TV']['R01_NoR_minus_Full']
    cards = [(-attach, 's', 'No-relative, Full 대비 3 tick 빠름', '#187f86'),
             (-auc, 'm·s', 'Full 대비 첫 0.9 s 위치 AUC 감소', '#187f86'),
             (tv, 'm/s', 'Full 대비 선형 명령 총변화량 증가', '#a06711')]
    for i, (v, unit, label, color) in enumerate(cards):
        x = .06 + i*.30
        fig.add_artist(FancyBboxPatch((x, .808), .28, .087, transform=fig.transFigure,
            boxstyle='round,pad=.007,rounding_size=.009', facecolor='white', edgecolor='#dce4ed', zorder=0))
        value = f'{v:.2f}' if i == 0 else f'{v:.6f}'
        fig.text(x+.014, .856, f'{value} {unit}', fontsize=24, color=color, weight='bold')
        fig.text(x+.014, .821, label, fontsize=12, color='#526376')
    ax = fig.add_axes([.07, .302, .355, .466], facecolor='white')
    cart = wkb.loads(side['cart_wkb_hex'], hex=True)
    for shape, fill in [(cart, True), (cart.buffer(.25), False)]:
        for i, g in enumerate(shape.geoms if hasattr(shape, 'geoms') else [shape]):
            if fill:
                ax.fill(*g.exterior.xy, color='#9f8b75', alpha=.5, label='카트' if i == 0 else None)
            else:
                ax.plot(*g.exterior.xy, ':', color='#9f8b75', lw=1.6,
                        label='로봇 중심 금지 경계 (반경 .20 + 여유 .05 m)' if i == 0 else None)
    original = np.array(side['methods']['M0_NATIVE']['reference'])
    ax.plot(original[:,0], original[:,1], '--', color=COLORS[0], lw=1.6, label='원래 FRESH')
    for name, color, marker in [('FULL_LOCAL_SE2', COLORS[2], '^'), ('NO_RELATIVE', COLORS[3], 'x')]:
        d = side['methods'][name]; ref = np.array(d['reference']); mt = d['metrics']
        p = np.array(mt['trace']['poses_world']); label = 'Full' if name == 'FULL_LOCAL_SE2' else 'No-relative'
        ax.plot(ref[:,0], ref[:,1], '--', marker=marker, ms=3, color=color, lw=1, label=label+' 참조')
        ax.plot(p[:,0], p[:,1], '-', marker=marker, markevery=15, ms=4, color=color, lw=1.7, label=label+' 실행')
        j = int(np.searchsorted(mt['trace']['times_s'], mt['full']['join_time_s']))
        ax.scatter(*p[j,:2], marker='D', color=color, s=45, zorder=4)
    for key, marker in [('A', 'x'), ('B', '*')]:
        ax.scatter(*side[key][:2], marker=marker, color='#172d42', s=75, zorder=5)
        ax.annotate(key, side[key][:2], xytext=(-17, 2), textcoords='offset points', weight='bold')
    ax.set(aspect='equal', xlabel='World X [m]', ylabel='World Y [m]', title='경로와 실제 실행  ·  ◆ 지속 부착 시점')
    ax.grid(alpha=.18); ax.legend(fontsize=8, loc='lower left', framealpha=.93)
    ax2 = fig.add_axes([.51, .499, .435, .253], facecolor='white')
    for name, label, color, style, marker in zip(ORDER, NAMES, COLORS, STYLES, MARKERS):
        tr = side['methods'][name]['metrics']['trace']
        ax2.plot(tr['times_s'], tr['distance_m'], color=color, ls=style, marker=marker,
                 markevery=20, ms=3, lw=1.8, label=label)
    ax2.axvspan(0, .9, color='#e8eef5', alpha=.65, zorder=0)
    ax2.axhline(.1, color='#526376', ls=':', lw=1)
    ax2.text(2.95, .102, '위치 기준 .10 m', ha='right', fontsize=10, color='#526376')
    ax2.set(xlabel='B 이후 경과 시간 [s]', ylabel='원래 FRESH까지 거리 [m]', title='초기 분리와 회복  ·  음영: 첫 0.9 s')
    ax2.grid(alpha=.18); ax2.legend(fontsize=9, loc='upper right')
    axt = fig.add_axes([.51, .305, .435, .133]); axt.axis('off')
    chosen = [('sustained_attachment_s','부착 시간 [s]'),('position_auc_09_m_s','위치 AUC 0.9 s [m·s]'),('linear_command_TV','선형 TV [m/s]')]
    cell = [[label, f"{effects[k]['R00_NoR_minus_Full']:+.6f}", f"{effects[k]['R01_NoR_minus_Full']:+.6f}"] for k,label in chosen]
    table = axt.table(cellText=cell, colLabels=['No-relative - Full', 'R00', 'R01'], colWidths=[.48,.26,.26], bbox=[0,0,1,.88])
    table.auto_set_font_size(False); table.set_fontsize(11)
    axt.set_title('두 반복에서 같은 변화 방향', loc='left', fontsize=14, pad=7)
    axb = fig.add_axes([.06, .132, .885, .123]); axb.axis('off')
    table2 = axb.table(cellText=[[label, f"{rows[n]['position_auc_09_m_s']:.6f}", f"{rows[n]['sustained_attachment_s']:.6f}",
        f"{rows[n]['execution_clearance_lower_bound_m']:.6f}", f"{rows[n]['linear_command_TV']:.6f}"] for n,label in zip(ORDER,NAMES)],
        colLabels=['R01 방법','위치 AUC 0.9 s [m·s]','지속 부착 [s]','실행 최소 여유 하한 [m]','선형 TV [m/s]'],
        bbox=[0,0,1,1], colWidths=[.18,.215,.19,.235,.18])
    table2.auto_set_font_size(False); table2.set_fontsize(11)
    for tab in [table, table2]:
        for (row,col), c in tab.get_celld().items():
            c.set_edgecolor('#dce4ed'); c.set_linewidth(.7)
            if row == 0: c.set_facecolor('#e8eef5'); c.set_text_props(weight='bold',color='#172d42')
            else: c.set_facecolor('white' if row%2 else '#f1f5f9')
    fig.text(.06, .095, '안전 검사 통과 · 일정 일치 · 13개 비교 지표의 변화 방향 일치. 경로 차이 최대 6.97 mm, 실행 차이 최대 19.98 mm.', fontsize=12, color='#172d42')
    fig.text(.06, .064, '해석 범위: R00·R01의 원시 FRESH는 동일합니다. 같은 시나리오의 재현이며, 다양한 경로나 온라인 배포의 우위를 입증하지 않습니다.', fontsize=11, color='#526376')
    fig.text(.06, .037, '부착: 위치 ≤ .10 m + yaw ≤ 15°를 이후 .30 s 유지  |  안전 여유 기준 .05 m  |  저장 결과만 시각화 · 새 최적화/MPC/LightNav/RGB/Isaac 0회', fontsize=10, color='#526376')
    path = out/'OSA03_R01_results_ko.png'
    fig.savefig(path, dpi=200, facecolor=fig.get_facecolor()); plt.close(fig)
    provenance = dict(figure=str(path), png_sha256=sha(path), pixels=[3200,2600],
        inputs={str(TRACKED):sha(TRACKED), str(RUN/'result_hashes.json'):sha(RUN/'result_hashes.json')},
        script_sha256=sha(Path(__file__)), settings=dict(dpi=200, figsize_inches=[16,13], font='NanumGothic'),
        primary_metrics=rows, paired_effects=summary['replication_effects'], new_scientific_calls=0)
    (out/'provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2)+'\n')
    print(path)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--out', type=Path, required=True)
    export(p.parse_args().out.resolve())
