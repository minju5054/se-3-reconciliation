#!/usr/bin/env python3
"""Rebuild the supplied research PDF, preserving evidence and source provenance.

Requires reportlab, pymupdf, Pillow plus the repository's existing dependencies.
No optimization, controller execution, SE(2) transform, or timing interpolation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pymupdf
from PIL import Image
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import Paragraph, Table, TableStyle

from reconciliation.data02_active_old import load_active_old_interval

ROOT = Path(__file__).resolve().parents[1]
BLUE = "#07549A"
INK = "#303840"
GRAY = "#59636E"
ORANGE = "#D77522"
GREEN = "#268252"
PURPLE = "#B83F98"
S2 = "data/exp02d_objective_execution/exp02d-objective-matched-20260913/movies_S2/S2_RAW_Exp02D.mp4"
R1 = "data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00/movies_old_fresh_context/R1_RAW_BEFORE_Exp02D.mp4"
ONLINE = "data/data02_online_successive_v1/data02-online-successive-primary-v1"
CHECK = "data/controller_effect_check/controller-current-20260913T062219Z-r2"
SOURCE_DOCS = [
    "EXP_02D_FINAL_PRESENTATION_SCRIPT_20260914.md",
    "EXP_02B_CONTROLLER_AWARE_RECONCILIATION.md",
    "EXP_02B_GUI_DIAGNOSIS.md",
    "EXP_02B_SLIDE02_GUI_VIDEOS.md",
    "EXP_02C_SLIDE03_DIRECTION_GUI.md",
    "CURRENT_CONTROLLER_EFFECT_CHECK.md",
    "DATA_02_ONLINE_SUCCESSIVE_OLD_FRESH.md",
    "DATA_02_V2_EXTENSION_AND_FINAL_SPLIT.md",
    "EXP_02D_LOOKAHEAD_DIRECTION.md",
    "EXP_02D_MATCHED_OBJECTIVE_VIDEOS.md",
    "EXP_02D_TURNING_EXCLUSIVE_SUCCESS.md",
]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("/home/gpuadmin/Downloads/연구 .pdf"))
    parser.add_argument("--output", type=Path, default=ROOT / "output/pdf")
    parser.add_argument("--ffmpeg", type=Path, required=True)
    args = parser.parse_args()
    cfg_path = ROOT / "configs/research_pdf_revision_20260914.json"
    cfg = json.loads(cfg_path.read_text())
    if sha(args.source) != cfg["source_pdf_sha256"]:
        raise ValueError("Source PDF changed; inspect and explicitly update the revision configuration")
    for relative, expected in cfg["source_sha256"].items():
        if sha(ROOT / relative) != expected:
            raise ValueError(f"Evidence changed: {relative}")
    args.output.mkdir(parents=True, exist_ok=True)
    assets = ROOT / "tmp/pdfs/revision/assets"
    assets.mkdir(parents=True, exist_ok=True)
    source = pymupdf.open(args.source)
    if len(source) != 10 or tuple(source[0].rect) != (0, 0, 720, 405):
        raise ValueError("Unexpected source page layout")
    for key, xref in cfg["source_image_xrefs"].items():
        item = source.extract_image(xref)
        (assets / f"{key}.{item['ext']}").write_bytes(item["image"])
    source[0].get_pixmap(matrix=pymupdf.Matrix(3, 3), clip=pymupdf.Rect(532, 365, 720, 396)).save(assets / "logos.png")

    inputs = {str(args.source): sha(args.source), **cfg["source_sha256"]}
    records = []
    for episode in (14, 44):
        item = load_active_old_interval(ROOT / ONLINE, f"episode_{episode:06d}", 4)
        records.append(item)
    record = records[1]
    source_metadata = record.transition_metadata
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10})
    fig, ax = plt.subplots(figsize=(5.2, 4.6))
    for values, color, label in [(record.old_world, BLUE, "Planned OLD"),
                                 (record.fresh_world, PURPLE, "Raw FRESH"),
                                 (record.display_actual_poses, GREEN, "Measured OLD motion")]:
        ax.plot(values[:, 0], values[:, 1], color=color, lw=2.1, label=label)
    for pose, marker, color, label in [
        (record.observation_pose_world_se2, "o", "#D8BA23", "FRESH observation"),
        (record.p_pose_world_se2, "s", ORANGE, "P"),
        (record.boundary_pose_world_se2, "x", "#C13737", "B: switch"),
    ]:
        ax.scatter(pose[0], pose[1], c=color, marker=marker, s=45, zorder=8, label=label)
    ax.set(xlabel="World X [m]", ylabel="World Y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=.18)
    for pose, label, offset in [
        (record.observation_pose_world_se2, "Obs", (5, -14)),
        (record.p_pose_world_se2, "P", (5, 7)),
        (record.boundary_pose_world_se2, "B", (-13, 6)),
    ]:
        ax.annotate(label, xy=pose[:2], xytext=offset, textcoords="offset points", fontsize=10)
    fig.tight_layout()
    fig.savefig(assets / "episode44.png", dpi=220, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(13, 2.6), gridspec_kw={"width_ratios": [2, 1]})
    replay_records = []
    for ax, active in zip(axes, records, strict=True):
        ref_path = ROOT / CHECK / "references" / f"{active.episode_id}_transition_04.npy"
        ref = np.load(ref_path, allow_pickle=False)
        if not np.array_equal(ref, active.old_world):
            raise ValueError("Online and replay OLD references differ")
        inputs[str(ref_path.relative_to(ROOT))] = sha(ref_path)
        ax.plot(ref[:, 0], ref[:, 1], c=BLUE, label="Saved OLD", lw=2)
        for mode, color, label in [("nominal", ORANGE, "Correction OFF"), ("calibrated", GREEN, "Correction ON")]:
            folder = ROOT / CHECK / "saved_old" / f"{active.episode_id}_transition_04" / mode / "repetition_00"
            raw = folder / "raw/actual_trajectory.npy"
            meta = json.loads((folder / "metadata.json").read_text())
            provenance = json.loads((folder / "derived/raw_provenance.json").read_text())
            # The stored provenance schema is preserved in the manifest without reinterpretation.
            points = np.load(raw, allow_pickle=False)
            inputs[str(raw.relative_to(ROOT))] = sha(raw)
            inputs[str((folder / "metadata.json").relative_to(ROOT))] = sha(folder / "metadata.json")
            ax.plot(points[:, 0], points[:, 1], c=color, label=label, lw=1.8)
            ax.scatter(points[0, 0], points[0, 1], c=color, marker="^", s=32, zorder=5)
            ax.scatter(points[-1, 0], points[-1, 1], c=color, marker="x", s=32, zorder=5)
            replay_records.append({"source": str(folder.relative_to(ROOT)), "metadata": meta, "raw_provenance": provenance})
        ax.set(xlabel="World X [m]", ylabel="World Y [m]", title=f"Episode {int(active.episode_id[-6:])} / transition 04")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.18)
    fig.tight_layout(rect=(0, 0, 1, .88))
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(.5, 1.01),
               ncol=3, fontsize=9, frameon=False)
    fig.savefig(assets / "controller_updated.png", dpi=220, bbox_inches="tight")
    plt.close(fig)

    media = args.output / "media"
    media.mkdir(exist_ok=True)
    for name, relative, seconds in [("s2", S2, 24), ("r1", R1, 14)]:
        movie = ROOT / relative
        shutil.copy2(movie, media / movie.name)
        subprocess.run([str(args.ffmpeg), "-hide_banner", "-loglevel", "error", "-ss", str(seconds),
                        "-i", str(movie), "-frames:v", "1", "-y", str(assets / f"{name}.png")], check=True)

    regular = Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    bold = Path("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf")
    pdfmetrics.registerFont(TTFont("Nanum", str(regular)))
    pdfmetrics.registerFont(TTFont("NanumBold", str(bold)))
    pdfmetrics.registerFontFamily("Nanum", normal="Nanum", bold="NanumBold")
    output = args.output / "연구_수정본.pdf"
    c = canvas.Canvas(str(output), pagesize=(720, 405), invariant=1)
    c.setTitle("온라인 경로 전환 보정 실험: Exp02B, Exp02C, Exp02D")
    c.setAuthor("조민주")
    c.setSubject("2026-09-14 발표 수정본. 원본 기록과 별도 재실행을 구분한 11쪽.")
    layout = []
    page_number = 0

    def text(x, y, message, size=12, width=642, color=INK, bold=False, leading=None):
        style = ParagraphStyle("body", fontName="NanumBold" if bold else "Nanum", fontSize=size,
                               leading=leading or size * 1.42, textColor=colors.HexColor(color), wordWrap="CJK")
        p = Paragraph(message.replace("\n", "<br/>"), style)
        w, h = p.wrap(width, 405)
        if x + w > 708.01 or y + h > 399:
            raise ValueError(f"Text outside slide {page_number}: {message}")
        p.drawOn(c, x, 405 - y - h)
        layout.append({"page": page_number, "kind": "text", "box": [x, y, x + w, y + h], "text": message})
        return h

    def pic(name, x, y, w, h=None):
        path = assets / name
        im = Image.open(path)
        ih = w * im.height / im.width
        if h is not None and ih > h:
            w *= h / ih
            ih = h
        c.drawImage(str(path), x, 405-y-ih, width=w, height=ih, mask="auto")
        layout.append({"page": page_number, "kind": "image", "box": [x, y, x + w, y + ih], "source": name})
        return w, ih

    def table(rows, x, y, widths, row_height=26, size=11, emphasis=()):
        normal = ParagraphStyle("cell", fontName="Nanum", fontSize=size, leading=size*1.3, wordWrap="CJK", textColor=colors.HexColor(INK))
        data = [[Paragraph(str(v), normal) for v in row] for row in rows]
        t = Table(data, colWidths=widths, rowHeights=[row_height]*len(rows))
        rules = [("BACKGROUND", (0,0), (-1,0), colors.HexColor("#E8F0F8")),
                 ("LINEBELOW", (0,0), (-1,0), .6, colors.HexColor("#799BBF")),
                 ("VALIGN", (0,0), (-1,-1), "MIDDLE"),
                 ("LEFTPADDING", (0,0), (-1,-1), 8), ("RIGHTPADDING", (0,0), (-1,-1), 6)]
        for r in range(1,len(rows)):
            rules.append(("LINEBELOW", (0,r), (-1,r), .3, colors.HexColor("#D8DDE3")))
        for r in emphasis:
            rules.append(("BACKGROUND", (0,r), (-1,r), colors.HexColor("#FFF2DB")))
        t.setStyle(TableStyle(rules))
        tw, th = t.wrap(700,405)
        t.drawOn(c,x,405-y-th)
        layout.append({"page":page_number,"kind":"table","box":[x,y,x+tw,y+th]})

    def page(title, subtitle=None, size=21):
        nonlocal page_number
        page_number += 1
        c.setFillColor(colors.HexColor(BLUE)); c.rect(18.5,351.5,17.5,53.5,fill=1,stroke=0)
        c.setFillColor(colors.HexColor("#3171BB")); c.rect(708,40,12,365,fill=1,stroke=0)
        c.setFillColor(colors.HexColor("#FFAB0F")); c.rect(0,4,720,4,fill=1,stroke=0)
        pic("logos.png", 534, 368, 174)
        text(48,16,title,size=size,bold=True,width=645,color="#555B60",leading=size*1.22)
        if subtitle:
            text(40,53,subtitle,size=10.8,color=GRAY)
        text(490,377,f"{page_number:02d}",size=8,width=25,color=GRAY)

    def foot(message):
        text(39,365,message,size=7.1,width=443,color=GRAY,leading=9.2)

    def done():
        c.showPage()

    def movie(name, poster, y, caption):
        w,h=pic(poster,35,y,650)
        c.linkURL(f"media/{name}",(35,405-y-h,35+w,405-y),relative=0,thickness=0,kind="GoToR")
        text(40,y+h+4,caption,size=8.6,color=BLUE,width=645)

    page("온라인 경로 전환 보정 실험", "조민주    연구 기간: 2026.09.07 ~ 2026.09.13", size=24)
    text(42,100,"OLD에서 FRESH로 바뀔 때의 명령 변화를 줄일 수 있는가?",size=18,bold=True)
    table([
        ["연구 단계", "이번에 확인한 내용"],
        ["EXP-02B / 02C", "기존 경로 보정의 악화 사례와 direction 항의 과보정 원인"],
        ["Online data", "주행 중 전환 기록 수집, 44번의 계획 경로와 실제 이동 차이"],
        ["EXP-02D", "lookahead 기준으로 방향 항 변경, 조건별 이득과 한계 평가"],
    ],40,157,[146,496],row_height=36,size=12)
    text(42,322,"개선은 정상 전환의 과보정 감소와 일부 곡선 사례에 한정된다.",size=15,bold=True,color=BLUE)
    foot("Controller의 회전·종료 검증이 남은 상태에서 진행한 개발용 분석과 사례 비교.")
    done()

    page("EXP-02B: 경로 보정과 실행 오차를 분리", "실제 LightNav 전환 3개 × 진입점 k=0,3,6 × 방법 5개 = 45개 branch")
    table([["사례 / 전환 직후 희망 명령 변화", "RAW", "기존 graph"],
           ["회전 사례 B, k=3: |Δω| (rad/s)","1.4280","2.2092"],
           ["정상 사례 C, k=0: |Δv| (m/s)","0.0044","0.2410"]],40,79,[382,130,130],row_height=24,size=11.1,emphasis=(2,))
    pic("failure.png",40,165,296)
    text(41,351,"발표 영상: exp02b_failure.webm",size=9,width=310,color=BLUE)
    text(362,165,"보정 전 OLD 실행 진단",size=14,bold=True,width=326)
    text(362,190,"평균 회전 요청 0.755, 측정 0.139 rad/s\nOLD까지 공간 RMS 6.46 cm",size=12,width=326)
    text(362,236,"실행 보정 후 재검사: Exp02B-R",size=13.5,bold=True,width=326)
    text(362,260,"B k=3 graph의 회전 실행 RMSE는\n1.1825에서 0.4816 rad/s로 감소.\n같은 B에서 첫 희망 명령의 악화는 유지.",size=11.5,width=326)
    text(362,322,"실행 오차가 남아 있는 상태에서\nobjective의 원인을 별도로 분석했다.",size=12,bold=True,width=326,color=BLUE)
    foot("영상 화면은 보정 전 저장 OLD 자세 재생. 바퀴·접촉 동역학의 재실행이 아님.\n근거: EXP_02B_GUI_DIAGNOSIS, EXP_02B_SLIDE02_GUI_VIDEOS")
    done()

    page("EXP-02C: direction 항이 만든 과보정", "Case C, k=0. 같은 입력·solver·follower에서 factor 구성만 비교")
    table([["경로 보정 구성","첫 |Δv| (m/s)","진입점 이동 (cm)"],
           ["RAW","0.00442","0"],
           ["FULL: E+D+Y+F","0.24097","38.2"],
           ["D 제거: E+Y+F","0.00442","거의 0"],
           ["Y 제거: E+D+F","0.24097","38.2"]],40,77,[328,155,159],row_height=23,size=10.7,emphasis=(2,))
    pic("ablation.png",27,209,449)
    text(510,207,"D와 follower의 기준점 차이",size=12.3,width=182,bold=True)
    text(510,233,"기존 D는 뒤쪽 첫 점 F0,\nfollower는 앞쪽 F3를 봄.\nD가 진입점을 앞으로 옮기면\nF가 뒤쪽 경로까지 이동시킴.",size=11.5,width=182)
    text(510,302,"38.2 cm = 경로 진입점 변위\n실제 추가 주행 거리는 아님",size=11.5,width=182,bold=True,color=ORANGE)
    text(42,346,"이 고정 조건에서 D를 제거하자 큰 경로 이동과 전진 명령 악화가 사라졌다.",size=12.5,bold=True,color=BLUE)
    foot("E: 진입점 보존, D: 진입 방향, Y: yaw 연결, F: 경로 내부 상대운동 보존.\n저장 world XY[m]의 경로 최적화 결과. 새 물리 주행 결과가 아님. 근거: EXP_02C_SLIDE03_DIRECTION_GUI")
    done()

    page("온라인 전환 데이터 수집", "Hospital의 Jackal이 OLD를 실행하는 동안 LightNav에 다음 FRESH를 요청")
    for y, head, body in [
        (92,"1. 초기 관측과 OLD 생성","카메라 64장을 4 Hz로 수집하고 첫 경로를 실행."),
        (143,"2. 주행 중 FRESH 요청","실행 시작 0.5초 뒤 요청. 추론 중에도 OLD 실행을 지속."),
        (194,"3. 다음 제어 시점에 전환","FRESH가 준비되면 B에서 원본 경로로 전환. 수집 단계 보정 없음."),
        (245,"4. 같은 에피소드에서 반복","받은 FRESH를 다음 OLD로 사용. 에피소드당 최대 6회 시도."),
    ]:
        text(42,y,head,size=14,bold=True,width=462)
        text(42,y+23,body,size=11.2,width=462)
    text(535,99,"959",size=34,bold=True,width=155,color=BLUE)
    text(535,150,"유효 전환 (v1+v2)",size=12,width=155)
    text(535,198,"191",size=34,bold=True,width=155,color=BLUE)
    text(535,250,"고유 raw pair",size=12,width=155)
    text(42,312,"원본 경로·영상, 관측·ready·P·B의 자세와 시각, 명령·차체·바퀴 기록을 저장.",size=12)
    text(42,335,"수집 당시 실행 보정 pi_strong은 이미 ON. 반복 pair가 있어 개발용 분석으로 한정.",size=11.6,bold=True,color=BLUE)
    foot("경로는 각 관측 자세로 world에 배치. B로 재정렬하지 않음. waypoint 고유 시각 없음.\n근거: DATA_02_ONLINE_SUCCESSIVE_OLD_FRESH, DATA_02_V2_EXTENSION_AND_FINAL_SPLIT")
    done()

    page("온라인 수집 예시: 계획 경로와 실제 움직임", "전체 데이터 중 12개 전환 예시. 다음 쪽에서 episode 44 / transition 04를 확대")
    pic("online12.png",20,73,416)
    text(465,89,"파랑  계획 OLD",size=13,bold=True,color=BLUE,width=225)
    text(465,117,"자홍  새로 추론한 FRESH",size=13,bold=True,color=PURPLE,width=225)
    text(465,145,"초록  당시 측정한 OLD 주행",size=13,bold=True,color=GREEN,width=225)
    table([["표식","의미"],["노란 원","FRESH 관측 위치"],["주황 P","B의 0.1초 전 위치"],["빨강 X, B","FRESH 전환 위치"]],462,190,[75,154],row_height=29,size=10.6)
    text(466,323,"44번은 전진 명령이 0인데도\nOLD에서 멀어지는 이동을 보임.",size=12.1,bold=True,width=222)
    foot("축: 원본 Isaac world XY[m]. 계획 선과 실제 OLD 활성 구간을 구분. B는 전환점.\n원본 PDF의 12개 패널과 사례 순서를 보존. 44번은 다음 쪽에서 상세 분석.")
    done()

    page("44번: 경로 간 간격과 실제 이동의 차이", "v1: episode_000044_transition_04. 온라인 OLD 활성 구간 약 1.4초")
    pic("episode44.png",29,86,302,250)
    text(42,340,"파랑 OLD / 자홍 FRESH / 초록 실제 OLD",size=9.5,width=302)
    text(354,85,"OLD / FRESH는 서로 다른 관측에서 생성",size=13,bold=True,width=339)
    text(354,111,"OLD 길이 6.93 cm, FRESH 길이 8.94 cm.\n각 관측 자세에 배치하므로 시작점이 다르다.\nFRESH 관측 후 B까지도 차체가 9.19 cm 이동.",size=11.7,width=339)
    text(354,181,"명령과 실제 이동이 일치하지 않은 구간",size=13,bold=True,width=339)
    text(354,207,"전진 요청 0 m/s, 회전 요청 +1.5 rad/s.\nOLD까지 최근접 거리는 3.92에서 17.65 cm로 증가.\n초록 선은 실제 XY 기록이며, 원본 OLD와 다르다.",size=11.7,width=339)
    text(354,277,"원인 판단의 범위",size=13,bold=True,width=339)
    text(354,303,"별도 검사에서도 회전 실행의 한계는 남았다.\n이 44번의 장애물 접촉·바퀴 헛돎·관성 기여는\n분리하지 않아 하나의 원인으로 확정할 수 없다.",size=11.5,width=339,color=BLUE)
    foot("원본 sim s: OLD 관측 21.533, FRESH 관측 22.783, ready 23.467, 전환 B 23.533.\n원본 world XY[m] 그대로, 추가 정렬 없음. 근거: transition.json, audit_saved_old.json")
    done()

    page("실행 보정: 일부 곡선 개선, 회전·종료 한계 잔존", size=19.4)
    text(40,52,"파란 OLD는 앞쪽 온라인 사례와 동일. 평지 reset 후 보정 OFF / ON으로 별도 실행.",size=11.4)
    text(40,72,"직전 속도·PI·접촉·전환 이력은 미복원. 1초 settling 후 도달 또는 최대 8초까지 실행.",size=10.7,color=GRAY)
    pic("controller_updated.png",36,94,652,142)
    table([["저장 OLD","위치 RMS (cm), OFF / ON","최대 이탈 (cm), OFF / ON"],
           ["14번 / 04","6.71 / 1.45","8.41 / 2.93"],
           ["44번 / 04","21.80 / 17.32","27.04 / 37.33"]],40,246,[128,257,257],row_height=21,size=10.7,emphasis=(2,))
    text(42,316,"종료는 위치 8 cm와 yaw 0.08 rad 기준. 회전 중 위치 범위를 벗어나면 재접근할 수 있다.",size=10.9)
    text(42,341,"당시 이 한계를 충분히 확인하지 못한 채 다음 Exp02D 실험으로 진행했다.",size=12.3,bold=True,color="#A64C22")
    foot("삼각형: 새 실행 시작, X: 종료. 표는 모드별 3회 평균, 그래프는 동일 결과의 첫 반복.\n실행 전체 RMS이며 모드별 지속 시간이 다름. 근거: CURRENT_CONTROLLER_EFFECT_CHECK")
    done()

    page("EXP-02D: lookahead의 효과는 조건별로 다름", "기존 D의 진입점 Fk 대신 follower의 앞쪽 목표점 Fq를 방향 기준으로 사용")
    text(41,79,"다른 항과 가중치는 유지. 동일 B에서 OLD 마지막 명령과 새 경로의 첫 희망 명령을 비교.",size=11.3)
    table([["명령 변화 구간","RAW","기존 FULL","D 제거","Lookahead"],
           ["전체 959개","1.394","0.918","1.208","0.872"],
           ["정상 685개","0.180","0.544","0.163","0.177"],
           ["중간 178개","0.678","0.648","0.638","0.905"],
           ["큰 변화 96개","2.978","1.419","2.532","1.383"]],40,114,[190,105,117,111,119],row_height=28,size=11.2)
    text(41,268,"정상 전환의 불필요한 보정은 감소. 중간 집단의 종합 점수는 악화.",size=13,bold=True)
    text(41,296,"전체 FULL 대비 차이의 95% 구간은 [-0.150, +0.067]로 0을 포함.\n평균 회전 명령 변화도 0.1621에서 0.1809 rad/s로 증가했다.",size=11.7)
    text(41,336,"확인한 개선 범위는 제한적이며, 일반적인 주행 우위는 입증하지 못했다.",size=13,bold=True,color=BLUE)
    foot("표: 정규화 명령 변화 J, 낮을수록 좋음. raw pair 균등 평균.\nJ = sqrt((|Δv|/0.15 m/s)² + (|Δω|/0.30 rad/s)²). 근거: EXP_02D_LOOKAHEAD_DIRECTION")
    done()

    page("S2: 명령 개선이 실제 도달을 보장하지 않음", "episode 27 / transition 01. 왼쪽 RAW, 오른쪽 Exp02D")
    movie(Path(S2).name,"s2.png",79,"S2_RAW_Exp02D.mp4  (화면 클릭으로 별도 영상 열기)")
    text(41,297,"각자 경로 끝점 도달: RAW 3/3, 이전 objective 0/3, Exp02D 0/3",size=13,bold=True)
    text(41,324,"Exp02D는 수납 카트 앞에서 바퀴가 회전해도 차체가 정체.\n보정으로 경로가 이동했고 장애물 여유 제약은 없으나, 접촉의 단일 원인은 미확정.",size=11.7)
    foot("같은 B에서 reset 후 물리 실행, 원래 온라인 동역학 미복원, 2배 느린 영상.\n주황은 이전 경로 참고선. 이전 방식의 0/3은 별도 실행 결과. 근거: EXP_02D_MATCHED_OBJECTIVE_VIDEOS")
    done()

    page("R1: 이전 방식 대비 국소적인 곡선 개선", "episode 16 / transition 02. 왼쪽 RAW, 가운데 이전 objective, 오른쪽 Exp02D")
    movie(Path(R1).name,"r1.png",77,"R1_RAW_BEFORE_Exp02D.mp4  (화면 클릭으로 별도 영상 열기)")
    table([["방법","각자 끝점 도달","yaw RMS (rad)","종료 (s)"],
           ["RAW","3/3","0.100543","4.7"],
           ["이전 objective","3/3","0.852554","13.2"],
           ["Exp02D","3/3","0.092446","4.1"]],40,230,[185,154,164,139],row_height=22,size=10.8)
    text(41,327,"이전 방식의 반복 회전·재접근은 감소. RAW도 끝점에는 도달했으며\nyaw 기준 0.1 rad를 근소하게 초과해, RAW 대비 큰 우위로 일반화할 수 없다.",size=11.7,bold=True,color=BLUE)
    foot("87개 곡선 후보의 사후 검색 사례, 같은 초기화 3회, 각자 경로 기준 평가.\n공유 follower의 종료 문제를 해결했다는 결과가 아님. 근거: EXP_02D_TURNING_EXCLUSIVE_SUCCESS")
    done()

    page("결론과 다음 검증", "Direction 설계의 국소적 개선과 실행 계층의 미해결 문제를 함께 보고")
    text(42,89,"Lookahead는 정상 전환의 과보정을 줄였지만,\n항상 더 좋은 명령이나 실제 주행 성공을 보장하지 않았다.",size=18,bold=True,color=BLUE)
    table([["이번에 확인한 개선","남아 있는 한계"],
           ["정상 전환의 불필요한 경로 보정 감소","중간 집단 악화, 전체 우위 불확실"],
           ["R1에서 이전 objective 대비 종료 지연 감소","회전·종료 제어와 장애물 앞 정체 미해결"]],40,166,[321,321],row_height=32,size=11.8)
    text(42,278,"다음 검증 순서",size=14,bold=True)
    text(42,302,"1. 저장 경로로 follower의 접근·최종 방향·종료 처리와 하위 회전 응답 검증\n2. 같은 전환 조건에서 경로 변형·장애물 여유·실제 도달을 나누어 평가\n3. Controller 버전을 고정한 뒤 새 온라인 데이터를 수집하고 재평가",size=11.5,leading=18.5)
    foot("기존 raw 기록은 당시 실행 조건의 증거로 보존. 장애물 factor는 필요성 확인 후 설계할 후속 작업.")
    done()
    c.save()

    for i, a in enumerate(layout):
        for b in layout[i+1:]:
            if a["page"] != b["page"]:
                continue
            ax, ay, bx, by = a["box"]
            cx, cy, dx, dy = b["box"]
            if min(bx, dx)-max(ax, cx) > 1 and min(by, dy)-max(ay, cy) > 1:
                raise ValueError(f"Overlapping layout objects on page {a['page']}: {a} / {b}")

    # A GoToR action is for a second PDF. Convert the media actions to file launch
    # links so supporting viewers hand MP4s to the system's video player.
    linked = pymupdf.open(output)
    for p in linked:
        for link in p.get_links():
            target = link.get("file") or link.get("uri")
            area = link["from"]
            p.delete_link(link)
            p.insert_link({"kind": pymupdf.LINK_LAUNCH, "from": area, "file": target})
    linked_path = assets / "linked.pdf"
    linked.save(linked_path, garbage=4, deflate=True, no_new_id=True)
    linked.close()
    shutil.copy2(linked_path, output)

    # Ensure every media link is relative and resolves in the portable package.
    pdf = pymupdf.open(output)
    if len(pdf) != 11:
        raise ValueError("Unexpected output page count")
    if any(abs(p.rect.width-720) > .001 or abs(p.rect.height-405) > .001 for p in pdf):
        raise ValueError("Page size changed")
    all_text = "\n".join(p.get_text() for p in pdf)
    for expected in ["38.2", "17.65", "37.33", "0.100543", "충분히 확인하지 못한", "exp02b_failure.webm"]:
        if expected not in all_text:
            raise ValueError(f"Missing slide content: {expected}")
    links = [v for p in pdf for v in p.get_links()]
    if len(links) != 2:
        raise ValueError(f"Expected two media links, got {links}")
    for link in links:
        target = link.get("file") or link.get("uri")
        if not target or not (args.output / target).is_file():
            raise ValueError(f"Broken media link: {link}")
    pdf.close()

    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": str(cfg_path.relative_to(ROOT)), "configuration_sha256": sha(cfg_path),
        "builder_sha256": sha(Path(__file__)), "source_inputs_sha256": inputs,
        "output_pdf_sha256": sha(output), "page_count": 11, "page_size_pt": [720,405],
        "coordinate_convention": "Source and target: original Isaac world XY[m], yaw[rad], CCW about +Z. No transform, alignment, axis swap, or pose interpolation. Display axes use equal aspect. Unit display cm=m*100 only.",
        "timing_convention": "Original simulation seconds. OLD activation, OLD/FRESH observation, readiness, P and B remain distinct. Display may prepend the prior saved switch pose using the existing validated loader; it is not a new measurement. Replay starts after reset and settling; original online dynamics are not restored.",
        "online_records": [{"episode": a.episode_id, "source_hashes": dict(a.source_sha256),
                            "activation_sim_s": a.activation_sim_time_s, "observation_sim_s": a.observation_sim_time_s,
                            "readiness_sim_s": a.model_ready_sim_time_s, "P_sim_s": a.p_sim_time_s, "B_sim_s": a.switch_sim_time_s,
                            "activation_boundary_prepended": a.activation_boundary_prepended} for a in records],
        "replay_records": replay_records,
        "episode44_source_metrics": source_metadata["metrics"],
        "media": {"S2": S2, "R1": R1, "poster_video_seconds": {"S2":24,"R1":14},
                  "exp02b_failure.webm": "Not supplied/found. Retained original PDF still and requested filename. No video link or substitute recording invented."},
        "layout_elements": layout,
        "validation": {"page_count_and_dimensions":True, "required_copy":True,
                       "media_links_resolve":True, "layout_objects_do_not_overlap":True},
        "no_new_experiment": True,
    }
    (args.output / "revision_manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n")
    readme = """연구 발표 수정본 (11쪽)

연구_수정본.pdf와 media 폴더를 같은 폴더에 둡니다.
9, 10쪽의 화면은 실제 MP4에서 추출한 정지 화면입니다.
화면 클릭을 지원하지 않는 PDF 뷰어에서는 media의 MP4를 직접 재생합니다.
PDF 내부 자동 재생 영상은 아닙니다. 동영상은 모두 2배 느린 재생입니다.

exp02b_failure.webm 원본은 제공 경로와 저장소에서 찾지 못했습니다.
2쪽에는 기존 PDF의 화면과 요청된 파일명을 유지했습니다.
해당 화면은 보정 전 OLD 자세 기록 재생이며, 보정 후 물리 실행 영상이 아닙니다.

수정 요지: 38.2cm는 경로 진입점 변위. 온라인/평지 재실행의 44번 구분.
Controller 검증 미완료 상태로 진행한 한계 명시. Lookahead의 효과는 조건별·국소적.
원본 PDF 및 raw 기록은 변경하지 않았습니다.
"""
    (args.output / "영상_안내.txt").write_text(readme)
    bundle = args.output / "연구_발표자료.zip"
    with zipfile.ZipFile(bundle,"w",zipfile.ZIP_DEFLATED) as z:
        for path in [output,args.output/"영상_안내.txt",*sorted(media.glob("*.mp4"))]:
            z.write(path,path.relative_to(args.output))
    print(json.dumps({"pdf":str(output),"bundle":str(bundle),"pages":11,
                      "media_targets":[link.get("file") or link.get("uri") for link in links],
                      "source_unchanged":sha(args.source)==cfg["source_pdf_sha256"]},ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
