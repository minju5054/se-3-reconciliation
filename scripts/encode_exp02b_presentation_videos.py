#!/usr/bin/env python3
"""Validate and encode real EXP-02B-R GUI recordings for presentation slide 2."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
from reconciliation.online_switch import sha256_file
from reconciliation.video_timing import validate_video_frames
from encode_exp02d_objective_videos import synchronized_frame_indices, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffmpeg", type=Path, required=True)
    args = parser.parse_args()
    sources = {}
    for path in sorted(args.recordings.resolve().glob("*/recording.json")):
        folder = path.parent
        data = json.loads(path.read_text())
        provenance = json.loads((folder / "provenance.json").read_text())
        if sha256_file(folder / "provenance.json") != data["provenance_sha256"]:
            raise ValueError("recording provenance changed")
        for file, digest in provenance["input_sha256"].items():
            if sha256_file(Path(file)) != digest:
                raise ValueError(f"recording input changed: {file}")
        actual = np.load(folder / "calibrated_actual.npy", allow_pickle=False)
        primary = np.load(Path(provenance["primary_branch"]) / "actual_trajectory.npy", allow_pickle=False)
        if not data["primary_poses_exactly_equal"] or not np.array_equal(primary, actual):
            raise ValueError("recording differs from frozen primary motion")
        validate_video_frames(data["frames"], provenance["physics_dt_s"], provenance["capture_stride"],
                              np.arange(len(actual)) * provenance["physics_dt_s"], actual)
        if not data["comparability_gate"]["valid"] or not data["first_desired_invariant"]["passed"]:
            raise ValueError("boundary or first-command invariant failed")
        for frame in data["frames"]:
            if sha256_file(folder / frame["file"]) != frame["sha256"]:
                raise ValueError("recorded frame changed")
        if sha256_file(folder / "end_card.png") != data["end_card_sha256"]:
            raise ValueError("end card changed")
        alias = {("case_high_delta_omega", 3): "B_k3", ("case_benign_delayed", 0): "C_k0"}[data["case"], data["k"]]
        key = (alias, data["method"])
        if key in sources:
            raise ValueError("duplicate recording")
        sources[key] = (folder, data)
    expected = {(case, method) for case in ("B_k3", "C_k0") for method in ("raw_k", "graph")}
    if set(sources) != expected:
        raise ValueError("both methods for both requested cases are required")
    for case in ("B_k3", "C_k0"):
        cameras = [json.loads((sources[case, method][0] / "camera.json").read_text()) for method in ("raw_k", "graph")]
        if cameras[0] != cameras[1]:
            raise ValueError("comparison camera differs between methods")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    ffmpeg = str(args.ffmpeg.resolve())
    manifest = {"processing_script_sha256": sha256_file(Path(__file__)), "ffmpeg_sha256": sha256_file(Path(ffmpeg)),
        "ffmpeg_version": subprocess.check_output([ffmpeg, "-version"], text=True).splitlines()[0],
        "timing": "2 s initial still + 2 s physics at 4x slow motion + 3 s final still; 13 s per case",
        "frame_processing": "unaltered Isaac window PNGs, aspect-preserving resize, H264; no generated/interpolated poses",
        "comparison": "left RAW and right graph are separate physics reruns at identical B, aligned by simulation time",
        "videos": {}}

    def encode(name, inputs, filters):
        path = output / name
        command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-n", *inputs, *filters,
                   "-c:v", "libx264", "-preset", "fast", "-crf", "18", "-pix_fmt", "yuv420p",
                   "-movflags", "+faststart", str(path)]
        subprocess.run(command, check=True)
        subprocess.run([ffmpeg, "-v", "error", "-i", str(path), "-f", "null", "-"], check=True)
        manifest["videos"][name] = {"sha256": sha256_file(path), "command": command}

    for (case, method), (folder, data) in sources.items():
        if data["timing"]["nominal_simulation_fps"] != 20 or len(data["frames"]) != 41:
            raise ValueError("expected frozen 2 s recording at 20 simulation frames/s")
        sequence = output / "sequences" / f"{case}_{method}"
        sequence.mkdir(parents=True)
        indices = [0] * 10 + synchronized_frame_indices(41, 40, 15)
        for i, index in enumerate(indices):
            original = folder / (data["frames"][index]["file"] if index is not None else "end_card.png")
            (sequence / f"{i:06d}.png").symlink_to(original)
        name = f"{case}_{method}.mp4"
        encode(name, ["-framerate", "5", "-i", str(sequence / "%06d.png")],
               ["-vf", "scale=1920:-2:flags=lanczos,fps=30"])
        manifest["videos"][name].update({"recording": str(folder), "recording_sha256": sha256_file(folder / "recording.json"),
                                         "duration_s": 13, "primary_poses_exactly_equal": True})
    for case in ("B_k3", "C_k0"):
        encode(f"{case}_RAW_GRAPH.mp4", ["-i", str(output / f"{case}_raw_k.mp4"), "-i", str(output / f"{case}_graph.mp4")],
               ["-filter_complex", "[0:v]scale=1920:-2[l];[1:v]scale=1920:-2[r];[l][r]hstack=inputs=2[v]", "-map", "[v]"])
    encode("SLIDE02_B_THEN_C.mp4", ["-i", str(output / "B_k3_RAW_GRAPH.mp4"), "-i", str(output / "C_k0_RAW_GRAPH.mp4")],
           ["-filter_complex", "[0:v][1:v]concat=n=2:v=1:a=0[v]", "-map", "[v]"])
    write_json(output / "video_manifest.json", manifest)
    html = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Exp02B-R · 2쪽 발표 영상</title>
<style>body{font:18px/1.6 sans-serif;background:#10151e;color:#e8edf5;max-width:1600px;margin:32px auto;padding:0 24px}h1{font-size:30px}h2{margin-top:44px}video{width:100%;background:black}a{color:#82b9ff}table{border-collapse:collapse}td,th{padding:9px 20px;border-bottom:1px solid #465365;text-align:left}.note{color:#afbdd0}</style>
<h1>Exp02B-R: controller 보정 후에도 남은 명령 변화와 추종 오차</h1>
<p>왼쪽 RAW suffix, 오른쪽 이전 objective(graph). 두 실행 모두 전환 이후 보정 controller(pi_strong)를 사용했습니다.</p>
<p class="note">Isaac Sim의 실제 물리 재실행 녹화 · 두 실행의 궤적은 기존 정량 실험 기록과 정확히 일치 · 4배 느린 재생</p>
<h2>2쪽 주 영상: 회전 명령 변화가 큰 Case B, k=3</h2>
<video controls preload="metadata" src="B_k3_RAW_GRAPH.mp4"></video>
<p>첫 회전 명령 변화 |Δω|: RAW <b>1.428</b>, graph <b>2.209 rad/s</b>. graph의 회전 추종 RMSE는 보정 전 1.182에서 보정 후 <b>0.482 rad/s</b>로 줄었지만 오차가 남았습니다.</p>
<h2>보조 영상: 원래 명령 변화가 작았던 Case C, k=0</h2>
<video controls preload="metadata" src="C_k0_RAW_GRAPH.mp4"></video>
<p>첫 직진 명령 변화 |Δv|: RAW <b>0.0044</b>, graph <b>0.2410 m/s</b>. graph는 경로 내부 상대운동을 유지하면서 진입점을 <b>38.2 cm</b> 이동시켰습니다.</p>
<p>두 수치는 실제 속도 오차가 아니라 OLD의 마지막 명령과 FRESH의 첫 명령 사이의 차이입니다. 실제 추종 오차는 별도 RMSE로 표시했습니다. 공간 RMS에는 전환점 B와 후보 경로의 초기 간격도 포함되므로 controller만의 오차로 해석하지 않습니다.</p>
<p class="note">파랑: 계획 OLD · 청록: 저장된 실제 OLD · 회색: 원본 FRESH · 주황: graph · 초록: 보정 controller의 실제 주행<br>노랑: 전환점 B · 분홍: 원본 진입점 F_k · 흰색: 현재 실행 경로 진입점(겹칠 수 있음)</p>
<p class="note">OLD는 기존 nominal 실행 기록입니다. 영상의 새 물리 실행은 저장된 B로 정확히 reset한 뒤 시작하며 PI 적분은 0에서 시작합니다. 기존 OLD의 바퀴 목표값을 복구하는 Exp02B-R 절차를 유지했습니다. 각 2초 구간을 보여주는 진단이며, 목표 도달 성공·실패 평가가 아닙니다.</p>
<p><a href="SLIDE02_B_THEN_C.mp4">발표용 두 사례 연속 영상 (26초)</a> · <a href="B_k3_graph.mp4">Case B graph 확대</a> · <a href="C_k0_graph.mp4">Case C graph 확대</a></p>
</html>'''
    with (output / "index.html").open("x", encoding="utf-8") as stream:
        stream.write(html)
    print(f"EXP02B_PRESENTATION_READY={output}")


if __name__ == "__main__":
    main()
