#!/usr/bin/env python3
"""Explain saved real Exp02C Case C k=0 with trajectory figures and a native GUI.

This reads frozen results; it neither optimizes paths nor executes robot physics.
All plotted positions remain in the saved Isaac world frame, in metres.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from reconciliation.exp02b import load_source_case, transition_input
from reconciliation.exp02c_factor_isolation import controller_desired_metrics, geometry_metrics
from reconciliation.online_switch import sha256_file
from reconciliation.transition_graph import incoming_motion_residual

DEFAULT_RUN = ROOT / "data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z"
VARIANTS = {"RAW": "V0_RAW", "E+F": "V1_ENTRY_FRESH", "D added": "V2_ENTRY_DIRECTION_FRESH",
            "FULL": "V4_FULL_CURRENT_M4", "No D": "V6_NO_DIRECTION",
            "No propagation": "V8_DIAGNOSTIC_NO_PROPAGATION"}
COLORS = {"raw": "#65717d", "old": "#4774c5", "incoming": "#167c91", "D": "#cb3f39",
          "FULL": "#dc720f", "No D": "#16805d", "D added": "#c36515",
          "target": "#8763b0", "No propagation": "#a44291"}


def direction_geometry(previous, boundary, raw_entry, follower_target):
    """World-XY display geometry; target T makes only the direction residual zero."""
    p, b, f, q = (np.asarray(x, dtype=float) for x in (previous, boundary, raw_entry, follower_target))
    if any(x.shape != (3,) or not np.isfinite(x).all() for x in (p, b, f, q)):
        raise ValueError("expected finite world SE(2) poses")
    incoming, radial = b[:2] - p[:2], f[:2] - b[:2]
    step, radius = np.linalg.norm(incoming), np.linalg.norm(radial)
    if min(step, radius) <= 1e-6:
        raise ValueError("direction is undefined at zero displacement")
    unit = incoming / step
    target = b[:2] + radius * unit
    angle = float(np.degrees(np.arccos(np.clip((radial / radius) @ unit, -1, 1))))
    return {"incoming_unit_world_xy": unit.tolist(), "old_step_m": float(step), "raw_radius_m": float(radius),
            "direction_zero_target_world_xy": target.tolist(), "raw_to_direction_target_m": float(np.linalg.norm(target - f[:2])),
            "incoming_vs_raw_entry_angle_deg": angle, "raw_entry_forward_projection_m": float(radial @ unit),
            "raw_follower_target_forward_projection_m": float((q[:2] - b[:2]) @ unit),
            "semantics": "T solves only incoming-direction residual=0 at frozen raw radius; not the FULL optimum or a robot target"}


def read_evidence(run):
    hashes = {}

    def record(path):
        path = Path(path)
        hashes[str(path)] = sha256_file(path)
        return path

    def read_json(path):
        return json.loads(record(path).read_text())

    def read_array(path):
        return np.load(record(path), allow_pickle=False)

    provenance = read_json(run / "source_provenance.json")
    source_record = provenance["real_cases"]["case_benign_delayed"]
    trial = Path(source_record["trial_directory"])
    for name, expected in source_record["source_file_sha256"].items():
        if sha256_file(record(trial / name)) != expected:
            raise ValueError(f"frozen source changed: {name}")
    config_path = ROOT / "configs/exp01b_controlled_latency.yaml"
    if sha256_file(record(config_path)) != provenance["exp01b_config_sha256"]:
        raise ValueError("frozen follower config changed")
    config = yaml.safe_load(config_path.read_text())
    source = load_source_case("case_benign_delayed", trial)
    inputs = transition_input(source, 0)
    variants = {}
    for name, variant in VARIANTS.items():
        directory = run / "real/case_benign_delayed/k_0" / variant
        raw = read_array(directory / "raw_fresh.npy")
        if not np.array_equal(raw, source.fresh_world):
            raise ValueError("variant input is not the exact frozen FRESH")
        path = read_array(directory / "optimized.npy")
        command = read_json(directory / "controller_desired_metrics.json")
        geometry = read_json(directory / "geometry_metrics.json")
        rigid = read_json(directory / "rigid_fit_metrics.json")
        metadata = read_json(directory / "metadata.json")
        fresh_command = controller_desired_metrics(path, boundary=source.boundary_pose_world,
            old_final_command=source.old_commands[-1], follower_values=config["closed_loop"])
        fresh_geometry = geometry_metrics(inputs, path, minimum_translation_m=1e-6)
        for key in ("delta_v_des_abs_mps", "delta_omega_des_abs_rps", "candidate_first_desired_v_omega"):
            np.testing.assert_allclose(fresh_command[key], command[key], rtol=0, atol=1e-12)
        if fresh_command["follower_state"] != command["follower_state"]:
            raise ValueError("follower target changed")
        for key in ("entry", "endpoint"):
            np.testing.assert_allclose(fresh_geometry[key]["translation_displacement_m"],
                                       geometry[key]["translation_displacement_m"], rtol=0, atol=1e-12)
        variants[name] = {"variant": variant, "path": path, "command": command, "geometry": geometry,
                          "rigid": rigid, "active_factors": metadata["active_factors"]}
    for name in ("RAW", "E+F"):
        if not np.array_equal(variants[name]["path"], source.fresh_world):
            raise ValueError("expected exact RAW / E+F no-op")
    raw = source.fresh_world
    target_index = variants["RAW"]["command"]["follower_state"][0]["target_index"]
    mechanism = direction_geometry(source.previous_pose_world, source.boundary_pose_world, raw[0], raw[target_index])
    target_pose = np.r_[mechanism["direction_zero_target_world_xy"], raw[0, 2]]
    residual = incoming_motion_residual(source.previous_pose_world, source.boundary_pose_world, target_pose, raw[0], minimum_translation_m=1e-6)
    np.testing.assert_allclose(residual[:2], 0, rtol=0, atol=1e-12)
    full_cost = read_json(run / "real/case_benign_delayed/k_0/V4_FULL_CURRENT_M4/factor_costs_initial.json")
    return {"run": run, "source": source, "variants": variants, "mechanism": mechanism,
            "target_index": target_index, "input_sha256": hashes, "full_initial_factor_costs": full_cost,
            "follower_values": config["closed_loop"]}


def plot_style():
    import matplotlib
    from matplotlib import font_manager
    font = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
    font_manager.fontManager.addfont(font)
    matplotlib.rcParams.update({"font.family": "NanumGothic", "font.size": 13, "axes.unicode_minus": False,
        "axes.spines.top": False, "axes.spines.right": False, "axes.titleweight": "bold",
        "axes.labelcolor": "#4f5963", "text.color": "#172633", "svg.fonttype": "none"})


def axes_style(ax, *, zoom=False):
    ax.set_aspect("equal", adjustable="box")
    ax.set_anchor("N")
    ax.set_xlim((.19, .94) if zoom else (.10, 2.03))
    ax.set_ylim((-8.035, -7.405) if zoom else (-8.10, -7.02))
    ax.set_xlabel("World X [m]", fontsize=11)
    ax.set_ylabel("World Y [m]", fontsize=11)
    ax.tick_params(labelsize=10)
    ax.grid(alpha=.16)


def arrow(ax, start, end, color, *, width=2.4, style="-|>", z=7):
    return ax.annotate("", xy=np.asarray(end)[:2], xytext=np.asarray(start)[:2],
        arrowprops={"arrowstyle": style, "color": color, "lw": width, "mutation_scale": 15}, zorder=z)


def label(ax, point, text, offset, *, color="#172633", size=12):
    return ax.annotate(text, xy=np.asarray(point)[:2], xytext=offset, textcoords="offset points",
        color=color, fontsize=size, ha="left", va="center", zorder=20,
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9, "pad": 2},
        arrowprops={"arrowstyle": "-", "color": color, "lw": .8})


def base(ax, e, *, zoom=False):
    source = e["source"]
    raw, old = source.fresh_world, source.old_world
    ax.plot(old[:, 0], old[:, 1], color=COLORS["old"], lw=5, alpha=.30, label="계획 OLD")
    ax.plot(raw[:, 0], raw[:, 1], "o--", color=COLORS["raw"], lw=1.9, ms=4, label="원본 FRESH")
    p, b = source.previous_pose_world, source.boundary_pose_world
    arrow(ax, p, b, COLORS["incoming"], width=3)
    ax.scatter(*p[:2], color=COLORS["incoming"], s=24, zorder=12)
    ax.scatter(*b[:2], color="#ffd666", edgecolor="#172633", s=100, zorder=12)
    axes_style(ax, zoom=zoom)
    return raw, p, b


def mechanism_plot(ax, e, *, detailed=False):
    raw, p, b = base(ax, e, zoom=True)
    q = e["target_index"]
    f, target = raw[0], raw[q]
    arrow(ax, b, f, COLORS["D"], width=3)
    arrow(ax, b, target, COLORS["No D"], width=3)
    ax.scatter(*f[:2], marker="s", color=COLORS["D"], s=75, zorder=15)
    ax.scatter(*target[:2], marker="D", color=COLORS["No D"], s=75, zorder=15)
    label(ax, f, "F0: 뒤에 남은 첫 점", (-12, -30), color=COLORS["D"])
    label(ax, p, "P: 직전 위치", (-90, 34), color=COLORS["incoming"], size=11)
    label(ax, b, "B: 현재 위치", (-16, -34), size=12)
    label(ax, target, f"F{q}: follower 목표점", (-20, 48), color=COLORS["No D"])
    ax.text(.03, .95, "D는 B→F0를 본다\nfollower는 B→F3를 본다", transform=ax.transAxes,
            va="top", fontsize=13, linespacing=1.6)
    ax.text(.02, .03, f"진행 방향 P→B와 B→F0의 각도: {e['mechanism']['incoming_vs_raw_entry_angle_deg']:.1f}°",
            transform=ax.transAxes, fontsize=11)
    if detailed:
        point = e["mechanism"]["direction_zero_target_world_xy"]
        ax.scatter(*point, marker="x", s=100, color=COLORS["target"], zorder=16)
        label(ax, point, "T: direction 항만 0이 되는 위치\n전체 objective의 해는 아님", (20, -85),
              color=COLORS["target"], size=12)
    return ax


def variant_plot(ax, e, name, *, explanatory=True):
    raw, p, b = base(ax, e)
    item = e["variants"][name]
    path = item["path"]
    color = COLORS.get(name, COLORS["raw"])
    ax.plot(path[:, 0], path[:, 1], "o-", lw=2.7, ms=4.5, color=color, label=name, zorder=5)
    ax.scatter(*raw[0, :2], marker="s", color=COLORS["raw"], s=45, zorder=8)
    ax.scatter(*path[0, :2], marker="s", color=color, s=70, zorder=10)
    label(ax, b, "B", (-2, -28), size=11)
    if name in ("FULL", "D added"):
        arrow(ax, raw[0], path[0], color, width=1.4, style="<->")
        arrow(ax, raw[-1], path[-1], color, width=1.4, style="<->")
        shift = item["geometry"]["entry"]["translation_displacement_m"] * 100
        label(ax, (raw[0] + path[0]) / 2, f"진입점 {shift:.1f} cm 이동", (-45, 45), color=color)
        label(ax, (raw[-1] + path[-1]) / 2, "끝점도 약 38.2 cm 이동", (-107, 34), color=color)
        if explanatory:
            ax.text(.03, .02, "상대 모양을 유지한 채\n뒤쪽 waypoint까지 함께 이동", transform=ax.transAxes,
                    va="bottom", fontsize=12, linespacing=1.5)
    elif name == "No propagation":
        label(ax, path[0], "첫 점만 이동", (10, -50), color=color)
        label(ax, raw[1], "뒤쪽 점을 고정해 생긴 역방향 꺾임", (0, 65), color=color, size=11)
        label(ax, raw[-1], "끝점은 원본 그대로", (-65, 35), color=color)
    else:
        label(ax, raw[0], "진입점 유지", (-4, 31), color=color)
        label(ax, raw[-1], "원본 FRESH와 거의 겹침", (-105, 38), color=color)
    ax.legend(loc="upper left", fontsize=10, frameon=False)
    return ax


def export_figures(e, output):
    import matplotlib.pyplot as plt
    output.mkdir(parents=True, exist_ok=False)
    fig = plt.figure(figsize=(18, 10.125), facecolor="white")
    fig.suptitle("Direction factor는 왜 정상 경로를 크게 바꿨나?", x=.045, y=.965, ha="left", fontsize=27, weight="bold")
    fig.text(.047, .909, "Exp02C · 실제 수집 Case C, k=0 · 같은 OLD / FRESH / P / B · factor만 변경", fontsize=16)
    positions = [[.055, .285, .275, .56], [.38, .285, .275, .56], [.705, .285, .275, .56]]
    titles = ["① 첫 점과 실제 추종 목표가 다름", "② D 포함: FULL (E+D+Y+F)", "③ D 제거: E+Y+F"]
    for index, (pos, title, name) in enumerate(zip(positions, titles, (None, "FULL", "No D"))):
        ax = fig.add_axes(pos)
        if name is None:
            mechanism_plot(ax, e)
        else:
            variant_plot(ax, e, name, explanatory=False)
        ax.set_title(title, fontsize=17, pad=21, loc="left")
        item = e["variants"][name or "RAW"]
        delta = item["command"]["delta_v_des_abs_mps"]
        fig.text(pos[0], .213, f"첫 전진 명령 변화 |Δv| = {delta:.4f} m/s", fontsize=16, weight="bold")
    fig.text(.05, .128, "D는 뒤쪽 첫 점을 앞쪽으로 당김  ·  F는 경로의 상대 모양을 지키며 그 이동을 뒤까지 전달", fontsize=19, weight="bold")
    fig.text(.05, .078, "추가 확인: E+F에 D만 추가해도 진입점 38.2 cm 이동, |Δv| 0.2410 m/s — FULL과 거의 같음", fontsize=15)
    fig.text(.05, .032, "저장된 최적화 결과와 B에서 계산한 desired 명령 비교 · 실제 주행 영상 아님 · 좌표는 원본 world XY[m], 재정렬 없음", fontsize=11, color="#52606c")
    save_figure(fig, output / "slide03_direction_factor")
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 8), facecolor="white")
    mechanism_plot(ax, e, detailed=True)
    ax.set_title("첫 점은 뒤에 있지만 follower의 목표는 앞에 있다", fontsize=20, pad=22)
    fig.subplots_adjust(left=.10, right=.95, top=.86, bottom=.19)
    fig.text(.1, .075, "P→B: 방금 움직인 방향   ·   빨강: 기존 D가 보는 방향   ·   초록: 실제 follower 목표\n보라 T는 기존 direction residual의 기하학적 목표를 표시한 해설점", fontsize=12, linespacing=1.7)
    save_figure(fig, output / "direction_reference_detail")
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(16, 7), facecolor="white")
    for ax, name in zip(axes, ("FULL", "No propagation")):
        variant_plot(ax, e, name)
        ax.set_title("FULL: 상대운동 보존으로 함께 이동" if name == "FULL" else "전파 차단 진단: 첫 점만 이동", fontsize=19)
    fig.subplots_adjust(left=.065, right=.975, top=.88, bottom=.18, wspace=.18)
    fig.text(.07, .06, "전파 차단은 첫 점 외의 waypoint를 원본으로 고정한 진단 조건입니다. 실제 주행 방법이나 개선안이 아닙니다.", fontsize=13)
    save_figure(fig, output / "fresh_motion_propagation")
    plt.close(fig)
    manifest = {"source_run": str(e["run"]), "case": "case_benign_delayed", "k": 0,
        "created_utc": datetime.now(timezone.utc).isoformat(), "input_sha256": e["input_sha256"],
        "processing_script_sha256": sha256_file(Path(__file__)), "coordinate_frame": "saved Isaac world XY[m], yaw[rad], +Z CCW; identity transform",
        "source_timing": e["source"].source_attempt["timing"], "source_anchor": e["source"].source_metadata["new_world_anchor"],
        "observation_ready_execution_are_distinct": True, "physics_executed": False, "optimization_rerun": False,
        "waypoint_time_base": None, "P_world_se2": e["source"].previous_pose_world.tolist(),
        "B_world_se2": e["source"].boundary_pose_world.tolist(), "raw_target_index": e["target_index"],
        "direction_geometry": e["mechanism"], "plot_bounds": {"mechanism": [.19, .94, -8.035, -7.405], "variants": [.10, 2.03, -8.10, -7.02]},
        "variant_metrics": {name: {k: value for k, value in item.items() if k != "path"} for name, item in e["variants"].items()},
        "outputs": {p.name: sha256_file(p) for p in output.iterdir() if p.is_file()},
        "scope": "conditional real-case factor attribution; no claim about physical navigation or all transitions"}
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(f"EXP02C_DIRECTION_FIGURES={output}", flush=True)


def save_figure(fig, base_path):
    for extension in ("png", "svg"):
        path = base_path.with_suffix("." + extension)
        if path.exists():
            raise FileExistsError(path)
        fig.savefig(path, dpi=180, facecolor="white")


def show_gui(e, output, *, smoke=False):
    import tkinter as tk
    from tkinter import ttk
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    root = tk.Tk()
    root.title("Exp02C · direction factor 원인 진단 · 저장 경로 비교")
    root.geometry("1500x920")
    root.configure(background="white")
    panel = ttk.Frame(root, padding=18)
    panel.pack(side=tk.RIGHT, fill=tk.Y)
    frame = ttk.Frame(root)
    frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    fig = Figure(figsize=(10, 7), facecolor="white")
    ax = fig.add_subplot()
    fig.subplots_adjust(left=.09, right=.97, top=.87, bottom=.12)
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    NavigationToolbar2Tk(canvas, frame).update()
    ttk.Label(panel, text="Case C, k=0\n같은 입력, factor만 변경", font=("NanumGothic", 18, "bold")).pack(anchor="w", pady=(0, 18))
    selected = tk.StringVar(value="원인: 기준점 비교")
    options = {"원인: 기준점 비교": None, "RAW 원본": "RAW", "E+F: D 추가 전": "E+F", "D만 추가: E+D+F": "D added",
               "FULL: D 포함": "FULL", "D 제거: E+Y+F": "No D", "전파 차단 진단": "No propagation"}
    info = ttk.Label(panel, text="", wraplength=340, justify="left", font=("NanumGothic", 14))

    def update():
        ax.clear()
        name = options[selected.get()]
        if name is None:
            mechanism_plot(ax, e, detailed=True)
            info.configure(text="FRESH 첫 점 F0는 B의 뒤쪽입니다.\n\nD는 B→F0가 진행 방향과 반대라고 판단합니다.\n\n하지만 follower는 앞쪽 F3를 목표로 삼습니다. 원본의 명령 변화는 이미 작습니다.\n\n보라 T는 direction 항만 0이 되는 위치이며 FULL의 해는 아닙니다.")
        else:
            variant_plot(ax, e, name)
            item = e["variants"][name]
            command, geometry = item["command"], item["geometry"]
            message = (f"첫 전진 명령 변화 |Δv|\n{command['delta_v_des_abs_mps']:.6f} m/s\n\n"
                f"진입점 이동: {geometry['entry']['translation_displacement_m'] * 100:.2f} cm\n"
                f"끝점 이동: {geometry['endpoint']['translation_displacement_m'] * 100:.2f} cm\n\n"
                f"B에서의 첫 desired v\n{command['candidate_first_desired_v_omega'][0]:.4f} m/s\n"
                f"직전 OLD desired v\n{command['old_final_desired_v_omega'][0]:.4f} m/s")
            if name == "No propagation":
                message += "\n\n뒤쪽 점을 고정한 반사실 진단입니다. 실제 실행 방법이 아닙니다."
            info.configure(text=message)
        ax.set_title(selected.get(), fontsize=22, pad=22)
        canvas.draw()

    for title in options:
        ttk.Radiobutton(panel, text=title, variable=selected, value=title, command=update).pack(anchor="w", pady=7)
    ttk.Separator(panel).pack(fill=tk.X, pady=18)
    info.pack(anchor="w", pady=8)
    ttk.Label(panel, text="저장된 실제 수집 경로의 진단\n새 최적화 / 물리 주행 없음\nWorld XY [m], 좌표 재정렬 없음",
              justify="left", font=("NanumGothic", 11)).pack(side=tk.BOTTOM, anchor="w", pady=12)
    update()
    root.update()
    if smoke:
        directory = output / "gui_validation"
        directory.mkdir(exist_ok=False)
        for index, title in enumerate(options):
            selected.set(title)
            update()
            root.update()
            if not ax.lines or not ax.get_title():
                raise RuntimeError("GUI selection did not update the figure")
            fig.savefig(directory / f"{index:02d}.png", dpi=100)
        (directory / "result.json").write_text(json.dumps({"states_checked": list(options), "passed": True}, ensure_ascii=False, indent=2))
        root.destroy()
        print("EXP02C_DIRECTION_GUI_SMOKE=passed", flush=True)
        return
    print("EXP02C_DIRECTION_GUI_READY=interactive stored-path comparison; physics_executed=false", flush=True)
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--gui-smoke-test", action="store_true")
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("TkAgg" if args.gui or args.gui_smoke_test else "Agg")
    plot_style()
    evidence = read_evidence(args.run.resolve())
    if args.gui or args.gui_smoke_test:
        show_gui(evidence, args.output.resolve(), smoke=args.gui_smoke_test)
    else:
        export_figures(evidence, args.output.resolve())


if __name__ == "__main__":
    main()
