#!/usr/bin/env python3
"""Encode sampled live GUI frames and synchronized historical/Exp02D comparisons."""

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
from reconciliation.online_switch import sha256_file
from reconciliation.closed_loop_execution_validation import validate_closed_loop_trial
from reconciliation.video_timing import validate_video_frames


def write_json(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def synchronized_frame_indices(sample_count, maximum_intervals, hold_frames):
    """None means the measured end-state card, starting at the last sample time."""
    if sample_count < 1 or maximum_intervals < sample_count - 1 or hold_frames < 1:
        raise ValueError("invalid sampled-video duration or final hold")
    return [i if i < sample_count - 1 else None
            for i in range(maximum_intervals + hold_frames)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", nargs="+", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffmpeg", type=Path, required=True)
    args = parser.parse_args()
    ffmpeg = str(args.ffmpeg.resolve())
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sources = {}
    for recording in args.recordings:
        recording = recording.resolve()
        completed = json.loads((recording / "completed.json").read_text())
        provenance = json.loads((recording / "provenance.json").read_text())
        primary_run = recording.parents[1]
        for filename, key in [("protocol.json", "primary_protocol_sha256"), ("summary.json", "primary_summary_sha256")]:
            if sha256_file(primary_run / filename) != provenance[key]:
                raise ValueError("recording primary provenance changed")
        for row in completed["results"]:
            directory = Path(row["directory"])
            if not directory.is_relative_to(recording):
                raise ValueError("recording directory escapes its parent")
            data = json.loads((directory / "recording.json").read_text())
            if not data["primary_poses_exactly_equal"]:
                raise ValueError("recorded execution does not match the quantitative primary")
            validate_closed_loop_trial(directory / "trial")
            with (directory / "trial/raw/telemetry.csv").open() as stream:
                times = np.array([float(row["sim_time_s"]) for row in csv.DictReader(stream)])
            poses = np.load(directory / "trial/raw/actual_trajectory.npy", allow_pickle=False)
            primary_poses = np.load(primary_run / "trials" / data["case"] / data["method"] /
                                    "repetition_00/raw/actual_trajectory.npy", allow_pickle=False)
            if not np.array_equal(poses, primary_poses):
                raise ValueError("recorded motion differs from primary telemetry")
            validate_video_frames(data["frames"], provenance["physics_dt"], provenance["capture_stride"], times, poses)
            for frame in data["frames"]:
                if sha256_file(directory / frame["file"]) != frame["sha256"]:
                    raise ValueError("raw video frame changed")
            if sha256_file(directory / "end_card.png") != data["end_card_sha256"]:
                raise ValueError("end card changed")
            key = (data["case"], data["method"])
            if key in sources:
                raise ValueError("duplicate case/method recording")
            sources[key] = (directory, data)
    fps = {data["timing"]["nominal_simulation_fps"] for _, data in sources.values()}
    if len(fps) != 1:
        raise ValueError("comparison recordings have different sampling rates")
    input_fps = fps.pop() / 2.0
    hold_frames = int(round(input_fps * 2.0))
    manifest = {"processing_script_sha256": sha256_file(Path(__file__)),
        "ffmpeg_version": subprocess.check_output([ffmpeg, "-version"], text=True).splitlines()[0],
        "ffmpeg_sha256": sha256_file(Path(ffmpeg)), "slow_motion_factor": 2.0,
        "input_fps": input_fps, "output_fps": 30, "final_hold_wall_seconds": 2,
        "timing": "align sample zero at settled B; 2x slow motion; shorter execution holds its end card while longer runs; no pose interpolation",
        "image_processing": "whole Isaac window, aspect-preserving resize and H264 encoding; side by side uses two measured runs, not simultaneous robots",
        "videos": {}, "comparisons": {}}
    methods = sorted({method for _, method in sources})
    for case in sorted({case for case, _ in sources}):
        selected = {method: sources[case, method] for method in methods if (case, method) in sources}
        maximum_intervals = max(len(data["frames"]) - 1 for _, data in selected.values())
        total_frames = maximum_intervals + hold_frames
        for method, (directory, data) in selected.items():
            sequence = output / "sequences" / case / method
            sequence.mkdir(parents=True)
            for i, source_index in enumerate(synchronized_frame_indices(len(data["frames"]), maximum_intervals, hold_frames)):
                source = directory / (data["frames"][source_index]["file"] if source_index is not None else "end_card.png")
                (sequence / f"{i:06d}.png").symlink_to(source)
            path = output / f"{case}_{method}.mp4"
            command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-n", "-framerate", str(input_fps),
                "-i", str(sequence / "%06d.png"), "-vf", "scale=1920:-2:flags=lanczos,fps=30",
                "-c:v", "libx264", "-preset", "fast", "-crf", "18", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", str(path)]
            subprocess.run(command, check=True)
            subprocess.run([ffmpeg, "-v", "error", "-i", str(path), "-f", "null", "-"], check=True)
            manifest["videos"][path.name] = {"sha256": sha256_file(path), "recording": str(directory),
                "recording_sha256": sha256_file(directory / "recording.json"), "command": command,
                "sample_count": len(data["frames"]), "video_duration_s": total_frames / input_fps,
                "end_card_starts_at_video_s": (len(data["frames"]) - 1) / input_fps,
                "primary_poses_exactly_equal": True}
        for left, right, suffix in [("M1_HISTORICAL_M4", "M3_LOOKAHEAD", "BEFORE_AFTER"),
                                    ("M0_RAW", "M3_LOOKAHEAD", "RAW_Exp02D")]:
            if left not in selected or right not in selected:
                continue
            path = output / f"{case}_{suffix}.mp4"
            command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-n",
                "-i", str(output / f"{case}_{left}.mp4"), "-i", str(output / f"{case}_{right}.mp4"),
                "-filter_complex", "[0:v]scale=1280:-2[l];[1:v]scale=1280:-2[r];[l][r]hstack=inputs=2[v]",
                "-map", "[v]", "-c:v", "libx264", "-preset", "fast", "-crf", "18",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
            subprocess.run(command, check=True)
            subprocess.run([ffmpeg, "-v", "error", "-i", str(path), "-f", "null", "-"], check=True)
            manifest["comparisons"][path.name] = {"left": left, "right": right,
                "sha256": sha256_file(path), "command": command, "synchronized_at": "settled B, simulation t=0"}
        print(f"ENCODED_CASE={case}", flush=True)
    write_json(output / "manifest.json", manifest)
    print(f"OBJECTIVE_MOVIES_READY={output}", flush=True)


if __name__ == "__main__":
    main()
