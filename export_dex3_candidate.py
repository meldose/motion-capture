#!/usr/bin/env python3
"""Create a conservative, offline Dex3 gesture candidate from Motion Studio CSV.

This script never imports ROS and never communicates with a robot.  Its output
is deliberately *not* a command file: the installed Dex3 hands must first be
calibrated to establish directions and a safe neutral pose.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


SOURCE_BONES = {
    "thumb_0": "ThumbUnder",
    "thumb_1": "ThumbMid",
    "thumb_2": "ThumbUp",
    "index_0": "ForeFingerUnder",
    "index_1": "ForeFingerMid",
    "middle_0": "MiddleFingerUnder",
    "middle_1": "MiddleFingerMid",
}


def quat(row: dict[str, str], side: str, bone: str) -> tuple[float, float, float, float]:
    prefix = f"{side}{bone}-handBone-Quat-"
    try:
        values = tuple(float(row[prefix + axis]) for axis in "wxyz")
    except (KeyError, ValueError) as exc:
        raise ValueError(f"missing quaternion for {side}{bone}") from exc
    magnitude = math.sqrt(sum(value * value for value in values))
    if magnitude < 1e-8:
        raise ValueError(f"invalid zero quaternion for {side}{bone}")
    return tuple(value / magnitude for value in values)  # type: ignore[return-value]


def relative_angle(reference: tuple[float, float, float, float], current: tuple[float, float, float, float]) -> float:
    """Return the unsigned angle (radians) from a calibrated reference pose."""
    rw, rx, ry, rz = reference
    cw, cx, cy, cz = current
    # conjugate(reference) * current
    w = rw * cw + rx * cx + ry * cy + rz * cz
    x = rw * cx - rx * cw - ry * cz + rz * cy
    y = rw * cy + rx * cz - ry * cw - rz * cx
    z = rw * cz - rx * cy + ry * cx - rz * cw
    vector = math.sqrt(x * x + y * y + z * z)
    return 2.0 * math.atan2(vector, abs(w))


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError("CSV contains no motion frames")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_json", type=Path)
    # test_1.msd records 3,259 frames over ~27 seconds (120.7 Hz).  Its
    # metadata field named `fps` is 60, but the recorded elapsed timestamps
    # and `play1xFps` establish the actual frame cadence used here.
    parser.add_argument("--input-fps", type=float, default=120.7037037037)
    parser.add_argument("--output-fps", type=float, default=20.0)
    parser.add_argument("--smoothing", type=float, default=0.25,
                        help="EMA coefficient in (0, 1]; higher follows input more closely")
    args = parser.parse_args()
    if args.input_fps <= 0 or args.output_fps <= 0 or not 0 < args.smoothing <= 1:
        parser.error("FPS values must be positive and smoothing must be in (0, 1]")

    rows = read_rows(args.input_csv)
    interval = max(1, round(args.input_fps / args.output_fps))
    sampled = rows[::interval]
    references = {
        side: {name: quat(sampled[0], side, bone) for name, bone in SOURCE_BONES.items()}
        for side in ("left", "right")
    }
    smoothed = {side: {name: 0.0 for name in SOURCE_BONES} for side in ("left", "right")}
    frames = []
    for frame_number, row in enumerate(sampled):
        hands = {}
        for side in ("left", "right"):
            gesture = {}
            for name, bone in SOURCE_BONES.items():
                angle = relative_angle(references[side][name], quat(row, side, bone))
                # 1.2 rad is a deliberately broad 'full flexion' reference.  We
                # preserve a normalized gesture, rather than inventing unsafe motor q.
                target = min(1.0, angle / 1.2)
                previous = smoothed[side][name]
                smoothed[side][name] = previous + args.smoothing * (target - previous)
                gesture[name] = round(smoothed[side][name], 6)
            hands[side] = gesture
        source_index = frame_number * interval
        frames.append({"time_s": round(source_index / args.input_fps, 4), "hands": hands})

    output = {
        "format": "g1-dex3-gesture-candidate/v1",
        "safety": {
            "robot_commandable": False,
            "reason": "Requires hand-specific direction and neutral-pose calibration before q targets can be generated.",
            "required_before_live_test": [
                "Keep G1 stationary and clear the hand workspace.",
                "Read both Dex3 HandState topics and record the actual neutral q positions.",
                "Jog one motor at a time at low gain to confirm sign and motor-to-finger mapping.",
                "Generate clamped q targets from this normalized gesture and validate them without contact.",
            ],
        },
        "source": {
            "file": str(args.input_csv),
            "input_frames": len(rows),
            "input_fps": args.input_fps,
            "output_fps": args.output_fps,
            "reference": "first sampled frame",
        },
        "dex3_motor_order": ["thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1"],
        "frames": frames,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {len(frames)} offline gesture frames to {args.output_json}")


if __name__ == "__main__":
    main()
