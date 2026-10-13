#!/usr/bin/env python3
"""Replay a calibrated Motion Studio Dex3 gesture on the real robot.

Dry-run is the default.  Physical playback requires --arm and a calibration
created by calibrate_dex3_hands.py.  The script never commands arm or body
joints; it publishes only Dex3 HandCmd targets.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any


MODULES_DIR = Path(__file__).resolve().parents[1] / "EF/ef_ws/g1/dev/modules"
sys.path.insert(0, str(MODULES_DIR))

from sdk_hand import Dex3HandController  # noqa: E402


JOINT_ORDER = ["thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1"]
HANDS = ("left", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path(__file__).with_name("dex3_gesture_candidate.json"))
    parser.add_argument("--calibration", type=Path, default=Path(__file__).with_name("dex3_motion_calibration.json"))
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier; 0.1 to 1.0 only.")
    parser.add_argument("--kp", type=float, default=0.30, help="Low hand position gain.")
    parser.add_argument("--kd", type=float, default=0.10)
    parser.add_argument("--neutral-tolerance", type=float, default=0.12,
                        help="Maximum current-vs-calibrated-neutral error before refusing playback.")
    parser.add_argument("--arm", action="store_true", help="Required before publishing physical motion.")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot read {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise SystemExit(f"{path} must contain a JSON object.")
    return payload


def validate(candidate: dict[str, Any], calibration: dict[str, Any]) -> list[dict[str, Any]]:
    if candidate.get("dex3_motor_order") != JOINT_ORDER:
        raise SystemExit("Candidate joint order is not the expected Dex3 order; refusing to run.")
    frames = candidate.get("frames")
    if not isinstance(frames, list) or not frames:
        raise SystemExit("Candidate has no frames.")
    if calibration.get("format") != "g1-dex3-motion-calibration/v1" or calibration.get("joint_order") != JOINT_ORDER:
        raise SystemExit("Calibration format or joint order is invalid.")
    hand_profiles = calibration.get("hands")
    if not isinstance(hand_profiles, dict):
        raise SystemExit("Calibration has no hand profiles.")
    for hand in HANDS:
        profile = hand_profiles.get(hand)
        if not isinstance(profile, dict) or profile.get("validated") is not True:
            raise SystemExit(f"Calibration for {hand} is missing or not validated.")
        for key in ("neutral_q", "flex_delta_rad", "joint_min", "joint_max"):
            values = profile.get(key)
            if not isinstance(values, list) or len(values) != 7:
                raise SystemExit(f"Calibration {hand}.{key} must contain seven values.")
    previous = -1.0
    for frame in frames:
        if not isinstance(frame, dict) or not isinstance(frame.get("time_s"), (int, float)):
            raise SystemExit("Candidate frame has no valid time_s.")
        if float(frame["time_s"]) < previous:
            raise SystemExit("Candidate frame times are not monotonic.")
        previous = float(frame["time_s"])
    return frames


def targets(frame: dict[str, Any], calibration: dict[str, Any], hand: str) -> list[float]:
    profile = calibration["hands"][hand]
    normalized = frame.get("hands", {}).get(hand, {})
    if not isinstance(normalized, dict):
        raise RuntimeError(f"Frame has no {hand} hand data.")
    result = []
    for idx, name in enumerate(JOINT_ORDER):
        value = normalized.get(name)
        if not isinstance(value, (int, float)):
            raise RuntimeError(f"Frame has invalid {hand}.{name} value.")
        flexion = min(1.0, max(0.0, float(value)))
        q = float(profile["neutral_q"][idx]) + flexion * float(profile["flex_delta_rad"][idx])
        result.append(max(float(profile["joint_min"][idx]), min(float(profile["joint_max"][idx]), q)))
    return result


def current_positions(controller: Dex3HandController, timeout_s: float = 4.0) -> list[float]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        positions = controller._read_actual_positions(max_age=0.5)
        if positions is not None:
            return positions
        time.sleep(0.05)
    raise RuntimeError(f"No recent {controller.hand} Dex3 HandState received.")


def main() -> int:
    args = parse_args()
    if not 0.1 <= args.speed <= 1.0:
        raise SystemExit("--speed must be between 0.1 and 1.0.")
    candidate = load_json(args.input)
    calibration = load_json(args.calibration)
    frames = validate(candidate, calibration)
    duration = float(frames[-1]["time_s"])
    first_targets = {hand: targets(frames[0], calibration, hand) for hand in HANDS}
    print(f"Capture: {len(frames)} frames, {duration:.2f}s at {args.speed:.2f}x speed")
    for hand in HANDS:
        print(f"  first {hand} target: {[round(value, 4) for value in first_targets[hand]]}")
    if not args.arm:
        print("Dry-run complete. Review targets, clear the workspace, then re-run with --arm.")
        return 0

    print("PLAYBACK WILL MOVE BOTH DEX3 HANDS ONLY. Keep robot stationary and E-stop accessible.")
    if input("Type 'play' to start: ").strip().lower() != "play":
        print("Playback cancelled.")
        return 1
    controllers = {hand: Dex3HandController(hand=hand, iface=args.iface, domain_id=args.domain_id) for hand in HANDS}
    try:
        for hand, controller in controllers.items():
            current = current_positions(controller)
            neutral = [float(value) for value in calibration["hands"][hand]["neutral_q"]]
            max_error = max(abs(actual - expected) for actual, expected in zip(current, neutral))
            if max_error > args.neutral_tolerance:
                raise RuntimeError(
                    f"{hand} hand is {max_error:.3f} rad from calibrated neutral "
                    f"(limit {args.neutral_tolerance:.3f}); reposition it and retry."
                )

        start = time.monotonic()
        for frame in frames:
            due = start + float(frame["time_s"]) / args.speed
            while time.monotonic() < due:
                time.sleep(min(0.005, max(0.0, due - time.monotonic())))
            for hand, controller in controllers.items():
                # State freshness is checked during playback; lost feedback stops commands.
                current_positions(controller, timeout_s=0.6)
                controller.write_targets_once(targets(frame, calibration, hand), kp=args.kp, kd=args.kd, tau=0.0)
        print("Playback complete.")
        return 0
    finally:
        for controller in controllers.values():
            controller.release_fingers(hold_s=0.4, rate_hz=30.0)


if __name__ == "__main__":
    raise SystemExit(main())
