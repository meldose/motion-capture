#!/usr/bin/env python3
"""Interactively create a conservative Dex3 calibration for motion playback.

The script only jogs one motor at a time.  It never discovers a full joint
range: the saved flex delta is the small, visually confirmed movement that is
safe to replay.  Run it beside the robot with a clear hand workspace.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


MODULES_DIR = Path(__file__).resolve().parents[1] / "EF/ef_ws/g1/dev/modules"
sys.path.insert(0, str(MODULES_DIR))

from sdk_hand import (  # noqa: E402
    Dex3HandController,
    HAND_MAX_LIMITS,
    HAND_MIN_LIMITS,
)


JOINT_ORDER = ["thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1"]
HANDS = ("left", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("dex3_motion_calibration.json"))
    parser.add_argument("--jog-rad", type=float, default=0.08,
                        help="Maximum one-motor test displacement; must be 0.02 to 0.15 rad.")
    parser.add_argument("--move-s", type=float, default=0.8, help="Seconds for each jog or return.")
    parser.add_argument("--kp", type=float, default=0.25, help="Low proportional gain used for calibration.")
    parser.add_argument("--kd", type=float, default=0.10)
    parser.add_argument("--arm", action="store_true", help="Required before any robot command is sent.")
    return parser.parse_args()


def current_positions(controller: Dex3HandController, timeout_s: float = 4.0) -> list[float]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        positions = controller._read_actual_positions(max_age=0.5)  # State-only read; controller owns subscriber.
        if positions is not None:
            return positions
        time.sleep(0.05)
    raise RuntimeError(f"No recent {controller.hand} Dex3 HandState received.")


def move(controller: Dex3HandController, target: list[float], args: argparse.Namespace) -> None:
    controller.set_targets(target, hold_s=args.move_s, ramp_s=args.move_s, rate_hz=40.0,
                           kp=args.kp, kd=args.kd, tau=0.0)


def ask(prompt: str, allowed: set[str]) -> str:
    while True:
        value = input(prompt).strip().lower()
        if value in allowed:
            return value
        print("Please enter one of: " + ", ".join(sorted(allowed)))


def main() -> int:
    args = parse_args()
    if not 0.02 <= args.jog_rad <= 0.15:
        raise SystemExit("--jog-rad must be between 0.02 and 0.15 rad.")
    if not args.arm:
        print("Dry safety check only. Re-run with --arm after clearing the workspace.")
        print(f"Would write calibration to: {args.output}")
        return 0

    print("CALIBRATION WILL MOVE ONE DEX3 MOTOR AT A TIME.")
    print("Keep the robot stationary, remove all objects, and keep the E-stop accessible.")
    if ask("Type 'go' when the hands are open/neutral and clear: ", {"go"}) != "go":
        return 1

    controllers = {
        hand: Dex3HandController(hand=hand, iface=args.iface, domain_id=args.domain_id)
        for hand in HANDS
    }
    neutral = {hand: current_positions(controller) for hand, controller in controllers.items()}
    print("Captured neutral positions:")
    for hand in HANDS:
        print(f"  {hand}: {[round(value, 4) for value in neutral[hand]]}")

    calibration: dict[str, object] = {
        "format": "g1-dex3-motion-calibration/v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "iface": args.iface,
        "domain_id": args.domain_id,
        "joint_order": JOINT_ORDER,
        "jog_rad": args.jog_rad,
        "hands": {},
    }
    try:
        for hand in HANDS:
            controller = controllers[hand]
            deltas: list[float] = []
            print(f"\n--- {hand.upper()} HAND ---")
            for idx, name in enumerate(JOINT_ORDER):
                while True:
                    sign = ask(
                        f"{hand} {idx}:{name}: test '+' or '-' q direction; 's' abort calibration: ",
                        {"+", "-", "s"},
                    )
                    if sign == "s":
                        raise RuntimeError("Calibration aborted by operator.")
                    requested_delta = args.jog_rad if sign == "+" else -args.jog_rad
                    target = list(neutral[hand])
                    target[idx] = max(HAND_MIN_LIMITS[hand][idx], min(HAND_MAX_LIMITS[hand][idx], target[idx] + requested_delta))
                    actual_delta = target[idx] - neutral[hand][idx]
                    if abs(actual_delta) < 0.02:
                        print("Neutral is too close to this limit; reposition the hand and restart.")
                        raise RuntimeError("Insufficient safe jog range.")
                    print(f"  Jogging q by {actual_delta:+.3f} rad. Watch only {name}.")
                    move(controller, target, args)
                    move(controller, neutral[hand], args)
                    flexes = ask("  Did that direction FLEX the expected joint? [y/n]: ", {"y", "n"})
                    if flexes == "y":
                        deltas.append(actual_delta)
                        break
                    print("  Try the opposite sign. If a different finger moved, stop and inspect hardware/topic mapping.")

            calibration["hands"][hand] = {
                "neutral_q": neutral[hand],
                "flex_delta_rad": deltas,
                "joint_min": HAND_MIN_LIMITS[hand],
                "joint_max": HAND_MAX_LIMITS[hand],
                "validated": True,
            }
    finally:
        for controller in controllers.values():
            controller.release_fingers(hold_s=0.3, rate_hz=30.0)

    args.output.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    print(f"\nSaved conservative calibration: {args.output}")
    print("Its deltas are deliberately small. Playback will not exceed these verified displacements.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
