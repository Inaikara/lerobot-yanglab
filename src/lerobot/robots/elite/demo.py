"""Simple validation script for the Elite robot adapter.

Usage example (run from repo root):
    python -m lerobot.robots.elite.demo "[[0, 0, -1], [0, 0, 0, 2]]"

Each entry inside the JSON list represents a delta action with components
``delta_x``, ``delta_y``, ``delta_z`` and an optional ``gripper`` command.
Commands are streamed sequentially to the robot with a configurable pause.
"""

from __future__ import annotations

import argparse
import json
import time

from .config_elite import EliteConfig
from .elite_robot import EliteRobot

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move Elite arm to a target configuration.")
    parser.add_argument(
        "actions",
        type=str,
        help=(
            "JSON list of delta actions. Example: "
            "\"[[0, 0, -1, 1], [0, 0, 0, 2]]\" applies a downward step followed by an open-gripper command."
        ),
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.01,
        help="Delay between interpolated steps in seconds.",
    )
    parser.add_argument(
        "--pose-coordinate-num",
        type=int,
        default=-1,
        help="Coordinate frame index passed to the controller when querying Cartesian pose.",
    )
    parser.add_argument(
        "--pose-tool-num",
        type=int,
        default=-1,
        help="Tool number passed to the controller when querying Cartesian pose.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = EliteConfig(
        pose_coordinate_num=args.pose_coordinate_num,
        pose_tool_num=args.pose_tool_num,
    )

    robot = EliteRobot(config)
    robot.connect()
    print("Connected to Elite robot.")

    try:
        actions = json.loads(args.actions)
        if not isinstance(actions, list) or not actions:
            raise ValueError("Actions JSON must be a non-empty list of delta commands.")

        for idx, raw_action in enumerate(actions):
            if not isinstance(raw_action, (list, tuple)):
                raise ValueError(f"Delta action at index {idx} must be a list or tuple, got {type(raw_action)!r}.")
            if len(raw_action) not in (3, 4):
                raise ValueError(
                    f"Delta action at index {idx} must contain 3 or 4 values (delta_x, delta_y, delta_z, [gripper])."
                )
            delta_values = list(float(val) for val in raw_action)
            if len(delta_values) == 3:
                delta_values.append(1.0)
            delta_action = dict(zip(("delta_x", "delta_y", "delta_z", "gripper"), delta_values))
            applied = robot.send_action(delta_action)
            print(f"Applied action {idx}: {applied}")
            if args.sleep > 0:
                time.sleep(args.sleep)
    finally:
        robot.disconnect()
        print("Disconnected from Elite robot.")


if __name__ == "__main__":
    main()
