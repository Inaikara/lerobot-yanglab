#!/usr/bin/env python

"""Configuration dataclass for the Elite robot adapter."""

from __future__ import annotations

from dataclasses import dataclass, field
from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("elite")
@dataclass
class EliteConfig(RobotConfig):
    """Configuration for the Elite robot.

    Attributes:
        ip: Controller IP address.
        port: Controller TCP port.
        timeout_s: Socket timeout used by the client.
        lookahead_ms: Transparent transmission lookahead parameter.
        sample_time_ms: Transparent transmission sample time parameter.
        smoothness: Transparent transmission smoothness parameter.
        include_cartesian_observation: Whether to read Cartesian pose alongside joints.
        pose_coordinate_num: Elite coordinate frame identifier used for pose queries.
        pose_tool_num: Tool number to use for pose queries.
        delta_position_step: Distance in meters applied per delta action tick.
        delta_orientation_step: Orientation increment (radians) applied per delta action tick.
        gripper_open_position: Joint target corresponding to an open gripper.
        gripper_closed_position: Joint target corresponding to a closed gripper.
        default_gripper_state: Default logical gripper state (0 = closed, 1 = open).
        gripper_serial_port: Serial port that controls the external gripper.
        gripper_baudrate: Serial baudrate for the external gripper.
        gripper_timeout_s: Serial timeout for the external gripper.
        gripper_joint_index: Index of the gripper joint in the Elite controller joint vector.
        cameras: Two RealSense cameras already supported in LeRobot.
        action_type: Control input format. Use "delta_ee" for teleoperation style cartesian deltas,
            or "joint" when commands provide absolute joint targets.
    """

    ip: str = "192.168.1.200"
    port: int = 8055
    timeout_s: float = 2.0

    lookahead_ms: float = 400.0
    sample_time_ms: float = 10.0
    smoothness: float = 0.1

    max_relative_target: float | dict[str, float] | None = None
    max_joint_velocity: float = 1.2  # TODO: Update to Elite hardware joint velocity limit.
    max_joint_acceleration: float = 2.5  # TODO: Update to Elite hardware joint acceleration limit.

    include_cartesian_observation: bool = False
    pose_coordinate_num: int = -1
    pose_tool_num: int = -1
    delta_position_step: float = 0.2
    delta_orientation_step: float = 0.05
    gripper_open_position: float = 1000.0
    gripper_closed_position: float = 0.0
    default_gripper_state: float = 1000.0
    gripper_serial_port: str = 'COM0'
    gripper_baudrate: int = 115200
    gripper_timeout_s: float = 0.01
    gripper_joint_index: int = 7
    action_type: str = "delta_ee"

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "front": RealSenseCameraConfig(
                serial_number_or_name="046122250226",
                fps=30,
                width=640,
                height=480,
            ),
            "wrist": RealSenseCameraConfig(
                serial_number_or_name="036522071980",
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    def __post_init__(self):
        super().__post_init__()
