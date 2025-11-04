#!/usr/bin/env python

"""Elite industrial arm adapter for LeRobot."""

from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import Any, Sequence

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_elite import EliteConfig
from .elite_client import EliteClient, EliteClientConfig, EliteClientError
from .elite_gripper import EliteGripper

logger = logging.getLogger(__name__)

JOINT_COUNT = 8
DELTA_TRANSLATION_KEYS: tuple[str, ...] = ("delta_x", "delta_y", "delta_z")
DELTA_ROTATION_KEYS: tuple[str, ...] = ("delta_rx", "delta_ry", "delta_rz")
GRIPPER_KEY = "gripper"
ACTION_TYPES: tuple[str, ...] = ("delta_ee", "joint")


class EliteRobot(Robot):
    """Elite arm controlled through transparent transmission."""

    config_class = EliteConfig
    name = "elite"

    def __init__(self, config: EliteConfig):
        super().__init__(config)
        self.config = config

        self._joint_keys = [f"joint_{idx}.pos" for idx in range(1, JOINT_COUNT + 1)]
        self._cartesian_keys = (
            "ee.x",
            "ee.y",
            "ee.z",
            "ee.wx",
            "ee.wy",
            "ee.wz",
        )
        self._client = EliteClient(
            EliteClientConfig(ip=config.ip, port=config.port, timeout_s=config.timeout_s)
        )
        self.cameras = make_cameras_from_configs(config.cameras)

        self._connected = False
        self._last_joint_positions: list[float] | None = None
        self._delta_translation_keys = DELTA_TRANSLATION_KEYS
        self._delta_rotation_keys = DELTA_ROTATION_KEYS
        self._gripper_key = GRIPPER_KEY
        self._gripper_joint_index = self.config.gripper_joint_index
        if not 0 <= self._gripper_joint_index < JOINT_COUNT:
            raise ValueError(
                f"Elite gripper joint index {self.config.gripper_joint_index} outside valid range [0, {JOINT_COUNT})."
            )
        self._gripper_joint_key = self._joint_keys[self._gripper_joint_index]
        self._arm_joint_indices: tuple[int, ...] = tuple(
            idx for idx in range(JOINT_COUNT) if idx != self._gripper_joint_index
        )[:6]
        if len(self._arm_joint_indices) != 6:
            raise ValueError("Elite robot expects exactly 6 arm joints for observation/action recording.")
        self._inactive_joint_indices: tuple[int, ...] = tuple(
            idx
            for idx in range(JOINT_COUNT)
            if idx not in self._arm_joint_indices and idx != self._gripper_joint_index
        )
        self._arm_joint_keys = [self._joint_keys[idx] for idx in self._arm_joint_indices]
        self._default_gripper_state = float(self.config.default_gripper_state)
        self._gripper = EliteGripper(self.config)
        self._action_type = config.action_type.lower()
        if self._action_type not in ACTION_TYPES:
            raise ValueError(f"Unsupported Elite action_type '{config.action_type}'. Expected one of {ACTION_TYPES}.")

    # ------------------------------------------------------------------
    # Feature spaces
    # ------------------------------------------------------------------
    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features = {key: float for key in self._arm_joint_keys}
        features[self._gripper_key] = float
        for cam_key, cam in self.cameras.items():
            features[cam_key] = (cam.height, cam.width, 3)
        if self.config.include_cartesian_observation:
            for key in self._cartesian_keys:
                features[key] = float
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {key: float for key in self._arm_joint_keys}
        features[self._gripper_key] = float
        return features

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected and self._client.is_connected

    def connect(self, calibrate: bool = True) -> None:  # noqa: ARG002 - calibrate unused for now
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            self._client.connect()
            self._bootstrap_controller_state()
        except Exception:
            self._client.disconnect()
            raise

        for cam in self.cameras.values():
            cam.connect()

        self._gripper.initialise()

        self._connected = True
        logger.info("%s connected.", self)

    def _bootstrap_controller_state(self) -> None:
        if not self._client.get_servo_status():
            logger.debug("Enabling Elite servo power.")
            self._client.set_servo_status(1)
            time.sleep(1.0)

        if not self._client.get_motor_status():
            logger.debug("Synchronising Elite motors.")
            self._client.sync_motor_status()
            time.sleep(0.5)

        # Clear any leftover buffered commands before enabling transparent transmission.
        while self._client.get_transparent_transmission_state() == 1:
            logger.debug("Clearing Elite transparent transmission buffer.")
            self._client.tt_clear_servo_joint_buf()
            time.sleep(0.1)

        joint_positions = self._client.get_robot_pos()
        if len(joint_positions) != JOINT_COUNT:
            raise EliteClientError(
                f"Expected {JOINT_COUNT} joint positions, received {len(joint_positions)} from Elite controller."
            )

        was_initialised = self._client.transparent_transmission_init(
            lookahead=self.config.lookahead_ms,
            sample_time_ms=self.config.sample_time_ms,
            smoothness=self.config.smoothness,
        )
        if not was_initialised:
            raise EliteClientError("Elite transparent transmission initialisation failed.")

        self._client.tt_set_current_servo_joint(joint_positions)
        self._client.tt_put_servo_joint_to_buf(joint_positions)
        self._last_joint_positions = joint_positions
        self._gripper.set_last_command(self._default_gripper_state)

    # ------------------------------------------------------------------
    # Calibration + configuration
    # ------------------------------------------------------------------
    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    # ------------------------------------------------------------------
    # Runtime API
    # ------------------------------------------------------------------
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        joint_positions = self._client.get_robot_pos()
        if len(joint_positions) != JOINT_COUNT:
            raise EliteClientError(
                f"Expected {JOINT_COUNT} joint positions, received {len(joint_positions)} from Elite controller."
            )

        for idx in self._inactive_joint_indices:
            joint_positions[idx] = 0.0
        joint_positions[self._gripper_joint_index] = 0.0

        self._last_joint_positions = joint_positions
        observation: dict[str, Any] = {}
        for idx in self._arm_joint_indices:
            key = self._joint_keys[idx]
            observation[key] = joint_positions[idx]

        gripper_state = self._gripper.read_state()
        observation[self._gripper_key] = gripper_state

        for cam_key, cam in self.cameras.items():
            before_read = time.perf_counter()
            observation[cam_key] = cam.async_read()
            logger.debug(
                "Elite camera %s async_read latency: %.3f s",
                cam_key,
                time.perf_counter() - before_read,
            )

        if self.config.include_cartesian_observation:
            pose = self._client.get_robot_pose(
                coordinate_num=self.config.pose_coordinate_num,
                tool_num=self.config.pose_tool_num,
            )
            if len(pose) < len(self._cartesian_keys):
                raise EliteClientError(
                    f"Expected at least {len(self._cartesian_keys)} pose components, received {len(pose)}."
                )
            observation.update(
                {
                    key: float(pose[idx])
                    for idx, key in enumerate(self._cartesian_keys)
                }
            )

        return observation

    def _ensure_last_joint_positions(self) -> list[float]:
        if self._last_joint_positions is None:
            joint_positions = self._client.get_robot_pos()
            if len(joint_positions) != JOINT_COUNT:
                raise EliteClientError(
                    f"Expected {JOINT_COUNT} joint positions, received {len(joint_positions)} from Elite controller."
                )
            self._last_joint_positions = joint_positions
        return list(self._last_joint_positions)

    def _pad_joint_targets(self, positions: Sequence[float]) -> list[float]:
        padded = list(positions[:JOINT_COUNT])
        if len(padded) < JOINT_COUNT:
            padded.extend([0.0] * (JOINT_COUNT - len(padded)))
        elif len(padded) > JOINT_COUNT:
            padded = padded[:JOINT_COUNT]
        return padded

    def _prepare_delta_action(self, action: dict[str, Any]) -> dict[str, float]:
        if not action:
            raise EliteClientError("Delta control action is empty.")

        missing = [key for key in self._delta_translation_keys if key not in action]
        if missing:
            raise EliteClientError(f"Missing delta control keys {missing} in Elite action.")

        processed: dict[str, float] = {}
        for key in self._delta_translation_keys:
            try:
                processed[key] = float(action[key])
            except (TypeError, ValueError) as exc:
                raise EliteClientError(
                    f"Invalid value for Elite delta control key '{key}': {action[key]!r}."
                ) from exc

        for key in self._delta_rotation_keys:
            value = action.get(key, 0.0)
            try:
                processed[key] = float(value)
            except (TypeError, ValueError) as exc:
                raise EliteClientError(
                    f"Invalid value for Elite delta control key '{key}': {value!r}."
                ) from exc

        gripper_value = action.get(self._gripper_key, self._default_gripper_state)
        try:
            processed[self._gripper_key] = float(gripper_value)
        except (TypeError, ValueError) as exc:
            raise EliteClientError(
                f"Invalid value for Elite delta control key '{self._gripper_key}': {gripper_value!r}."
            ) from exc
        return processed

    def _prepare_joint_action(self, action: dict[str, Any]) -> list[float]:
        if not action:
            raise EliteClientError("Joint control action is empty.")

        missing = [key for key in self._arm_joint_keys if key not in action]
        if self._gripper_key not in action:
            missing.append(self._gripper_key)
        if missing:
            raise EliteClientError(f"Missing joint control keys {missing} in Elite action.")

        joint_positions: list[float] = [0.0] * JOINT_COUNT
        for idx, key in zip(self._arm_joint_indices, self._arm_joint_keys):
            try:
                joint_positions[idx] = float(action[key])
            except (TypeError, ValueError) as exc:
                raise EliteClientError(f"Invalid value for Elite joint key '{key}': {action[key]!r}.") from exc

        value = action[self._gripper_key]
        try:
            joint_positions[self._gripper_joint_index] = self._gripper.send_command(float(value))
        except (TypeError, ValueError) as exc:
            raise EliteClientError(
                f"Invalid value for Elite gripper key '{self._gripper_key}': {value!r}."
            ) from exc

        for idx in self._inactive_joint_indices:
            joint_positions[idx] = 0.0
        return joint_positions

    def _delta_action_to_joint_targets(self, action: dict[str, Any], base_positions: list[float]) -> list[float]:
        delta_action = self._prepare_delta_action(action)

        current_pose = self._client.get_robot_pose(
            coordinate_num=self.config.pose_coordinate_num,
            tool_num=self.config.pose_tool_num,
        )
        if len(current_pose) < len(self._cartesian_keys):
            raise EliteClientError(
                f"Expected at least {len(self._cartesian_keys)} pose components, received {len(current_pose)}."
            )

        position_scale = float(self.config.delta_position_step)
        if position_scale <= 0.0:
            raise EliteClientError("Elite delta_position_step must be positive.")
        target_pose = list(current_pose[: len(self._cartesian_keys)])
        target_pose[0] += delta_action["delta_x"] * position_scale
        target_pose[1] += delta_action["delta_y"] * position_scale
        target_pose[2] += delta_action["delta_z"] * position_scale
        orientation_scale = float(self.config.delta_orientation_step)
        if orientation_scale < 0.0:
            raise EliteClientError("Elite delta_orientation_step must be non-negative.")
        target_pose[3] += delta_action["delta_rx"] * orientation_scale
        target_pose[4] += delta_action["delta_ry"] * orientation_scale
        target_pose[5] += delta_action["delta_rz"] * orientation_scale

        ik_positions = self._client.inverse_kinematic(target_pose)
        if len(ik_positions) != JOINT_COUNT:
            raise EliteClientError(
                f"Expected {JOINT_COUNT} joint targets from inverse kinematics, received {len(ik_positions)}."
            )

        target_positions = self._pad_joint_targets(ik_positions)

        target_positions[self._gripper_joint_index] = self._gripper.send_command(delta_action[self._gripper_key])
        for idx in self._inactive_joint_indices:
            target_positions[idx] = 0.0

        return target_positions

    def send_action(self, action: dict[str, Any]) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        base_positions = self._ensure_last_joint_positions()
        for idx in self._inactive_joint_indices:
            base_positions[idx] = 0.0

        if self._action_type == "joint":
            target_positions = self._prepare_joint_action(action)
        elif self._action_type == "delta_ee":
            target_positions = self._delta_action_to_joint_targets(action, base_positions)
        else:
            raise EliteClientError(f"Unsupported Elite action_type '{self._action_type}'.")

        if self.config.max_relative_target is not None:
            goal_present = {
                key.removesuffix(".pos"): (target_positions[idx], base_positions[idx])
                for idx, key in enumerate(self._joint_keys)
            }
            safe_joint_positions = ensure_safe_goal_position(goal_present, self.config.max_relative_target)
            target_positions = [safe_joint_positions[key.removesuffix(".pos")] for key in self._joint_keys]

        success = self._client.tt_put_servo_joint_to_buf(target_positions)
        if not success:
            raise EliteClientError("Elite controller failed to queue transparent transmission command.")

        self._last_joint_positions = target_positions

        absolute_action: dict[str, float] = {
            self._joint_keys[idx]: float(target_positions[idx]) for idx in self._arm_joint_indices
        }

        gripper_state = self._gripper.last_command
        absolute_action[self._gripper_key] = gripper_state

        # Mutate the incoming action dict so downstream consumers (dataset writer, UI) see absolute joints/gripper state.
        action.clear()
        action.update(absolute_action)

        return absolute_action

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        self._gripper.shutdown()

        for cam in self.cameras.values():
            cam.disconnect()

        self._client.disconnect()
        self._connected = False
        logger.info("%s disconnected.", self)
