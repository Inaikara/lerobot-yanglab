#!/usr/bin/env python

"""Touch X teleoperator implementation for LeRobot."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from ..utils import TeleopEvents
from .config_touchx import TouchXTeleopConfig

logger = logging.getLogger(__name__)


hd = None  # type: ignore[assignment]
hd_callback = None  # type: ignore
hdAsyncSheduler = None  # type: ignore
hdSyncSheduler = None  # type: ignore

try:
    import pyOpenHaptics.hd as _hd
    from pyOpenHaptics.hd_callback import (
        hdAsyncSheduler as _hdAsyncSheduler,
        hdSyncSheduler as _hdSyncSheduler,
        hd_callback as _hd_callback,
    )

    hd = _hd
    hd_callback = _hd_callback
    hdAsyncSheduler = _hdAsyncSheduler
    hdSyncSheduler = _hdSyncSheduler
except Exception as inner_exc:  # pragma: no cover - optional dependency
    logger.debug("System pyOpenHaptics import failed: %s", inner_exc)


@dataclass(slots=True)
class _TouchXSample:
    position: tuple[float, float, float]
    rotation: tuple[float, float, float]
    button: bool
    timestamp_s: float


class TouchXTeleop(Teleoperator):
    """Translate Touch X stylus motions into delta end-effector commands."""

    config_class = TouchXTeleopConfig
    name = "touchx"

    def __init__(self, config: TouchXTeleopConfig):
        super().__init__(config)
        self.config = config
        self._device_handle: int | None = None
        self._callback = None
        self._lock = threading.Lock()
        self._latest_sample: _TouchXSample | None = None
        self._previous_sample: _TouchXSample | None = None
        self._smoothed_translation: list[float] | None = None
        self._smoothed_rotation: list[float] | None = None
        self._state_lock = threading.Lock()
        self._gripper_state = float(self.config.default_gripper_state)
        self._keyboard_listener: Any | None = None
        self._connected = False
        self._is_calibrated = False
        self._scheduler_running = False

    # ------------------------------------------------------------------
    # Feature spaces
    # ------------------------------------------------------------------
    @property
    def action_features(self) -> dict:
        action_names = {
            "delta_x": 0,
            "delta_y": 1,
            "delta_z": 2,
            "delta_rx": 3,
            "delta_ry": 4,
            "delta_rz": 5,
        }
        action_names["gripper"] = 6
        return {
            "dtype": "float32",
            "shape": (len(action_names),),
            "names": action_names,
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        if hd is None or hd_callback is None or hdAsyncSheduler is None or hdSyncSheduler is None:
            raise DeviceNotConnectedError(
                "pyOpenHaptics is not available. Ensure the Touch X drivers and pyOpenHaptics bindings are installed."
            )

        scheduler_type = self.config.scheduler_type.lower()
        if scheduler_type not in {"async", "sync"}:
            raise ValueError(f"Unsupported Touch X scheduler_type '{self.config.scheduler_type}'.")

        device_handle = hd.init_device(self.config.device_name)
        if device_handle is None or device_handle == hd.HD_BAD_HANDLE:
            raise DeviceNotConnectedError(f"Unable to initialise Touch X device '{self.config.device_name}'.")

        if hasattr(hd, "make_current_device"):
            hd.make_current_device(device_handle)  # type: ignore[attr-defined]
        hd.start_scheduler()
        if hd.get_error():
            hd.stop_scheduler()
            hd.close_device(device_handle)
            raise DeviceNotConnectedError("Touch X scheduler reported an error during startup.")

        self._device_handle = device_handle
        self._callback = self._build_callback()
        if scheduler_type == "async":
            hdAsyncSheduler(self._callback)
        else:
            hdSyncSheduler(self._callback)

        self._connected = True
        self._scheduler_running = True

        self._start_keyboard_listener()

        if calibrate:
            try:
                self.calibrate()
            except Exception:
                self.disconnect()
                raise

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def calibrate(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self._lock:
                sample = self._latest_sample
            if sample is not None:
                self._previous_sample = sample
                self._reset_filters()
                self._is_calibrated = True
                logger.info("Touch X teleop calibrated with neutral sample at %.3f s.", sample.timestamp_s)
                return
            time.sleep(0.01)

        raise TimeoutError("Unable to calibrate Touch X teleop: no samples received from device.")

    def configure(self) -> None:
        # No additional runtime configuration required.
        return

    # ------------------------------------------------------------------
    # Runtime API
    # ------------------------------------------------------------------
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with self._lock:
            latest = self._latest_sample
            previous = self._previous_sample

            if latest is None or previous is None:
                return self._empty_action()

            if not latest.button:
                self._previous_sample = latest
                self._reset_filters()
                return self._empty_action()

            delta_translation = [
                latest.position[idx] - previous.position[idx] for idx in range(3)
            ]
            delta_rotation = [
                latest.rotation[idx] - previous.rotation[idx] for idx in range(3)
            ]

            scaled_translation = [
                delta * self.config.translation_scale for delta in delta_translation
            ]
            scaled_rotation = [
                delta * self.config.rotation_scale for delta in delta_rotation
            ]

            smoothed_translation = self._smooth_sequence(
                self._smoothed_translation,
                scaled_translation,
                self.config.translation_smoothing_alpha,
            )
            smoothed_rotation = self._smooth_sequence(
                self._smoothed_rotation,
                scaled_rotation,
                self.config.rotation_smoothing_alpha,
            )
            self._previous_sample = latest
            self._smoothed_translation = smoothed_translation
            self._smoothed_rotation = smoothed_rotation

        action = {
            "delta_x": -smoothed_translation[2],
            "delta_y": -smoothed_translation[0],
            "delta_z": smoothed_translation[1],
            "delta_rx": smoothed_rotation[0],
            "delta_ry": smoothed_rotation[1],
            "delta_rz": smoothed_rotation[2],
        }

        action["gripper"] = self._get_gripper_state()

        return action

    def get_teleop_events(self) -> dict[str, Any]:
        with self._lock:
            latest = self._latest_sample

        button_pressed = latest.button if latest is not None else False
        return {
            TeleopEvents.IS_INTERVENTION: button_pressed,
            TeleopEvents.TERMINATE_EPISODE: False,
            TeleopEvents.SUCCESS: False,
            TeleopEvents.RERECORD_EPISODE: False,
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        # Force feedback is intentionally disabled for safety.
        return

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self._device_handle is not None:
            if self._scheduler_running and hasattr(hd, "stop_scheduler"):
                hd.stop_scheduler()
            hd.close_device(self._device_handle)

        self._device_handle = None
        self._callback = None
        self._connected = False
        self._is_calibrated = False
        self._scheduler_running = False
        self._stop_keyboard_listener()
        with self._lock:
            self._latest_sample = None
            self._previous_sample = None
            self._smoothed_translation = None
            self._smoothed_rotation = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _empty_action(self) -> dict[str, Any]:
        action = {
            "delta_x": 0.0,
            "delta_y": 0.0,
            "delta_z": 0.0,
            "delta_rx": 0.0,
            "delta_ry": 0.0,
            "delta_rz": 0.0,
        }
        action["gripper"] = self._get_gripper_state()
        return action

    def _build_callback(self):
        @hd_callback  # type: ignore[misc]
        def _touchx_callback():
            if self._device_handle is None:
                return

            if hasattr(hd, "make_current_device"):
                hd.make_current_device(self._device_handle)  # type: ignore[attr-defined]

            transform = hd.get_transform()
            joints = hd.get_gimbals()
            button_state = hd.get_buttons()

            position = (
                float(transform[3][0]),
                float(transform[3][1]),
                float(transform[3][2]),
            )
            rotation = (
                float(joints[0]),
                float(joints[1]),
                float(joints[2]),
            )
            button = bool(button_state)
            timestamp_s = time.perf_counter()

            sample = _TouchXSample(position=position, rotation=rotation, button=button, timestamp_s=timestamp_s)
            with self._lock:
                self._latest_sample = sample
                if self._previous_sample is None:
                    self._previous_sample = sample

        return _touchx_callback

    @staticmethod
    def _smooth_sequence(
        previous: list[float] | None, current: list[float], alpha: float
    ) -> list[float]:
        if not current:
            return []
        if previous is None or alpha >= 1.0:
            return list(current)
        if alpha <= 0.0:
            return list(previous)
        return [prev + alpha * (curr - prev) for prev, curr in zip(previous, current)]

    def _reset_filters(self) -> None:
        self._smoothed_translation = [0.0, 0.0, 0.0]
        self._smoothed_rotation = [0.0, 0.0, 0.0]

    # ------------------------------------------------------------------
    # Gripper helpers
    # ------------------------------------------------------------------
    def _set_gripper_state(self, value: float) -> None:
        command = value
        with self._state_lock:
            if self._gripper_state == command:
                return
            self._gripper_state = command
        logger.debug("Touch X gripper state updated to %.1f", command)

    def _get_gripper_state(self) -> float:
        with self._state_lock:
            return float(self._gripper_state)

    def _start_keyboard_listener(self) -> None:
        if self._keyboard_listener is not None:
            return

        try:
            from pynput import keyboard
        except Exception as exc:  # pragma: no cover - optional dependency
            logger.warning("Keyboard listener unavailable for Touch X gripper control: %s", exc)
            return

        def on_press(key) -> None:
            try:
                char = key.char.lower()
            except AttributeError:
                return

            if char == "r":
                self._set_gripper_state(1000.0)
            elif char == "g":
                self._set_gripper_state(0.0)

        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        self._keyboard_listener = listener
        logger.info("Touch X gripper keyboard listener started (G=close, R=open).")

    def _stop_keyboard_listener(self) -> None:
        listener = self._keyboard_listener
        if listener is None:
            return
        try:
            listener.stop()
        except Exception as exc:  # pragma: no cover - defensive cleanup
            logger.debug("Failed to stop Touch X keyboard listener cleanly: %s", exc)
        finally:
            self._keyboard_listener = None
