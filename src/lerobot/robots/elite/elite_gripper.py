"""Gripper helper utilities for the Elite robot."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import serial
from serial.tools import list_ports

if TYPE_CHECKING:
    from .config_elite import EliteConfig

SerialException = serial.SerialException

logger = logging.getLogger(__name__)


class EliteGripperController:
    """Low-level serial interface for the Elite gripper."""

    def __init__(self, port: str = "COM0", baudrate: int = 115200, timeout: float = 0.01) -> None:
        if port == "COM0":
            port = self.get_first_available_port()
            if port is None:
                raise ValueError("No available serial port detected for Elite gripper.")

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.gripper_id = 1
        self._scan_id()

    def _scan_id(self) -> None:
        for candidate in range(1, 255):
            if self._getid(candidate) == 7:
                self.gripper_id = candidate
                break

    def _data2bytes(self, data: int) -> list[int]:
        if data == -1:
            return [0xFF, 0xFF]
        return [data & 0xFF, (data >> 8) & 0xFF]

    def _num2str(self, num: int) -> bytes:
        return bytes.fromhex(f"{num:02x}")

    def _checknum(self, data: list[int], length: int) -> int:
        return sum(data[2:length]) & 0xFF

    def _getid(self, candidate: int) -> int:
        datanum = 0x05
        payload = [0xEB, 0x90, candidate, datanum, 0x12] + self._data2bytes(1000) + self._data2bytes(0)
        payload.append(self._checknum(payload, datanum + 4))

        message = b"".join(self._num2str(x) for x in payload)
        self.ser.write(message)
        return len(self.ser.read(7))

    def set_id(self, new_id: int) -> None:
        if not 1 <= new_id <= 254:
            raise ValueError("Gripper ID must be within 1-254.")

        datanum = 0x02
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0x04, new_id]
        payload.append(self._checknum(payload, datanum + 4))

        self._send_command(payload, datanum + 5)
        self.gripper_id = new_id

    def _send_command(self, command: list[int], expected_length: int) -> bytes:
        message = b"".join(self._num2str(x) for x in command)
        self.ser.write(message)
        return self.ser.read(expected_length)

    def move_to_target(self, target: int) -> None:
        if not 0 <= target <= 1000:
            raise ValueError("Target position must be within 0-1000.")

        datanum = 0x03
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0x54] + self._data2bytes(target)
        payload.append(self._checknum(payload, datanum + 4))

        self._send_command(payload, 7)

    def get_current_position(self) -> int:
        datanum = 0x01
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0xD9]
        payload.append(self._checknum(payload, datanum + 4))

        response = self._send_command(payload, 8)
        if len(response) < 8:
            return -1
        return response[5] + (response[6] << 8)

    def get_state(self) -> dict[str, int]:
        datanum = 0x01
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0x41]
        payload.append(self._checknum(payload, datanum + 4))

        response = self._send_command(payload, 13)
        if len(response) < 13:
            return {}
        return {
            "position": response[5] + (response[6] << 8),
            "status": response[7],
            "temperature": response[8],
            "current": response[9] + (response[10] << 8),
            "power": response[11] + (response[12] << 8),
        }

    def emergency_stop(self) -> None:
        datanum = 0x01
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0x16]
        payload.append(self._checknum(payload, datanum + 4))
        self._send_command(payload, 7)

    def set_open_limit(self, max_open: int, min_open: int) -> None:
        if not 0 <= max_open <= 1000 or not 0 <= min_open <= 1000:
            raise ValueError("Open limits must be within 0-1000.")
        if max_open <= min_open:
            raise ValueError("Maximum open limit must exceed minimum open limit.")

        datanum = 0x05
        payload = (
            [0xEB, 0x90, self.gripper_id, datanum, 0x12]
            + self._data2bytes(max_open)
            + self._data2bytes(min_open)
        )
        payload.append(self._checknum(payload, datanum + 4))
        self._send_command(payload, 7)

    def set_speed(self, speed: int) -> None:
        if not 1 <= speed <= 1000:
            raise ValueError("Speed must be within 1-1000.")

        datanum = 0x03
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0x51] + self._data2bytes(speed)
        payload.append(self._checknum(payload, datanum + 4))

        self._send_command(payload, 7)

    def force_grip(self, speed: int, force_threshold: int) -> None:
        if not 1 <= speed <= 1000:
            raise ValueError("Speed must be within 1-1000.")
        if not 50 <= force_threshold <= 1000:
            raise ValueError("Force threshold must be within 50-1000.")

        datanum = 0x05
        payload = (
            [0xEB, 0x90, self.gripper_id, datanum, 0x10]
            + self._data2bytes(speed)
            + self._data2bytes(force_threshold)
        )
        payload.append(self._checknum(payload, datanum + 4))

        self._send_command(payload, 7)

    def continuous_force_grip(self, speed: int, force_threshold: int) -> None:
        if not 1 <= speed <= 1000:
            raise ValueError("Speed must be within 1-1000.")
        if not 50 <= force_threshold <= 1000:
            raise ValueError("Force threshold must be within 50-1000.")

        datanum = 0x05
        payload = (
            [0xEB, 0x90, self.gripper_id, datanum, 0x18]
            + self._data2bytes(speed)
            + self._data2bytes(force_threshold)
        )
        payload.append(self._checknum(payload, datanum + 4))

        self._send_command(payload, 7)

    def release(self, speed: int) -> None:
        if not 1 <= speed <= 1000:
            raise ValueError("Speed must be within 1-1000.")

        datanum = 0x03
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0x11] + self._data2bytes(speed)
        payload.append(self._checknum(payload, datanum + 4))

        self._send_command(payload, 7)

    def get_speed(self) -> int:
        datanum = 0x01
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0xD2]
        payload.append(self._checknum(payload, datanum + 4))

        response = self._send_command(payload, 8)
        if len(response) < 8:
            return -1
        return response[5] + (response[6] << 8)

    def get_force_threshold(self) -> int:
        datanum = 0x01
        payload = [0xEB, 0x90, self.gripper_id, datanum, 0xD3]
        payload.append(self._checknum(payload, datanum + 4))

        response = self._send_command(payload, 8)
        if len(response) < 8:
            return -1
        return response[5] + (response[6] << 8)

    def list_available_ports(self) -> list[str]:
        return [port.device for port in list_ports.comports()]

    def is_port_available(self, port_name: str) -> bool:
        try:
            temp_serial = serial.Serial(port_name)  # type: ignore[attr-defined, union-attr]
            temp_serial.close()
            return True
        except SerialException:
            return False

    def get_first_available_port(self) -> str | None:
        for port in self.list_available_ports():
            if self.is_port_available(port):
                return port
        return None

    def close(self) -> None:
        if getattr(self.ser, "is_open", False):
            self.ser.close()

    def __del__(self) -> None:  # pragma: no cover - destructor behaviour
        try:
            self.close()
        except Exception:
            pass


class EliteGripper:
    """Utility wrapper responsible for Elite gripper state and controller lifecycle."""

    def __init__(self, config: "EliteConfig", *, controller_cls: type["EliteGripperController"] = EliteGripperController):
        self._config = config
        self._controller_cls = controller_cls
        self._controller: EliteGripperController | None = None

        # Elite gripper firmware expects absolute targets in [0, 1000].
        self._range_min = 0.0
        self._range_max = 1000.0

        self._last_command = config.default_gripper_state

    @property
    def last_command(self) -> float:
        return self._last_command

    def set_last_command(self, value: float) -> None:
        self._last_command = value

    def initialise(self) -> None:
        self.shutdown()

        port = self._config.gripper_serial_port or "COM0"

        try:
            self._controller = self._controller_cls(
                port=port,
                baudrate=self._config.gripper_baudrate,
                timeout=self._config.gripper_timeout_s,
            )
            logger.info("Elite gripper controller initialised on port %s.", port)
            self._last_command = self._config.default_gripper_state
        except Exception as exc:  # pragma: no cover - hardware interaction
            self._controller = None
            raise ValueError(f"Failed to initialise Elite gripper controller on {port}: {exc}") from exc

    def shutdown(self) -> None:
        controller = self._controller
        if controller is None:
            return

        try:
            controller.close()
        except Exception as exc:  # pragma: no cover - hardware interaction
            logger.debug("Failed to close Elite gripper controller cleanly: %s", exc)
        finally:
            self._controller = None

    def send_command(self, command: float) -> float:
        target_position = command
        self.set_last_command(target_position)
        controller = self._controller
        try:
            controller.move_to_target(int(round(target_position)))
        except Exception as exc:  # pragma: no cover - hardware interaction
            logger.warning("Failed to send Elite gripper command: %s", exc)
        return target_position

    def read_state(self) -> float:
        return self._last_command

