"""Socket-based JSON-RPC client for the Elite controller.

This client is intentionally minimal and only exposes endpoints used by the
LeRobot ``Elite`` adapter. The Elite API returns JSON strings nested in JSON
objects, so we decode those whenever possible to work with native Python
structures.
"""

from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from typing import Any

JSONRPC_VERSION = "2.0"


class EliteClientError(RuntimeError):
    """Raised when the Elite controller reports an error or the socket fails."""


@dataclass(slots=True)
class EliteClientConfig:
    """Configuration options for :class:`EliteClient`."""

    ip: str = "192.168.1.200"
    port: int = 8055
    timeout_s: float = 2.0


class EliteClient:
    """JSON-RPC client for the Elite controller."""

    def __init__(self, config: EliteClientConfig):
        self.config = config
        self._socket: socket.socket | None = None
        self._last_request_id = 0

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def connect(self) -> None:
        if self._socket is not None:
            return

        sock = socket.create_connection((self.config.ip, self.config.port), timeout=self.config.timeout_s)
        sock.settimeout(self.config.timeout_s)
        self._socket = sock

    def disconnect(self) -> None:
        if self._socket is None:
            return

        self._socket.close()
        self._socket = None

    # ------------------------------------------------------------------
    # RPC helpers
    # ------------------------------------------------------------------
    def _next_request_id(self) -> int:
        self._last_request_id += 1
        return self._last_request_id

    def _call(self, method: str, params: dict[str, Any] | list[Any] | None = None) -> Any:
        if self._socket is None:
            raise EliteClientError("EliteClient is not connected.")

        request_id = self._next_request_id()
        payload = {
            "jsonrpc": JSONRPC_VERSION,
            "method": method,
            "params": params or {},
            "id": request_id,
        }
        message = json.dumps(payload) + "\n"
        try:
            self._socket.sendall(message.encode("utf-8"))
            response_bytes = self._socket.recv(4096)
        except OSError as exc:
            raise EliteClientError(f"Elite controller communication failed for method '{method}'.") from exc

        try:
            response = json.loads(response_bytes.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise EliteClientError(f"Elite controller returned non-JSON response for method '{method}'.") from exc

        if response.get("id") != request_id:
            raise EliteClientError("Elite controller returned response with unexpected id.")

        if "error" in response:
            raise EliteClientError(f"Elite controller error for method '{method}': {response['error']}")

        result = response.get("result")
        if isinstance(result, str):
            # The reference client wraps JSON strings inside JSON responses.
            try:
                return json.loads(result)
            except json.JSONDecodeError:
                return result
        return result

    # ------------------------------------------------------------------
    # Servo & controller state methods
    # ------------------------------------------------------------------
    def get_servo_status(self) -> bool:
        return bool(self._call("getServoStatus"))

    def set_servo_status(self, status: int) -> bool:
        return bool(self._call("set_servo_status", {"status": status}))

    def sync_motor_status(self) -> bool:
        return bool(self._call("syncMotorStatus"))

    def clear_alarm(self) -> bool:
        return bool(self._call("clearAlarm"))

    def get_motor_status(self) -> bool:
        return bool(self._call("getMotorStatus"))

    def get_robot_state(self) -> int:
        return int(self._call("getRobotState"))

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------
    def get_robot_pos(self) -> list[float]:
        result = self._call("getRobotPos")
        if not isinstance(result, list):
            raise EliteClientError("Elite controller returned malformed joint positions.")
        return [float(val) for val in result]

    def get_robot_pose(self, coordinate_num: int = -1, tool_num: int = -1) -> list[float]:
        result = self._call(
            "getRobotPose",
            {"coordinate_num": coordinate_num, "tool_num": tool_num},
        )
        if not isinstance(result, list):
            raise EliteClientError("Elite controller returned malformed pose data.")
        return [float(val) for val in result]

    # ------------------------------------------------------------------
    # Transparent transmission helpers
    # ------------------------------------------------------------------
    def get_transparent_transmission_state(self) -> int:
        return int(self._call("get_transparent_transmission_state"))

    def transparent_transmission_init(self, lookahead: float, sample_time_ms: float, smoothness: float) -> bool:
        return bool(
            self._call(
                "transparent_transmission_init",
                {
                    "lookahead": lookahead,
                    "t": sample_time_ms,
                    "smoothness": smoothness,
                },
            )
        )

    def tt_set_current_servo_joint(self, target_pos: list[float]) -> bool:
        return bool(self._call("tt_set_current_servo_joint", {"targetPos": target_pos}))

    def tt_put_servo_joint_to_buf(self, target_pos: list[float]) -> bool:
        return bool(self._call("tt_put_servo_joint_to_buf", {"targetPos": target_pos}))

    def tt_clear_servo_joint_buf(self, clear: int = 0) -> bool:
        return bool(self._call("tt_clear_servo_joint_buf", {"clear": clear}))

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def inverse_kinematic(self, target_pose: list[float]) -> list[float]:
        result = self._call("inverseKinematic", {"targetPose": target_pose})
        if not isinstance(result, list):
            raise EliteClientError("Elite controller returned malformed joint configuration.")
        return [float(val) for val in result]

    def stop(self) -> bool:
        return bool(self._call("stop"))

    def run(self) -> bool:
        return bool(self._call("run"))

    def pause(self) -> bool:
        return bool(self._call("pause"))
