"""RealMan RM_API2 TCP transport.

The vendor package is imported only inside ``connect``. RM_API2 ships a synchronous ctypes
client, so calls are sent through one lock and one worker thread; the non-blocking RM_MOVEJ
mode keeps the stop command able to acquire that lock while a trajectory is running.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import os
import threading
import time
from collections.abc import AsyncIterator
from typing import Any
from urllib.parse import urlsplit

from quackd.adapters.base import AdapterError, AdapterNotInstalled
from quackd.transport.base import Ack, DuckState, HeartbeatError, Intent, TransportError

DEFAULT_PORT = 8080
DEFAULT_SPEED_PERCENT = 10
SPEED_ENV = "QUACKD_REALMAN_SPEED_PERCENT"


def parse_address(address: str | None) -> tuple[str, int]:
    if not address:
        raise AdapterError("realman:real needs --address HOST[:PORT] (default port: 8080)")
    raw = address.strip()
    parts = urlsplit(raw if "://" in raw else f"tcp://{raw}")
    if parts.scheme != "tcp" or not parts.hostname or parts.path not in ("", "/"):
        raise AdapterError("realman:real address must be tcp://HOST[:PORT] or HOST[:PORT]")
    try:
        port = parts.port or DEFAULT_PORT
    except ValueError as exc:
        raise AdapterError(f"realman:real has an invalid port in {address!r}") from exc
    if not 1 <= port <= 65535 or parts.query or parts.fragment:
        raise AdapterError(f"realman:real has an invalid address {address!r}")
    return parts.hostname, port


def configured_speed_percent() -> int:
    raw = os.getenv(SPEED_ENV)
    if raw is None:
        return DEFAULT_SPEED_PERCENT
    try:
        value = int(raw)
    except ValueError as exc:
        raise AdapterError(f"{SPEED_ENV} must be an integer from 1 to 100") from exc
    if not 1 <= value <= 100:
        raise AdapterError(f"{SPEED_ENV} must be an integer from 1 to 100")
    return value


def _status(result: Any, operation: str) -> int:
    try:
        code = int(result)
    except (TypeError, ValueError) as exc:
        raise TransportError(f"realman {operation} returned a non-numeric status") from exc
    if code:
        raise TransportError(f"realman {operation} failed with SDK status {code}", code=code)
    return code


class RealManTcp:
    name = "real"

    def __init__(
        self,
        address: str | None,
        *,
        speed_percent: int = DEFAULT_SPEED_PERCENT,
        robot: Any | None = None,
    ) -> None:
        self.address = address
        self.speed_percent = speed_percent
        self.robot = robot
        self.connected = False
        self.model = "realman-arm"
        self.arm_dof = 0
        self.joint_names: tuple[str, ...] = ()
        self.joint_ranges_deg: dict[str, tuple[float, float]] = {}
        self._owns_robot = False
        self._sdk_lock = threading.RLock()

    def _invoke(self, method: str, *args: Any) -> Any:
        if self.robot is None:
            raise TransportError("realman SDK is not connected")
        with self._sdk_lock:
            return getattr(self.robot, method)(*args)

    async def _call(self, method: str, *args: Any) -> Any:
        # ponytail: one lock is enough for RM_API2's synchronous client; use a worker pool only
        # if measured SDK throughput or cancellation latency makes this insufficient.
        return await asyncio.to_thread(self._invoke, method, *args)

    async def connect(self) -> None:
        if self.connected:
            return
        if self.robot is None:
            try:
                from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
            except ModuleNotFoundError as exc:
                raise AdapterNotInstalled("realman", "quackd[realman]") from exc
            except ImportError as exc:
                raise AdapterNotInstalled("realman", "quackd[realman]") from exc
            except (NameError, OSError) as exc:
                raise AdapterError(
                    "RealMan SDK is installed but its native RM_API2 library is unavailable "
                    "for this operating system/architecture"
                ) from exc

            host, port = parse_address(self.address)
            self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self._owns_robot = True
            await self._call("rm_set_timeout", 1000)
            handle = await self._call("rm_create_robot_arm", host, port, 3)
            if int(getattr(handle, "id", -1)) < 0:
                self.robot = None
                raise TransportError(f"realman controller at {host}:{port} refused the connection")
        await self._refresh_info()
        self.connected = True

    async def _refresh_info(self) -> None:
        result = await self._call("rm_get_robot_info")
        if not isinstance(result, (tuple, list)) or len(result) < 2:
            raise TransportError("realman rm_get_robot_info returned an invalid result")
        _status(result[0], "rm_get_robot_info")
        info = result[1] if isinstance(result[1], dict) else {}
        raw_dof = info.get("arm_dof") or getattr(self.robot, "arm_dof", 0)
        self.arm_dof = int(str(raw_dof))
        if self.arm_dof <= 0:
            raise TransportError("realman controller reported no arm degrees of freedom")
        self.model = str(info.get("arm_model") or "unknown")
        self.joint_names = tuple(f"joint_{index}" for index in range(1, self.arm_dof + 1))
        self.joint_ranges_deg = await self._read_ranges()

    async def _read_ranges(self) -> dict[str, tuple[float, float]]:
        minimum = getattr(self.robot, "rm_get_joint_min_pos", None)
        maximum = getattr(self.robot, "rm_get_joint_max_pos", None)
        if not callable(minimum) or not callable(maximum):
            return {}
        try:
            lows = await self._call("rm_get_joint_min_pos")
            highs = await self._call("rm_get_joint_max_pos")
            if not all(
                isinstance(item, (tuple, list)) and len(item) >= 2 for item in (lows, highs)
            ):
                return {}
            _status(lows[0], "rm_get_joint_min_pos")
            _status(highs[0], "rm_get_joint_max_pos")
            low_values, high_values = lows[1], highs[1]
            return {
                joint: (float(low_values[index]), float(high_values[index]))
                for index, joint in enumerate(self.joint_names)
                if index < len(low_values) and index < len(high_values)
            }
        except (TransportError, TypeError, ValueError):
            return {}

    def _require_connected(self) -> None:
        if not self.connected or self.robot is None:
            raise TransportError("realman: not connected")

    async def _tuple_result(self, method: str) -> tuple[Any, Any]:
        self._require_connected()
        result = await self._call(method)
        if not isinstance(result, (tuple, list)) or len(result) < 2:
            raise TransportError(f"realman {method} returned an invalid result")
        _status(result[0], method)
        return result[0], result[1]

    async def _current_joints(self) -> list[float]:
        _, state = await self._tuple_result("rm_get_joint_degree")
        if not isinstance(state, (tuple, list)):
            raise TransportError("realman rm_get_joint_degree returned no joint list")
        return [float(value) for value in state[: self.arm_dof]]

    @staticmethod
    def _errors(current: dict[str, Any], complete: dict[str, Any], controller: Any) -> list[str]:
        errors: list[str] = []
        for source in (current.get("err"), complete.get("err")):
            if isinstance(source, dict) and int(source.get("err_len", 0) or 0):
                errors.extend(str(item) for item in source.get("err", []))
        for index, code in enumerate(complete.get("joint_err_code", []), start=1):
            if int(code):
                errors.append(f"joint_{index}:{int(code)}")
        if isinstance(controller, dict) and int(controller.get("system_error", 0) or 0):
            errors.append(f"controller:{int(controller['system_error'])}")
        return errors

    async def get_state(self) -> DuckState:
        _, current = await self._tuple_result("rm_get_current_arm_state")
        _, complete = await self._tuple_result("rm_get_arm_all_state")
        controller: dict[str, Any] = {}
        with contextlib.suppress(TransportError):
            value = await self._call("rm_get_controller_state")
            if isinstance(value, dict):
                controller = value
        power: int | None = None
        with contextlib.suppress(TransportError):
            value = await self._call("rm_get_arm_power_state")
            if isinstance(value, (tuple, list)) and len(value) >= 2 and int(value[0]) == 0:
                power = int(value[1])
        joints = [float(value) for value in current.get("joint", [])[: self.arm_dof]]
        errors = self._errors(current, complete, controller)
        return DuckState(
            t=self.now(),
            policy="controller",
            posture="unknown",
            extras={
                "connected": True,
                "arm_model": self.model,
                "arm_dof": self.arm_dof,
                "joint_names": list(self.joint_names),
                "joints": dict(zip(self.joint_names, joints, strict=False)),
                "pose": list(current.get("pose", [])),
                "joint_temperature": list(complete.get("joint_temperature", []))[: self.arm_dof],
                "joint_current_ma": list(complete.get("joint_current", []))[: self.arm_dof],
                "joint_voltage": list(complete.get("joint_voltage", []))[: self.arm_dof],
                "joint_enabled": list(complete.get("joint_en_flag", []))[: self.arm_dof],
                "joint_ranges_deg": dict(self.joint_ranges_deg),
                "controller": controller,
                "power": power,
                "errors": errors,
                "fall_detection": False,
            },
        )

    async def get_frame(self) -> None:
        return None

    async def _move_joints(self, params: dict[str, Any]) -> None:
        raw_positions = params.get("positions", {})
        if not isinstance(raw_positions, dict) or not raw_positions:
            raise TransportError("move_joints needs at least one joint", code=1)
        current = await self._current_joints()
        target = list(current)
        by_name = {name: index for index, name in enumerate(self.joint_names)}
        for raw_name, raw_goal in raw_positions.items():
            name = str(raw_name)
            if name not in by_name:
                raise TransportError(f"unknown RealMan joint {name!r}", code=1)
            goal = float(raw_goal)
            if not math.isfinite(goal):
                raise TransportError(f"{name} must be a finite angle", code=1)
            if name in self.joint_ranges_deg:
                low, high = self.joint_ranges_deg[name]
                if not low <= goal <= high:
                    raise TransportError(
                        f"{name}={goal:g} is outside the controller range "
                        f"{low:g}..{high:g} degrees",
                        code=1,
                    )
            target[by_name[name]] = goal
        # ponytail: fixed conservative speed until a shared joint-plan intent carries speed.
        _status(
            await self._call("rm_movej", target, self.speed_percent, 0, 0, 0),
            "rm_movej",
        )

    async def send_intent(self, intent: Intent) -> Ack:
        try:
            if intent.kind == "stop":
                _status(await self._call("rm_set_arm_stop"), "rm_set_arm_stop")
            elif intent.kind == "joint":
                await self._move_joints(intent.params)
            else:
                return Ack(accepted=False, reason=f"RealMan does not implement {intent.kind!r}")
        except TransportError as exc:
            if exc.code is not None and exc.code > 0:
                return Ack(accepted=False, reason=str(exc))
            raise
        return Ack()

    async def heartbeat(self) -> None:
        try:
            await self._tuple_result("rm_get_current_arm_state")
        except Exception as exc:
            if isinstance(exc, HeartbeatError):
                raise
            raise HeartbeatError(f"realman heartbeat failed: {exc}") from exc

    async def stop(self) -> None:
        self._require_connected()
        _status(await self._call("rm_set_arm_stop"), "rm_set_arm_stop")

    async def close(self) -> None:
        if self.robot is None:
            self.connected = False
            return
        with contextlib.suppress(Exception):
            await self.stop()
        if self._owns_robot:
            with contextlib.suppress(Exception):
                await self._call("rm_delete_robot_arm")
            self.robot = None
            self._owns_robot = False
        self.connected = False

    def subscribe(self, topic: str) -> AsyncIterator[dict[str, Any]]:
        return self._subscribe(topic)

    async def _subscribe(self, topic: str) -> AsyncIterator[dict[str, Any]]:
        while self.connected:
            yield {"topic": topic, **(await self.get_state()).model_dump()}
            await asyncio.sleep(0.5)

    def now(self) -> float:
        return time.monotonic()

    async def sleep(self, seconds: float) -> None:
        await asyncio.sleep(seconds)


__all__ = [
    "DEFAULT_PORT",
    "DEFAULT_SPEED_PERCENT",
    "RealManTcp",
    "configured_speed_percent",
    "parse_address",
]
