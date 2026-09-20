"""A deterministic RealMan arm for adapter and executor tests."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from typing import Any

from quackd.transport.base import Ack, DuckState, HeartbeatError, Intent

JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
DEFAULT_RANGES = {joint: (-180.0, 180.0) for joint in JOINTS}


class RealManMock:
    name = "mock"
    model = "RM_65"
    arm_dof = len(JOINTS)
    joint_names = JOINTS
    joint_ranges_deg = DEFAULT_RANGES

    def __init__(
        self,
        *,
        positions: dict[str, float] | None = None,
        refuse_kinds: set[str] | None = None,
        errors: list[str] | None = None,
    ) -> None:
        self.joints = {joint: 0.0 for joint in JOINTS}
        if positions:
            self.joints.update(positions)
        self.refuse_kinds = refuse_kinds or set()
        self.errors = list(errors or [])
        self.connected = False
        self.intents: list[Intent] = []
        self.heartbeats = 0
        self.stops = 0
        self._t = 0.0

    async def connect(self) -> None:
        self.connected = True

    async def close(self) -> None:
        self.connected = False

    async def get_frame(self) -> None:
        return None

    async def get_state(self) -> DuckState:
        return DuckState(
            t=self._t,
            policy="controller",
            posture="unknown",
            extras={
                "connected": self.connected,
                "arm_model": self.model,
                "arm_dof": self.arm_dof,
                "joint_names": list(self.joint_names),
                "joints": dict(self.joints),
                "joint_ranges_deg": dict(self.joint_ranges_deg),
                "errors": list(self.errors),
                "fall_detection": False,
            },
        )

    async def send_intent(self, intent: Intent) -> Ack:
        self.intents.append(intent)
        if intent.kind in self.refuse_kinds:
            return Ack(accepted=False, reason=f"mock refuses {intent.kind}")
        if not self.connected:
            return Ack(accepted=False, reason="mock RealMan is not connected")
        if intent.kind == "stop":
            self.stops += 1
            return Ack()
        if intent.kind != "joint":
            return Ack(accepted=False, reason=f"RealMan does not implement {intent.kind!r}")
        for joint, raw in dict(intent.params.get("positions", {})).items():
            if joint not in self.joints:
                return Ack(accepted=False, reason=f"unknown joint {joint!r}")
            goal = float(raw)
            lo, hi = self.joint_ranges_deg[joint]
            if not lo <= goal <= hi:
                return Ack(accepted=False, reason=f"{joint}={goal} is outside {lo}..{hi} degrees")
        self.joints.update({str(k): float(v) for k, v in dict(intent.params["positions"]).items()})
        return Ack()

    def subscribe(self, topic: str) -> AsyncIterator[dict[str, Any]]:
        return self._subscribe(topic)

    async def _subscribe(self, topic: str) -> AsyncIterator[dict[str, Any]]:
        while self.connected:
            yield {"topic": topic, **(await self.get_state()).model_dump()}
            await self.sleep(0.5)

    async def heartbeat(self) -> None:
        self.heartbeats += 1
        if not self.connected or self.errors:
            raise HeartbeatError("mock RealMan heartbeat failed")

    async def stop(self) -> None:
        await self.send_intent(Intent.stop())

    def now(self) -> float:
        return self._t

    async def sleep(self, seconds: float) -> None:
        self._t += seconds
        await asyncio.sleep(0)

    def intents_of(self, kind: str) -> list[Intent]:
        return [intent for intent in self.intents if intent.kind == kind]


__all__ = ["DEFAULT_RANGES", "JOINTS", "RealManMock"]
