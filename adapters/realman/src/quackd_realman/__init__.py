"""RealMan RM_API2 adapter for quackd.

The package is deliberately conservative: RM_API2 proves a TCP arm controller, joint state,
joint goals, limits, and stop. It does not prove a camera, gripper, pose planner, or external
e-stop, so none of those become quackd capabilities until a concrete device is tested.
"""

from __future__ import annotations

from collections.abc import AsyncIterator, Sequence
from typing import Any

from PIL import Image

from quackd.adapters.base import AdapterError, RestResult, one_camera_url, refuse_rest_pose
from quackd.adapters.manifest import (
    Datasheet,
    Figure,
    Frame,
    Health,
    RobotManifest,
    SafetyAuthority,
    verb_spec,
)
from quackd.transport.base import (
    Ack,
    DuckState,
    DuckTransport,
    HeartbeatError,
    Intent,
    TransportError,
)
from quackd.verbs.core import CORE
from quackd.verbs.registry import Precondition, Verb
from quackd_realman.real import (
    DEFAULT_SPEED_PERCENT,
    RealManTcp,
    configured_speed_percent,
)
from quackd_realman.verbs import realman_conditions, realman_verbs

__version__ = "0.10.0"
BACKENDS = ("mock", "real")
DEFAULT_ID = "realman-01"
STATUS = "✅ mock · 🧪 real (RM_API2 TCP SDK; no physical run yet)"
SUMMARY = "a RealMan industrial arm over the RM_API2 TCP SDK"
EXTRA = "quackd[realman]"
SDK = "Robotic_Arm"


def realman_manifest(
    backend: str,
    robot_id: str | None = None,
    *,
    model: str | None = None,
    arm_dof: int | None = None,
    joint_names: Sequence[str] = (),
    joint_ranges_deg: dict[str, tuple[float, float]] | None = None,
    speed_percent: int = DEFAULT_SPEED_PERCENT,
) -> RobotManifest:
    own = realman_verbs()
    datasheet = (
        Datasheet(
            dof=Figure(
                value=float(arm_dof),
                confidence="official",
                source="RealMan controller rm_get_robot_info",
            ),
            manipulator="none",
            notes=[
                "No end-effector capability is declared until the attached hardware is identified."
            ],
        )
        if arm_dof
        else Datasheet(
            manipulator="none",
            notes=["Arm model and degree count are read from the controller at connect time."],
        )
    )
    return RobotManifest(
        id=robot_id or DEFAULT_ID,
        vendor="RealMan",
        model=model or "realman-arm",
        embodiment="arm",
        mobility="none",
        intents=["joint"],
        sensors=["joint_state"],
        verbs=[
            verb_spec(CORE["report_state"], core=True),
            verb_spec(CORE["stop"], core=True),
            verb_spec(own["move_joints"], core=False),
        ],
        preconditions={"move_joints": ["connected", "no_error"]},
        safety_authority=SafetyAuthority(native="none", deadman=False, heartbeat_hz=2.0),
        frame=Frame(
            reference="base", note="joint angles are degrees; controller TCP pose is m/rad"
        ),
        limits={"max_joint_speed_percent": float(speed_percent)},
        backend=backend,
        blurb=SUMMARY,
        datasheet=datasheet,
        extras={
            "arm_model": model,
            "arm_dof": arm_dof,
            "joint_names": list(joint_names),
            "joint_ranges_deg": dict(joint_ranges_deg or {}),
            "speed_percent": speed_percent,
            "sdk": "RM_API2 Python ctypes SDK",
            "camera": "none",
            "gripper": "not declared",
        },
    )


class RealManAdapter:
    name = "realman"

    def __init__(self, transport: DuckTransport, *, robot_id: str | None = None) -> None:
        self.transport = transport
        self.backend = transport.name
        self.robot_id = robot_id or DEFAULT_ID
        self.manifest: RobotManifest | None = None

    async def connect(self) -> RobotManifest:
        await self.transport.connect()
        self.manifest = realman_manifest(
            self.backend,
            self.robot_id,
            model=getattr(self.transport, "model", None),
            arm_dof=getattr(self.transport, "arm_dof", None) or None,
            joint_names=getattr(self.transport, "joint_names", ()),
            joint_ranges_deg=getattr(self.transport, "joint_ranges_deg", {}),
            speed_percent=int(getattr(self.transport, "speed_percent", DEFAULT_SPEED_PERCENT)),
        )
        return self.manifest

    async def disconnect(self) -> None:
        await self.transport.close()

    async def close(self) -> None:
        await self.disconnect()

    async def get_state(self) -> DuckState:
        return await self.transport.get_state()

    async def get_frame(self) -> Image.Image | None:
        return await self.transport.get_frame()

    async def send_intent(self, intent: Intent) -> Ack:
        return await self.transport.send_intent(intent)

    async def health(self) -> Health:
        try:
            await self.transport.heartbeat()
            state = await self.transport.get_state()
        except (HeartbeatError, TransportError) as exc:
            return Health(ok=False, reason=str(exc))
        return Health(
            ok=True,
            extras={
                "model": state.extras.get("arm_model"),
                "arm_dof": state.extras.get("arm_dof"),
                "errors": state.extras.get("errors", []),
            },
        )

    async def heartbeat(self) -> None:
        await self.transport.heartbeat()

    async def stop(self) -> None:
        await self.transport.stop()

    async def go_to_rest(self) -> RestResult:
        return RestResult.none("RealMan has no adapter-defined rest pose")

    def subscribe(self, topic: str) -> AsyncIterator[dict[str, Any]]:
        return self.transport.subscribe(topic)

    def now(self) -> float:
        return self.transport.now()

    async def sleep(self, seconds: float) -> None:
        await self.transport.sleep(seconds)

    def preconditions(self) -> dict[str, Precondition]:
        return realman_conditions()

    def implementations(self) -> dict[str, Verb]:
        return realman_verbs()


def describe(backend: str, robot_id: str | None = None) -> RobotManifest:
    if backend not in BACKENDS:
        raise ValueError(f"unknown realman backend {backend!r}; choose one of {BACKENDS}")
    if backend == "mock":
        from quackd_realman.mock import DEFAULT_RANGES, JOINTS

        return realman_manifest(
            backend,
            robot_id,
            model="RM_65",
            arm_dof=len(JOINTS),
            joint_names=JOINTS,
            joint_ranges_deg=DEFAULT_RANGES,
        )
    return realman_manifest(backend, robot_id)


def implementations() -> dict[str, Verb]:
    return realman_verbs()


def conditions() -> dict[str, Precondition]:
    return realman_conditions()


def make(
    backend: str,
    *,
    robot_id: str | None = None,
    seed: int | None = None,
    address: str | None = None,
    live: bool = False,
    camera_url: str | Sequence[str] | None = None,
    token: str | None = None,
    rest_pose: dict[str, float] | None = None,
) -> RealManAdapter:
    del seed, live, token
    refuse_rest_pose("realman", rest_pose)
    if one_camera_url(camera_url, spec=f"realman:{backend}") is not None:
        raise AdapterError("realman does not provide a camera; --camera-url is unsupported")
    if backend == "mock":
        from quackd_realman.mock import RealManMock

        return RealManAdapter(RealManMock(), robot_id=robot_id)
    if backend == "real":
        return RealManAdapter(
            RealManTcp(address, speed_percent=configured_speed_percent()),
            robot_id=robot_id,
        )
    raise ValueError(f"unknown realman backend {backend!r}; choose one of {BACKENDS}")


def _upstream_rows() -> tuple[tuple[str, object, str, str], ...]:
    from quackd_realman import upstream_api

    return (
        ("realman", upstream_api, "docs/adapters/realman.md", "no physical controller run yet"),
    )


UPSTREAMS = _upstream_rows()

__all__ = [
    "BACKENDS",
    "DEFAULT_ID",
    "RealManAdapter",
    "conditions",
    "describe",
    "implementations",
    "make",
    "realman_manifest",
]
