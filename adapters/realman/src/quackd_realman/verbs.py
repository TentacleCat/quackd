"""RealMan's small quackd verb surface: state, stop, and joint goals in degrees."""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field, field_validator

from quackd.transport.base import DuckState, Intent
from quackd.verbs.core import send_or_fail
from quackd.verbs.registry import Precondition, Verb, VerbContext, VerbResult


class MoveJointsParams(BaseModel):
    model_config = ConfigDict(extra="forbid")

    positions: dict[str, float] = Field(
        ...,
        description=(
            "Joint name -> target angle in degrees. Omitted joints keep their current angle."
        ),
    )
    duration_s: float = Field(
        default=1.0,
        ge=0.2,
        le=10.0,
        description="How long to wait before reading the result; controller speed stays capped.",
    )

    @field_validator("positions")
    @classmethod
    def _non_empty_finite(cls, value: dict[str, float]) -> dict[str, float]:
        if not value:
            raise ValueError("give at least one joint")
        if any(not math.isfinite(float(angle)) for angle in value.values()):
            raise ValueError("joint angles must be finite numbers")
        return value


def connected(state: DuckState) -> str | None:
    if state.extras.get("connected") is not True:
        return "the RealMan controller is not connected"
    return None


def no_error(state: DuckState) -> str | None:
    errors = state.extras.get("errors", [])
    if errors:
        return f"the controller reports errors: {', '.join(map(str, errors))}"
    return None


def realman_conditions() -> dict[str, Precondition]:
    return {"connected": connected, "no_error": no_error}


async def move_joints(ctx: VerbContext, p: MoveJointsParams) -> VerbResult:
    if (fail := await send_or_fail(ctx, Intent.joint(dict(p.positions), p.duration_s))) is not None:
        return fail
    await ctx.transport.sleep(p.duration_s)
    state = await ctx.transport.get_state()
    return VerbResult.success(
        "commanded " + ", ".join(f"{name}={angle:.1f}°" for name, angle in p.positions.items()),
        goal=dict(p.positions),
        joints=state.extras.get("joints", {}),
        units="degrees",
    )


def realman_verbs() -> dict[str, Verb]:
    move = Verb(
        "move_joints",
        "Move selected RealMan joints to absolute angles in degrees. Unnamed joints keep moving "
        "towards their current targets. The controller speed is capped by the adapter.",
        move_joints,
        MoveJointsParams,
        timeout_s=30,
        safety_class="confirm",
        done_condition="the controller accepted the joint goal and the latest state was read",
    )
    return {move.name: move}


__all__ = ["MoveJointsParams", "realman_conditions", "realman_verbs"]
