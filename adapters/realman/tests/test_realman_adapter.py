from __future__ import annotations

import pytest
from quackd_realman import RealManAdapter, make
from quackd_realman.real import RealManTcp, parse_address

from quackd.adapters.base import AdapterError
from quackd.safety import Executor
from quackd.transport.base import Intent
from quackd.verbs.registry import registry_from_manifest


def test_address_parser() -> None:
    assert parse_address("192.168.1.18") == ("192.168.1.18", 8080)
    assert parse_address("tcp://robot.local:9000") == ("robot.local", 9000)
    with pytest.raises(AdapterError):
        parse_address("serial:///dev/ttyUSB0")


@pytest.mark.asyncio
async def test_mock_manifest_executor_and_stop() -> None:
    adapter = make("mock")
    manifest = await adapter.connect()
    assert set(manifest.verb_names()) == {"report_state", "stop", "move_joints"}
    assert manifest.intents == ["joint"]

    executor = Executor(
        registry_from_manifest(manifest, adapter),
        adapter,
        confirm=lambda _name, _params: True,
    )
    result = await executor.run_verb(
        "move_joints", {"positions": {"joint_1": 25.0}, "duration_s": 0.2}
    )
    assert result.ok
    assert (await adapter.get_state()).extras["joints"]["joint_1"] == 25.0

    await adapter.stop()
    assert adapter.transport.intents_of("stop")  # type: ignore[attr-defined]
    await adapter.close()


class FakeSdkRobot:
    def __init__(self) -> None:
        self.joints = [0.0] * 6
        self.moves: list[list[float]] = []
        self.stops = 0

    def rm_get_robot_info(self):
        return 0, {"arm_dof": 6, "arm_model": "RM_65"}

    def rm_get_joint_min_pos(self):
        return 0, [-90.0] * 6

    def rm_get_joint_max_pos(self):
        return 0, [90.0] * 6

    def rm_get_current_arm_state(self):
        return 0, {
            "joint": list(self.joints),
            "pose": [0.0] * 6,
            "err": {"err_len": 0, "err": []},
        }

    def rm_get_arm_all_state(self):
        return 0, {
            "joint_current": [0.0] * 6,
            "joint_en_flag": [1] * 6,
            "joint_temperature": [30.0] * 6,
            "joint_voltage": [24.0] * 6,
            "joint_err_code": [0] * 6,
            "err": {"err_len": 0, "err": []},
        }

    def rm_get_controller_state(self):
        return {"system_error": 0}

    def rm_get_arm_power_state(self):
        return 0, 1

    def rm_get_joint_degree(self):
        return 0, list(self.joints)

    def rm_movej(self, target, _speed, _blend, _connect, _block):
        self.joints = list(target)
        self.moves.append(self.joints)
        return 0

    def rm_set_arm_stop(self):
        self.stops += 1
        return 0


@pytest.mark.asyncio
async def test_real_wrapper_with_fake_sdk() -> None:
    transport = RealManTcp("tcp://robot.local:8080", robot=FakeSdkRobot())
    adapter = RealManAdapter(transport)
    manifest = await adapter.connect()
    assert manifest.model == "RM_65"

    ack = await adapter.send_intent(Intent.joint({"joint_1": 45.0}))
    assert ack.accepted
    assert (await adapter.get_state()).extras["joints"]["joint_1"] == 45.0
    refused = await adapter.send_intent(Intent.joint({"joint_1": 100.0}))
    assert not refused.accepted and "outside" in (refused.reason or "")

    await adapter.stop()
    assert transport.robot.stops == 1
    await adapter.close()
