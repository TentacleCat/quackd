"""The RM_API2 names used by the RealMan adapter, pinned to one source tree."""

from __future__ import annotations

from quackd.upstream import UpstreamRef

REPO = "https://github.com/RealManRobot/RM_API2"
PIN = "9d75cc995f52095837dddca594531621be18cf7b"
READ_ON = "2026-09-20"
PYPI_VERSION_READ = "1.1.6"


def src(path: str, line: int | None = None) -> str:
    return f"{REPO}/blob/{PIN}/{path}" + (f"#L{line}" if line else "")


PYTHON_SDK = UpstreamRef(
    "Python/Robotic_Arm",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_robot_interface.py"),
    "the Python ctypes wrapper used by the adapter",
)
THREAD_MODE = UpstreamRef(
    "rm_thread_mode_e.RM_TRIPLE_MODE_E",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_ctypes_wrap.py", 927),
    "the SDK's three-thread mode",
)
CONNECT = UpstreamRef(
    "RoboticArm.rm_create_robot_arm(ip, port)",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_robot_interface.py", 7115),
    "TCP controller connection; the README uses port 8080",
)
ROBOT_INFO = UpstreamRef(
    "RoboticArm.rm_get_robot_info()",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_robot_interface.py", 7242),
    "reports model and arm_dof",
)
JOINT_STATE = UpstreamRef(
    "RoboticArm.rm_get_current_arm_state()",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_robot_interface.py", 1150),
    "joint angles are degrees and pose is [m, rad]",
)
JOINT_MOVE = UpstreamRef(
    "RoboticArm.rm_movej()",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_robot_interface.py", 1460),
    "joint targets are degrees; block=0 is non-blocking",
)
STOP = UpstreamRef(
    "RoboticArm.rm_set_arm_stop()",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_robot_interface.py", 2084),
    "fast trajectory stop",
)
JOINT_RANGES = UpstreamRef(
    "rm_get_joint_min_pos()/rm_get_joint_max_pos()",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_robot_interface.py", 306),
    "controller-reported degree limits",
)
NATIVE_LIBRARY = UpstreamRef(
    "libapi_c.so/api_c.dll",
    "VERIFIED",
    src("Python/Robotic_Arm/rm_ctypes_wrap.py"),
    "RM_API2 ships Linux and Windows libraries, not a macOS library in this checkout",
)
HARDWARE_RUN = UpstreamRef(
    "realman:real hardware run",
    "UNVERIFIED",
    REPO,
    "the adapter has not yet been run against a physical RealMan controller",
)


def all_refs() -> list[UpstreamRef]:
    return [value for value in globals().values() if isinstance(value, UpstreamRef)]


def refs_by_status(status: str) -> list[UpstreamRef]:
    return [ref for ref in all_refs() if ref.status == status]
