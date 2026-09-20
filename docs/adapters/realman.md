# RealMan RM_API2 adapter

This adapter connects a RealMan arm through the Python ctypes SDK in
[RM_API2](https://github.com/RealManRobot/RM_API2). The source was read at commit
[`9d75cc9`](https://github.com/RealManRobot/RM_API2/tree/9d75cc995f52095837dddca594531621be18cf7b)
on 2026-09-20. No physical controller has been run yet.

## What is wired

`realman:mock` and `realman:real` expose:

- `report_state`: controller-reported joint angles, TCP pose, joint temperatures/currents/
  voltages, enable flags, limits, and error codes;
- `move_joints`: confirmed absolute joint targets in degrees, using the controller's non-blocking
  `rm_movej` command at a conservative fixed speed;
- `stop`: `rm_set_arm_stop`.

The manifest intentionally does not expose a camera, gripper, pose command, voice, mobility,
or software emergency-stop command. RM_API2 has APIs for some of those areas, but the attached
hardware and safety semantics are not known from the SDK alone.

## Install and first connection

```bash
uv sync --extra realman
export PYTHONPATH=/path/to/RM_API2/Python
quackd list-adapters
quackd doctor --robot realman:real --address 192.168.1.18:8080
```

`--address` accepts `HOST[:PORT]` or `tcp://HOST[:PORT]`; port 8080 is the default. The SDK
checkout currently contains Linux x86/ARM64 and Windows native libraries, not macOS, so the
real backend must run on a supported target. `QUACKD_REALMAN_SPEED_PERCENT` changes the initial
controller speed cap and must stay between 1 and 100; the default is 10.

Before the first movement, clear the arm's sweep, keep the controller's physical stop within
reach, verify the model and joint limits printed by `report_state`, and test `stop` manually.
The real backend has not yet earned a hardware-tested status.

## Upstream map

The adapter keeps the exact RM_API2 names and pinned links in
[`upstream_api.py`](../../adapters/realman/src/quackd_realman/upstream_api.py):
`RoboticArm.rm_create_robot_arm`, `rm_get_robot_info`, `rm_get_current_arm_state`,
`rm_get_arm_all_state`, `rm_get_joint_min_pos`, `rm_get_joint_max_pos`, `rm_movej`, and
`rm_set_arm_stop`. SDK return codes are refusals for positive controller errors and transport
errors for negative communication failures.
