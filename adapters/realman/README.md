# quackd-realman

RealMan RM_API2 adapter for quackd.

The adapter exposes `report_state`, `stop`, and a confirmed `move_joints` verb. The mock is
usable everywhere; the real backend needs the vendor SDK and a Linux/Windows native library.

```bash
uv sync --extra realman
export PYTHONPATH=/path/to/RM_API2/Python
quackd doctor --robot realman:real --address 192.168.1.18:8080
```

See [the adapter guide](../../docs/adapters/realman.md) for the hardware checklist and the
current unsupported capabilities.
