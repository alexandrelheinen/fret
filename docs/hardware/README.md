# Hardware (v2.x)

This directory, together with `docs/robots/`, is where FRET documents
specific hardware targets. The rest of `docs/` is hardware-neutral
(`FR-SYS-13`): it does not assume a particular robot, microcontroller, bus,
or protocol.

## Status as of v1.5.0

The v2.x hardware target (the physical robot, its actuator bus, and its
wire protocol) is **not yet defined**. `FR-HW-01` through `FR-HW-03` in
[`../requirements.md`](../requirements.md) are marked "superseded in v2.0,
pending redefinition" without renumbering — they describe an Arduino +
Micro-ROS target from an earlier plan that this roadmap no longer assumes.

- [`fret.hardware.bridge_node.BridgeNode`](../../src/fret/hardware/bridge_node.py)
  is an intentional stub: constructing it raises `NotImplementedError`.
  `tests/hardware/test_bridge_node.py` keeps that behavior pinned with
  `xfail(strict=True)`.
- `ros2 launch fret hardware.py` fails fast with a clear "not available
  before v2.x" message rather than trying to start a hardware executable
  that is not registered in `pyproject.toml` / `CMakeLists.txt`.

## What comes next

Per the FRET v2 hardware roadmap: PR2 defines the SO-101 hardware study and
its simulation variant (`v2.0.0`); PR3 builds the hardware-free bridge test
infrastructure (mock Feetech bus, recorded replays, fault injection,
`fret-hw` tooling) and the real bridge developed TDD against it
(`v2.1.0`, mock only); PR4 adds the safety layer (`v2.2.0`); PR5 is the
Raspberry Pi 5 deployment and bring-up kit (`v2.3.0-rc.1`). Each phase's
durable spec lands here or under `docs/robots/` at merge time.
