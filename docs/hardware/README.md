# Hardware (v2.x)

This directory, together with `docs/robots/`, is where FRET documents
specific hardware targets. The rest of `docs/` is hardware-neutral
(`FR-SYS-13`): it does not assume a particular robot, microcontroller, bus,
or protocol.

## Status as of v2.0.0

The v2.x hardware target is the **Seeed SO-ARM101 Pro** (SO-101 follower: 5
revolute joints + a friction gripper, Feetech STS3215 USB smart servos, no
onboard firmware), driven by a Raspberry Pi 5. See
[`arm-selection.md`](arm-selection.md) for the decision record (requirements,
candidates, scored comparison, known risks) and
[`docs/robots/so101.md`](../robots/so101.md) for the model page. `FR-HW-01`
through `FR-HW-03` in [`../requirements.md`](../requirements.md) remain marked
"superseded in v2.0" (they described an earlier Arduino + Micro-ROS plan);
`FR-HW-10` onward define the SO-101 / Pi 5 line.

PR2 (`v2.0.0`) ships the hardware study and a **simulation twin only** — no
serial, servo, or camera driver code yet (that is PR3 onward):

- [`fret.hardware.bridge_node.BridgeNode`](../../src/fret/hardware/bridge_node.py)
  is still an intentional stub: constructing it raises `NotImplementedError`.
  `tests/hardware/test_bridge_node.py` keeps that behavior pinned with
  `xfail(strict=True)`.
- `ros2 launch fret hardware.py` still fails fast with a clear "not available
  before v2.x" message.
- The `so101` MuJoCo simulation variant (profile, pure-NumPy kinematics,
  position-actuator scene, `so101_reach` scenario) is real and tested; the
  friction-grasp pick-and-place and vision scenarios described in
  `.claude/roadmap/pr2-so101-variant/spec.md` (FR-SIM-18/19) are deferred —
  see `docs/decisions.md`.

## What comes next

Per the FRET v2 hardware roadmap: PR2 defines the SO-101 hardware study and
its simulation variant (`v2.0.0`); PR3 builds the hardware-free bridge test
infrastructure (mock Feetech bus, recorded replays, fault injection,
`fret-hw` tooling) and the real bridge developed TDD against it
(`v2.1.0`, mock only); PR4 adds the safety layer (`v2.2.0`); PR5 is the
Raspberry Pi 5 deployment and bring-up kit (`v2.3.0-rc.1`). Each phase's
durable spec lands here or under `docs/robots/` at merge time.
