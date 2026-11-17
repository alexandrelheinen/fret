# Hardware (v2.x)

This directory, together with `docs/robots/`, is where FRET documents
specific hardware targets. The rest of `docs/` is hardware-neutral
(`FR-SYS-13`): it does not assume a particular robot, microcontroller, bus,
or protocol.

## Status as of v2.1.0

The v2.x hardware target is the **Seeed SO-ARM101 Pro** (SO-101 follower: 5
revolute joints + a friction gripper, Feetech STS3215 USB smart servos, no
onboard firmware), driven by a Raspberry Pi 5. See
[`arm-selection.md`](arm-selection.md) for the decision record and
[`docs/robots/so101.md`](../robots/so101.md) for the model page.

PR3 (`v2.1.0`) ships the **hardware-free test infrastructure** and the
SO-101 bridge developed TDD against it. **Mock only — not validated on
real hardware.** Torque enable on a real serial port is refused until
PR4 (`FR-SYS-18`).

| Doc | Contents |
|---|---|
| [testing-without-hardware.md](testing-without-hardware.md) | Mock bus, pty fake, record/replay |
| [servo-setup.md](servo-setup.md) | `fret-hw` commands and calibration |
| [bridge.md](bridge.md) | Bridge module overview |
| [arm-selection.md](arm-selection.md) | Hardware decision record |
| [../modules/hardware.md](../modules/hardware.md) | Module map and FR checklist |

## What comes next

PR4 (`v2.2.0`): safety layer (watchdog, limits, soft start, fault hold).
PR5: Raspberry Pi 5 deployment and bring-up kit.
