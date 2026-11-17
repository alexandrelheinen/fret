# Hardware bridge (SO-101)

See the module doc: [../modules/hardware.md](../modules/hardware.md).

**v2.1.0 is mock-only.** Develop against `MockStsBus`, `fret-hw fake-bus`
(pty), or recorded replays. The bridge refuses torque enable on
`/dev/ttyUSB*` until PR4 (`FR-SYS-18`).
