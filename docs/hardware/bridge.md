# Hardware bridge (SO-101)

See the module doc: [../modules/hardware.md](../modules/hardware.md).

**v2.2.0** adds the safety layer (`SafetySupervisor`) between the
integrator and the bus write. Develop against `MockStsBus`,
`fret-hw fake-bus` (pty), or recorded replays. Torque enable on a real
serial port requires an active supervisor (`FR-SYS-19`). **Not validated
on real hardware.**

Safety details: [safety.md](safety.md).
