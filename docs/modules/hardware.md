# Hardware Bridge Module

**Package:** `fret.hardware`
**Source:** `src/fret/hardware/`
**Tests:** `tests/hardware/`

---

## Responsibility

Serial bridge between the ROS 2 graph and Feetech STS3215 servos on the
Seeed SO-ARM101 Pro (SO-101 follower). Active from `v2.1.0` (mock infra)
with the safety layer from `v2.2.0`. Criticality **C2**: wrong output is
a wrong command to a machine.

---

## Status

> **v2.2.0 — safety layer.** `SafetySupervisor` sits between the
> integrator and the bus write (and in the SO-101 MuJoCo position-actuator
> IO). Mock + sim validated; **not validated on real hardware**. See
> [safety.md](../hardware/safety.md).

---

## Components

### `sts/` — Feetech STS3215 protocol

Packet framing (`FF FF ID LEN INST PARAMS CHK`), PING / READ / WRITE /
SYNC READ / SYNC WRITE, sign-magnitude encode/decode, and the register
table cited from LeRobot's Feetech tables (Apache-2.0) and Feetech's
STS/SMS memory table.

### `safety.py` — `SafetySupervisor`

ROS-free C2 supervisor: command watchdog, limit + step clamp, enable
lease, latched `stop` (category 2), soft-start helpers, and diagnostics.
Used by the hardware bridge and the SO-101 sim twin (`FR-HW-69`).

### `bridge_core.py` — ROS-free bridge

State machine
(`DISCONNECTED` → `CONNECTED_DISABLED` → `ENABLED` → `FAULT`),
one SYNC READ + one SYNC WRITE per cycle,
`VelocityToPositionIntegrator` (FR-CTL-07), protection-register
write/readback on connect, soft start, hold-on-fault, and the
`SafetySupervisor` seam.

### `bridge_node.py` — rclpy shell

Same topic contract as `MuJoCoBridgeNode`:
`/joint_commands` (`std_msgs/Float64MultiArray`, BEST_EFFORT) in,
`/joint_states` (`sensor_msgs/JointState`, ≥ 50 Hz) out. Services
`~/enable`, `~/disable`, `~/clear`, `~/stop` (`std_srvs/Trigger`).
Subscribes to `/controller_fault`.

### `robot_io.py` — hardware `RobotIO`

Implements `fret.robots.protocols.RobotIO` so the pick-place FSM can
drive the bridge without MuJoCo.

### `cli.py` — `fret-hw`

Servo tooling: `scan`, `ping`, `read`, `watch`, `set-id`, `set-baud`,
`torque off`, `calibrate`, `import-lerobot-calibration`, `record`,
`replay`, `fake-bus`.

---

## Satisfies Requirements

| Requirement | Description | Status |
|---|---|---|
| FR-HW-20 | Connect reads absolute positions; refuse on missing/wrong model/out of range | v2.1.0 |
| FR-HW-21 | `/joint_states` ≥ 50 Hz from one SYNC READ | v2.1.0 |
| FR-HW-22 / FR-HW-23 | Velocity commands → integrator → SYNC WRITE | v2.1.0 |
| FR-HW-24 | USB serial at configured baud (default 1 Mbps) | v2.1.0 |
| FR-HW-25 | Topic parity with `MuJoCoBridgeNode` | v2.1.0 |
| FR-HW-26 | `fret-hw calibrate` + LeRobot import | v2.1.0 |
| FR-HW-27 | `fret-hw set-id` single-servo + PING verify | v2.1.0 |
| FR-HW-40 | Bus power-loss → FAULT ≤5 cycles; no auto re-enable | v2.2.0 |
| FR-HW-41 | 100 ms command watchdog → hold | v2.2.0 |
| FR-HW-42 | Bridge-side limit + step clamp | v2.2.0 |
| FR-HW-43 | Protection registers write/readback on connect | v2.2.0 |
| FR-HW-44 | Soft start; torque off on shutdown | v2.2.0 |
| FR-HW-45 | Integrity rejects counted; N failures / overload → FAULT hold | v2.1.0 / v2.2.0 |
| FR-HW-46 | `/controller_fault` or FAULT → hold; clear then enable | v2.2.0 |
| FR-HW-47 | Enable lease; expired lease holds | v2.2.0 |
| FR-HW-68 | Record/replay deterministic JSONL | v2.1.0 |
| FR-HW-69 | Same supervisor in SO-101 MuJoCo IO; sim/mock parity | v2.2.0 |
| FR-HW-70 | `~/stop` latched category-2 hold (not emergency) | v2.2.0 |
| FR-HW-71 | Every mock fault has a bridge reaction test | v2.1.0 |
| FR-SYS-17 | `launch/hardware.py robot:=so101` against pty | v2.1.0 |
| FR-SYS-18 | (v2.1.0) Mock-only torque interlock — removed in v2.2.0 | removed |
| FR-SYS-19 | Enable on non-mock port only with active supervisor | v2.2.0 |
