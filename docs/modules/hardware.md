# Hardware Bridge Module

**Package:** `fret.hardware`
**Source:** `src/fret/hardware/`
**Tests:** `tests/hardware/`

---

## Responsibility

Serial bridge between the ROS 2 graph and Feetech STS3215 servos on the
Seeed SO-ARM101 Pro (SO-101 follower). Active from `v2.1.0` (mock-only
until the PR4 safety layer). Criticality **C2**: wrong output is a wrong
command to a machine.

---

## Status

> **v2.1.0 — mock only.** The bridge refuses to enable torque on a real
> serial port (`FR-SYS-18`) until PR4 lands the safety supervisor. Develop
> and test against `MockStsBus`, the pty fake device, or recorded
> replays. See [testing-without-hardware.md](../hardware/testing-without-hardware.md)
> and [servo-setup.md](../hardware/servo-setup.md).

---

## Components

### `sts/` — Feetech STS3215 protocol

Packet framing (`FF FF ID LEN INST PARAMS CHK`), PING / READ / WRITE /
SYNC READ / SYNC WRITE, sign-magnitude encode/decode, and the register
table cited from LeRobot's Feetech tables (Apache-2.0) and Feetech's
STS/SMS memory table.

### `bridge_core.py` — ROS-free bridge

State machine (`DISCONNECTED` → `CONNECTED_DISABLED` → `ENABLED`), one
SYNC READ + one SYNC WRITE per cycle, `VelocityToPositionIntegrator`
(FR-CTL-07), diagnostics counters, and an identity `SafetySupervisor`
seam held for PR4.

### `bridge_node.py` — rclpy shell

Same topic contract as `MuJoCoBridgeNode`:
`/joint_commands` (`std_msgs/Float64MultiArray`, BEST_EFFORT) in,
`/joint_states` (`sensor_msgs/JointState`, ≥ 50 Hz) out.

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
| FR-HW-45 (integrity) | Bad checksum / wrong id / length / timeout rejected and counted | v2.1.0 |
| FR-HW-68 | Record/replay deterministic JSONL | v2.1.0 |
| FR-HW-71 | Every mock fault has a bridge reaction test | v2.1.0 |
| FR-SYS-17 | `launch/hardware.py robot:=so101` against pty | v2.1.0 |
| FR-SYS-18 | Refuse torque enable on real serial until PR4 | v2.1.0 |
