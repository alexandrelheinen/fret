# Hardware Module

**Package:** `fret.hardware`  
**Source:** `src/fret/hardware/`  
**Tests:** `tests/hardware/`

---

## Responsibility

The hardware module provides the serial communication bridge between the ROS 2
high-level controller (Raspberry Pi 5) and the low-level actuator controller
(Arduino Mega). It is a **Phase 3 deliverable** and is currently implemented as
a stub.

---

## Status

> **Phase 3 — Not yet implemented.**
> The `bridge_node.py` stub exists to define the interface and maintain the
> architecture structure. Hardware integration begins after SITL validation
> (Milestones 1–5) is complete.

---

## Components

### `bridge_node.py` — BridgeNode (stub)

Future ROS 2 node that will:

1. Subscribe to `/joint_commands` (`std_msgs/Float64MultiArray`) and relay
   velocity commands to the Arduino Mega via Micro-ROS serial link.
2. Receive joint encoder feedback from the Arduino and publish it to
   `/joint_states` at ≥ 50 Hz.
3. Validate message integrity (checksum / frame delimiter) before forwarding
   any command to the actuator driver.

---

## Planned Serial Protocol

```
RPi 5 (ROS 2)  ──[USB/UART]──  Arduino Mega (Micro-ROS)
    │                                    │
    │  /joint_commands ──► BridgeNode ──► serial frame ──► motor drivers
    │                                    │
    │  /joint_states  ◄── BridgeNode ◄── encoder data ◄── encoders
```

The protocol will be finalized during Phase 3. Micro-ROS is the preferred
transport; a custom ASCII protocol is the fallback.

---

## Hardware Targets

| Component | Target |
|---|---|
| High-level controller | Raspberry Pi 5 (Ubuntu 24.04) |
| Low-level controller | Arduino Mega |
| Communication | USB serial or UART (Micro-ROS) |
| Actuators | Nema 17 stepper class |
| Drivers | TMC series (e.g., TMC2209) |

---

## Satisfies Requirements (Phase 3)

| Requirement | Description |
|---|---|
| FR-HW-01 | Relay `/joint_commands` to Arduino via Micro-ROS |
| FR-HW-02 | Publish encoder feedback to `/joint_states` at ≥ 50 Hz |
| FR-HW-03 | Validate message integrity before forwarding commands |
