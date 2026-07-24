# Hardware Module

**Package:** `fret.hardware`  
**Source:** `src/fret/hardware/`  
**Tests:** `tests/hardware/`

---

## Responsibility

Serial / Micro-ROS bridge between the ROS 2 high-level stack (e.g. Raspberry Pi
5) and low-level actuation (e.g. Arduino Mega). This is a **v2.x** deliverable.

---

## Status

> **v2.x — Not yet implemented.**  
> The `bridge_node.py` stub defines the interface. Hardware integration starts
> **after** the simulation CV line (**v1.5**) is validated.  
> See [releases.md § v2.x](../releases.md#v2x--hardware-integration-line) and
> [roadmap.md](../roadmap.md).

**v1.3–v1.5 are simulation-only** (including computer vision). Do not schedule
HITL work under those tags.

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

## Target stack (indicative)

```
RPi 5 (ROS 2)  ──[USB/UART]──  Arduino Mega (Micro-ROS)
                                      │
                                 motor drivers / encoders
```

Protocol details are finalized when **v2.0** planning opens. Micro-ROS is the
preferred transport; a custom ASCII fallback may exist.

| Item | Guidance |
| --- | --- |
| Communication | USB serial or UART (Micro-ROS) |
| Command rate | Match control (≥ 50 Hz when closed-loop) |
| Feedback | Encoders → `/joint_states` ≥ 50 Hz |

---

## Modular v2.x steps

| Tag | Focus |
| --- | --- |
| v2.0 | Bridge + encoder loop |
| v2.1 | Real cameras → same `fret.vision` contracts |
| v2.2 | Real arm actuation |
| v2.3 | Full HITL pick-and-place |

---

## Satisfies Requirements (v2.x)

| Requirement | Description |
|---|---|
| FR-HW-01 | Relay `/joint_commands` to Arduino via Micro-ROS |
| FR-HW-02 | Publish encoder feedback on `/joint_states` ≥ 50 Hz |
| FR-HW-03 | Validate message integrity before actuation |
