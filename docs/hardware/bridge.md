# Hardware Bridge Module

**Package:** `fret.hardware`
**Source:** `src/fret/hardware/`
**Tests:** `tests/hardware/`

---

## Responsibility

A serial bridge between the ROS 2 high-level stack and low-level robot
actuation. This is a **v2.x** deliverable.

---

## Status

> **v2.x — Not yet implemented; target undefined.**
> The `bridge_node.py` stub defines the interface shape only.
> FR-HW-01 through FR-HW-03 below are **superseded in v2.0, pending
> redefinition** (not renumbered): they described an earlier
> Arduino + Micro-ROS plan that the current roadmap no longer assumes.
> See [releases.md § v2.x](../releases.md#v2x--hardware-integration-line) and
> [roadmap.md](../roadmap.md) for the current plan.

**v1.x is simulation-only** (including computer vision). Do not schedule
HITL work under v1.x tags.

---

## Components

### `bridge_node.py` — BridgeNode (stub)

Future ROS 2 node that will:

1. Subscribe to `/joint_commands` (`std_msgs/Float64MultiArray`) and relay
   velocity commands to the robot over a serial link.
2. Receive joint feedback from the robot and publish it to `/joint_states`
   at ≥ 50 Hz.
3. Validate message integrity (checksum / frame delimiter) before forwarding
   any command to the actuator driver.

---

## Target stack

Not yet defined as of v1.5.0. The actual robot, bus, and protocol are
decided when hardware planning for the relevant phase opens; see
[roadmap.md](../roadmap.md) for the current plan and phase order.

---

## Satisfies Requirements

| Requirement | Description | Status |
|---|---|---|
| FR-HW-01 | Relay `/joint_commands` to the robot | Superseded in v2.0, pending redefinition |
| FR-HW-02 | Publish feedback on `/joint_states` ≥ 50 Hz | Superseded in v2.0, pending redefinition |
| FR-HW-03 | Validate message integrity before actuation | Superseded in v2.0, pending redefinition |
