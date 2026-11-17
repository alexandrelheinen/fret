# Safety layer (SO-101)

**Release:** `v2.2.0` (PR4) · **Criticality:** C2 · **Validated:** mock bus +
MuJoCo twin only — **not validated on real hardware**.

This document describes the bridge safety layer
(`fret.hardware.safety.SafetySupervisor`) and the stop configurations
for the Seeed SO-ARM101 Pro follower. It follows
`.guidelines/domains/robotics.md` § Stopping and § Supervision: a library
cannot own an emergency stop; "cleared" and "enabled" are separate
states; a reset permits restarting, it does not restart; a method named
`stop` is a normal (category 2) stop, never an emergency stop.

No amount of software test coverage produces a performance level or SIL.
This layer does not claim either.

---

## Stop categories (IEC 60204-1 vocabulary)

| Path | Category | Effect |
|---|---|---|
| `~/stop` | 2 (controlled stop) | Hold last measured pose; torque stays on; latched until `~/clear` |
| Fault / power loss / overload / overheat | 2 hold | Same hold; state `FAULT`; requires `~/clear` then `~/enable` |
| `~/disable` or node shutdown (SIGINT/SIGTERM) | torque off | Servos go limp — **support the arm by hand first** |
| Physical e-stop on the 12 V line | supply removal | Bus power disappears; bridge enters `FAULT` within 5 cycles (100 ms) |

`~/stop` is **not** an emergency stop. Do not name or document it as one.

---

## Configuration (Q9)

Alexandre decided (2026-11-17) that the primary path is **case (a)**: a
physical latching e-stop fitted on the 12 V servo supply. Case (b) remains
documented as a fallback only.

### Case (a) — physical e-stop fitted (primary)

Wire a latching mushroom e-stop rated for at least 12 V DC and the servo
supply current (5 A or more) **in series with the 12 V servo supply only**
(never the Pi 5 V supply). Pressing it removes bus power independently of
the Pi, Linux, or FRET.

- The bridge detects bus power loss (missing replies / low voltage) and
  enters `FAULT` within 5 control cycles (100 ms at 50 Hz) — `FR-HW-40`.
- Releasing the e-stop does **not** re-enable motion. Power returning
  never auto-enables.
- Recovery: `~/clear` (only if power is back), then `~/enable` with a
  fresh lease.

### Case (b) — no physical e-stop (fallback only)

Case (b) has **no** stop independent of software other than unplugging the
12 V barrel connector. Use only if an e-stop is not yet fitted; fit one
before first torque-on on real hardware (OPS-08).

1. Call `~/stop` — latched category-2 hold.
2. Support the arm by hand, then call `~/disable` — torque off; the arm
   goes limp.
3. For anything urgent or unexpected, unplug the 12 V servo supply. The
   bridge reacts exactly as in case (a).

No software stop is an emergency stop.

---

## State machine

```
DISCONNECTED → CONNECTED_DISABLED → ENABLED → FAULT
FAULT → (clear) → CONNECTED_DISABLED
```

Nothing reaches `ENABLED` except an explicit `~/enable` with a fresh
lease. `clear` ≠ `enable`. Enable is a level/lease (re-assert at least
every `enable_lease_s`, default 0.5 s), not an edge.

Services (`std_srvs/Trigger`): `~/enable`, `~/disable`, `~/clear`,
`~/stop`.

---

## Supervisor behaviors

Thresholds live in `config/hardware/so101.yml` under `safety:`
(`FR-SYS-04`). Defaults: watchdog 100 ms, lease 0.5 s, limit margin
0.03 rad, fault after 5 failed cycles.

| ID | Behavior |
|---|---|
| `FR-HW-41` | No valid command for 100 ms in `ENABLED` → hold measured pose; `watchdog` in diagnostics |
| `FR-HW-42` | Clamp goals to profile ∩ calibrated range − margin; per-cycle step ≤ `max_joint_velocity × dt` |
| `FR-HW-43` | On connect, write and read back torque limit, protection current, over-temperature |
| `FR-HW-44` | Soft start: goals = measured, then torque on; shutdown disables torque ("arm goes limp") |
| `FR-HW-45` | N failed cycles → `FAULT`; overload/overheat → `FAULT` immediately; hold on fault |
| `FR-HW-46` | `/controller_fault` or `FAULT` → stop integrating, hold; need `clear` then `enable` |
| `FR-HW-47` | Motion only while enable lease is fresh; expired lease holds like the watchdog |
| `FR-HW-70` | `~/stop` = latched category-2 hold until `clear` |

Fault policy (D9): **hold position** on fault. Torque off only on shutdown
or explicit `~/disable`. There is no delayed torque-off.

---

## Supervision stack

The watcher is outside the watched:

1. Bridge / `SafetySupervisor` — command freshness, limits, lease.
2. systemd watchdog (PR5) — supervises the bridge process.
3. Hardware e-stop on 12 V — supervises everything above.

A library-level `stop` is never presented as risk reduction.

SIGKILL skips shutdown hooks: servos keep their last goal. The physical
e-stop is the answer.

---

## Simulation parity

The same `SafetySupervisor` runs in the SO-101 MuJoCo position-actuator
IO adapter (`FR-HW-69`). Safety scenarios are exercised on the twin and
on the PR3 mock bus with matching expected outcomes.

---

## Related

- [bridge.md](bridge.md) — bridge overview
- [testing-without-hardware.md](testing-without-hardware.md) — mock / pty
- [../modules/hardware.md](../modules/hardware.md) — module map
- Requirements: `FR-HW-40`..`47`, `FR-HW-69`, `FR-HW-70`, `FR-SYS-19`
