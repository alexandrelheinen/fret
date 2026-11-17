# Hardware bring-up

Day-of-arrival checklist for the Seeed SO-ARM101 Pro on a Raspberry Pi 5.
OPS-08 follows this document step by step.

## Arrival checklist (OPS-08)

1. [ ] Power: official 5 V / 5 A Pi supply + separate 12 V servo supply wired
      per `deployment-rpi5.md` / `safety.md`. **Do not** power servos from the Pi.
2. [ ] Cooling: official Active Cooler installed and spinning.
3. [ ] microSD: A2 card ≥ 64 GB; Ubuntu Server 24.04 headless first boot done.
4. [ ] SSH: can log in as the deploy user; password login disabled.
5. [ ] Install: `sudo bash scripts/deploy/rpi5_install.sh --tag <tag>` exits 0.
6. [ ] udev: capture follower + camera; `/dev/fret-servo-follower` and
      `/dev/fret-cam0` exist after replug.
7. [ ] `fret-diag pi` — pass (or skip with reason for absent sensors only).
8. [ ] `fret-diag usb` then `bus` then `servos` on the mock or live bus.
9. [ ] `fret-diag calibration` — skip until calibrations written, then pass.
10. [ ] `fret-diag estop` — FAULT on power cut; no auto re-enable.
11. [ ] Camera: `ros2 launch fret camera.py` publishes images;
      `fret-diag camera` reports format / resolution / fps.
12. [ ] `fret-calib intrinsics` / `extrinsics` / `base-check` per
      `calibration.md`; VisionConfig under `/var/lib/fret/calibration/`.
13. [ ] `fret-diag vision` — ball at the printed mark within tolerance.
14. [ ] `fret-diag loop` then `fret-diag load` (budget; Active Cooler on).
15. [ ] Motion only with guards: `fret-diag joints --allow-motion --yes` then
      `gripper` (speed ≤ 0.3, bridge `CONNECTED_DISABLED` first).
16. [ ] Supervised pick: SC-v23a (`so101_pick_place_hw.yml`) at 30 % speed with
      a hand on the e-stop / power switch.
17. [ ] Attach JSON reports from `/var/log/fret/diag/` (or `--save` copies) to
      the tracking issue.

## Safety recap

- Units start torque-disabled (FR-HW-57).
- Motion diagnostics refuse without `--allow-motion` + confirmation (FR-HW-66).
- Hold on fault; torque off on shutdown or explicit disable (PR4).
- Physical e-stop wiring: see `safety.md` (Q9 still owned by Alexandre).
