# Raspberry Pi 5 deployment (PR5 / v2.3.0-rc.1)

This guide makes "deploy tag X to the Pi" one reproducible, reversible
operation. Everything that moves the arm runs on the Pi; the PC is only for
simulation, development, and monitoring.

**Not validated on real hardware in this release.** Arm64 container builds on
the PC are the reproducibility check while CI is off. OPS-06/07/08 cover Pi
smoke and hardware arrival.

## What runs where

| Host | Processes |
|---|---|
| Raspberry Pi 5 | bridge, safety, controller, scene, planner (ARCO, `Nice=10`), vision, Zenoh router |
| PC | MuJoCo twin, RViz2, PlotJuggler, development |

A Wi-Fi loss only cuts monitoring. It cannot leave a command latched on the
PC because the control path does not depend on the network (FR-HW-63).

## OS baseline

- Ubuntu Server 24.04 LTS arm64 for Raspberry Pi
- ROS 2 Jazzy `ros-base` from apt (plus `ros-jazzy-rmw-zenoh-cpp`)
- Python 3.12 venv with `--system-site-packages` (PEP 668; rclpy from apt)
- `pip install -e ".[hw]"` — no MuJoCo / `sim` extra on the Pi
- ARCO from the published aarch64 wheel (see `docs/decisions.md`, OPS-05)

Fallback (unsupported by scripts): Raspberry Pi OS + `ros:jazzy-ros-base`
container with `--device` passthrough and `--network host`.

## Headless first boot (FR-HW-74)

1. Flash Ubuntu Server 24.04 arm64 with **Raspberry Pi Imager**, or write a
   cloud-init `user-data` on the boot partition.
2. Set hostname (example `fret-pi`), user, SSH public key, Wi-Fi SSID/PSK.
3. Disable password login.
4. Boot the Pi; SSH in over Wi-Fi with no monitor or keyboard.
5. Run `scripts/deploy/rpi5_install.sh --tag vX.Y.Z` over SSH.

## Install / update / rollback

```bash
sudo bash scripts/deploy/rpi5_install.sh --tag v2.3.0-rc.1
sudo bash scripts/deploy/rpi5_update.sh v2.3.0-rc.1
sudo bash scripts/deploy/rpi5_rollback.sh
```

Layout: `/opt/fret/src`, `/opt/fret/releases/<tag>`, `/opt/fret/current`,
`/var/lib/fret/`, `/var/log/fret/` (tmpfs). The `current` symlink switches
only after build + smoke succeed (FR-HW-56).

## Power and cooling (FR-HW-62)

- Official **5 V / 5 A** USB-C supply for the Pi 5.
- Servos on their own **12 V** supply — never powered from the Pi.
- Official Raspberry Pi **Active Cooler** required (bridge + planner + vision
  all run on the Pi; passive cooling throttles under sustained load).

## microSD storage (FR-HW-72)

- Recommend an **A2** card, 64 GB or more.
- journald capped (`Storage=volatile` / `SystemMaxUse=` ≤ 100 MB).
- `/var/log/fret` and `ROS_LOG_DIR` on a 256 MB tmpfs; persist reports with
  `fret-diag --save` or `fret-diag export`.
- `noatime` on the root filesystem; **zram** swap (~2 GB); no swap file on
  the card.
- NVMe via an M.2 HAT is an optional upgrade, not required.

## Wi-Fi and RMW (FR-HW-63)

Recommended: `rmw_zenoh_cpp` with `fret-zenohd.service` on the Pi. On the PC:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42   # same as /etc/fret/ros.env
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<pi-hostname-or-ip>:7447"]'
```

Fallback: CycloneDDS with multicast disabled and unicast peers in
`/etc/fret/cyclonedds.xml` (from `deploy/dds/cyclonedds.xml.in`). Keep time
synchronized (NTP/chrony).

## systemd units (FR-HW-57, FR-HW-64)

- `fret-hardware.service` — starts with torque **disabled**; `Type=notify`
  + `WatchdogSec=`; bridge `sd_notify` keep-alive.
- `fret-planner.service` — `Nice=10`
- `fret-vision.service`
- `fret-zenohd.service`

Torque enable is always an explicit operator command (`~/enable`).

## udev (FR-HW-58)

```bash
sudo bash scripts/deploy/udev_capture.sh --follower /dev/ttyUSB0
sudo bash scripts/deploy/udev_capture.sh --camera /dev/video0
# --leader is reserved (leader arm out of scope)
```

Symlinks: `/dev/fret-servo-follower`, `/dev/fret-cam0` (optional
`/dev/fret-servo-leader`). ModemManager ignore is set in the rules.

## Performance budget

See the table in the PR5 spec / `src/fret/config/hardware/budget.yml`.
Measure with `fret-diag load` (FR-HW-73).

## Host check

```bash
fret-diag pi --timer-duration 60
```

## Arm64 container reproducibility (FR-HW-60)

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
bash scripts/deploy/build_arm64_container.sh
```

## Remote monitoring from the PC

With Zenoh connected, run RViz2 / PlotJuggler on the PC against the same
`ROS_DOMAIN_ID`. Do not start controllers on the PC while the Pi owns the arm.
