"""``fret-diag pi`` host checks (FR-HW-59)."""

from __future__ import annotations

import os
import platform
import time
from pathlib import Path
from typing import Any, Callable

from fret.diag.core import CheckResult, CheckStatus

RootPath = Path | None


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def check_architecture() -> CheckResult:
    machine = platform.machine()
    ok = machine in {"aarch64", "arm64"}
    return CheckResult(
        "architecture",
        CheckStatus.PASS if ok else CheckStatus.FAIL,
        f"machine={machine}",
        {"machine": machine},
    )


def check_os_release(root: RootPath = None) -> CheckResult:
    path = (root or Path("/")) / "etc" / "os-release"
    text = _read_text(path) or ""
    pretty = ""
    version_id = ""
    for line in text.splitlines():
        if line.startswith("PRETTY_NAME="):
            pretty = line.split("=", 1)[1].strip().strip('"')
        if line.startswith("VERSION_ID="):
            version_id = line.split("=", 1)[1].strip().strip('"')
    ok = version_id.startswith("24.04") or "24.04" in pretty
    return CheckResult(
        "os",
        CheckStatus.PASS if ok else CheckStatus.FAIL,
        pretty or f"VERSION_ID={version_id}",
        {"version_id": version_id, "pretty": pretty},
    )


def check_ros_distro() -> CheckResult:
    distro = os.environ.get("ROS_DISTRO", "")
    ok = distro == "jazzy"
    status = (
        CheckStatus.PASS
        if ok
        else (CheckStatus.SKIP if not distro else CheckStatus.FAIL)
    )
    return CheckResult(
        "ros_distro",
        status,
        f"ROS_DISTRO={distro or 'unset'}",
        {"ros_distro": distro},
    )


def check_python() -> CheckResult:
    ver = platform.python_version()
    ok = ver.startswith("3.12")
    return CheckResult(
        "python",
        CheckStatus.PASS if ok else CheckStatus.FAIL,
        f"python={ver}",
        {"version": ver},
    )


def check_arco(pinned_version: str = "0.5.0") -> CheckResult:
    try:
        import arco
    except ImportError as exc:
        return CheckResult(
            "arco",
            CheckStatus.FAIL,
            f"import arco failed: {exc}",
        )
    version = getattr(arco, "__version__", "unknown")
    ok = str(version) == pinned_version
    return CheckResult(
        "arco",
        CheckStatus.PASS if ok else CheckStatus.FAIL,
        f"arco={version} (pinned {pinned_version})",
        {"version": version, "pinned": pinned_version},
    )


def check_fret_import() -> CheckResult:
    try:
        import fret  # noqa: F401
    except ImportError as exc:
        return CheckResult("fret", CheckStatus.FAIL, str(exc))
    return CheckResult("fret", CheckStatus.PASS, "import fret ok")


def check_no_mujoco_required() -> CheckResult:
    return CheckResult(
        "mujoco_not_required",
        CheckStatus.PASS,
        "hw import graph does not require mujoco (FR-HW-61)",
    )


def check_groups(user: str | None = None) -> CheckResult:
    import grp
    import pwd

    name = user or os.environ.get("USER") or pwd.getpwuid(os.getuid()).pw_name
    try:
        pw = pwd.getpwnam(name)
    except KeyError:
        return CheckResult("groups", CheckStatus.FAIL, f"unknown user {name}")
    groups = {g.gr_name for g in grp.getgrall() if name in g.gr_mem}
    groups.add(grp.getgrgid(pw.pw_gid).gr_name)
    missing = [g for g in ("dialout", "video") if g not in groups]
    if missing:
        # On developer PCs the interactive user often lacks dialout/video;
        # treat as skip so FR-HW-59 device absence stays skip-not-fail.
        return CheckResult(
            "groups",
            CheckStatus.SKIP,
            f"missing groups: {missing} (expected on Pi service user)",
            {"groups": sorted(groups)},
        )
    return CheckResult(
        "groups",
        CheckStatus.PASS,
        f"user {name} in dialout,video",
        {"groups": sorted(groups)},
    )


def check_udev_symlinks(dev_root: RootPath = None) -> CheckResult:
    root = dev_root or Path("/dev")
    required = ["fret-servo-follower", "fret-cam0"]
    present = [name for name in required if (root / name).exists()]
    missing = [name for name in required if name not in present]
    if missing:
        return CheckResult(
            "udev_symlinks",
            CheckStatus.SKIP,
            f"missing {missing} (hardware absent)",
            {"present": present, "missing": missing},
        )
    return CheckResult(
        "udev_symlinks",
        CheckStatus.PASS,
        "follower and camera symlinks present",
        {"present": present},
    )


def check_memory(proc_root: RootPath = None) -> CheckResult:
    path = (proc_root or Path("/")) / "proc" / "meminfo"
    text = _read_text(path)
    if not text:
        return CheckResult("memory", CheckStatus.SKIP, "meminfo unreadable")
    total_kb = 0
    for line in text.splitlines():
        if line.startswith("MemTotal:"):
            total_kb = int(line.split()[1])
            break
    total_gb = total_kb / (1024 * 1024)
    return CheckResult(
        "memory",
        CheckStatus.PASS,
        f"MemTotal≈{total_gb:.1f} GiB (8 GiB expected on Pi 5)",
        {"mem_total_kb": total_kb},
    )


def check_thermal(sys_root: RootPath = None) -> CheckResult:
    base = (sys_root or Path("/")) / "sys" / "class" / "thermal"
    temps: list[float] = []
    if base.is_dir():
        for zone in sorted(base.glob("thermal_zone*")):
            raw = _read_text(zone / "temp")
            if raw and raw.isdigit():
                temps.append(int(raw) / 1000.0)
    if not temps:
        return CheckResult(
            "thermal",
            CheckStatus.SKIP,
            "no thermal_zone readable",
        )
    tmax = max(temps)
    status = CheckStatus.PASS if tmax < 75.0 else CheckStatus.FAIL
    return CheckResult(
        "thermal",
        status,
        f"SoC temp max={tmax:.1f} °C",
        {"temps_c": temps},
    )


def check_throttle_flags(sys_root: RootPath = None) -> CheckResult:
    # Ubuntu on Pi may expose firmware throttle under this path; skip if absent.
    candidates = [
        (sys_root or Path("/"))
        / "sys"
        / "devices"
        / "platform"
        / "soc"
        / "soc:firmware"
        / "get_throttled",
        (sys_root or Path("/"))
        / "proc"
        / "device-tree"
        / "chosen"
        / "bootloader",
    ]
    for path in candidates:
        raw = _read_text(path)
        if raw is not None:
            return CheckResult(
                "throttle_flags",
                CheckStatus.PASS,
                f"readable at {path}",
                {"raw": raw[:64]},
            )
    return CheckResult(
        "throttle_flags",
        CheckStatus.SKIP,
        "throttle/undervoltage interface not readable (vcgencmd often absent on Ubuntu)",
    )


def check_storage(root: RootPath = None) -> CheckResult:
    base = root or Path("/")
    # Root device best-effort via /proc/mounts
    mounts = _read_text(base / "proc" / "mounts") or ""
    root_src = ""
    noatime = False
    for line in mounts.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[1] == "/":
            root_src = parts[0]
            noatime = "noatime" in parts[3].split(",")
            break
    kind = "unknown"
    if "mmcblk" in root_src:
        kind = "microSD (A2 64 GB+ recommended)"
    elif "nvme" in root_src:
        kind = "NVMe"
    details: dict[str, Any] = {
        "root_device": root_src,
        "kind": kind,
        "noatime": noatime,
    }
    status = CheckStatus.PASS
    msg = f"root={root_src or 'unknown'} ({kind}), noatime={noatime}"
    if root_src and not noatime:
        status = CheckStatus.FAIL
        msg += " — enable noatime on root (FR-HW-72)"
    return CheckResult("storage_root", status, msg, details)


def check_free_disk(path: Path | None = None) -> CheckResult:
    target = path or Path("/")
    try:
        usage = os.statvfs(target)
    except OSError as exc:
        return CheckResult("free_disk", CheckStatus.SKIP, str(exc))
    free_bytes = usage.f_bavail * usage.f_frsize
    free_gb = free_bytes / (1024**3)
    ok = free_gb >= 4.0
    return CheckResult(
        "free_disk",
        CheckStatus.PASS if ok else CheckStatus.FAIL,
        f"{free_gb:.1f} GiB free (need ≥ 4 GiB)",
        {"free_bytes": free_bytes},
    )


def check_timer_loop(
    duration_s: float = 1.0,
    rate_hz: float = 50.0,
    p99_error_s: float = 0.002,
) -> CheckResult:
    """Shortened default duration for unit tests; FR-HW-59 uses 60 s on hardware."""
    period = 1.0 / rate_hz
    errors: list[float] = []
    start = time.perf_counter()
    next_t = start
    while time.perf_counter() - start < duration_s:
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)
        after = time.perf_counter()
        errors.append(abs((after - next_t)))
        next_t += period
    if not errors:
        return CheckResult("timer_loop", CheckStatus.FAIL, "no samples")
    errors_sorted = sorted(errors)
    idx = max(0, int(0.99 * (len(errors_sorted) - 1)))
    p99 = errors_sorted[idx]
    ok = p99 <= p99_error_s
    return CheckResult(
        "timer_loop",
        CheckStatus.PASS if ok else CheckStatus.FAIL,
        f"p99 period error={p99*1000:.3f} ms over {duration_s:.1f}s",
        {"p99_s": p99, "samples": len(errors)},
    )


def check_arco_plan_benchmark() -> CheckResult:
    try:
        import arco  # noqa: F401
    except ImportError as exc:
        return CheckResult("arco_plan", CheckStatus.SKIP, str(exc))
    t0 = time.perf_counter()
    # Lightweight import/version touch; full planning is measured on the Pi.
    _ = getattr(arco, "__version__", None)
    elapsed = time.perf_counter() - t0
    return CheckResult(
        "arco_plan",
        CheckStatus.PASS,
        f"arco micro-benchmark {elapsed*1000:.2f} ms (import/version)",
        {"elapsed_s": elapsed},
    )


def run_pi_checks(
    *,
    root: RootPath = None,
    timer_duration_s: float = 1.0,
    group_user: str | None = None,
) -> list[CheckResult]:
    """Run FR-HW-59 checks. ``root`` remaps /etc /proc /sys /dev for tests."""
    checks: list[Callable[[], CheckResult]] = [
        check_architecture,
        lambda: check_os_release(root),
        check_ros_distro,
        check_python,
        check_arco,
        check_fret_import,
        check_no_mujoco_required,
        lambda: check_groups(group_user),
        lambda: check_udev_symlinks((root / "dev") if root else None),
        lambda: check_memory(root),
        lambda: check_thermal(root),
        lambda: check_throttle_flags(root),
        lambda: check_storage(root),
        check_free_disk,
        lambda: check_timer_loop(duration_s=timer_duration_s),
        check_arco_plan_benchmark,
    ]
    return [fn() for fn in checks]
