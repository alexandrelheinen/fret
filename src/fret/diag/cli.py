"""``fret-diag`` CLI (FR-HW-65, FR-HW-66)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

from fret.diag.core import (
    ALL_COMMANDS,
    MOTION_COMMANDS,
    READ_ONLY_ORDER,
    CheckResult,
    CheckStatus,
    exit_code_for,
)
from fret.diag.report import format_human, write_report


def _run_command(
    name: str,
    *,
    allow_motion: bool,
    confirm: bool,
    report_dir: Path | None,
    save: bool,
    timer_duration_s: float,
) -> list[CheckResult]:
    if name in MOTION_COMMANDS and not allow_motion:
        return [
            CheckResult(
                name,
                CheckStatus.FAIL,
                "refused without --allow-motion (FR-HW-66)",
            )
        ]
    if name in MOTION_COMMANDS and allow_motion and not confirm:
        return [
            CheckResult(
                name,
                CheckStatus.SKIP,
                "needs interactive confirmation (--yes to confirm in scripts)",
            )
        ]

    if name == "pi":
        from fret.diag.pi import run_pi_checks

        return run_pi_checks(timer_duration_s=timer_duration_s)
    if name == "usb":
        from fret.diag.usb import run_usb_checks

        return run_usb_checks()
    if name == "bus":
        from fret.diag.bus import run_bus_checks

        return run_bus_checks()
    if name == "servos":
        from fret.diag.servos import run_servos_checks

        return run_servos_checks()
    if name == "calibration":
        from fret.diag.calibration import run_calibration_checks

        return run_calibration_checks()
    if name == "estop":
        from fret.diag.estop import run_estop_checks

        return run_estop_checks()
    if name == "loop":
        from fret.diag.loop import run_loop_checks

        return run_loop_checks()
    if name == "load":
        from fret.diag.load import run_load_checks

        return run_load_checks()
    if name == "camera":
        from fret.diag.camera import run_camera_checks

        return run_camera_checks()
    if name == "vision":
        from fret.diag.vision import run_vision_checks

        return run_vision_checks()
    if name == "joints":
        from fret.diag.joints import run_joints_checks

        return run_joints_checks(allow_motion=allow_motion, confirmed=confirm)
    if name == "gripper":
        from fret.diag.gripper import run_gripper_checks

        return run_gripper_checks(allow_motion=allow_motion, confirmed=confirm)
    if name == "e2e":
        from fret.diag.e2e import run_e2e_checks

        return run_e2e_checks(allow_motion=allow_motion, confirmed=confirm)
    if name == "all":
        results: list[CheckResult] = []
        for cmd in READ_ONLY_ORDER:
            chunk = _run_command(
                cmd,
                allow_motion=False,
                confirm=False,
                report_dir=report_dir,
                save=save,
                timer_duration_s=timer_duration_s,
            )
            results.extend(chunk)
            if any(r.status is CheckStatus.FAIL for r in chunk):
                break
        return results
    return [CheckResult(name, CheckStatus.FAIL, f"unknown command {name!r}")]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fret-diag",
        description="Per-subsystem diagnostics for FRET on Raspberry Pi 5",
    )
    parser.add_argument(
        "command",
        choices=ALL_COMMANDS,
        help="subsystem check (all = read-only chain)",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="permit joints/gripper/e2e (FR-HW-66)",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="confirm motion without interactive prompt",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="copy JSON report to /var/lib/fret/diag",
    )
    parser.add_argument(
        "--report-dir",
        type=Path,
        default=None,
        help="override JSON report directory",
    )
    parser.add_argument(
        "--timer-duration",
        type=float,
        default=1.0,
        help="pi timer-loop duration seconds (use 60 on hardware)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    report_dir = args.report_dir
    if report_dir is None:
        # Prefer writable location when /var/log/fret is unavailable.
        default = Path("/var/log/fret/diag")
        try:
            default.mkdir(parents=True, exist_ok=True)
            report_dir = default
        except OSError:
            report_dir = Path.cwd() / ".fret-diag"
    results = _run_command(
        args.command,
        allow_motion=bool(args.allow_motion),
        confirm=bool(args.yes),
        report_dir=report_dir,
        save=bool(args.save),
        timer_duration_s=float(args.timer_duration),
    )
    print(format_human(results))
    path = write_report(
        args.command,
        results,
        report_dir=report_dir,
        save_persistent=bool(args.save),
    )
    print(f"report: {path}")
    return exit_code_for(results)


if __name__ == "__main__":
    sys.exit(main())
