"""``fret-calib`` entry point."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

import numpy as np
import numpy.typing as npt
import yaml

from fret.calib.base_check import evaluate_base_check
from fret.calib.boards import print_board_specs
from fret.calib.extrinsics import estimate_extrinsics_from_pose
from fret.calib.intrinsics import (
    calibrate_intrinsics_from_corners,
    synthesize_checkerboard_views,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="fret-calib")
    sub = parser.add_subparsers(dest="command", required=True)

    intr = sub.add_parser("intrinsics", help="checkerboard intrinsics")
    intr.add_argument("--synthetic", action="store_true")
    intr.add_argument("--output", type=Path, required=True)
    intr.add_argument("--camera-id", default="cam0")
    intr.add_argument("--min-views", type=int, default=15)

    extr = sub.add_parser("extrinsics", help="ChArUco extrinsics")
    extr.add_argument("--synthetic", action="store_true")
    extr.add_argument("--output", type=Path, required=True)
    extr.add_argument("--camera-id", default="cam0")

    base = sub.add_parser("base-check", help="FK tip vs board points")
    base.add_argument("--tips", type=Path, required=True)
    base.add_argument("--board-points", type=Path, required=True)
    base.add_argument("--threshold-m", type=float, default=0.010)

    boards = sub.add_parser("boards", help="printable board specs")
    boards.add_argument("--print", action="store_true")
    return parser


def _write_vision_snippet(
    path: Path,
    *,
    camera_id: str,
    camera_matrix: npt.NDArray[np.float64] | None = None,
    dist: npt.NDArray[np.float64] | None = None,
    t_world_cam: npt.NDArray[np.float64] | None = None,
) -> None:
    payload: dict[str, object] = {"camera_id": camera_id}
    if camera_matrix is not None:
        payload["camera_matrix"] = camera_matrix.tolist()
    if dist is not None:
        payload["dist_coeffs"] = dist.tolist()
    if t_world_cam is not None:
        payload["t_world_cam"] = t_world_cam.tolist()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    if args.command == "boards":
        print(print_board_specs())
        return 0
    if args.command == "intrinsics":
        if not args.synthetic:
            print(
                "live capture not implemented in this environment; "
                "use --synthetic for offline recovery tests",
                file=sys.stderr,
            )
            return 2
        objs, imgs, size = synthesize_checkerboard_views(
            views=max(args.min_views, 16)
        )
        intr = calibrate_intrinsics_from_corners(
            objs, imgs, size, min_views=args.min_views
        )
        status = (
            "pass"
            if intr.rms_px <= 0.5
            else ("warn" if intr.rms_px <= 1.0 else "fail")
        )
        print(
            json.dumps(
                {
                    "rms_px": intr.rms_px,
                    "views": intr.views_accepted,
                    "status": status,
                }
            )
        )
        _write_vision_snippet(
            args.output,
            camera_id=args.camera_id,
            camera_matrix=intr.camera_matrix,
            dist=intr.dist_coeffs,
        )
        return 0 if intr.rms_px <= 1.0 else 1
    if args.command == "extrinsics":
        if not args.synthetic:
            print("use --synthetic without a live board", file=sys.stderr)
            return 2
        extr = estimate_extrinsics_from_pose(
            np.zeros(3),
            np.array([0.0, 0.0, 0.5]),
            reprojection_error_px=0.2,
        )
        print(
            json.dumps({"reprojection_error_px": extr.reprojection_error_px})
        )
        _write_vision_snippet(
            args.output,
            camera_id=args.camera_id,
            t_world_cam=extr.t_world_cam,
        )
        return 0
    if args.command == "base-check":
        tips = yaml.safe_load(args.tips.read_text(encoding="utf-8"))
        boards = yaml.safe_load(args.board_points.read_text(encoding="utf-8"))
        base = evaluate_base_check(
            tips, boards, pass_threshold_m=args.threshold_m
        )
        print(
            json.dumps(
                {
                    "errors_m": list(base.errors_m),
                    "max_error_m": base.max_error_m,
                    "passed": base.passed,
                }
            )
        )
        return 0 if base.passed else 1
    return 2


if __name__ == "__main__":
    sys.exit(main())
