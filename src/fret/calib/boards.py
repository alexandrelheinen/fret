"""Printable calibration board specifications (mm)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CheckerboardSpec:
    """OpenCV checkerboard: inner corner counts and square size."""

    cols: int = 9
    rows: int = 6
    square_mm: float = 25.0

    def describe(self) -> str:
        return (
            f"checkerboard {self.cols}x{self.rows} inner corners, "
            f"{self.square_mm:.1f} mm squares"
        )


@dataclass(frozen=True)
class CharucoSpec:
    """ChArUco board for extrinsics (table-plane)."""

    squares_x: int = 5
    squares_y: int = 7
    square_mm: float = 30.0
    marker_mm: float = 22.0
    dictionary: str = "DICT_4X4_50"

    def describe(self) -> str:
        return (
            f"ChArUco {self.squares_x}x{self.squares_y}, "
            f"square={self.square_mm:.1f} mm, marker={self.marker_mm:.1f} mm, "
            f"{self.dictionary}"
        )


def print_board_specs() -> str:
    checker = CheckerboardSpec()
    charuco = CharucoSpec()
    return (
        "# FRET printable board specs (US letter / A4)\n"
        f"- Intrinsics: {checker.describe()}\n"
        f"- Extrinsics: {charuco.describe()}\n"
        "- Place the ChArUco origin at the table mark used by SC-v23a.\n"
    )
