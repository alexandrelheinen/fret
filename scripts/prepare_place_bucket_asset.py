#!/usr/bin/env python3
"""Build a bottom-centered place-bucket OBJ from the AWS warehouse bucket.

Source: ``src/fret/mjcf/assets/aws_warehouse/meshes/bucket.obj`` (MIT-0).
Output: ``src/fret/mjcf/assets/place_bucket.obj`` with origin at the bottom
centre of the axis-aligned bounding box so MJCF ``pos`` is the place XY.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
_SRC = _REPO / "src/fret/mjcf/assets/aws_warehouse/meshes/bucket.obj"
_OUT = _REPO / "src/fret/mjcf/assets/place_bucket.obj"


def _load_obj(path: Path) -> tuple[list[np.ndarray], list[str]]:
    verts: list[np.ndarray] = []
    other: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            parts = line.split()
            verts.append(np.array([float(parts[1]), float(parts[2]), float(parts[3])]))
        else:
            other.append(line)
    if not verts:
        raise ValueError(f"No vertices in {path}")
    return verts, other


def prepare(src: Path = _SRC, out: Path = _OUT) -> Path:
    verts, other = _load_obj(src)
    arr = np.stack(verts, axis=0)
    mins = arr.min(axis=0)
    maxs = arr.max(axis=0)
    center_xy = 0.5 * (mins[:2] + maxs[:2])
    arr[:, 0] -= center_xy[0]
    arr[:, 1] -= center_xy[1]
    arr[:, 2] -= mins[2]
    size = arr.max(axis=0) - arr.min(axis=0)
    header = (
        "# Place bucket visual for FRET manipulator pick-and-place cells.\n"
        "# Derived from AWS RoboMaker Small Warehouse Bucket_01 (MIT-0).\n"
        f"# Native extents after centering (m): "
        f"{size[0]:.6f} x {size[1]:.6f} x {size[2]:.6f}\n"
        "# Origin: bottom centre (z=0 on floor plane).\n"
    )
    lines = [header.rstrip()]
    for v in arr:
        lines.append(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}")
    for line in other:
        if line.startswith("#") and "Blender" in line:
            continue
        lines.append(line)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src", type=Path, default=_SRC)
    parser.add_argument("--out", type=Path, default=_OUT)
    args = parser.parse_args()
    path = prepare(args.src, args.out)
    print(f"Wrote {path}")


if __name__ == "__main__":
    main()
