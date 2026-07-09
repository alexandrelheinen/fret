#!/usr/bin/env python3
"""Import MIT-0 AWS RoboMaker warehouse meshes for FRET MuJoCo scenes.

Converts selected upstream DAE visuals to OBJ (meters, bottom-corner origin)
and copies floor/wall textures into ``src/fret/mjcf/assets/aws_warehouse/``.

Source: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

Example::

    python3 scripts/import_aws_warehouse_assets.py
    python3 scripts/import_aws_warehouse_assets.py --src /path/to/aws-robomaker-small-warehouse-world
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import trimesh

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_OUT = _REPO_ROOT / "src/fret/mjcf/assets/aws_warehouse"

# DAE unit quirks: shelf is centimeters; cluttering models are already meters.
_MODELS: tuple[tuple[str, str, float], ...] = (
    ("aws_robomaker_warehouse_ShelfD_01", "shelf.obj", 0.01),
    ("aws_robomaker_warehouse_ClutteringA_01", "clutter_a.obj", 1.0),
    ("aws_robomaker_warehouse_ClutteringC_01", "clutter_c.obj", 1.0),
    ("aws_robomaker_warehouse_ClutteringD_01", "clutter_d.obj", 1.0),
)

_TEXTURES: tuple[tuple[str, str, str], ...] = (
    (
        "aws_robomaker_warehouse_GroundB_01",
        "aws_robomaker_warehouse_GroundB_01.png",
        "ground.png",
    ),
    (
        "aws_robomaker_warehouse_WallB_01",
        "aws_robomaker_warehouse_WallB_01.png",
        "wall.png",
    ),
)


def _export_visual(
    aws_models: Path, model_name: str, out_path: Path, unit_scale: float
) -> tuple[float, float, float]:
    """Export one visual DAE as OBJ with bottom-corner at the origin."""
    dae = aws_models / model_name / "meshes" / f"{model_name}_visual.DAE"
    if not dae.is_file():
        raise FileNotFoundError(f"Missing visual mesh: {dae}")

    scene = trimesh.load(dae)
    meshes = []
    for geom in scene.geometry.values():
        mesh = geom.copy()
        mesh.apply_scale(unit_scale)
        meshes.append(mesh)

    combined = trimesh.util.concatenate(meshes)
    combined.vertices -= combined.bounds[0]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    combined.export(out_path)
    extents = combined.extents
    return float(extents[0]), float(extents[1]), float(extents[2])


def import_assets(src_root: Path, out_dir: Path) -> None:
    """Convert AWS models and textures into the FRET MJCF asset tree."""
    aws_models = src_root / "models"
    if not aws_models.is_dir():
        raise FileNotFoundError(
            f"Expected AWS models directory at {aws_models}. "
            "Clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world"
        )

    meshes_dir = out_dir / "meshes"
    textures_dir = out_dir / "textures"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    textures_dir.mkdir(parents=True, exist_ok=True)

    print(f"Writing meshes to {meshes_dir}")
    for model_name, out_name, unit_scale in _MODELS:
        extents = _export_visual(aws_models, model_name, meshes_dir / out_name, unit_scale)
        print(f"  {out_name}: extents={extents[0]:.3f} x {extents[1]:.3f} x {extents[2]:.3f} m")

    print(f"Writing textures to {textures_dir}")
    for model_name, src_name, dst_name in _TEXTURES:
        src_tex = aws_models / model_name / "materials" / "textures" / src_name
        if not src_tex.is_file():
            raise FileNotFoundError(f"Missing texture: {src_tex}")
        shutil.copy2(src_tex, textures_dir / dst_name)
        print(f"  {dst_name}")

    license_src = src_root / "LICENSE"
    if license_src.is_file():
        shutil.copy2(license_src, out_dir / "LICENSE")


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--src",
        type=Path,
        default=None,
        help="Path to aws-robomaker-small-warehouse-world clone",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=_DEFAULT_OUT,
        help="Output directory for converted assets",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    src = args.src
    if src is None:
        tmp = Path("/tmp/aws-robomaker-small-warehouse-world")
        if not tmp.is_dir():
            print(
                "Cloning aws-robomaker-small-warehouse-world into /tmp ...",
                file=sys.stderr,
            )
            import subprocess

            subprocess.run(
                [
                    "git",
                    "clone",
                    "--depth",
                    "1",
                    "https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git",
                    str(tmp),
                ],
                check=True,
            )
        src = tmp

    import_assets(src.resolve(), args.out.resolve())
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
