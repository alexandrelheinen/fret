#!/usr/bin/env python3
"""Generate Dubins race MJCF structure geoms from obstacle YAML.

Keeps ``dubins_race_obstacles.yml`` as the planning source of truth and
writes collision boxes plus **per-instance scaled** AWS RoboMaker mesh
assets into ``dubins_race.xml``.

MuJoCo ignores ``geom size`` on meshes, so each structure gets its own
``<mesh … scale="…">`` entry (otherwise native ~2 m clutter piles into a blob).

Example::

    python3 scripts/generate_dubins_race_mjcf.py
    python3 scripts/generate_dubins_race_mjcf.py --dry-run
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_YAML = (
    _REPO_ROOT / "src/fret/config/worlds/dubins_race_obstacles.yml"
)
_DEFAULT_MJCF = _REPO_ROOT / "src/fret/mjcf/dubins_race.xml"

_MESH_FILE: dict[str, str] = {
    "shelf": "assets/aws_warehouse/meshes/shelf.obj",
    "shelf_e": "assets/aws_warehouse/meshes/shelf_e.obj",
    "clutter_a": "assets/aws_warehouse/meshes/clutter_a.obj",
    "clutter_c": "assets/aws_warehouse/meshes/clutter_c.obj",
    "clutter_d": "assets/aws_warehouse/meshes/clutter_d.obj",
    "bucket": "assets/aws_warehouse/meshes/bucket.obj",
    "desk": "assets/aws_warehouse/meshes/desk.obj",
    "lamp": "assets/aws_warehouse/meshes/lamp.obj",
    "pallet": "assets/aws_warehouse/meshes/pallet.obj",
    "trash": "assets/aws_warehouse/meshes/trash.obj",
}

_MESH_EXTENTS: dict[str, tuple[float, float, float]] = {
    "shelf": (3.917, 0.880, 2.613),
    "shelf_e": (3.917, 0.880, 2.613),
    "clutter_a": (2.717, 2.51, 1.339),
    "clutter_c": (2.167, 1.582, 1.8),
    "clutter_d": (1.831, 1.034, 1.778),
    "bucket": (0.941, 1.218, 1.404),
    "desk": (0.882, 1.555, 1.437),
    "lamp": (1.023, 1.008, 1.514),
    "pallet": (1.152, 0.532, 0.964),
    "trash": (1.446, 0.892, 1.268),
}

_VISUAL_MATERIAL: dict[str, str] = {
    "shelf": "shelf",
    "shelf_e": "shelf",
    "clutter_a": "clutter_cardboard",
    "clutter_c": "clutter_crate",
    "clutter_d": "clutter_pallet",
    "bucket": "clutter_cardboard",
    "desk": "clutter_crate",
    "lamp": "tb3_hardware",
    "pallet": "clutter_pallet",
    "trash": "clutter_cardboard",
}

_STRUCTURES_BEGIN = "    <!-- Warehouse structure block (auto-generated) -->"
_STRUCTURES_END = "    <!-- End warehouse structure block -->"
_MESH_ASSETS_BEGIN = "    <!-- Warehouse structure meshes (auto-generated) -->"
_MESH_ASSETS_END = "    <!-- End warehouse structure meshes -->"


def _load_mesh_extents() -> None:
    """Refresh extents from on-disk OBJ bounds when available."""
    try:
        import trimesh
    except ImportError:
        return
    meshes_dir = _REPO_ROOT / "src/fret/mjcf/assets/aws_warehouse/meshes"
    for name in list(_MESH_EXTENTS):
        path = meshes_dir / f"{name}.obj"
        if not path.is_file():
            continue
        mesh = trimesh.load(path, force="mesh")
        extents = tuple(float(v) for v in mesh.extents)
        _MESH_EXTENTS[name] = (extents[0], extents[1], extents[2])


def _mesh_scale(
    mesh: str, hx: float, hy: float, height: float
) -> tuple[float, float, float]:
    ex, ey, ez = _MESH_EXTENTS[mesh]
    return (2.0 * hx / ex, 2.0 * hy / ey, height / max(ez, 1e-6))


def _load_structures(yaml_path: Path) -> list[dict[str, object]]:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    structures = list(data.get("structures", []))
    for alcove in data.get("dead_ends", []):
        structures.extend(alcove.get("parts", []))
    return structures


def _visual_for(spec: dict[str, object]) -> str:
    raw = str(spec.get("visual", "")).strip()
    if raw in _MESH_FILE:
        return raw
    height = float(spec.get("height", 1.0))
    if height <= 0.7:
        return "bucket"
    if float(spec.get("hx", 0.0)) >= 0.4 or float(spec.get("hy", 0.0)) >= 0.4:
        return "shelf"
    return "clutter_c"


def generate_mesh_assets(structures: list[dict[str, object]]) -> str:
    """Return ``<mesh>`` assets with per-structure non-uniform scale."""
    _load_mesh_extents()
    lines: list[str] = [_MESH_ASSETS_BEGIN]
    for idx, spec in enumerate(structures):
        mesh = _visual_for(spec)
        sx, sy, sz = _mesh_scale(
            mesh,
            float(spec["hx"]),
            float(spec["hy"]),
            float(spec["height"]),
        )
        lines.append(
            f'    <mesh name="str_{idx:03d}_mesh" file="{_MESH_FILE[mesh]}" '
            f'scale="{sx:.6f} {sy:.6f} {sz:.6f}"/>'
        )
    lines.append(_MESH_ASSETS_END)
    return "\n".join(lines)


def generate_structure_geoms(structures: list[dict[str, object]]) -> str:
    """Return MJCF geoms for analytic collision + AWS mesh visuals."""
    lines: list[str] = [_STRUCTURES_BEGIN]
    for idx, spec in enumerate(structures):
        x = float(spec["x"])
        y = float(spec["y"])
        hx = float(spec["hx"])
        hy = float(spec["hy"])
        height = float(spec["height"])
        half_z = height / 2.0
        mesh = _visual_for(spec)
        material = _VISUAL_MATERIAL[mesh]
        name = f"str_{idx:03d}"
        lines.append(
            f'    <geom name="{name}_col" class="structure_col" '
            f'pos="{x} {y} {half_z:.2f}" size="{hx} {hy} {half_z:.2f}"/>'
        )
        corner_x = x - hx
        corner_y = y - hy
        lines.append(
            f'    <geom name="{name}_vis" type="mesh" mesh="{name}_mesh" '
            f'pos="{corner_x:.3f} {corner_y:.3f} 0" material="{material}" '
            f'contype="0" conaffinity="0"/>'
        )
    lines.append(_STRUCTURES_END)
    return "\n".join(lines)


def _patch_block(text: str, begin: str, end: str, block: str) -> str:
    pattern = re.compile(
        re.escape(begin) + r".*?" + re.escape(end),
        re.DOTALL,
    )
    if not pattern.search(text):
        raise ValueError(
            f"Block markers not found ({begin!r} … {end!r}). "
            "Add the begin/end comments before running this script."
        )
    return pattern.sub(block, text)


def patch_mjcf(
    mjcf_path: Path,
    structure_block: str,
    mesh_assets_block: str,
) -> str:
    """Replace auto-generated mesh assets and structure geoms."""
    text = mjcf_path.read_text(encoding="utf-8")
    text = _patch_block(
        text, _MESH_ASSETS_BEGIN, _MESH_ASSETS_END, mesh_assets_block
    )
    text = _patch_block(
        text, _STRUCTURES_BEGIN, _STRUCTURES_END, structure_block
    )
    return text


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaml", type=Path, default=_DEFAULT_YAML)
    parser.add_argument("--mjcf", type=Path, default=_DEFAULT_MJCF)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print generated block without writing MJCF",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    structures = _load_structures(args.yaml.resolve())
    mesh_assets = generate_mesh_assets(structures)
    geoms = generate_structure_geoms(structures)
    if args.dry_run:
        print(mesh_assets)
        print(geoms)
        print(f"# structures: {len(structures)}", file=sys.stderr)
        return 0

    updated = patch_mjcf(args.mjcf.resolve(), geoms, mesh_assets)
    args.mjcf.write_text(updated, encoding="utf-8")
    print(f"Wrote {len(structures)} structures into {args.mjcf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
