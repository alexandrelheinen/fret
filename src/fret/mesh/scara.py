#!/usr/bin/env python3
"""Generate STL mesh files for the FRET SCARA robot model.

Geometry is read directly from scara.xacro so this script never goes out of
sync with the URDF — edit the .xacro, rebuild, and the meshes update too.

Usage:
    python3 src/fret/mesh/scara.py [--output-dir PATH]
"""

import argparse
import math
import os
import re
import struct
import xml.etree.ElementTree as ET

# Path to the source of truth for all geometry constants.
_HERE = os.path.dirname(os.path.abspath(__file__))
XACRO_PATH = os.path.normpath(os.path.join(_HERE, "..", "urdf", "scara.xacro"))

XACRO_NS = "http://www.ros.org/wiki/xacro"


# ---------------------------------------------------------------------------
# Xacro property parser (stdlib only — no xacro runtime dependency)
# ---------------------------------------------------------------------------


def read_geometry_from_xacro(xacro_path: str = XACRO_PATH) -> dict:
    """Parse scara.xacro and return a dict of evaluated numeric properties.

    Processes all ``xacro:property`` elements in document order, evaluating
    ``${expr}`` values in the context of previously resolved properties.
    Non-numeric and unresolvable properties are silently skipped.
    """
    tree = ET.parse(xacro_path)
    root = tree.getroot()

    props: dict = {"PI": math.pi}

    def _resolve(value: str) -> float:
        expanded = re.sub(
            r"\$\{([^}]+)\}",
            lambda m: str(
                eval(m.group(1), {"__builtins__": {}}, props)
            ),  # noqa: S307
            value,
        )
        return float(expanded)

    for elem in root.iter(f"{{{XACRO_NS}}}property"):
        name = elem.get("name")
        value = elem.get("value")
        if not name or not value:
            continue
        try:
            props[name] = _resolve(value)
        except Exception:
            pass  # skip non-numeric or forward-reference properties

    return props


# ---------------------------------------------------------------------------
# Binary STL helpers
# ---------------------------------------------------------------------------


def _normal(v0, v1, v2):
    """Compute outward unit normal for CCW triangle."""
    e1 = [v1[i] - v0[i] for i in range(3)]
    e2 = [v2[i] - v0[i] for i in range(3)]
    n = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ]
    mag = math.sqrt(sum(x * x for x in n))
    if mag < 1e-12:
        return [0.0, 0.0, 1.0]
    return [x / mag for x in n]


def write_stl_binary(triangles, path):
    """Write a list of (v0, v1, v2) triangles to a binary STL file.

    Each vertex is a 3-tuple of floats (x, y, z) in metres.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"\x00" * 80)  # header
        f.write(struct.pack("<I", len(triangles)))  # triangle count
        for v0, v1, v2 in triangles:
            n = _normal(v0, v1, v2)
            f.write(struct.pack("<3f", *n))
            f.write(struct.pack("<3f", *v0))
            f.write(struct.pack("<3f", *v1))
            f.write(struct.pack("<3f", *v2))
            f.write(struct.pack("<H", 0))  # attribute byte count


# ---------------------------------------------------------------------------
# Mesh: headpiece body (trapezoidal side profile)
# ---------------------------------------------------------------------------
#
# The Link-2 headpiece is modelled as three parts in scara.xacro:
#
#   1. Rear end-cap cylinder  (R = arm_radius, H = HEAD_REAR_HEIGHT)  — URDF primitive
#   2. This mesh              (trapezoidal body)
#   3. Front end-cap cylinder (R = arm_radius, H = HEAD_FRONT_HEIGHT) — URDF primitive
#
# Side-view profile (trapezoidal):
#
#   Hr ──────── Hr
#   │  flat top  │ \  slope
#   │            │   \ Hf
#   └────────────┴────┘
#   x=0         x=Lr  x=L2
#
# The flat-top rectangle spans x = [0, Lr] at height Hr.
# The slope spans x = [Lr, L2], going linearly from Hr down to Hf.
# Lr = ARM_1_LENGTH - FRONT_BOX_LEN  (same breakpoint as the old step).
#
# Mesh origin: x=0 (J2 axis), z=0 (arm-slab top face in arm_1_link frame).
# In scara.xacro: <origin xyz="0 0 ${link_2_height}"/>.
#
# 12 vertices (×2 for ±Y), 20 triangles.
#
#   idx   x     y      z
#    0    0    -W/2    0       rear-bottom-left
#    1    0    +W/2    0       rear-bottom-right
#    2    Lr   -W/2    0       mid-bottom-left
#    3    Lr   +W/2    0       mid-bottom-right
#    4    L2   -W/2    0       front-bottom-left
#    5    L2   +W/2    0       front-bottom-right
#    6    0    -W/2    Hr      rear-top-left
#    7    0    +W/2    Hr      rear-top-right
#    8    Lr   -W/2    Hr      mid-top-left   (slope starts here)
#    9    Lr   +W/2    Hr      mid-top-right
#   10    L2   -W/2    Hf      front-top-left
#   11    L2   +W/2    Hf      front-top-right


def make_headpiece_wedge(properties: dict):
    """Return list of (v0, v1, v2) triangles for the headpiece trapezoidal body."""
    L2 = properties["arm_1_length"]
    Lr = (
        L2 - properties["front_box_len"]
    )  # x where the flat top ends and slope begins
    W2 = properties["arm_diameter"] / 2.0  # half-width (±Y extent)
    Hr = properties["head_rear_height"]
    Hf = properties["head_front_height"]

    v = [
        [0, -W2, 0],  #  0
        [0, +W2, 0],  #  1
        [Lr, -W2, 0],  #  2
        [Lr, +W2, 0],  #  3
        [L2, -W2, 0],  #  4
        [L2, +W2, 0],  #  5
        [0, -W2, Hr],  #  6
        [0, +W2, Hr],  #  7
        [Lr, -W2, Hr],  #  8
        [Lr, +W2, Hr],  #  9
        [L2, -W2, Hf],  # 10
        [L2, +W2, Hf],  # 11
    ]

    # CCW winding when viewed from the outside face (outward normals).
    tris = [
        # Bottom (normal -Z) — two quads
        (v[0], v[1], v[2]),
        (v[1], v[3], v[2]),
        (v[2], v[3], v[4]),
        (v[3], v[5], v[4]),
        # Flat top (normal +Z)
        (v[6], v[8], v[7]),
        (v[7], v[8], v[9]),
        # Sloped top (normal ≈ +Z tilted toward +X)
        (v[8], v[10], v[9]),
        (v[9], v[10], v[11]),
        # Left side (normal -Y) — two quads
        (v[0], v[2], v[6]),
        (v[2], v[8], v[6]),
        (v[2], v[4], v[8]),
        (v[4], v[10], v[8]),
        # Right side (normal +Y) — two quads
        (v[1], v[7], v[3]),
        (v[3], v[7], v[9]),
        (v[3], v[9], v[5]),
        (v[5], v[9], v[11]),
        # Rear face (normal -X)
        (v[0], v[6], v[1]),
        (v[6], v[7], v[1]),
        # Front face (normal +X)
        (v[4], v[5], v[10]),
        (v[5], v[11], v[10]),
    ]
    return tris


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    default_out = os.path.join(os.getcwd(), "generated_meshes", "scara")

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--output-dir",
        default=default_out,
        help="Directory where STL files are written (default: src/fret/meshes)",
    )
    parser.add_argument(
        "--xacro",
        default=XACRO_PATH,
        help=f"Path to scara.xacro (default: {XACRO_PATH})",
    )
    args = parser.parse_args()

    props = read_geometry_from_xacro(args.xacro)

    out_dir = os.path.realpath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    path = os.path.join(out_dir, "headpiece_wedge.stl")
    tris = make_headpiece_wedge(props)
    write_stl_binary(tris, path)
    print(f"[mesh.py] Written {len(tris)} triangles → {path}")


if __name__ == "__main__":
    main()
