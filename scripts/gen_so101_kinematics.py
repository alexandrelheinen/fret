#!/usr/bin/env python3
"""Re-derive the SO-101 kinematic-chain constants from the vendored MJCF.

``fret.control.kinematics_so101`` hardcodes the fixed per-body transforms
(translation + orientation quaternion) and the joint order of the SO-101
arm chain as Python constants, rather than parsing the MJCF at import time
or depending on ``mujoco`` (FR-SIM-16: the Pi runtime has no MuJoCo). This
script is the single source of truth those constants are derived from: it
walks the vendored ``third_party/robotstudio_so101/so101.xml`` body tree
and prints (or checks) the same data structure.

Usage::

    python3 scripts/gen_so101_kinematics.py            # print derived chain
    python3 scripts/gen_so101_kinematics.py --check     # verify kinematics_so101.py matches
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
_DEFAULT_MJCF = _REPO_ROOT / "third_party" / "robotstudio_so101" / "so101.xml"

# Arm joints that move the gripper tip (the 6th joint, "gripper", operates
# the moving jaw only and does not move the "gripperframe" site).
JOINT_ORDER: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
_EE_SITE_NAME = "gripperframe"
_EE_SITE_BODY = "gripper"
Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(frozen=True)
class JointFrame:
    """Fixed body transform (parent -> this body) plus the joint it carries."""

    name: str
    pos: Vec3
    quat: Quat
    joint_range: tuple[float, float]


@dataclass(frozen=True)
class SiteFrame:
    """Fixed site transform relative to its parent body."""

    pos: Vec3
    quat: Quat


def _parse_floats(text: str | None, n: int) -> tuple[float, ...]:
    if text is None:
        raise ValueError("missing required MJCF attribute")
    values = tuple(float(x) for x in text.split())
    if len(values) != n:
        raise ValueError(f"expected {n} floats, got {values!r}")
    return values


def parse_so101_chain(
    xml_path: str | pathlib.Path = _DEFAULT_MJCF,
) -> tuple[list[JointFrame], SiteFrame]:
    """Walk the vendored MJCF body tree and return the SO-101 arm chain.

    Returns:
        ``(chain, ee_site)`` where ``chain`` holds one :class:`JointFrame`
        per entry of :data:`JOINT_ORDER`, in that order, and ``ee_site`` is
        the fixed ``gripperframe`` site transform relative to the
        ``gripper`` body.

    Raises:
        ValueError: If a joint's axis is not ``(0, 0, 1)`` (the hardcoded
            ``Rz`` chain in ``kinematics_so101.py`` assumes this), or if a
            joint from :data:`JOINT_ORDER` or the end-effector site is
            missing.
    """
    root = ET.parse(str(xml_path)).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"no <worldbody> in {xml_path}")

    found: dict[str, JointFrame] = {}
    ee_site: SiteFrame | None = None

    def walk(body_el: ET.Element) -> None:
        nonlocal ee_site
        pos = _parse_floats(body_el.get("pos", "0 0 0"), 3)
        quat = _parse_floats(body_el.get("quat", "1 0 0 0"), 4)
        joint_el = body_el.find("joint")
        if joint_el is not None:
            name = joint_el.get("name")
            if name in JOINT_ORDER:
                axis = _parse_floats(joint_el.get("axis", "0 0 1"), 3)
                if axis != (0.0, 0.0, 1.0):
                    raise ValueError(
                        f"joint {name!r} has axis {axis!r}, expected "
                        "(0, 0, 1) (required by the hardcoded Rz chain)"
                    )
                rng = _parse_floats(joint_el.get("range", "0 0"), 2)
                found[name] = JointFrame(
                    name=name,
                    pos=pos,  # type: ignore[arg-type]
                    quat=quat,  # type: ignore[arg-type]
                    joint_range=rng,
                )
        if body_el.get("name") == _EE_SITE_BODY:
            for site_el in body_el.findall("site"):
                if site_el.get("name") == _EE_SITE_NAME:
                    ee_site = SiteFrame(
                        pos=_parse_floats(  # type: ignore[arg-type]
                            site_el.get("pos", "0 0 0"), 3
                        ),
                        quat=_parse_floats(  # type: ignore[arg-type]
                            site_el.get("quat", "1 0 0 0"), 4
                        ),
                    )
        for child in body_el.findall("body"):
            walk(child)

    for top_body in worldbody.findall("body"):
        walk(top_body)

    missing = [name for name in JOINT_ORDER if name not in found]
    if missing:
        raise ValueError(f"MJCF missing expected joint(s): {missing}")
    if ee_site is None:
        raise ValueError(
            f"MJCF missing site {_EE_SITE_NAME!r} on body {_EE_SITE_BODY!r}"
        )
    return [found[name] for name in JOINT_ORDER], ee_site


def _format_chain_source(chain: list[JointFrame], ee_site: SiteFrame) -> str:
    lines = ["_CHAIN: tuple[_JointFrame, ...] = ("]
    for jf in chain:
        lines.append(
            f"    _JointFrame({jf.name!r}, {jf.pos!r}, {jf.quat!r}, "
            f"{jf.joint_range!r}),"
        )
    lines.append(")")
    lines.append(
        f"_EE_SITE: _SiteFrame = _SiteFrame({ee_site.pos!r}, {ee_site.quat!r})"
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mjcf",
        default=str(_DEFAULT_MJCF),
        help="Path to the vendored so101.xml (default: %(default)s)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help=(
            "Verify fret.control.kinematics_so101's hardcoded constants "
            "match the MJCF-derived chain; exit 1 on mismatch."
        ),
    )
    args = parser.parse_args(argv)

    chain, ee_site = parse_so101_chain(args.mjcf)

    if not args.check:
        print(_format_chain_source(chain, ee_site))
        return 0

    sys.path.insert(0, str(_REPO_ROOT / "src"))
    from fret.control import kinematics_so101 as kso101

    ok = True
    for jf, (name, pos, quat, _rng) in zip(chain, kso101.chain_constants()):
        if jf.name != name or jf.pos != pos or jf.quat != quat:
            print(f"MISMATCH joint {jf.name!r}: MJCF={jf} hardcoded={name}")
            ok = False
    hc_pos, hc_quat = kso101.ee_site_constants()
    if ee_site.pos != hc_pos or ee_site.quat != hc_quat:
        print(f"MISMATCH ee site: MJCF={ee_site} hardcoded={hc_pos, hc_quat}")
        ok = False
    if ok:
        print("OK: kinematics_so101.py constants match the vendored MJCF")
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
