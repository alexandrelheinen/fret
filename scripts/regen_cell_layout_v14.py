#!/usr/bin/env python3
"""Regenerate OM-X / OMY pick-place cells: rear structural gate + dual cams + front cone.

Updates MJCF templates and scenario YAML joint waypoints (IK). Vision YAML
extrinsics are filled from live MuJoCo after the MJCF write.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))


def _mj_xyaxes(eye: np.ndarray, target: np.ndarray) -> str:
    forward = target - eye
    forward = forward / float(np.linalg.norm(forward))
    z_cam = -forward
    up = np.array([0.0, 0.0, 1.0])
    x_cam = np.cross(up, z_cam)
    n = float(np.linalg.norm(x_cam))
    if n < 1e-9:
        x_cam = np.array([1.0, 0.0, 0.0])
    else:
        x_cam = x_cam / n
    y_cam = np.cross(z_cam, x_cam)
    return " ".join(f"{v:.6f}" for v in (*x_cam, *y_cam))


def _quat_rot_x(angle_rad: float) -> str:
    c = math.cos(angle_rad * 0.5)
    s = math.sin(angle_rad * 0.5)
    return f"{c:.6f} {s:.6f} 0.000000 0.000000"


def gate_body_xml(
    *,
    x: float,
    half_y: float,
    z_top: float,
    bar: float,
    lookat: tuple[float, float, float],
    fovy: float,
) -> str:
    """Rear structural gate in the YZ plane with X-brace and corner cameras."""
    post_h = 0.5 * z_top
    brace_dy = 2.0 * half_y
    brace_dz = z_top - 4.0 * bar
    brace_len = 0.5 * math.hypot(brace_dy, brace_dz)
    ang = math.atan2(brace_dz, brace_dy)
    look = np.asarray(lookat, dtype=float)
    eye_l = np.array([x + 0.04, half_y, z_top - 0.02])
    eye_r = np.array([x + 0.04, -half_y, z_top - 0.02])
    xy_l = _mj_xyaxes(eye_l, look)
    xy_r = _mj_xyaxes(eye_r, look)
    q_a = _quat_rot_x(ang)
    q_b = _quat_rot_x(-ang)
    return f"""    <!-- Structural rear vision gate + dual corner Cam-A/B (v1.4). -->
    <body name="vision_gate" pos="{x:.5f} 0 0">
      <geom name="gate_post_l" type="box" size="{bar:.4f} {bar:.4f} {post_h:.4f}"
            pos="0 {half_y:.5f} {post_h:.5f}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <geom name="gate_post_r" type="box" size="{bar:.4f} {bar:.4f} {post_h:.4f}"
            pos="0 {-half_y:.5f} {post_h:.5f}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <geom name="gate_beam_top" type="box" size="{bar:.4f} {half_y + bar:.5f} {bar:.4f}"
            pos="0 0 {z_top:.5f}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <geom name="gate_beam_bot" type="box" size="{bar:.4f} {half_y + bar:.5f} {bar:.4f}"
            pos="0 0 {2.0 * bar:.5f}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <geom name="gate_brace_a" type="box" size="{0.7 * bar:.4f} {brace_len:.5f} {0.7 * bar:.4f}"
            pos="0 0 {0.5 * z_top:.5f}" quat="{q_a}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <geom name="gate_brace_b" type="box" size="{0.7 * bar:.4f} {brace_len:.5f} {0.7 * bar:.4f}"
            pos="0 0 {0.5 * z_top:.5f}" quat="{q_b}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <geom name="gate_cam_plate_l" type="box" size="0.03 0.025 0.012"
            pos="0.04 {half_y:.5f} {z_top - 0.02:.5f}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <geom name="gate_cam_plate_r" type="box" size="0.03 0.025 0.012"
            pos="0.04 {-half_y:.5f} {z_top - 0.02:.5f}" material="vision_portal"
            contype="5" conaffinity="5" group="1"/>
      <camera name="gate_cam_left" pos="0.04 {half_y:.5f} {z_top - 0.02:.5f}"
              xyaxes="{xy_l}" fovy="{fovy:.1f}"/>
      <camera name="gate_cam_right" pos="0.04 {-half_y:.5f} {z_top - 0.02:.5f}"
              xyaxes="{xy_r}" fovy="{fovy:.1f}"/>
    </body>
"""


def _shift_pos_line(line: str, dx: float, dy: float) -> str:
    def repl(m: re.Match[str]) -> str:
        x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
        return f'pos="{x + dx:.5f} {y + dy:.5f} {z:.5f}"'

    return re.sub(
        r'pos="([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)"',
        repl,
        line,
        count=1,
    )


def rewrite_pick_place_xml(
    path: Path,
    *,
    d_pick: tuple[float, float],
    d_place: tuple[float, float],
    gate_xml: str,
    overview_cam: str,
) -> None:
    text = path.read_text(encoding="utf-8")
    out: list[str] = []
    skipping_portal = False
    for line in text.splitlines(True):
        if (
            '<body name="vision_portal"' in line
            or '<body name="vision_gate"' in line
        ):
            skipping_portal = True
            continue
        if skipping_portal:
            if line.strip().startswith("</body>"):
                skipping_portal = False
                out.append(gate_xml)
                if not gate_xml.endswith("\n"):
                    out.append("\n")
            continue
        if '<camera name="overhead"' in line:
            continue
        if any(
            k in line
            for k in (
                "place_bucket",
                "place_cone",
                "place_plate",
                "place_cone_rim",
                "funnel_",
                "goal_zone",
            )
        ):
            line = _shift_pos_line(line, d_place[0], d_place[1])
        if any(k in line for k in ("pedestal_pick", "start_zone", "pick_box")):
            line = _shift_pos_line(line, d_pick[0], d_pick[1])
        out.append(line)
    text2 = "".join(out)
    # Ensure overview looks at new cell centre
    text2 = re.sub(
        r'<camera name="overview"[^/]*/>',
        overview_cam.strip(),
        text2,
        count=1,
    )
    path.write_text(text2, encoding="utf-8")
    print(f"updated {path}")


def update_scenario_yaml(
    path: Path,
    *,
    pick_xy: list[float],
    place_xy: list[float],
    configs: dict[str, list[float]],
) -> None:
    text = path.read_text(encoding="utf-8")
    text = re.sub(
        r"pick_xy:\s*\[[^\]]+\]",
        f"pick_xy: [{pick_xy[0]:.3f}, {pick_xy[1]:.3f}]",
        text,
    )
    text = re.sub(
        r"place_xy:\s*\[[^\]]+\]",
        f"place_xy: [{place_xy[0]:.3f}, {place_xy[1]:.3f}]",
        text,
    )
    for key, vals in configs.items():
        joined = ", ".join(f"{v:.4f}" for v in vals)
        text = re.sub(
            rf"{key}:\s*\[[^\]]+\]",
            f"{key}: [{joined}]",
            text,
        )
    path.write_text(text, encoding="utf-8")
    print(f"updated {path}")


def ik_omx(
    pick_xy: list[float], place_xy: list[float]
) -> dict[str, list[float]]:
    from fret.control.kinematics_open_manipulator_x import (
        OpenManipulatorXKinematics,
    )

    kin = OpenManipulatorXKinematics()
    s = 0.72
    seeds = [
        np.array([0.0, -1.05, 0.7, 0.7]),
        np.array([-0.55, 0.60, -0.65, 0.65]),
        np.array([0.55, 0.60, -0.65, 0.65]),
        np.array([0.0, 0.5, -0.5, 0.5]),
        np.zeros(4),
    ]

    def solve(xyz: list[float]) -> list[float]:
        best_err = 1e9
        best_q = seeds[0]
        for seed in seeds:
            q = kin.inverse_kinematics(np.asarray(xyz), seed=seed)
            err = float(np.linalg.norm(kin.forward_kinematics(q)[:3, 3] - xyz))
            if err < best_err:
                best_err = err
                best_q = q
        if best_err > 0.005:
            raise RuntimeError(
                f"OM-X IK poor at {xyz}: {best_err*1000:.1f} mm"
            )
        return [float(v) for v in best_q]

    return {
        "pick_hover_configuration": solve(
            [pick_xy[0] * s, pick_xy[1] * s, 0.20]
        ),
        "pick_grasp_configuration": solve(
            [pick_xy[0] * s, pick_xy[1] * s, 0.175]
        ),
        "place_hover_configuration": solve(
            [place_xy[0] * s, place_xy[1] * s, 0.20]
        ),
        "place_grasp_configuration": solve(
            [place_xy[0] * s, place_xy[1] * s, 0.175]
        ),
    }


def ik_omy(
    pick_xy: list[float], place_xy: list[float]
) -> dict[str, list[float]]:
    """Pad-mid IK (not link6) — matches ``scripts/tune_omy_pad_mid_waypoints``."""
    import mujoco as mj

    from fret.control.omy_pad_mid_ik import (
        OMY_ARM_JOINTS,
        OMY_GRIPPER_PINCH,
        pad_mid_ik,
    )
    from fret.mjcf.omy import ensure_omy_pick_place_mjcf

    idle = np.array([0.7506, -0.8762, 1.7473, 0.3442, 1.4352, 0.0001])
    # Prefer pad-mid IK (not link6 FK). Multi-seed because side picks are
    # farther from the folded idle and local minima are common.
    xml = ensure_omy_pick_place_mjcf()
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    pad_right = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right"))
    pad_left = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left"))
    grip_adr = int(
        model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "rh_r1")]
    )
    limits = np.array(
        [
            model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]
            for n in OMY_ARM_JOINTS
        ],
        dtype=np.float64,
    )
    box = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box"))
    pick_z = float(data.xpos[box][2])
    ball = np.array([pick_xy[0], pick_xy[1], pick_z], dtype=np.float64)
    drop = np.array(
        [place_xy[0], place_xy[1], 0.280 + 0.095], dtype=np.float64
    )
    via = np.array(
        [0.5 * (pick_xy[0] + place_xy[0]), 0.5 * pick_xy[1], 0.42],
        dtype=np.float64,
    )
    specs = [
        ("idle", np.array([0.20, 0.0, 0.35]), 0.0),
        ("pick_hover", ball + np.array([0.0, 0.0, 0.10]), 0.0),
        ("pick_grasp", ball.copy(), OMY_GRIPPER_PINCH),
        ("lift_hover", ball + np.array([0.0, 0.0, 0.14]), OMY_GRIPPER_PINCH),
        ("transfer_via", via, OMY_GRIPPER_PINCH),
        ("place_hover", drop + np.array([0.0, 0.0, 0.10]), OMY_GRIPPER_PINCH),
        ("place_grasp", drop.copy(), OMY_GRIPPER_PINCH),
    ]
    seeds = [
        idle,
        np.array([-0.1001, 0.683, 2.0814, -0.1359, -0.7662, 0.0036]),
        np.array([-0.1096, 0.5268, 1.9468, -0.203, -0.7949, -0.0003]),
    ]
    out: dict[str, list[float]] = {}
    seed = idle.copy()
    for name, target, gv in specs:
        best_q, best_err = None, 1e9
        for cand in [seed, *seeds]:
            q, err = pad_mid_ik(
                model,
                data,
                mj,
                target=target,
                grip_val=float(gv),
                seed=np.asarray(cand, dtype=np.float64),
                limits=limits,
                pad_right=pad_right,
                pad_left=pad_left,
                grip_adr=grip_adr,
            )
            if err < best_err:
                best_q, best_err = q, err
        assert best_q is not None
        if best_err > 0.008:
            raise RuntimeError(
                f"OMY pad-mid IK poor at {name}: {best_err * 1000:.1f} mm"
            )
        out[f"{name}_configuration"] = [round(float(v), 4) for v in best_q]
        seed = best_q
    out["retreat_configuration"] = list(out["idle_configuration"])
    return out


def main() -> int:
    # OM-X: pick on −Y side (±90° yaw), place cone on +X front.
    omx_pick = [0.22, -0.20]
    omx_place = [0.26, 0.0]
    omx_old_pick = [0.273, -0.160]
    omx_old_place = [0.273, 0.160]
    omx_gate = gate_body_xml(
        x=-0.16,
        half_y=0.30,
        z_top=0.72,
        bar=0.012,  # 24 mm square tube
        lookat=(0.22, -0.08, 0.08),
        fovy=42.0,
    )
    omx_overview = (
        '<camera name="overview" pos="0.85 -0.55 0.75"\n'
        '            xyaxes="0.55 0.84 0 -0.40 0.26 0.88" fovy="40"/>'
    )
    rewrite_pick_place_xml(
        REPO / "src/fret/mjcf/omx_pick_place.xml",
        d_pick=(omx_pick[0] - omx_old_pick[0], omx_pick[1] - omx_old_pick[1]),
        d_place=(
            omx_place[0] - omx_old_place[0],
            omx_place[1] - omx_old_place[1],
        ),
        gate_xml=omx_gate,
        overview_cam=omx_overview,
    )
    # Same layout deltas for sibling OM-X cells (shared vision geometry intent).
    for sibling in ("omx_desk_clutter.xml", "omx_wall_maze.xml"):
        rewrite_pick_place_xml(
            REPO / "src/fret/mjcf" / sibling,
            d_pick=(
                omx_pick[0] - omx_old_pick[0],
                omx_pick[1] - omx_old_pick[1],
            ),
            d_place=(
                omx_place[0] - omx_old_place[0],
                omx_place[1] - omx_old_place[1],
            ),
            gate_xml=omx_gate,
            overview_cam=omx_overview,
        )

    update_scenario_yaml(
        REPO / "src/fret/config/scenarios/omx_pick_place.yml",
        pick_xy=omx_pick,
        place_xy=omx_place,
        configs=ik_omx(omx_pick, omx_place),
    )

    # OMY
    omy_pick = [0.35, -0.30]
    omy_place = [0.48, 0.0]
    omy_old_pick = [0.40, -0.28]
    omy_old_place = [0.50, 0.20]
    omy_gate = gate_body_xml(
        x=-0.22,
        half_y=0.42,
        z_top=1.00,
        bar=0.016,  # 32 mm tube
        lookat=(0.38, -0.10, 0.10),
        fovy=45.0,
    )
    omy_overview = (
        '<camera name="overview" pos="1.05 -0.70 1.00"\n'
        '            xyaxes="0.55 0.84 0 -0.40 0.26 0.88" fovy="40"/>'
    )
    for sibling in ("omy_pick_place.xml", "omy_clutter.xml"):
        rewrite_pick_place_xml(
            REPO / "src/fret/mjcf" / sibling,
            d_pick=(
                omy_pick[0] - omy_old_pick[0],
                omy_pick[1] - omy_old_pick[1],
            ),
            d_place=(
                omy_place[0] - omy_old_place[0],
                omy_place[1] - omy_old_place[1],
            ),
            gate_xml=omy_gate,
            overview_cam=omy_overview,
        )

    update_scenario_yaml(
        REPO / "src/fret/config/scenarios/omy_pick_place.yml",
        pick_xy=omy_pick,
        place_xy=omy_place,
        configs=ik_omy(omy_pick, omy_place),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
