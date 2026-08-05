#!/usr/bin/env python3
"""Pin OMX wall-maze wall-contact failure mode after full-arm contype=5.

Writes JSON (+ optional PNG) under ``/opt/cursor/artifacts/wall_maze_diag/``.

Classifies contacts as:
  - plan_mesh: static mj_forward contacts on dense planned waypoints
  - track: contacts during JointSpaceMPC dry-run of that path
  - fsm_only_contacts: plan+track clear, but full FSM still hits walls

Pinned finding (seed 204, contype=5): plan/track clear; FSM FAULT with
``transfer_wall_stem`` ↔ ``gripper_left`` palm (geom 51, bit 4) near
EE ≈ (0.13, −0.10, 0.23) — pick-side Γ during early MOVE_PLACE when the
path was still planned from ``pick_hover``.

Resolution: plan from ``lift_hover``, require zero-hit MPC dry-run on Γ
paths, densify C-space barriers; acceptance = DONE with wall_contact_steps=0.


Usage::

    PYTHONPATH=src python3 scripts/diagnose_wall_maze_contacts.py
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
_OUT = Path("/opt/cursor/artifacts/wall_maze_diag")


def _geom_name(mj: Any, model: Any, gid: int) -> str:
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or f"geom_{gid}"


def _wall_contacts(mj: Any, model: Any, data: Any) -> list[tuple[str, str]]:
    hits: list[tuple[str, str]] = []
    for ci in range(data.ncon):
        c = data.contact[ci]
        g1 = _geom_name(mj, model, c.geom1)
        g2 = _geom_name(mj, model, c.geom2)
        if g1.startswith("transfer_wall") or g2.startswith("transfer_wall"):
            hits.append((g1, g2))
    return hits


def _ee_xyz(mj: Any, model: Any, data: Any) -> list[float]:
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "link5")
    if bid < 0:
        bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "gripper_left")
    return [float(x) for x in data.xpos[bid]]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        default="src/fret/config/scenarios/omx_wall_maze.yml",
    )
    parser.add_argument(
        "--fsm", action="store_true", help="Run full FSM cycle"
    )
    parser.add_argument(
        "--duration-s", type=float, default=55.0, help="FSM duration"
    )
    args = parser.parse_args()

    sys.path.insert(0, str(_REPO / "src"))

    import mujoco as mj

    from fret.control.kinematics import Kinematics
    from fret.control.pick_place_clutter_sim import (
        _barrier_occupancy_for_scenario,
        _dry_run_transfer,
        _path_clear_of_wall_meshes,
        plan_transfer_path,
        run_pick_place_clutter,
    )
    from fret.control.pick_place_sim import waypoints_from_scenario
    from fret.sitl_config import load_scenario_parameters, mjcf_path

    scenario = Path(args.scenario)
    params = load_scenario_parameters(scenario)
    wp = waypoints_from_scenario(scenario)
    kin = Kinematics("open_manipulator_x")

    report: dict[str, Any] = {
        "scenario": str(scenario),
        "planner_rng_seed": int(params.get("planner_rng_seed", -1)),
        "wall_inflate_m": float(params.get("wall_inflate_m", 0.0)),
        "mpc_cspace_clearance_rad": float(
            params.get("mpc_cspace_clearance_rad", 0.0)
        ),
        "mpc_weight_obstacle": float(params.get("mpc_weight_obstacle", 0.0)),
    }

    # --- Plan ---
    try:
        path, straight = plan_transfer_path(
            wp.pick_hover, wp.place_hover, scenario_path=scenario
        )
        report["plan_ok"] = True
        report["straight_collides"] = bool(straight)
        report["path_len"] = len(path)
    except Exception as exc:  # noqa: BLE001 — diagnostic surface
        report["plan_ok"] = False
        report["plan_error"] = f"{type(exc).__name__}: {exc}"
        _OUT.mkdir(parents=True, exist_ok=True)
        (_OUT / "report.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
        print(json.dumps(report, indent=2))
        return 1

    ee = np.array(
        [kin.forward_kinematics(q)[:3, 3] for q in path], dtype=np.float64
    )
    report["path_ee_min_r"] = float(np.min(np.linalg.norm(ee[:, :2], axis=1)))
    report["path_ee_min_z"] = float(np.min(ee[:, 2]))
    report["path_ee_peak_z"] = float(np.max(ee[:, 2]))

    # --- Static mesh contacts on planned waypoints ---
    xml = mjcf_path("open_manipulator_x", "omx_wall_maze")
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    names = ("Joint1", "Joint2", "Joint3", "Joint4")
    qadrs = [
        int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)])
        for n in names
    ]
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    data.qpos[
        int(model.jnt_qposadr[box_jid]) : int(model.jnt_qposadr[box_jid]) + 3
    ] = [
        0.5,
        0.5,
        0.5,
    ]
    plan_hits: list[dict[str, Any]] = []
    pair_counts: Counter[str] = Counter()
    stride = max(1, len(path) // 50)
    for idx, q in enumerate(path[::stride]):
        for i, adr in enumerate(qadrs):
            data.qpos[adr] = float(q[i])
        data.qvel[:] = 0.0
        mj.mj_forward(model, data)
        hits = _wall_contacts(mj, model, data)
        if hits:
            for a, b in hits:
                pair_counts[f"{a}|{b}"] += 1
            plan_hits.append(
                {
                    "waypoint_index": int(idx * stride),
                    "q": [float(x) for x in q],
                    "ee": _ee_xyz(mj, model, data),
                    "pairs": hits,
                }
            )
    report["plan_mesh_clear"] = _path_clear_of_wall_meshes(
        path, scenario_id="omx_wall_maze"
    )
    report["plan_mesh_contact_waypoints"] = len(plan_hits)
    report["plan_mesh_pair_counts"] = dict(pair_counts.most_common(20))
    report["plan_mesh_first_hits"] = plan_hits[:5]

    # --- MPC dry-run tracking ---
    mpc_occ = _barrier_occupancy_for_scenario(
        params,
        robot_model="open_manipulator_x",
        scenario_id="omx_wall_maze",
        seed=int(params.get("planner_rng_seed", 7)),
    )
    track_ok = _dry_run_transfer(
        path,
        wp.pick_hover,
        wp.place_hover,
        joint_tol_rad=0.12,
        scenario_id="omx_wall_maze",
        robot_model="open_manipulator_x",
        occupancy=mpc_occ,
        params=params,
    )
    report["track_dry_run_ok"] = bool(track_ok)

    # Instrumented dry-run contact log (same tracker as sim).
    from fret.control.joint_mpc import JointPathMPCTracker
    from fret.control.pick_place_clutter_sim import _joint_mpc_for_model

    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    data.qpos[
        int(model.jnt_qposadr[box_jid]) : int(model.jnt_qposadr[box_jid]) + 3
    ] = [
        0.5,
        0.5,
        0.5,
    ]
    for i, n in enumerate(names):
        data.qpos[
            model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]
        ] = float(wp.pick_hover[i])
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)
    mpc = _joint_mpc_for_model(
        "open_manipulator_x", occupancy=mpc_occ, params=params
    )
    tracker = JointPathMPCTracker(
        path,
        mpc,
        race_speed=1.2,
        max_carrot_lag=0.40,
        goal_tol=0.12,
    )
    tracker.reset(wp.pick_hover.copy())
    track_events: list[dict[str, Any]] = []
    track_pairs: Counter[str] = Counter()
    dt = float(model.opt.timestep)
    ctrl_period = 0.02
    accum = 0.0
    q_cmd = wp.pick_hover.copy()
    for step in range(int(12.0 / dt)):
        accum += dt
        if accum >= ctrl_period:
            accum -= ctrl_period
            if not tracker.complete:
                q_cmd = tracker.step(ctrl_period)
        for i, n in enumerate(names):
            data.ctrl[mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, n)] = (
                float(q_cmd[i])
            )
        mj.mj_step(model, data)
        hits = _wall_contacts(mj, model, data)
        if hits:
            for a, b in hits:
                track_pairs[f"{a}|{b}"] += 1
            if len(track_events) < 40:
                track_events.append(
                    {
                        "t_s": float(step * dt),
                        "ee": _ee_xyz(mj, model, data),
                        "pairs": hits,
                    }
                )
        if tracker.complete and step * dt > 2.0:
            break
    report["track_contact_events"] = len(track_events)
    report["track_pair_counts"] = dict(track_pairs.most_common(20))
    report["track_first_events"] = track_events[:8]

    # Classification
    if report["plan_mesh_contact_waypoints"] > 0:
        kind = "bad_plan_mesh"
    elif report["track_contact_events"] > 0:
        kind = "good_plan_bad_tracking"
    else:
        kind = "plan_and_track_clear"
    report["failure_class"] = kind

    if args.fsm:
        # Instrument FSM: record phase + geom pairs on every wall contact tick.
        import fret.control.pick_place_clutter_sim as _clutter

        fsm_events: list[dict[str, Any]] = []
        fsm_pairs: Counter[str] = Counter()
        fsm_phases: Counter[str] = Counter()
        _orig = _clutter.arm_contacts_transfer_wall

        def _wrapped(mj_: Any, model_: Any, data_: Any) -> bool:
            hit = _orig(mj_, model_, data_)
            if hit and len(fsm_events) < 80:
                # Best-effort phase from FSM if present on the stack caller.
                phase = "?"
                pairs = _wall_contacts(mj_, model_, data_)
                for a, b in pairs:
                    fsm_pairs[f"{a}|{b}"] += 1
                fsm_events.append(
                    {
                        "ee": _ee_xyz(mj_, model_, data_),
                        "pairs": pairs,
                        "phase": phase,
                    }
                )
            if hit:
                fsm_phases["contact_ticks"] += 1
            return hit

        _clutter.arm_contacts_transfer_wall = _wrapped  # type: ignore[assignment]
        try:
            result = run_pick_place_clutter(
                duration_s=float(args.duration_s),
                scenario_path=scenario,
                max_attempts=3,
            )
            report["fsm_state"] = result.state.name
            report["fsm_wall_contact_steps"] = int(result.wall_contact_steps)
            report["fsm_faulted_on_wall"] = bool(
                result.faulted_on_wall_contact
            )
        except RuntimeError as exc:
            report["fsm_error"] = str(exc)
            report["fsm_state"] = "ERROR"
            report["fsm_wall_contact_steps"] = -1
            report["fsm_faulted_on_wall"] = True
        finally:
            _clutter.arm_contacts_transfer_wall = _orig  # type: ignore[assignment]
        report["fsm_contact_pair_counts"] = dict(fsm_pairs.most_common(20))
        report["fsm_contact_events"] = fsm_events[:20]
        report["fsm_contact_ticks_logged"] = int(fsm_phases["contact_ticks"])
        if (
            report.get("plan_mesh_contact_waypoints", 0) == 0
            and report.get("track_contact_events", 0) == 0
        ):
            if int(
                report.get("fsm_wall_contact_steps", 0) or 0
            ) > 0 or report.get("fsm_faulted_on_wall"):
                report["failure_class"] = "fsm_only_contacts"
                kind = "fsm_only_contacts"

    _OUT.mkdir(parents=True, exist_ok=True)
    (_OUT / "report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    # Lightweight EE path dump for charts
    np.save(_OUT / "path_ee.npy", ee)
    print(json.dumps(report, indent=2))
    print(f"wrote {_OUT / 'report.json'}", file=sys.stderr)
    return 0 if kind == "plan_and_track_clear" and report.get("plan_ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
