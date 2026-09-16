#!/usr/bin/env python3
"""Render the FRET illustration gallery: 16:9 stills of real runs.

Every plate is a still frame of the *same* simulation the release videos
are rendered from — planner, controller, and MuJoCo physics all run for
real, then the frame is lit and graded for publication use (website card,
pitch deck, paper figure).

What makes a plate "artistic" is staging, never fiction:

* a studio light rig (key / fill / cool rim / warm kicker) replaces the
  flat default headlight, and the backdrop becomes a dark gradient;
* the arm or the vehicles are multi-exposed along the recorded path, so
  the motion the planner produced is visible in one frame;
* the executed end-effector (or vehicle) track is drawn as a glowing
  ribbon coloured by the FSM phase, and planned paths are drawn thin;
* bloom, contrast, and a vignette are applied in post.

Geometry, poses, and trajectories are never edited: only materials,
lights, camera framing, and the grade.

Example::

    MUJOCO_GL=egl python3 scripts/release/render_gallery.py --all \\
        --output-dir docs/images/gallery

Dependencies (not required for core FRET algorithms)::

    pip install -e ".[sim]"
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "scripts" / "release"))
sys.path.insert(0, str(_REPO_ROOT / "scripts"))

from gallery_manifest import (  # noqa: E402
    GalleryManifest,
    GalleryPlate,
    GhostSpec,
    assert_sixteen_by_nine,
    box_downscale,
    ghost_indices,
    ghost_weights,
    grade_image,
    hero_index,
    load_gallery_manifest,
    screen,
)
from render_mujoco import resolve_mjcf_path  # noqa: E402

# Ghost passes hide every non-actor geom by moving it to this group.
_GHOST_GROUP = 5
_SCENARIO_DIR = _REPO_ROOT / "src/fret/config/scenarios"

# FRET plate palette. Cyan = approach, mint = grasp, amber = transfer,
# violet = place/retreat. Also reused per agent in the Dubins race
# (cyan = RRT*, amber = SST, slate = the fixed dummy foil).
_CYAN = np.array([0.24, 0.76, 0.98])
_MINT = np.array([0.35, 0.92, 0.70])
_AMBER = np.array([0.99, 0.66, 0.18])
_VIOLET = np.array([0.72, 0.45, 0.98])
_SLATE = np.array([0.55, 0.62, 0.72])

# PickPlaceState -> ribbon colour (IntEnum values; see pick_place_fsm).
_PHASE_COLORS: dict[int, npt.NDArray[np.float64]] = {
    1: _CYAN,  # APPROACH_PICK
    2: _CYAN,  # DESCEND_PICK
    3: _MINT,  # GRASP
    4: _MINT,  # LIFT
    5: _AMBER,  # MOVE_PLACE
    6: _AMBER,  # DESCEND_PLACE
    7: _VIOLET,  # RELEASE
    8: _VIOLET,  # RETREAT
    9: _VIOLET,  # DONE
}

_AGENT_COLORS: dict[str, npt.NDArray[np.float64]] = {
    "rrt": _CYAN,
    "sst": _AMBER,
    "dummy": _SLATE,
}
_DUBINS_JOINTS: dict[str, str] = {
    "rrt": "rrt_base_joint",
    "sst": "sst_base_joint",
    "dummy": "dummy_base_joint",
}
_DUBINS_TRACE_Z_M = 0.05
# Scene markers whose colour lives on the geom, not on a material.
_MARKER_TINTS: dict[str, tuple[float, float, float, float]] = {
    "start_zone": (0.10, 0.34, 0.22, 0.45),
    "goal_zone": (0.38, 0.12, 0.12, 0.45),
    "goal_beacon": (0.34, 0.11, 0.10, 1.0),
}
_AGENT_MATERIALS: dict[str, npt.NDArray[np.float64]] = {
    "car_rrt": _CYAN,
    "car_sst": _AMBER,
    "car_dummy": _SLATE,
}

_OMX_SCENARIO_YAML: dict[str, str] = {
    "omx_wall_maze_rrt": "omx_wall_maze_rrt.yml",
    "omx_wall_maze_sst": "omx_wall_maze_sst.yml",
    "omx_desk_clutter": "omx_desk_clutter.yml",
    "omy_clutter_rrt": "omy_clutter_rrt.yml",
    "omy_clutter_sst": "omy_clutter_sst.yml",
}
_TOOL_BODIES: dict[str, tuple[str, ...]] = {
    "omy": ("rh_r2", "rh_l2"),
    "open_manipulator_x": ("gripper_left", "gripper_right"),
}


def _require_mujoco() -> Any:
    """Import MuJoCo with an actionable error when the extra is missing."""
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - environment guard
        raise SystemExit(
            "mujoco is required: pip install -e '.[sim]' "
            "(and set MUJOCO_GL=egl for headless rendering)"
        ) from exc
    return mujoco


@dataclass(frozen=True)
class SimRun:
    """Recorded samples of one scenario run, cacheable as ``.npz``."""

    kind: str
    q_arm: npt.NDArray[np.float64] | None = None
    gripper: npt.NDArray[np.float64] | None = None
    box_qpos: npt.NDArray[np.float64] | None = None
    states: npt.NDArray[np.int_] | None = None
    poses: dict[str, npt.NDArray[np.float64]] | None = None
    plans: dict[str, npt.NDArray[np.float64]] | None = None

    @property
    def n_samples(self) -> int:
        """Number of recorded samples in the run."""
        if self.q_arm is not None:
            return int(self.q_arm.shape[0])
        if self.poses:
            return int(next(iter(self.poses.values())).shape[0])
        return 0

    def to_npz(self, path: Path) -> None:
        """Write the run to ``path`` as a compressed archive."""
        payload: dict[str, npt.NDArray[Any]] = {"kind": np.array([self.kind])}
        if self.q_arm is not None:
            payload["q_arm"] = self.q_arm
        if self.gripper is not None:
            payload["gripper"] = self.gripper
        if self.box_qpos is not None:
            payload["box_qpos"] = self.box_qpos
        if self.states is not None:
            payload["states"] = self.states
        for name, values in (self.poses or {}).items():
            payload[f"pose_{name}"] = values
        for name, values in (self.plans or {}).items():
            payload[f"plan_{name}"] = values
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, **payload)

    @classmethod
    def from_npz(cls, path: Path) -> SimRun:
        """Read a run previously written by :meth:`to_npz`."""
        with np.load(path, allow_pickle=False) as data:
            poses = {
                key[len("pose_") :]: data[key]
                for key in data.files
                if key.startswith("pose_")
            }
            plans = {
                key[len("plan_") :]: data[key]
                for key in data.files
                if key.startswith("plan_")
            }
            return cls(
                kind=str(data["kind"][0]),
                q_arm=data["q_arm"] if "q_arm" in data else None,
                gripper=data["gripper"] if "gripper" in data else None,
                box_qpos=data["box_qpos"] if "box_qpos" in data else None,
                states=data["states"] if "states" in data else None,
                poses=poses or None,
                plans=plans or None,
            )


# ---------------------------------------------------------------------------
# Simulation drivers (the honest part: every plate replays a real run)
# ---------------------------------------------------------------------------


def _simulate_pick_place(plate: GalleryPlate) -> SimRun:
    """Run the OMY floor pick-and-place cycle (SC-v14b / SC-v16b)."""
    from fret.control.omy_pick_place_sim import simulate_omy_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    state, samples = simulate_omy_pick_place(
        duration_s=55.0,
        joint_tol_rad=0.22,
        record_every_steps=25,
        scenario_path=_SCENARIO_DIR / f"{plate.scenario}.yml",
    )
    if state != PickPlaceState.DONE:
        raise RuntimeError(
            f"{plate.scenario} ended in {state.name}, expected DONE"
        )
    return _samples_to_run(samples, kind="pick_place")


def _simulate_clutter(plate: GalleryPlate) -> SimRun:
    """Run an obstacle-aware cell (OM-X Γ maze, OMY clutter)."""
    from fret.control.pick_place_clutter_sim import run_pick_place_clutter
    from fret.control.pick_place_fsm import PickPlaceState

    scenario_yaml = _OMX_SCENARIO_YAML.get(
        plate.scenario, f"{plate.scenario}.yml"
    )
    result = run_pick_place_clutter(
        duration_s=45.0,
        joint_tol_rad=0.22,
        scenario_path=_SCENARIO_DIR / scenario_yaml,
        max_attempts=8,
        record_every_steps=25,
    )
    if result.state != PickPlaceState.DONE:
        raise RuntimeError(
            f"{plate.scenario} ended in {result.state.name}, expected DONE"
        )
    if int(result.wall_contact_steps) > 0:
        raise RuntimeError(
            f"{plate.scenario} had {result.wall_contact_steps} wall contact "
            "steps (honesty gate: a plate must show a clean run)"
        )
    return _samples_to_run(result.samples, kind="clutter")


def _samples_to_run(samples: list[Any], *, kind: str) -> SimRun:
    """Pack recorded ``PickPlaceSample`` objects into a :class:`SimRun`."""
    if len(samples) < 2:
        raise RuntimeError("run recorded too few samples")
    return SimRun(
        kind=kind,
        q_arm=np.asarray([s.q_arm for s in samples], dtype=np.float64),
        gripper=np.asarray(
            [float(s.gripper) for s in samples], dtype=np.float64
        ),
        box_qpos=np.asarray([s.box_qpos for s in samples], dtype=np.float64),
        states=np.asarray([int(s.state) for s in samples], dtype=np.int64),
    )


def _simulate_dubins(plate: GalleryPlate) -> SimRun:
    """Run the SC-v11 Dubins race in physics mode and keep both plans."""
    from fret.scenario.dubins_race_runner import DubinsRaceRunner
    from fret.scenario.planner_rng import SHOWCASE_PLANNER_RNG_SEED

    result = DubinsRaceRunner().run(
        record_poses=True,
        physics_mode=True,
        planner_rng_seed=SHOWCASE_PLANNER_RNG_SEED,
    )
    if not result.both_reached_goal:
        raise RuntimeError(
            "Dubins race did not finish "
            f"(race_duration_s={result.race_duration_s:.1f})"
        )
    poses = {
        "rrt": np.asarray(result.rrt_pose_history, dtype=np.float64),
        "sst": np.asarray(result.sst_pose_history, dtype=np.float64),
        "dummy": np.asarray(result.dummy_pose_history, dtype=np.float64),
    }
    plans = {
        "rrt": np.asarray(
            [
                np.asarray(p, dtype=np.float64)[:2]
                for p in result.rrt_plan.path
            ],
            dtype=np.float64,
        ),
        "sst": np.asarray(
            [
                np.asarray(p, dtype=np.float64)[:2]
                for p in result.sst_plan.path
            ],
            dtype=np.float64,
        ),
    }
    del plate
    return SimRun(kind="dubins", poses=poses, plans=plans)


_DRIVERS = {
    "pick_place": _simulate_pick_place,
    "clutter": _simulate_clutter,
    "dubins": _simulate_dubins,
}


def load_run(
    plate: GalleryPlate, *, cache_dir: Path, refresh: bool = False
) -> SimRun:
    """Return the recorded run for ``plate``, using the npz cache."""
    cache_path = cache_dir / f"{plate.scenario}.npz"
    if cache_path.is_file() and not refresh:
        run = SimRun.from_npz(cache_path)
        print(f"[gallery] cache hit {cache_path}", flush=True)
        return run
    print(f"[gallery] simulating {plate.scenario} …", flush=True)
    run = _DRIVERS[plate.source](plate)
    run.to_npz(cache_path)
    print(f"[gallery] cached {cache_path}", flush=True)
    return run


# ---------------------------------------------------------------------------
# Studio rig: lights, backdrop, materials (never geometry or poses)
# ---------------------------------------------------------------------------


def _apply_render_size(spec: Any, width: int, height: int) -> None:
    spec.visual.global_.offwidth = int(width)
    spec.visual.global_.offheight = int(height)
    spec.visual.quality.offsamples = 8
    spec.visual.quality.shadowsize = 8192


def beauty_model(
    mujoco: Any, mjcf_path: Path, *, width: int, height: int, fovy: float
) -> Any:
    """Compile the lit "beauty" model used for the sharp pose."""
    spec = mujoco.MjSpec.from_file(str(mjcf_path))
    _apply_render_size(spec, width, height)
    spec.visual.global_.fovy = float(fovy)
    spec.visual.headlight.ambient = [0.20, 0.22, 0.27]
    spec.visual.headlight.diffuse = [0.16, 0.17, 0.21]
    spec.visual.headlight.specular = [0.0, 0.0, 0.0]
    spec.visual.rgba.haze = [0.03, 0.05, 0.09, 1.0]
    spec.visual.map.haze = 0.3

    # The scene's own lights are tuned for the video look; the plate rig
    # replaces them so every scenario is lit the same way.
    for light in spec.lights:
        light.diffuse = [0.0, 0.0, 0.0]
        light.specular = [0.0, 0.0, 0.0]
        light.castshadow = False

    for texture in spec.textures:
        if texture.type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
            texture.rgb1 = [0.05, 0.08, 0.13]
            texture.rgb2 = [0.01, 0.02, 0.04]
        if texture.name in {"groundplane", "floor"}:
            texture.rgb1 = [0.075, 0.090, 0.115]
            texture.rgb2 = [0.050, 0.062, 0.080]
            texture.markrgb = [0.12, 0.28, 0.38]

    for material in spec.materials:
        if material.name == "black":
            material.rgba = [0.17, 0.18, 0.21, 1.0]
            material.specular = 0.90
            material.shininess = 0.70
            material.reflectance = 0.08
        elif material.name in {"place_bin", "transfer_wall"}:
            material.rgba = [0.16, 0.19, 0.24, 1.0]
            material.specular = 0.35
            material.shininess = 0.4
            material.reflectance = 0.04
        elif material.name == "vision_portal":
            # The CV gate's translucent portal panel is a visualisation
            # aid, not physics: hiding it keeps the arm unveiled. The
            # gate frame itself stays in shot.
            material.rgba = [0.14, 0.24, 0.32, 0.0]
        elif material.name == "pick_ball":
            material.emission = 0.10
            material.specular = 0.5
        elif material.name == "floor":
            material.rgba = [0.085, 0.10, 0.125, 1.0]
            material.specular = 0.25
            material.shininess = 0.35
            material.reflectance = 0.10
        elif material.name == "start_zone":
            material.rgba = [0.10, 0.42, 0.24, 0.80]
            material.emission = 0.05
        elif material.name == "goal_zone":
            material.rgba = [0.52, 0.10, 0.12, 0.80]
            material.emission = 0.05
        elif material.name in _AGENT_MATERIALS:
            # Paint each vehicle in its planner's plate colour so the
            # car and its track read as one object.
            material.rgba = [*_AGENT_MATERIALS[material.name], 1.0]
            material.specular = 0.45
            material.shininess = 0.45
            material.emission = 0.06
        elif material.name in {
            "shelf",
            "clutter_pallet",
            "clutter_crate",
            "clutter_cardboard",
            "tb3_hardware",
        }:
            material.rgba = [
                *(0.55 * np.asarray(material.rgba[:3])),
                material.rgba[3],
            ]
            material.specular = 0.25
            material.shininess = 0.3

    for geom in spec.worldbody.geoms:
        tint = _MARKER_TINTS.get(geom.name)
        if tint is not None:
            geom.rgba = list(tint)

    _add_light_rig(
        mujoco, spec, extent=max(1.0, float(spec.stat.extent or 1.0))
    )
    return spec.compile()


def _add_light_rig(mujoco: Any, spec: Any, *, extent: float) -> None:
    """Add a four-light studio rig (directional, so scale-independent).

    Directional lights keep the same look from a 0.65 m tabletop cell to
    an 11 m warehouse floor: no distance falloff, no hot spot on the
    ground plane, parallel shadows.
    """
    key = spec.worldbody.add_light()
    key.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    key.pos = [1.0 * extent, -1.5 * extent, 2.0 * extent]
    key.dir = [-0.40, 0.62, -0.68]
    key.diffuse = [1.15, 1.09, 1.00]
    key.specular = [0.35, 0.35, 0.35]
    key.castshadow = True

    fill = spec.worldbody.add_light()
    fill.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    fill.pos = [-1.8 * extent, -1.2 * extent, 1.2 * extent]
    fill.dir = [0.72, 0.48, -0.50]
    fill.diffuse = [0.26, 0.32, 0.44]
    fill.specular = [0.05, 0.05, 0.05]
    fill.castshadow = False

    rim = spec.worldbody.add_light()
    rim.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    rim.pos = [-0.25 * extent, 1.9 * extent, 1.0 * extent]
    rim.dir = [0.20, -0.94, -0.28]
    rim.diffuse = [0.44, 0.82, 1.06]
    rim.specular = [0.85, 0.85, 0.85]
    rim.castshadow = False

    kicker = spec.worldbody.add_light()
    kicker.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    kicker.pos = [1.7 * extent, 1.3 * extent, 0.9 * extent]
    kicker.dir = [-0.70, -0.62, -0.35]
    kicker.diffuse = [0.46, 0.30, 0.14]
    kicker.specular = [0.45, 0.40, 0.30]
    kicker.castshadow = False


def ghost_model(
    mujoco: Any,
    mjcf_path: Path,
    *,
    width: int,
    height: int,
    fovy: float,
    ghosts: GhostSpec,
) -> Any:
    """Compile a model that renders only the actor, flat, on black.

    Screening such a pass over the beauty frame adds motion trails
    without touching the rest of the scene.
    """
    spec = mujoco.MjSpec.from_file(str(mjcf_path))
    _apply_render_size(spec, width, height)
    spec.visual.global_.fovy = float(fovy)
    spec.visual.headlight.ambient = [1.0, 1.0, 1.0]
    spec.visual.headlight.diffuse = [0.0, 0.0, 0.0]
    spec.visual.headlight.specular = [0.0, 0.0, 0.0]
    spec.visual.rgba.haze = [0.0, 0.0, 0.0, 1.0]
    spec.visual.map.haze = 0.0
    for light in spec.lights:
        light.diffuse = [0.0, 0.0, 0.0]
        light.specular = [0.0, 0.0, 0.0]
        light.castshadow = False
    for texture in spec.textures:
        if texture.type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
            texture.rgb1 = [0.0, 0.0, 0.0]
            texture.rgb2 = [0.0, 0.0, 0.0]
    for material in spec.materials:
        material.rgba = [*ghosts.rgb, 1.0]
        material.emission = 0.0
        material.specular = 0.0
        material.reflectance = 0.0
        material.textures = [""] * len(material.textures)
    actors = set(ghosts.bodies)
    for body in spec.bodies:
        if body.name in actors:
            continue
        for geom in body.geoms:
            geom.group = _GHOST_GROUP
    for geom in spec.worldbody.geoms:
        geom.group = _GHOST_GROUP
    return spec.compile()


# ---------------------------------------------------------------------------
# Overlay geometry (planner and controller output drawn into the scene)
# ---------------------------------------------------------------------------


def add_segment(
    mujoco: Any,
    scene: Any,
    start: npt.NDArray[np.float64],
    end: npt.NDArray[np.float64],
    rgb: npt.NDArray[np.float64],
    *,
    width: float,
    alpha: float = 0.95,
    emission: float = 0.85,
) -> None:
    """Append one glowing capsule segment to ``scene``."""
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3),
        np.zeros(3),
        np.zeros(9),
        np.asarray([*rgb, alpha], dtype=np.float32),
    )
    mujoco.mjv_connector(
        geom, mujoco.mjtGeom.mjGEOM_CAPSULE, float(width), start, end
    )
    geom.emission = float(emission)
    geom.specular = 0.0
    scene.ngeom += 1


def add_marker(
    mujoco: Any,
    scene: Any,
    position: npt.NDArray[np.float64],
    rgb: npt.NDArray[np.float64],
    *,
    radius: float,
    alpha: float = 1.0,
) -> None:
    """Append one glowing sphere marker to ``scene``."""
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.full(3, float(radius)),
        np.asarray(position, dtype=np.float64),
        np.eye(3).flatten(),
        np.asarray([*rgb, alpha], dtype=np.float32),
    )
    geom.emission = 1.0
    scene.ngeom += 1


def draw_polyline(
    mujoco: Any,
    scene: Any,
    points: npt.NDArray[np.float64],
    rgb: npt.NDArray[np.float64],
    *,
    width: float,
    alpha: float = 0.9,
    dashed: bool = False,
) -> None:
    """Draw a 3-D polyline, optionally dashed (for *planned* paths)."""
    for index in range(1, len(points)):
        if dashed and index % 2 == 0:
            continue
        add_segment(
            mujoco,
            scene,
            points[index - 1],
            points[index],
            rgb,
            width=width,
            alpha=alpha,
        )


# ---------------------------------------------------------------------------
# Plate rendering
# ---------------------------------------------------------------------------


def _track_point(
    plate: GalleryPlate, run: SimRun, hero: int
) -> npt.NDArray[np.float64] | None:
    """Return the world point the camera should centre on, if any."""
    target = plate.camera.track
    if not target or run.kind != "dubins" or not run.poses:
        return None
    if target == "pack":
        names = [a for a in ("rrt", "sst") if a in run.poses]
    else:
        names = [target]
    points = []
    for name in names:
        history = run.poses[name]
        points.append(history[min(hero, history.shape[0] - 1)][:2])
    if not points:
        return None
    centre = np.mean(points, axis=0)
    return np.array([centre[0], centre[1], plate.camera.lookat[2]])


def _make_camera(
    mujoco: Any,
    plate: GalleryPlate,
    lookat: npt.NDArray[np.float64] | None = None,
) -> Any:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    target = lookat if lookat is not None else np.asarray(plate.camera.lookat)
    camera.lookat[0] = float(target[0])
    camera.lookat[1] = float(target[1])
    camera.lookat[2] = float(target[2])
    camera.distance = plate.camera.distance
    camera.azimuth = plate.camera.azimuth
    camera.elevation = plate.camera.elevation
    return camera


def _tool_point(
    mujoco: Any, model: Any, data: Any, bodies: tuple[str, ...]
) -> npt.NDArray[np.float64]:
    positions = []
    for name in bodies:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"Body not found in MJCF: {name}")
        positions.append(np.asarray(data.xpos[body_id], dtype=np.float64))
    return np.mean(positions, axis=0)


@dataclass
class _ArmStage:
    """Bound arm scene: apply a recorded sample, read the tool point."""

    mujoco: Any
    model: Any
    data: Any
    box_qadr: int
    run: SimRun
    apply_sample: Any
    tool_bodies: tuple[str, ...]

    def apply(self, index: int) -> None:
        """Write sample ``index`` into the model state."""
        assert self.run.q_arm is not None
        assert self.run.gripper is not None
        assert self.run.box_qpos is not None
        self.apply_sample(
            self.mujoco,
            self.model,
            self.data,
            q_arm=self.run.q_arm[index],
            gripper=float(self.run.gripper[index]),
            box_qpos=self.run.box_qpos[index],
            box_qadr=self.box_qadr,
        )

    def tool_point(self, index: int) -> npt.NDArray[np.float64]:
        """Return the tool centre point at sample ``index``."""
        self.apply(index)
        return _tool_point(
            self.mujoco, self.model, self.data, self.tool_bodies
        )


def _arm_stage(
    mujoco: Any, plate: GalleryPlate, model: Any, run: SimRun
) -> _ArmStage:
    if plate.model == "omy":
        from render_mujoco import (  # noqa: PLC0415
            _apply_omy_pick_place_sample as apply_sample,
        )
    else:
        from render_mujoco import (  # noqa: PLC0415
            _apply_omx_pick_place_sample as apply_sample,
        )
    joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    if joint_id < 0:
        raise ValueError("pick_box_joint missing from scene")

    def _apply(mj: Any, m: Any, d: Any, **kwargs: Any) -> None:
        if plate.model == "omy":
            apply_sample(mj, m, d, physics_replay=False, **kwargs)
        else:
            apply_sample(mj, m, d, **kwargs)

    return _ArmStage(
        mujoco=mujoco,
        model=model,
        data=mujoco.MjData(model),
        box_qadr=int(model.jnt_qposadr[joint_id]),
        run=run,
        apply_sample=_apply,
        tool_bodies=_TOOL_BODIES[plate.model],
    )


def _render(renderer: Any, data: Any, camera: Any, options: Any) -> Any:
    renderer.update_scene(data, camera=camera, scene_option=options)
    return renderer.render().astype(np.float64) / 255.0


def _composite_ghosts(
    mujoco: Any,
    plate: GalleryPlate,
    mjcf_path: Path,
    run: SimRun,
    base: npt.NDArray[np.float64],
    *,
    hero: int,
    size: tuple[int, int],
    camera: Any,
) -> npt.NDArray[np.float64]:
    """Screen multi-exposure ghost poses over the beauty frame."""
    ghosts = plate.ghosts
    indices = ghost_indices(hero=hero, count=ghosts.count, span=ghosts.span)
    if not indices:
        return base
    width, height = size
    model = ghost_model(
        mujoco,
        mjcf_path,
        width=width,
        height=height,
        fovy=plate.camera.fovy,
        ghosts=ghosts,
    )
    options = mujoco.MjvOption()
    options.geomgroup[_GHOST_GROUP] = 0
    renderer = mujoco.Renderer(model, height=height, width=width, max_geom=64)
    weights = ghost_weights(
        len(indices),
        weight_min=ghosts.weight_min,
        weight_max=ghosts.weight_max,
    )
    out = base
    try:
        if run.kind == "dubins":
            data = mujoco.MjData(model)
            for weight, index in zip(weights, indices, strict=True):
                _apply_dubins_sample(mujoco, model, data, run, index)
                out = screen(
                    out, _render(renderer, data, camera, options), weight
                )
        else:
            stage = _arm_stage(mujoco, plate, model, run)
            for weight, index in zip(weights, indices, strict=True):
                stage.apply(index)
                out = screen(
                    out,
                    _render(renderer, stage.data, camera, options),
                    weight,
                )
    finally:
        renderer.close()
    return out


def _apply_dubins_sample(
    mujoco: Any, model: Any, data: Any, run: SimRun, index: int
) -> None:
    from render_mujoco import _set_freejoint_pose  # noqa: PLC0415

    assert run.poses is not None
    for agent, joint in _DUBINS_JOINTS.items():
        history = run.poses[agent]
        clamped = min(index, history.shape[0] - 1)
        _set_freejoint_pose(mujoco, model, data, joint, history[clamped])
    mujoco.mj_forward(model, data)


def _draw_arm_overlays(
    mujoco: Any,
    scene: Any,
    plate: GalleryPlate,
    run: SimRun,
    track: npt.NDArray[np.float64],
    hero: int,
) -> None:
    """Draw the executed tool path, phase-coloured, plus end markers."""
    trace = plate.trace
    states = run.states
    for index in range(1, len(track)):
        phase = int(states[index]) if states is not None else 1
        color = (
            _PHASE_COLORS.get(phase, _CYAN) if trace.phase_colors else _CYAN
        )
        past = index <= hero
        if not past and not trace.show_future:
            continue
        add_segment(
            mujoco,
            scene,
            track[index - 1],
            track[index],
            color if past else color * trace.future_scale,
            width=trace.width if past else trace.future_width,
            alpha=0.95 if past else 0.75,
        )
    add_marker(
        mujoco, scene, track[0], _CYAN, radius=trace.marker_radius * 0.85
    )
    add_marker(mujoco, scene, track[hero], _AMBER, radius=trace.marker_radius)


def _draw_dubins_overlays(
    mujoco: Any,
    scene: Any,
    plate: GalleryPlate,
    run: SimRun,
    hero: int,
) -> None:
    """Draw planned paths (dashed) and travelled tracks (solid) per agent."""
    assert run.poses is not None
    trace = plate.trace
    agents = plate.agents or tuple(run.poses)
    for agent in agents:
        color = _AGENT_COLORS.get(agent, _CYAN)
        plan = (run.plans or {}).get(agent)
        if plan is not None and len(plan) > 1:
            planned = np.column_stack(
                [plan[:, 0], plan[:, 1], np.full(len(plan), 0.02)]
            )
            draw_polyline(
                mujoco,
                scene,
                planned,
                color * 0.55,
                width=trace.future_width,
                alpha=0.75,
                dashed=True,
            )
        history = run.poses[agent]
        upto = min(hero, history.shape[0] - 1)
        travelled = np.column_stack(
            [
                history[: upto + 1, 0],
                history[: upto + 1, 1],
                np.full(upto + 1, _DUBINS_TRACE_Z_M),
            ]
        )
        draw_polyline(
            mujoco, scene, travelled, color, width=trace.width, alpha=0.95
        )
        add_marker(
            mujoco,
            scene,
            travelled[-1],
            color,
            radius=trace.marker_radius,
            alpha=0.9,
        )


def render_plate(
    plate: GalleryPlate,
    *,
    manifest: GalleryManifest,
    output_dir: Path,
    cache_dir: Path,
    refresh: bool = False,
) -> Path:
    """Render one gallery plate and return the written PNG path."""
    import imageio.v2 as imageio

    mujoco = _require_mujoco()
    out_width = plate.effective_width(manifest.width)
    out_height = plate.effective_height(manifest.height)
    assert_sixteen_by_nine(out_width, out_height, context=plate.id)
    scale = manifest.supersample
    width, height = out_width * scale, out_height * scale

    run = load_run(plate, cache_dir=cache_dir, refresh=refresh)
    mjcf_path = resolve_mjcf_path(plate.model, plate.scenario, None)
    model = beauty_model(
        mujoco, mjcf_path, width=width, height=height, fovy=plate.camera.fovy
    )
    hero = hero_index(
        run.states,
        n_samples=run.n_samples,
        state=plate.hero_state,
        at=plate.hero_at,
    )
    options = mujoco.MjvOption()
    renderer = mujoco.Renderer(
        model, height=height, width=width, max_geom=20000
    )

    try:
        if run.kind == "dubins":
            data = mujoco.MjData(model)
            _apply_dubins_sample(mujoco, model, data, run, hero)
            camera = _make_camera(
                mujoco, plate, _track_point(plate, run, hero)
            )
            base = _render(renderer, data, camera, options)
            stage_data = data
            track = None
        else:
            stage = _arm_stage(mujoco, plate, model, run)
            track = np.asarray(
                [stage.tool_point(i) for i in range(run.n_samples)]
            )
            lookat = track[hero] if plate.camera.track == "tool" else None
            camera = _make_camera(mujoco, plate, lookat)
            stage.apply(hero)
            base = _render(renderer, stage.data, camera, options)
            stage_data = stage.data

        composite = _composite_ghosts(
            mujoco,
            plate,
            mjcf_path,
            run,
            base,
            hero=hero,
            size=(width, height),
            camera=camera,
        )

        # Overlay pass: redraw the sharp pose with the trajectory geoms,
        # then keep only the pixels the overlay actually changed so the
        # ghost trail underneath survives.
        clean = _render(renderer, stage_data, camera, options)
        scene = renderer.scene
        if run.kind == "dubins":
            _draw_dubins_overlays(mujoco, scene, plate, run, hero)
        else:
            assert track is not None
            _draw_arm_overlays(mujoco, scene, plate, run, track, hero)
        overlay = renderer.render().astype(np.float64) / 255.0
        mask = (np.abs(overlay - clean).max(axis=2) > 0.02).astype(np.float64)
        composite = composite * (1.0 - mask[..., None]) + (
            overlay * mask[..., None]
        )
    finally:
        renderer.close()

    graded = grade_image(composite, plate.grade)
    downscaled = box_downscale(graded, scale)
    frame = (np.clip(downscaled, 0.0, 1.0) * 255.0).astype(np.uint8)
    if float(frame.mean()) <= 1.0:
        raise RuntimeError(f"plate {plate.id!r} rendered blank")
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / plate.output
    imageio.imwrite(path, frame)
    print(
        f"[gallery] wrote {path} ({frame.shape[1]}x{frame.shape[0]}, "
        f"hero sample {hero}/{run.n_samples})",
        flush=True,
    )
    return path


def build_parser() -> argparse.ArgumentParser:
    """Build the gallery renderer CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plate",
        action="append",
        dest="plates",
        default=None,
        help="Plate id to render (repeatable)",
    )
    parser.add_argument(
        "--all", action="store_true", help="Render every plate"
    )
    parser.add_argument(
        "--list", action="store_true", help="List plate ids and exit"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=_REPO_ROOT / "docs/images/gallery",
        help="Directory for the rendered PNG plates",
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=_REPO_ROOT / "build/gallery_cache",
        help="Directory for cached simulation runs (.npz)",
    )
    parser.add_argument(
        "--refresh-sim",
        action="store_true",
        help="Re-run the simulations instead of reusing the cache",
    )
    parser.add_argument(
        "--sim-only",
        action="store_true",
        help="Populate the simulation cache without rendering",
    )
    parser.add_argument(
        "--manifest", type=Path, default=None, help="Gallery manifest path"
    )
    parser.add_argument(
        "--supersample",
        type=int,
        default=None,
        help="Override the manifest supersample factor",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Render the requested gallery plates."""
    args = build_parser().parse_args(argv)
    manifest = load_gallery_manifest(args.manifest)
    if args.supersample is not None:
        manifest = GalleryManifest(
            width=manifest.width,
            height=manifest.height,
            supersample=int(args.supersample),
            plates=manifest.plates,
        )
    if args.list:
        for plate in manifest.plates:
            print(f"{plate.id:20s} {plate.scenario:20s} {plate.output}")
        return 0
    if args.all:
        selected = list(manifest.plates)
    elif args.plates:
        selected = [manifest.by_id(plate_id) for plate_id in args.plates]
    else:
        print("Nothing to do: pass --all, --plate ID, or --list")
        return 2
    if args.sim_only:
        for scenario in dict.fromkeys(p.scenario for p in selected):
            plate = next(p for p in selected if p.scenario == scenario)
            load_run(plate, cache_dir=args.cache_dir, refresh=args.refresh_sim)
        return 0
    for plate in selected:
        render_plate(
            plate,
            manifest=manifest,
            output_dir=args.output_dir,
            cache_dir=args.cache_dir,
            refresh=args.refresh_sim,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
