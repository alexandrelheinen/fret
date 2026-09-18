# Presentation stills (FR-SIM-14)

## Intent

The project website shows a stock photograph of somebody else's robot. This
feature replaces it with still frames produced by FRET's own MuJoCo
renderer, so that the thumbnail is the project's output rather than a
borrowed image.

## Scope

In scope: presentation-only MJCF wrappers that add cameras, fill light, and
renderer quality settings on top of two existing cells, plus a still
renderer that runs the scenario, searches that run for one instant, and
writes a single PNG per camera. The two cells are the OM-X SC-v13b
pick-and-place cell and the SC-v11 Dubins warehouse race. One composite is
also in scope, for the website card that has room for a single picture; it
is described in its own section below and is named so that it is never
taken for a render.

Out of scope: any change to the simulated cells themselves. Geometry,
materials, contact parameters, the scenes' own lights, the gate cameras,
and every `config/vision/` threshold stay exactly as they are. Also out of
scope, for the per-camera stills: overlays, annotations, compositing, and
any pixel-level edit after the renderer produced the frame. What the
renderer hands back is what lands in those files.

## The measured files this feature must not touch

These files are consumed by tests, the vision pipeline, and the release
render job. This feature treats them as read-only:

| File | Consumer |
|---|---|
| `src/fret/mjcf/omx_pick_place.xml` | `fret.vision` gate cameras, `simulate_pick_place`, showcase render |
| `src/fret/mjcf/dubins_race.xml` | `DubinsRaceRunner`, release showcase job, physics evidence scripts |
| `src/fret/mjcf/omx_desk_clutter.xml`, `omx_wall_maze.xml`, `omx_tabletop.xml` | OM-X scenario tests |
| `src/fret/mjcf/omy_*.xml` | OMY scenario and release tests |
| `src/fret/config/vision/*.yml` | `fret.vision` HSV thresholds |
| `src/fret/config/scenarios/*.yml` | FSM waypoints, scenario tests, release durations |
| `src/fret/config/release/showcase.yml` | release render matrix |

Lighting is the sharp edge here. `hsv_blob_overhead.yml` thresholds at
`hsv_lower: [5, 80, 80]`, so it needs saturation and value of at least 80.
Raising or moving a light inside `omx_pick_place.xml` shifts the value
channel seen by `gate_cam_left` and `gate_cam_right`, and the pipeline then
returns fewer `BallObservation` values with nothing raising. The
presentation lights therefore live in separate scenes that the gate cameras
never load.

## Design

`src/fret/mjcf/omx_pick_place_thumbnail.xml` and
`src/fret/mjcf/dubins_race_thumbnail.xml` are wrapper models. Each
`<include>`s its cell and appends its own cameras, fill lights, and
`<visual>` settings. MuJoCo merges the two, so the cell arrives unmodified
and the presentation elements exist only for callers that load the wrapper.
Nothing in `fret.vision`, the scenario runners, or the release matrix loads
them.

`scripts/render_still.py` reuses `scripts/render_mujoco.py` for the MuJoCo
import, the pose application, and the seeded race simulation, and reuses
`fret.control.pick_place_sim.simulate_pick_place` for the arm motion. It
adds only the still path: run the scenario, search the recorded samples,
apply the winning sample, render one frame per camera, and write PNG. It
does not call the video path's CV ghost placement or its gate-cam overlay,
so nothing is drawn that the plain scene does not already carry.

The renderer was added beside `render_mujoco.py` rather than inside it
because that script's CLI makes `--fps`, `--duration`, `--collision-backend`
and `--planner-algorithm` mandatory, and its showcase functions return video
results. Loosening those contracts would reach into the release render path.

### Choosing the instant

For the OM-X cell, the still is the last recorded sample of the FSM state
`DESCEND_PICK`: the arm committed to the approach, gripper open, one step
short of the grasp. The state is searched in the run; no joint vector is
typed in.

For the Dubins race, each camera is fixed in the wrapper and the instant
moves. Every recorded instant is scored by how far inside the frame the
worse-placed AGV sits, the instants are ranked by that score, and the
renderer walks the ranking until a segmentation probe shows both AGVs
unoccluded. Occlusion is judged on pixel count times the square of the
viewing distance, which is the same for both agents when neither is hidden,
because both are the same TurtleBot3. A rack standing in front of one of
them therefore rules that instant out instead of producing a still where one
AGV is a sliver behind a pallet.

### What counts as the subject

AC-STILL-06 measures framing against a segmentation render, so "subject"
needs a definition per scene. The ground plane and anything the renderer
draws fully transparent (the SC-v13b catcher funnel, the CV ghost) are
always backdrop. Beyond that:

- OM-X: the `gate_*` geoms are backdrop. The v1.4 vision gate is a 0.72 m
  portal that the arm stands inside, so it crosses the frame in any shot
  close enough to read; it is staging, not the thing being framed.
- Dubins: the subject is the two racing AGVs, `car_rrt` and `car_sst`.
  The warehouse racks, pallets, and tables are what the AGVs weave through,
  and the grey dummy foil is neither planner's result.

### Renderer settings, and why shadows are off

Both wrappers set `<quality shadowsize="0" offsamples="8"/>`. The release
clips render at 720p, where each scene's shadow map holds up. At 1920x1080
the same map breaks into acne stripes across the far floor, and no
combination of `shadowsize` and `<map shadowscale>` removed them: raising
the map resolution left the stripes untouched, and tightening the frustum
far enough to help also deleted the contact shadow under the ball. Shadow
mapping is therefore off for stills. It is a renderer quality setting, in
the same class as the frame size, and the choice is recorded here rather
than left for a reader to notice in a diff.

Both wrappers also override `<headlight>`. Each scene's key light points
straight down, which lands on the floor and leaves the black OM-X castings
and the small TurtleBot3 shells reading as silhouettes. Trading headlight
diffuse for ambient and specular, and doing the modelling with two grazing
fills, keeps the floor off the blowout point while the robots stay legible.
The OM-X is a matte black robot and this lights it; it does not repaint it,
which is why its subject pixels sit lower than the frame mean.

## The composed project thumbnail

The website card wants one picture, and this project has two robots. The
composer, `scripts/make_project_thumbnail.py`, cuts two stills along a
straight diagonal seam whose endpoints average 0.5, so the two halves have
exactly equal area. **The result is a composite, not a render**, and it is
written under its own name so the two are never confused. Nothing inside
either half is retouched: each half is the delivered PNG, panned and
cropped, and the only marks added are the seam.

Placing a subject against a sloped edge needs more than a bounding box,
because every one of these subjects is narrow at one end and wide at the
other. `render_still.py` therefore writes an `extent_profile` into its
metrics JSON: the subject's horizontal extent measured band by band down
the frame, from the same segmentation render as the bounding box. The
composer checks each band against the seam and against the outer border,
and fails rather than cut a robot.

The OM-X cell needed one more thing. At `fovy` 34 the arm fills half a
frame edge to edge, so the wrapper carries `thumb_split`: the same eye and
the same lighting as `thumb_pick_three_quarter`, with the field pulled back
to 46 degrees. It is a fourth camera, not a fourth point of view, and only
the composer loads it.

House style comes from the site's own CSS. Cards there are flat: square
corners, no shadow, a hairline border, a 16:9 poster plate. The accent
shard (`--shard-beta` at `--cubist-shard-opacity`) belongs to
`.projects-plate--mark`, the glyph plate a project falls back to when it
has no thumbnail, so a real poster gets no shard; `--shard` can draw one
anyway. The site also applies `filter: saturate(0.92) contrast(1.02)` to
poster images itself, so no tone adjustment is baked into the file.

## Acceptance criteria

**AC-STILL-01:** When the still renderer runs, the shared scene and config
files listed above shall be byte-identical before and after.

**AC-STILL-02:** When a camera is requested, it shall resolve from a
presentation wrapper only, and each wrapper shall expose at least three
cameras at distinct positions and viewing directions.

**AC-STILL-03:** When the still renderer selects a frame, it shall select it
by searching the recorded run, never by a hand-entered joint vector or a
hand-placed agent.

**AC-STILL-04:** When a still is written, it shall be a PNG of exactly the
requested pixel size, 8-bit RGB with no alpha channel, tagged sRGB.

**AC-STILL-05:** When a still is written, the renderer shall report measured
tone statistics, and the frame shall sit in the mid-tones: mean luma between
60 and 200, under 2 percent of pixels crushed at or below 8, and under
2 percent blown at or above 247.

**AC-STILL-06:** When a still is written, the scene's subject shall fall
inside the central 85 percent of the frame, measured from a segmentation
render rather than by eye.

**AC-STILL-07:** When the same command runs twice, the two PNG files shall
be byte-identical.

## Constraints

- Output is 1920 x 1080. The OM-X cell's `<global offwidth/offheight>` is
  1280 x 720, so the wrapper raises the offscreen framebuffer rather than
  the shared scene.
- Rendering needs EGL: `MUJOCO_GL=egl PYOPENGL_PLATFORM=egl`.
- The Dubins still needs the full seeded race, which takes roughly two
  minutes of wall clock before the first frame is rendered.
- Pillow arrives with `imageio`, already in the `sim` extra, so the still
  path adds no new dependency.
- PNG color space is declared with the one-byte `sRGB` chunk rather than an
  embedded ICC profile, because an ICC profile carries a creation timestamp
  and would break AC-STILL-07.

## Reproducing

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python3 scripts/render_still.py \
    --scenario omx_pick_place --all-cameras \
    --output-dir artifacts/stills --width 1920 --height 1080 \
    --metrics-json artifacts/stills/omx_pick_place_stills.json
```

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python3 scripts/render_still.py \
    --scenario dubins_race --all-cameras \
    --output-dir artifacts/stills --width 1920 --height 1080 \
    --metrics-json artifacts/stills/dubins_race_stills.json
```

The OM-X cycle carries no RNG. The Dubins race runs under
`SHOWCASE_PLANNER_RNG_SEED = 5` from `fret.scenario.planner_rng`, the same
seed the release physics clips use.

## Composing the thumbnail

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python3 scripts/render_still.py \
    --scenario omx_pick_place --camera thumb_split \
    --output-dir artifacts/stills/wide --width 2560 --height 1080 \
    --metrics-json artifacts/stills/wide/omx_split.json
```

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python3 scripts/render_still.py \
    --scenario dubins_race --camera thumb_aisle_pursuit \
    --output-dir artifacts/stills/wide --width 2560 --height 1080 \
    --metrics-json artifacts/stills/wide/dubins_race_stills.json
```

```bash
python3 scripts/make_project_thumbnail.py \
    --left artifacts/stills/wide/omx_pick_place_thumb_split.png \
    --left-metrics artifacts/stills/wide/omx_split.json \
    --left-camera thumb_split \
    --right artifacts/stills/wide/dubins_race_thumb_aisle_pursuit.png \
    --right-metrics artifacts/stills/wide/dubins_race_stills.json \
    --right-camera thumb_aisle_pursuit \
    --seam-tilt 0.12 --clearance 55 --edge-margin 30 \
    --left-offset -800 --right-offset 200 \
    --output artifacts/stills/fret_project_thumbnail.png
```

The sources are rendered wider than the canvas so each half has room to be
panned. The two offsets above were chosen by eye, from the candidates the
solver accepted, to keep the grey dummy foil off the right border; drop
both flags to let the solver place them.

## Traceability

Requirement `FR-SIM-14` in [requirements.md](requirements.md). Tests in
`tests/simulation/test_render_still.py` name the `AC-STILL-*` ids they
guard.
