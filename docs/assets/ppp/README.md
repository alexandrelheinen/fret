# PPP gantry reference images

Vendor product images included **for mechanical layout and axis convention
documentation only**. They are not FRET-owned assets and are not redistributed
for commercial use.

| File | Source URL | Purpose |
|---|---|---|
| `cartesian-robot-reference.jpg` | https://ecdn6.globalso.com/upload/p/2394/image_other/2024-09/cartesian-robot-1-1.jpg | Overhead dual-rail Cartesian gantry appearance |
| `gantry-xyz-axes-reference.png` | https://cdn.goodao.net/fuyumotion/Gantry-Robot-Linear-Motion-System-XYZ-Positioning-Stage.png | X / Y / Z slide labelling |

Used in:

- [docs/robots/ppp.md](../../robots/ppp.md)
- [docs/mujoco_for_dummies.md](../../mujoco_for_dummies.md)

FRET's MuJoCo scene (`src/fret/mjcf/ppp_warehouse.xml`) reproduces this
topology with procedural geoms because no equivalent public MJCF mesh kit was
found (see mujoco_for_dummies.md § Visual assets and realism).
