# WSL2 — MuJoCo visualization tutorial

This tutorial helps you get **MuJoCo visuals working** when you develop FRET
inside **WSL2** on Windows. The main FRET stack (Ubuntu 24.04, ROS 2 Jazzy,
colcon, pytest) is designed for Linux; WSL2 is a practical way to run that stack
on a Windows machine without dual-booting.

If you use WSL2 as your daily environment — as many contributors do — OpenGL and
display forwarding are the usual pain points for the interactive MuJoCo viewer
and for local MP4 export. This guide covers what works, what to try first, and
sensible fallbacks.

**Why this matters for FRET:** the project targets Ubuntu + ROS 2. WSL2 lets
you keep that toolchain while staying on Windows for everything else (editor,
browser, hardware tools). For the motivation behind that choice, see
[Why I chose WSL for embedded development](https://alexandrelheinen.pages.dev/articles/2026-04-22-wsl/).

**Related:** [tutorial.md](tutorial.md) · [simulation.md](simulation.md)

---

## What you need rendering for

FRET uses **MuJoCo only** for 3D visualization. Not every workflow needs a
working GPU inside WSL.

| Goal | Command | Needs OpenGL in WSL? |
|---|---|---|
| Algorithm dev / unit tests | `pytest tests/ -v --ignore=tests/integration` | No |
| MJCF load check (no window) | `python3 scripts/view_mujoco.py --dry-run` | No |
| Full ROS SITL | `ros2 launch fret sitl.py …` | No (sim I/O only) |
| **Interactive 3D viewer** | `./scripts/view.sh` | **Yes** |
| **Local MP4 export** | `./scripts/video.sh -o demo.mp4` | **Yes** (EGL/GL) |
| **Watch a showcase MP4** | `./scripts/download_showcase.sh` | **No** |

Keep all **development** in WSL. Treat visualization as a separate concern with
the options below.

---

## Recommendation

1. **Try WSL2 rendering first** (Windows 11 + WSLg). This is the intended path
   and avoids maintaining two full environments.
2. **Use a thin Windows Python venv** only as an escape hatch for the
   interactive viewer — not a duplicate ROS workspace.
3. **Download CI videos or render MP4** when you only need to confirm how the
   scene looks, not orbit it live.

Do **not** set up a full Windows-side ROS 2 stack just for visualization. The
cost is high and FRET does not require it.

---

## Option A — WSL2 rendering (recommended first)

### Prerequisites

- **Windows 11** with [WSLg](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
  (integrated Linux GUI support). WSLg forwards display and GPU without VcXsrv.
- **WSL2** (not WSL1): `wsl --status` should report version 2.
- **GPU driver on Windows** with WSL support (NVIDIA, AMD, or Intel — install or
  update the vendor driver on the Windows host; WSL uses it through GPU
  paravirtualization).

### 1. Update WSL

In **PowerShell (admin)**:

```powershell
wsl --update
wsl --shutdown
```

Re-open your Ubuntu terminal after shutdown.

### 2. Install OpenGL / EGL libraries in Ubuntu

Headless MP4 rendering and some MuJoCo builds need system GL libraries. On
Ubuntu 24.04:

```bash
sudo apt install libegl1 libegl-mesa0 libgles2 libgl1 libgl1-mesa-dri libosmesa6 mesa-utils
```

These packages match the hint in `scripts/render_mujoco.py` when EGL init fails.

### 3. Sanity checks

```bash
echo "DISPLAY=$DISPLAY"           # WSLg: often :0
echo "WAYLAND_DISPLAY=$WAYLAND_DISPLAY"   # often wayland-0
glxinfo -B | head -20             # should show a real GPU renderer
```

If `glxinfo` reports only `llvmpipe` (software rendering), update your Windows
GPU driver and restart WSL before continuing.

### 4. Install FRET sim deps and test

```bash
cd ~/Workspace/fret   # or your clone path
pip install -e ".[sim]"

# No window — confirms MJCF + Python path
python3 scripts/view_mujoco.py --dry-run

# Interactive viewer
./scripts/view.sh

# Headless MP4
./scripts/video.sh -o /tmp/demo.mp4
```

If the viewer works but MP4 export fails, try EGL explicitly:

```bash
export MUJOCO_GL=egl
./scripts/video.sh -o /tmp/demo.mp4
```

### Troubleshooting (WSL2)

| Symptom | Likely fix |
|---|---|
| `cannot open display` | Run `wsl --update`, `wsl --shutdown`, reopen terminal; confirm WSLg on Win11 |
| Black / empty viewer | Update Windows GPU driver; reinstall Mesa packages above |
| `MuJoCo is required` | `pip install -e ".[sim]"` or `pip install mujoco` |
| `eglQueryString` / EGL init error | Install apt packages in step 2 |
| Viewer OK, MP4 fails | `export MUJOCO_GL=egl` then retry `./scripts/video.sh` |
| `mujoco.viewer is unavailable` | `pip install -U mujoco` |

Spend roughly one focused session on driver and apt fixes. If the viewer still
shows a black window after that, use Option B or C instead of fighting GL
further.

---

## Option B — Thin Windows Python (viewer only)

Use native Windows MuJoCo when WSL OpenGL keeps failing. You only need Python,
`mujoco`, and `numpy` — **no ROS on Windows**.

From **PowerShell**, using the same repo on the WSL filesystem:

```powershell
cd \\wsl.localhost\Ubuntu\home\alexandre\Workspace\fret

py -3.12 -m venv .venv-win
.\.venv-win\Scripts\activate
pip install mujoco numpy

python scripts\view_mujoco.py
python scripts\view_mujoco.py --duration 45
```

Alternatively, clone the repo under `C:\dev\fret` if you prefer a Windows-local
copy for viewing only.

Keep **build, test, and SITL** in WSL. Switch to this venv only when you want
the live MuJoCo window (orbit, pan, zoom).

---

## Option C — No local rendering

When you only need to **see** a release showcase — not interact with it —
skip GL entirely.

### Download CI / release MP4 (WSL-friendly)

Release CI renders two POV clips per scenario (PPP warehouse + Dubins race:
overview and follow) and uploads them to Cloudflare R2. See
[tutorial.md § Download from R2](tutorial.md#download-from-r2-wsl-friendly--no-mujoco-rendering-needed).

```bash
cp .env.example .env    # fill R2_* values once; .env is gitignored
sudo apt install awscli # if needed
./scripts/download_showcase.sh
./scripts/download_showcase.sh --scenario dubins_race
./scripts/download_showcase.sh --all
```

Default output: `artifacts/r2/ppp_warehouse_latest.mp4`. Open the file in any
Windows video player.

### Local MP4 when EGL works

If Option A partially works (EGL but no stable viewer), `./scripts/video.sh`
may still produce an MP4 you can open on Windows.

---

## Suggested workflow

```
Daily dev (WSL)     →  pytest, colcon, ros2 launch fret sitl.py …
Quick visual check  →  ./scripts/video.sh  OR  ./scripts/download_showcase.sh
Interactive explore →  ./scripts/view.sh (WSLg)  OR  Windows venv (Option B)
```

1. **Develop algorithms** — `pytest tests/ -v --ignore=tests/integration`
2. **Validate E2E** — integration tests and SITL in WSL
3. **Preview visually** — viewer or MP4 using the options above
4. **Export for demos** — `./scripts/video.sh` locally or rely on CI artifacts

---

## Decision summary

| Question | Answer |
|---|---|
| Full Windows ROS env for viz? | **No** — too much overhead |
| Try WSL2 GL first? | **Yes** on Windows 11 with WSLg |
| Windows escape hatch? | **Yes** — minimal Python venv for `view_mujoco.py` only |
| Zero GL on WSL? | **Yes** — `download_showcase.sh` or CI artifacts |

---

## File map

```
docs/wsl.md                  ← this guide
docs/tutorial.md             ← MuJoCo controls, R2 download, scene assets
scripts/view.sh              ← interactive viewer (WSL or Windows)
scripts/view_mujoco.py
scripts/video.sh             ← headless MP4 (needs working GL)
scripts/render_mujoco.py
scripts/download_showcase.sh ← MP4 without local rendering
src/fret/mjcf/ppp_warehouse.xml
```
