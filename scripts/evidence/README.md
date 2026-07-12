# Evidence scripts

Optional scripts that generate **T3–T4** proof artifacts (charts, MuJoCo snapshots).

| Script | Purpose |
| --- | --- |
| `evidence_dubins_physics_seed.py` | Dubins physics flake vs seeded export; writes PNGs to `/opt/cursor/artifacts/dubins_physics_evidence/` |

Run:

```bash
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
python3 scripts/evidence_dubins_physics_seed.py
```

Agents: follow [`.cursor/skills/report-writing/SKILL.md`](../.cursor/skills/report-writing/SKILL.md).
