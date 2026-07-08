# FRET Robot Models

Each FRET release targets one robot class. Models are selectable via `model:=` at launch.

| Model | Release | Doc | Status |
|---|---|---|---|
| `ppp` | v1.0 | [ppp.md](ppp.md) | ✅ Shipped |
| `dubins` | v1.1 | [dubins.md](dubins.md) | ✅ Shipped |
| `rrp` / `scara` | v1.2 | [rrp.md](rrp.md) | 🟡 Bootstrap exists |
| `six_dof` | v1.3 | [six_dof.md](six_dof.md) | 🔲 Planned |

The bootstrap `scara` model (RRP, 3-DOF) validated MS-1–5 and remains in the codebase
as the foundation for v1.2.
