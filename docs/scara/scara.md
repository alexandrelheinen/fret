# SCARA Robot Model: RSR6600-Inspired

This document specifies the FRET SCARA robot model, which is geometrically inspired by the [HiCNC RSR6600](https://www.hcnc-group.com/industrial-robot/scara-robot/4-axis-scara-robot.html).
It serves as the reference for the URDF model in `src/fret/urdf/scara.xacro`.

> **Unit convention:** This document uses **millimeters** for human readability, consistent with the manufacturer datasheet. The URDF uses **SI units (meters)**.

---

## 1. Global Coordinate System

| Attribute | Value |
| --- | --- |
| Origin $O_0$ | Intersection of $J_1$ axis and mounting plate bottom face |
| $+X$ | Forward (aligned with Arm 1 at $0°$) |
| $+Z$ | Upward (normal to mounting plane) |
| $+Y$ | Right-hand rule |

---

## 2. Kinematic Structure

The robot is a 4-DOF **R-R-P-R** chain (Revolute – Revolute – Prismatic – Revolute).

```
Base  →[J1 R]→  Link 1  →[J2 R]→  Link 2  →[J3 P]→  Quill  →[J4 R]→  End Effector
```

---

## 3. Link Geometry

All dimensions in **mm**.

### Link 0 — Fixed Base

| Feature | Value |
| --- | --- |
| Mounting plate | $195 \times 169 \times 10$ ($x, y, z$) |
| Pedestal front | Semi-cylinder, $R = 60$, centered on $J_1$ axis |
| Pedestal rear | Rectangular box, $W = 120$, extending to plate rear |
| Total height | $166.5$ (plate + pedestal) |
| $J_1$ position | $(0,\ 0,\ 166.5)$ relative to $O_0$ |

### Link 1 — Inner Arm (driven by $J_1$)

| Feature | Value |
| --- | --- |
| Shape | Stadium (pill): central box + two semi-cylinders ($R = 50$) |
| Kinematic length $L_1$ | $325$ (axis-to-axis) |
| Physical length | $425$ |
| Height | $71.5$ |
| Assembly | Bottom face flush with base top ($Z = 166.5$) |

### Link 2 — Outer Arm (driven by $J_2$)

| Feature | Value |
| --- | --- |
| Lower plate | Stadium identical to Link 1 ($R = 50$, height $71.5$) |
| Kinematic length $L_2$ | $275$ (axis-to-axis) |
| Headpiece shape | Trapezoidal wedge — flat top at $H = 133$ (rear), tapering linearly to $H = 70$ over the front $159\ \text{mm}$, pill cross-section ($R = 50$); modelled as `headpiece_wedge.stl` with semi-cylinder end-caps |
| — Flat-top section (proximal) | $H = 133$, length $= L_2 - 159 = 116\ \text{mm}$ (from $J_2$ axis to slope start) |
| — Sloped section (distal) | $H$ ramps from $133$ to $70$ over $159\ \text{mm}$ to the $J_3$ axis |
| Quill guard | Cylinder $\varnothing 48$ ($R = 24$), centred on $J_3$ axis, spans from $5\ \text{mm}$ below lower-plate bottom to top of rear block ($H_{\text{guard}} \approx 209.5\ \text{mm}$) |
| Assembly | Bottom face flush with Link 1 top ($Z = 238$) |

### Link 3 — Quill (driven by $J_3$ and $J_4$)

| Feature | Value |
| --- | --- |
| Shape | Two-part: shaft cylinder $R = 10$, height $= 437.5$ + top disk $\varnothing 42$ ($R = 21$), thickness $= 5$ |
| Total height | $442.5$ |
| $J_3$ axis | $-Z$ (positive stroke extends **downward**) |
| Home pose ($J_3 = 0$) | Quill top face flush with Link 2 top face (fully retracted) |
| Full extension ($J_3 = 200\ \text{mm}$) | Quill bottom face $200\ \text{mm}$ below Link 2 bottom face |

> **Sign convention:** Increasing $J_3$ moves the end effector **downward** toward the workpiece, consistent with industrial SCARA standards.

---

## 4. Joint Parameters

All angles in degrees, distances in mm.

| Joint | Type | Axis | Range | Max Speed |
| --- | --- | --- | --- | --- |
| $J_1$ | Revolute | $+Z$ | $\pm 132°$ | $375°/s$ ($6.54\ \text{rad/s}$) |
| $J_2$ | Revolute | $+Z$ | $\pm 150°$ | $600°/s$ ($10.46\ \text{rad/s}$) |
| $J_3$ | Prismatic | $-Z$ | $0$ to $200\ \text{mm}$ | $1111\ \text{mm/s}$ |
| $J_4$ | Revolute | $+Z$ | $\pm 360°$ | $2000°/s$ ($34.88\ \text{rad/s}$) |

> Acceleration limits are not published in the manufacturer datasheet. They will be identified experimentally during HITL calibration (roadmap step 3).

---

## 5. DH Parameters

Standard Denavit–Hartenberg convention. Distances in meters.

| $i$ | Joint | $\theta_i$ | $d_i$ (m) | $a_i$ (m) | $\alpha_i$ |
| --- | --- | --- | --- | --- | --- |
| 1 | $J_1$ | $\theta_1^*$ | $0.1665$ | $0.325$ | $0$ |
| 2 | $J_2$ | $\theta_2^*$ | $0$ | $0.275$ | $\pi$ |
| 3 | $J_3$ | $0$ | $d_3^*$ | $0$ | $0$ |
| 4 | $J_4$ | $\theta_4^*$ | $0$ | $0$ | $0$ |

$d_1 = 0.1665$ m is the base height (offset from world origin to $J_1$).
$\alpha_2 = \pi$ flips the $Z$-axis so that the prismatic joint extends downward.
Variables are marked with $^*$.

---

## 6. Workspace Envelope

### Horizontal (XY plane)

The reachable region is an annular sector centered at $J_1$:

| Limit | Formula | Value |
| --- | --- | --- |
| Maximum reach | $L_1 + L_2$ | $600\ \text{mm}$ |
| Minimum reach (fully folded) | $\|L_1 - L_2\|$ | $50\ \text{mm}$ |
| Angular sweep | $2 \times 132°$ | $264°$ |

> The maximum reach of $600\ \text{mm}$ matches the manufacturer's stated working radius.

### Vertical (Z axis)

The end effector height above the base plate:

| Condition | Height above $O_0$ |
| --- | --- |
| $J_3 = 0$ (retracted) | $238\ \text{mm}$ |
| $J_3 = 200\ \text{mm}$ (extended) | $38\ \text{mm}$ |

---

## 7. Dynamics

| Property | Value |
| --- | --- |
| Total body mass | $21.5\ \text{kg}$ |
| — Base (Link 0) | $40\%\ (8.6\ \text{kg})$ |
| — Link 1 | $30\%\ (6.45\ \text{kg})$ |
| — Link 2 | $20\%\ (4.3\ \text{kg})$ |
| — Link 3 (quill) | $10\%\ (2.15\ \text{kg})$ |
| Nominal payload | $6\ \text{kg}$ |
| $J_4$ allowable inertia | $0.12\ \text{kg} \cdot \text{m}^2$ (with payload) / $0.01\ \text{kg} \cdot \text{m}^2$ (no payload) |
| $J_4$ allowable torque | $4.23\ \text{N} \cdot \text{m}$ |

---

## 8. Electrical & Environmental

| Property | Value |
| --- | --- |
| Rated power | $0.58\ \text{kW}$ |
| Supply | Single-phase AC $220\ \text{V}$ / $3.1\ \text{A}$ |
| Protection class | IP54 |
| Operating temperature | $0°\text{C}$ to $45°\text{C}$ |
| Operating humidity | $20\%$ to $80\%$ (non-condensing) |
| Mounting | Floor, ceiling (inverted), wall |

---

## 9. Repeatability

| Metric | Value |
| --- | --- |
| Repeat positioning accuracy | $\pm 0.02\ \text{mm}$ |
| Fastest cycle time | $0.4\ \text{s}$ |

> The manufacturer also claims $\pm 0.01\ \text{mm}$ end-effector repeatability using high-precision harmonic reducers and ball screw/spline assemblies. The $\pm 0.02\ \text{mm}$ figure from the datasheet is used as the conservative design target.
