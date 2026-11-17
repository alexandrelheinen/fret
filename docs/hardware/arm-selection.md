# Arm selection for the v2.x hardware line

Status: decision record (PR2, `v2.0.0`).
Prices, shipping and stock were read on seller pages on 2026-11-16 for delivery to
Paris, France, and will drift; treat them as a dated snapshot. Exchange rate: ECB,
1 EUR = 1.1014 USD (2026-11-16).

## Decision

**Seeed Studio SO-ARM101 Pro, assembled, bought from OpenELAB, 388.45 EUR delivered to
Paris** (VAT included, free EU shipping above 100 EUR, duties paid by the seller). It
arrives assembled with a leader and a follower arm, a 32 mm USB camera, two servo driver
boards, 5 V and 12 V supplies and four table clamps. The follower uses Feetech STS3215
servos (12 V, 1:345 gearing, 12-bit absolute encoders, 1 Mbps serial bus), which already
have a ROS 2 Jazzy ros2_control driver and a MuJoCo Menagerie model. It ranked first
under all three weighting profiles below.

Runner-up: Komatachi SO-101 follower, assembled, 12 V (Budapest, EU), 369 EUR plus 15
to 20 EUR shipping, no camera.

The arm is controlled by a Raspberry Pi 5 (arm64, Ubuntu 24.04, ROS 2 Jazzy), not by the
development PC. See `docs/hardware/deployment-rpi5.md` (added in v2.3).

## Requirements used for the comparison

Must-haves (R) are pass/fail; nice-to-haves (N) feed the scores.

| ID | Requirement | Type | Basis in FRET |
|---|---|---|---|
| R1 | Pick a 40 mm ball (about 13 g) from the table and drop it in a fixed bin | Must | `config/scenarios/omx_pick_place_cv.yml`, `mjcf/omx_pick_place.xml` |
| R2 | Reach of at least 0.32 m with a top-down approach | Must | OM-X scenario pick points up to 0.316 m from the base |
| R3 | Payload of at least 50 g | Must | Ball mass computed from the MJCF |
| R4 | At least 4 joints plus a gripper; full 6D orientation not needed | Must | 4-DOF OM-X passes V14-* in `docs/releases.md` |
| R5 | Absolute joint positions on `/joint_states` at 50 Hz or more | Must | FR-HW-02; `update_rate: 50.0` in controller configs |
| R6 | Accept commands at 50 Hz; velocity-to-position conversion if the servos are position-only | Must | FR-CTL-03 |
| R7 | Direct USB link to the host, no intermediate microcontroller firmware | Must | Replaces the earlier Micro-ROS/Arduino assumption |
| R8 | Fixed UVC camera over the table, 1280x720 or more, intrinsic and extrinsic calibration | Must | `config/vision/omx_portal_overhead.yml` |
| R9 | Clamps to a workbench; supply and controller included; little assembly | Must | Maintainer request |
| R10 | 500 EUR or less delivered to Paris (price, shipping, VAT, duties, fees) | Must | Maintainer request |
| R11 | Vendor with no military or dual-use orientation | Must | Maintainer policy |
| N1 | Ready ROS 2 Jazzy driver | Nice | |
| N2 | MuJoCo model of the exact arm | Nice | FR-SIM-04 |
| N3 | Precision compatible with 5 mm end-effector error | Nice | FR-CTL-02 |
| N4 | Assembled and tested, camera included, documentation and community | Nice | |
| N5 | Hooks for e-stop, watchdog, joint and torque limits | Nice | |

## Candidates and delivered cost

EU sellers show VAT-inclusive prices and charge no customs. For imports from China or
the USA the estimate is price + shipping (estimated 40 EUR from China, 50 EUR from the
USA) + 1.7 % duty (TARIC 8479 50 00, classification uncertain) + 20 % import VAT on goods,
shipping and duty + about 15 EUR carrier fee. Estimates are marked "est.".

| Kit (seller, origin) | Listed price | Delivered, Paris | Stock on 2026-11-16 |
|---|---|---|---|
| Seeed SO-ARM101 Pro, assembled (OpenELAB, EU warehouse, DDP) | 388.45 EUR incl. VAT | 388.45 EUR (391.71 EUR if 20 % instead of 19 % VAT is applied at checkout) | Available |
| Komatachi SO-101 follower, assembled 12 V (Budapest) | 369 EUR incl. VAT | 384 to 389 EUR; about 424 to 429 EUR with a camera (est.) | Available |
| WowRobo SO-101 Package 3, assembled (China) | 299 USD = 271.47 EUR | about 395 EUR (est.) | Available |
| Waveshare SO-ARM101 CAM Kit SE (Amazon.fr) | 444.49 EUR incl. VAT | 444.49 EUR | 5 left |
| Hiwonder SO-ARM101 Standard (Shenzhen, FOB, no IOSS) | 419.99 USD = 381.32 EUR | about 529 EUR (est.) | Available |
| Waveshare RoArm-M3-S (Amazon.fr) | 312.99 EUR | 312.99 EUR | In stock |
| Waveshare RoArm-M2-S (Amazon.fr) | 246.99 EUR | 246.99 EUR | 14 left |
| ROBOTIS Koch v1.1 follower kit (USA) | 212.92 USD = 193.32 EUR, no printed parts | about 312 EUR (est.) plus printed parts | Base kit only |
| Hiwonder xArm 1S, assembled (Shenzhen) | 239.99 USD = 217.90 EUR | about 330 EUR (est.) | Available |

What each kit includes, and the watch-outs:

| Kit | Contents | Watch-outs |
|---|---|---|
| Seeed Pro (OpenELAB) | Leader and follower, assembled; 32 mm USB camera; 2 driver boards; 5 V and 12 V supplies; 4 table clamps | 12 V supply listed as 2 A; camera resolution not found |
| Komatachi follower | Follower only; STS3215 12 V; Waveshare driver board; 12 V / 5 A supply; clamp | No camera; small, recent shop |
| WowRobo Package 3 | Leader and follower, assembled, 1 camera | Import VAT and duties collected on delivery |
| Waveshare CAM Kit SE | ST3215 12 V follower; encoder-only leader; printed parts; 2 cameras | Self-assembly |
| Hiwonder Standard | Leader and follower, assembled; wrist camera | HX-30HM/HX-10HM servos: no ROS 2 driver found; LeRobot fork only |
| RoArm-M3-S / M2-S | Assembled ESP32-based arms | Vendor repeatability of about 4 to 5 mm; JSON commands through ESP32 firmware |
| Koch v1.1 | Dynamixel XL430/XL330 | Parts must be printed; needs a 12 V to 6 V converter |
| xArm 1S | Assembled | Legacy bus servos; no ROS 2 driver or model found |

## Scored comparison

Each criterion is scored 0 to 5; total = sum of weight x score / 5, out of 100.

| # | Kit | Delivered | ROS 2 driver | MuJoCo/URDF model | Feedback/bridge | Reach, payload, DOF | Precision | Bench-ready | Docs/community | Cost/logistics | Balanced | FRET-first | Bench-first |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Seeed SO-ARM101 Pro | 388 EUR | 5 | 5 | 4 | 4 | 3 | 5 | 5 | 5 | 92 | 91 | 95 |
| 2 | Komatachi SO-101 follower | about 424 to 429 EUR | 5 | 5 | 4 | 4 | 3 | 4 | 4 | 4 | 83 | 85 | 82 |
| 3 | WowRobo SO-101 Package 3 | about 395 EUR | 5 | 5 | 4 | 4 | 3 | 4 | 4 | 3 | 80 | 83 | 79 |
| 4 | Waveshare SO-ARM101 CAM Kit SE | 444 EUR | 5 | 5 | 4 | 4 | 3 | 2 | 4 | 4 | 75 | 81 | 68 |
| 5 | Waveshare RoArm-M3-S | 313 EUR | 3 | 2 | 2 | 5 | 1 | 4 | 4 | 5 | 68 | 62 | 76 |
| 6 | Waveshare RoArm-M2-S | 247 EUR | 3 | 2 | 2 | 3 | 2 | 4 | 4 | 5 | 66 | 60 | 73 |
| 7 | Hiwonder SO-ARM101 Standard | about 529 EUR | 1 | 3 | 4 | 4 | 3 | 5 | 3 | 1 | 60 | 57 | 67 |
| 8 | ROBOTIS Koch v1.1 follower | about 312 EUR | 4 | 2 | 5 | 2 | 4 | 1 | 3 | 3 | 57 | 63 | 48 |
| 9 | Hiwonder xArm 1S | about 330 EUR | 1 | 1 | 2 | 3 | 1 | 4 | 3 | 4 | 51 | 43 | 61 |

| Profile | ROS 2 driver | Model | Feedback | Reach/payload/DOF | Precision | Bench-ready | Docs | Cost | Sum |
|---|---|---|---|---|---|---|---|---|---|
| Balanced | 15 | 10 | 10 | 10 | 10 | 20 | 10 | 15 | 100 |
| FRET-first | 20 | 15 | 15 | 10 | 10 | 10 | 10 | 10 | 100 |
| Bench-first | 10 | 5 | 5 | 10 | 5 | 35 | 15 | 15 | 100 |

## Excluded options

| Arm | Price checked | Reason |
|---|---|---|
| ROBOTIS OpenMANIPULATOR-X (RM-X52-TNM) | 1,542.69 EUR incl. VAT | About 3x the budget, although it matches the FRET OM-X simulation |
| Elephant Robotics myCobot 280 M5 | 735.00 EUR | Over budget; 280 mm reach is below 0.32 m |
| Elephant Robotics mechArm 270 Pi | 799 USD | Over budget |
| DOBOT Magician Lite | 752 EUR | Over budget |
| Hiwonder JetArm Standard | 479.99 USD without Jetson | Needs a Jetson |
| WowRobo TwinArm | 580 / 630 USD | Over budget before taxes |
| PartaBot SO-ARM101 | 329 to 479 USD | Sold out; ships from the USA |
| Letmeknow SO-ARM101 Pro kit | 399.00 EUR | Out of stock; self-assembly |

Defense and dual-use screen: none of the vendors listed showed a military or defense
orientation on the pages read. This was a light check, not an audit.

## Known risks of the chosen arm

- The 12 V supply is listed as 2 A; six STS3215 at 12 V can draw more under load. Check the label on arrival and plan a 12 V / 5 A supply if needed.
- Camera resolution was not found; FRET's simulated gate cameras are 1280x720.
- PLA frame, up to 0.5 degrees of backlash per joint, and gravity sag of about 0.05 rad at 25 % load (measured by the `so101_ros2` project) make the 5 mm end-effector target of FR-CTL-02 unlikely on hardware without compensation. The hardware tolerance is set in v2.4 from measured data.
- The leader arm is paid for but not used by FRET.

## Sources

- OpenELAB, Seeed SO-ARM101 Pro assembled kit: https://openelab.io/products/seeed-studio-so-arm101-pro-assembled-kit-lerobot-6dof-robotic-arm
- OpenELAB shipping policy: https://openelab.io/pages/shipping-policy
- Komatachi shop: https://komatachi.com/shop
- WowRobo SO-ARM101: https://shop.wowrobo.com/products/so-arm101-diy-kit-assembled-version-1
- Waveshare SO-ARM101 CAM Kit SE on Amazon.fr: https://www.amazon.fr/dp/B0G6SG4H7T
- Hiwonder LeRobot SO-ARM101: https://www.hiwonder.com/products/lerobot-so-101
- Waveshare RoArm-M3-S: https://www.amazon.fr/dp/B0GF272HXW ; RoArm-M2-S: https://www.amazon.fr/dp/B0CMLNPH8V
- ROBOTIS Koch v1.1 follower: https://robotis.us/products/koch-v1-1-low-cost-robot-arm-follower
- Hiwonder xArm 1S: https://www.hiwonder.com/products/xarm-1s
- robotis.sk OpenMANIPULATOR-X: https://www.robotis.sk/132-rm-x52-tnm
- Welectron myCobot 280 M5: https://www.welectron.com/Elephant-Robotics-myCobot-280-Robot-Arm-M5-Stack
- Elephant Robotics mechArm 270: https://shop.elephantrobotics.com/products/mecharm
- Hiwonder JetArm: https://www.hiwonder.com/products/jetarm
- WowRobo TwinArm: https://shop.wowrobo.com/products/wowrobo-twinarm-robotic-arm-set-inspired-by-koch-v1-1
- Letmeknow SO-ARM101 Pro kit: https://letmeknow.fr/fr/kit/4227-kit-bras-robotique-so-arm101-pro-lerobot-pieces-imprimees-3d-642613896672.html
- MuJoCo Menagerie `robotstudio_so101` (Apache-2.0): https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101
- `so101_ros2` (ROS 2 Jazzy, ros2_control, BSD-3-Clause): https://github.com/adoodevv/so101_ros2
- TheRobotStudio SO-ARM100 (Apache-2.0): https://github.com/TheRobotStudio/SO-ARM100
- Hugging Face LeRobot (Apache-2.0): https://github.com/huggingface/lerobot
- ECB euro reference rates: https://www.ecb.europa.eu/stats/policy_and_exchange_rates/euro_reference_exchange_rates/html/index.en.html
- EU TARIC consultation: https://ec.europa.eu/taxation_customs/dds2/taric/taric_consultation.jsp
