# FRET: Full-stack Robotic Effector Trajectories

<img src="docs/logo.png" alt="FRET Logo" width="200" align="left">

FRET is a robotics development project focused on end-to-end effector trajectory execution across simulation and physical hardware.
The project adopts a progressive validation strategy: Software-In-The-Loop (SITL), Hardware-In-The-Loop (HITL), and physical prototype operation.

Core objectives include reliable kinematic control, robust trajectory tracking, real-time communication between control layers, and future vision-based closed-loop autonomy.

> The [Project Roadmap](docs/roadmap.md) available in a dedicated file.

<br clear="all">

## System Specification

* **High-Level Controller:** Raspberry Pi 5 running Linux (Ubuntu).
* **Middleware:** ROS 2 (Humble/Jazzy) for high-level logic, kinematics, and communication.
* **Low-Level Controller:** Arduino Mega for deterministic actuation and signal processing.
* **Communication Layer:** Serial bridge (Micro-ROS or custom protocol) for command and telemetry exchange.
* **Simulation Stack:** URDF model with Gazebo/RViz for virtual validation.
* **Motion Planning:** Optimal (or near optimal) trajectory generation for pick-and-place applications.
* **Control Approach:** Jacobian-based trajectory tracking with feedback correction.
* **Mechanical/Electronic Baseline:** Stepper-class actuation (e.g., Nema 17) and precision drivers (e.g., TMC series).
* **Vision Expansion:** PiCam/Webcam integration for perception-driven replanning.

## Project Scope

FRET covers architectural design, simulation validation, hardware integration, physical calibration, and autonomous trajectory execution.
At this stage, the repository is dedicated to project definition and technical specification.
Tutorials and implementation-oriented documentation will be added incrementally as roadmap milestones are executed.

## Installing the development workspace

Install all build, simulation and flashing dependencies on a Debian-based system.
```bash
./scripts/setup.sh
```
