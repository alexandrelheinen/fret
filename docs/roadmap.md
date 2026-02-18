# FRET Project Roadmap

## Project goals

* **High-Level Controller:** Raspberry Pi 5 running Linux (Ubuntu) with the the high level task planning, motion planning, navigation, and control real time processes.
* **Middleware:** ROS 2 (Humble/Jazzy) for logic, kinematics, and communication.
* **Low-Level Controller:** Arduino Mega for real-time actuation and signal conversion to physical layer for the initial prototyping. To be reviewed and replaced by an actual driver afterwards.
* **Communication:** Serial-based bridge Micro-ROS for data exchange between RPi 5 and Arduino.
* **Topology**:  A SCARA-like robot (RRP) will be used for the SITL setup. The topology of the physical prototype will be determined at the step 4 depending on practical constraints. The simulation environment must extended to the new reality.

> The current system specification must be kept updated at the main project [README file](README.md), as decision are being made.

## Technical Roadmap

### 1. SITL (Software-In-The-Loop) Setup
* Configure the development environment on Linux.
* Create an initial robot's physical description (URDF) for a SCARA robot (initial setup)
* Launch the simulation environment (Gazebo/RViz) to validate the virtual model.

> Status: WIP

### 2. Intermediate Level Trajectory Implementation
* Implement the Kinematics Engine (Jacobian-based control).
* Execute predefined trajectories (straight lines, circles, etc.) in the simulator.
* Implement feedback terms to correct position errors and reject model disturbances.

> Status: TODO

### 3. HITL (Hardware-In-The-Loop) Setup
* Establish the communication link between RPi 5 and Arduino Mega.
* Validate the control stack by sending commands to the Arduino (without active drivers).
* Test telemetry loopback to verify data integrity and latency.

> Status: TODO

### 4. Hardware Procurement
* Define and purchase remaining mechanical and electronic components. This will defined the actual robot topology
* Focus on reliable actuators (e.g., Nema 17) and precise drivers (e.g., TMC series).
* Adapt the simulation stack to the new reality, implementing a simulation environment reproducing the actual prototype

> Status: TODO

### 5. Motion Planner Implementation
* Develop the high-level logic for path planning.
* Transition from hardcoded targets to dynamic trajectory generation.
* Deploy the full pipeline in the SITL and HITL setups.

> Status: TODO

### 6. Physical Prototype Assembly
* Finalize the mechanical build (e.g., 4-bar linkage or prismatic stages).
* Calibrate the low-level firmware constants against the physical hardware.
* Execute the first real-world trajectories.
* Replace or not the Arduino as low level driver by a dedicated, more compact, board.

> Status: TODO

### 7. Vision Integration
* Close the loop using optical sensors (PiCam/Webcam).
* Implement detection and real-time replanning based on visual feedback.
* Transition from known targets to autonomous object manipulation.

> Status: To be defined