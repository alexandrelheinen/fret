/**
 * @file scara_controller.hpp
 * @brief Concrete controller for the SCARA (R-R-P-R) robot model.
 *
 * Private header — not part of the public package API.
 * The controller self-registers in ControllerRegistry under the key "scara"
 * at static initialization time; no external code needs to include this file.
 */
#ifndef FRET_CONTROL_SCARA_CONTROLLER_HPP
#define FRET_CONTROL_SCARA_CONTROLLER_HPP

#include <vector>

#include "fret/control/controller_base.hpp"

namespace fret::control {

/**
 * @brief Concrete controller for the SCARA (R-R-P-R) robot model.
 *
 * Implements compute_command() for the four joints: joint_arm_0 (J1),
 * joint_arm_1 (J2), joint_extension (J3), and joint_tool_rotate (J4).
 *
 * This initial implementation returns zero velocity on all joints and serves
 * as the pipeline validation stub. The full Jacobian-based control law will
 * be added in a subsequent step.
 *
 * The controller is automatically registered in ControllerRegistry under the
 * key "scara" at static initialization time.
 */
class ScaraController : public ControllerBase {
  public:
    /**
     * @brief Compute a velocity command for the current SCARA state.
     * @param joint_state Latest joint state received from /joint_states.
     * @param ee_transform Current end-effector transform (base frame → EE
     * frame).
     * @return Ordered velocity command [J1, J2, J3, J4], zero for all joints
     *         in this stub implementation.
     */
    std::vector<double> do_compute_command(
        const sensor_msgs::msg::JointState &joint_state,
        const geometry_msgs::msg::TransformStamped &ee_transform) override;
};

} // namespace fret::control

#endif
