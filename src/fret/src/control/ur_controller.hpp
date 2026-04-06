/**
 * @file ur_controller.hpp
 * @brief Concrete controller for Universal Robots 6-DOF models.
 *
 * Private header — not part of the public package API.
 */
#ifndef FRET_CONTROL_UR_CONTROLLER_HPP
#define FRET_CONTROL_UR_CONTROLLER_HPP

#include <vector>

#include "fret/control/controller_base.hpp"

namespace fret::control {

/**
 * @brief Concrete controller for Universal Robots 6-DOF manipulators.
 *
 * This initial implementation returns zero velocity for every joint and
 * serves as a deterministic SITL validation stub.
 */
class UrController : public ControllerBase {
  public:
    /**
     * @brief Compute a velocity command for the current UR state.
     * @param joint_state Latest joint state received from /joint_states.
     * @param ee_transform Current end-effector transform (base frame -> EE
     * frame).
     * @return Ordered velocity command, zero for all joints in this stub.
     */
    std::vector<double> do_compute_command(
        const sensor_msgs::msg::JointState &joint_state,
        const geometry_msgs::msg::TransformStamped &ee_transform) override;
};

} // namespace fret::control

#endif
