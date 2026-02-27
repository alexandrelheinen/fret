/**
 * @file controller_base.hpp
 * @brief Abstract base class for robot controllers.
 */
#ifndef FRET_CONTROL_CONTROLLER_BASE_HPP
#define FRET_CONTROL_CONTROLLER_BASE_HPP

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace fret::control {

/**
 * @brief Abstract base class for all robot controllers.
 *
 * Uses the Non-Virtual Interface (NVI) pattern: the public compute_command()
 * handles shared bookkeeping (EE state estimation) before delegating to
 * do_compute_command() implemented by each concrete controller.
 *
 * The node calls configure() once on startup, set_reference() whenever a
 * new target pose arrives, and compute_command() on every control cycle.
 */
class ControllerBase {
  public:
    virtual ~ControllerBase() = default;

    /**
     * @brief Initialize the controller with the ordered list of joint names.
     * @param joint_names Ordered joint names matching the command vector.
     */
    virtual void configure(const std::vector<std::string> &joint_names) {
        joint_names_ = joint_names;
    }

    /**
     * @brief Update the target end-effector pose.
     *
     * Called by the node whenever a new reference message arrives.
     * Concrete controllers read reference_pose_ and has_reference_ to
     * compute their error signal.
     *
     * @param pose Desired end-effector pose in the base frame.
     */
    void set_reference(const geometry_msgs::msg::PoseStamped &pose);

    /**
     * @brief Compute a velocity command for the current robot state (NVI).
     *
     * Updates internal EE state then delegates to do_compute_command().
     * Do not override — override do_compute_command() instead.
     *
     * @param joint_state   Latest joint state from /joint_states.
     * @param ee_transform  Current base-frame → EE-frame transform.
     * @return Ordered velocity command, one value per joint.
     */
    std::vector<double>
    compute_command(const sensor_msgs::msg::JointState &joint_state,
                    const geometry_msgs::msg::TransformStamped &ee_transform);

  protected:
    /**
     * @brief Override point for the concrete control law.
     *
     * Called by compute_command() after EE state has been updated.
     * Access last_ee_pos_, last_ee_linear_vel_, etc. for state feedback and
     * reference_pose_ / has_reference_ for the error signal.
     *
     * @param joint_state   Latest joint state.
     * @param ee_transform  Current base-frame → EE-frame transform.
     * @return Ordered velocity command, one value per joint.
     */
    virtual std::vector<double> do_compute_command(
        const sensor_msgs::msg::JointState &joint_state,
        const geometry_msgs::msg::TransformStamped &ee_transform) = 0;

    /** @brief Ordered joint names set by configure(). */
    std::vector<std::string> joint_names_;

    /** @brief Most recently set target EE pose. */
    geometry_msgs::msg::PoseStamped reference_pose_;
    /** @brief True once set_reference() has been called at least once. */
    bool has_reference_{false};

    /** @brief Last known EE position in the base frame. */
    tf2::Vector3 last_ee_pos_{0.0, 0.0, 0.0};
    /** @brief Last known EE orientation in the base frame. */
    tf2::Quaternion last_ee_orientation_{0.0, 0.0, 0.0, 1.0};
    /** @brief Timestamp of the last EE state update. */
    rclcpp::Time last_ee_time_;
    /** @brief True once the first EE transform has been received. */
    bool has_last_ee_{false};
    /** @brief Linear velocity of the EE estimated by finite difference. */
    tf2::Vector3 last_ee_linear_vel_{0.0, 0.0, 0.0};
    /** @brief Angular velocity of the EE estimated by finite difference. */
    tf2::Vector3 last_ee_angular_vel_{0.0, 0.0, 0.0};

  private:
    /**
     * @brief Update EE state from the latest transform.
     *
     * Computes linear and angular velocity by finite difference and stores
     * them in last_ee_linear_vel_ and last_ee_angular_vel_.
     */
    void
    update_ee_state(const geometry_msgs::msg::TransformStamped &ee_transform);
};

} // namespace fret::control

#endif
