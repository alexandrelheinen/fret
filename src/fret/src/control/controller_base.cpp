#include "fret/control/controller_base.hpp"

#include <algorithm>
#include <cmath>

namespace fret::control {

namespace {

constexpr double NEAR_ZERO_AXIS_LENGTH_SQ = 1e-12;

} // namespace

void ControllerBase::set_reference(
    const geometry_msgs::msg::PoseStamped &pose) {
    reference_pose_ = pose;
    has_reference_ = true;
}

std::vector<double> ControllerBase::compute_command(
    const sensor_msgs::msg::JointState &joint_state,
    const geometry_msgs::msg::TransformStamped &ee_transform) {
    update_ee_state(ee_transform);
    return do_compute_command(joint_state, ee_transform);
}

void ControllerBase::update_ee_state(
    const geometry_msgs::msg::TransformStamped &ee_transform) {
    const auto &t = ee_transform.transform.translation;
    const auto &q = ee_transform.transform.rotation;
    tf2::Vector3 position(t.x, t.y, t.z);
    tf2::Quaternion orientation(q.x, q.y, q.z, q.w);

    if (has_last_ee_) {
        rclcpp::Time now = ee_transform.header.stamp;
        rclcpp::Duration dt = now - last_ee_time_;
        if (dt.seconds() > 0.0) {
            last_ee_linear_vel_ = (position - last_ee_pos_) / dt.seconds();
            tf2::Quaternion dq = last_ee_orientation_.inverse() * orientation;
            dq.normalize();
            double angle =
                2.0 * std::acos(std::max(-1.0, std::min(1.0, dq.w())));
            if (angle > M_PI) {
                angle -= 2.0 * M_PI;
            }
            tf2::Vector3 axis(dq.x(), dq.y(), dq.z());
            if (axis.length2() > NEAR_ZERO_AXIS_LENGTH_SQ) {
                axis.normalize();
            }
            last_ee_angular_vel_ = axis * (angle / dt.seconds());
        }
    }

    last_ee_pos_ = position;
    last_ee_orientation_ = orientation;
    last_ee_time_ = ee_transform.header.stamp;
    has_last_ee_ = true;
}

} // namespace fret::control
