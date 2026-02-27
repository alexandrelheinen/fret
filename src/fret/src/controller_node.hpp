/**
 * @file controller_node.hpp
 * @brief ROS 2 node that reads robot state and publishes velocity commands.
 *
 * Private header — not part of the public package API.
 */
#ifndef FRET_CONTROLLER_NODE_HPP
#define FRET_CONTROLLER_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "fret/control/controller_base.hpp"

/**
 * @brief ROS 2 node that closes the loop between robot state and velocity
 * commands.
 *
 * On startup the node reads the robot_model parameter, instantiates the
 * matching controller from ControllerRegistry, and starts a fixed-rate timer.
 * Each tick reads the latest /joint_states and queries TF for the
 * end-effector transform, then delegates command computation to the
 * controller.
 */
class ControllerNode : public rclcpp::Node {
  public:
    ControllerNode();

  private:
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
    void on_reference(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_command_timer();

    bool lookup_end_effector_transform(
        geometry_msgs::msg::TransformStamped &transform);
    void publish_command(const geometry_msgs::msg::TransformStamped &transform);
    void publish_zero_velocity();
    size_t get_joint_count() const;

    std::string robot_model_;
    std::string joint_states_topic_;
    std::string command_topic_;
    std::string reference_topic_;
    std::string base_frame_;
    std::string ee_frame_;
    double command_rate_hz_;
    std::vector<std::string> joint_names_;

    sensor_msgs::msg::JointState last_joint_state_;
    rclcpp::Time last_joint_state_time_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unique_ptr<fret::control::ControllerBase> controller_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr command_timer_;
};

#endif
