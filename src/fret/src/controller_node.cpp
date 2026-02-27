#include "controller_node.hpp"

#include <utility>

#include "control/controller_registry.hpp"

namespace {

constexpr double DEFAULT_COMMAND_RATE_HZ = 50.0;
constexpr int QUEUE_DEPTH = 10;
constexpr int TF_WARN_THROTTLE_MS = 5000;

} // namespace

ControllerNode::ControllerNode()
    : Node("controller"), tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
    declare_parameter<std::string>("robot_model", "scara");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<std::string>("command_topic",
                                   "/joint_group_velocity_controller/commands");
    declare_parameter<std::string>("reference_topic", "/controller/reference");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("ee_frame", "end_effector_link");
    declare_parameter<double>("command_rate_hz", DEFAULT_COMMAND_RATE_HZ);
    declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"joint_arm_0", "joint_arm_1", "joint_extension", "joint_tool_rotate"});

    robot_model_ = get_parameter("robot_model").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    command_topic_ = get_parameter("command_topic").as_string();
    reference_topic_ = get_parameter("reference_topic").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    ee_frame_ = get_parameter("ee_frame").as_string();
    command_rate_hz_ = get_parameter("command_rate_hz").as_double();
    joint_names_ = get_parameter("joint_names").as_string_array();

    controller_ =
        fret::control::ControllerRegistry::instance().create(robot_model_);
    if (!controller_) {
        RCLCPP_ERROR(get_logger(),
                     "No controller registered for robot model '%s'",
                     robot_model_.c_str());
    } else {
        controller_->configure(joint_names_);
    }

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, QUEUE_DEPTH,
        std::bind(&ControllerNode::on_joint_state, this,
                  std::placeholders::_1));

    reference_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        reference_topic_, QUEUE_DEPTH,
        std::bind(&ControllerNode::on_reference, this, std::placeholders::_1));

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        command_topic_, QUEUE_DEPTH);

    auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
    command_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ControllerNode::on_command_timer, this));

    RCLCPP_INFO(get_logger(),
                "Controller node started. model=%s "
                "joint_states=%s command=%s reference=%s base=%s ee=%s",
                robot_model_.c_str(), joint_states_topic_.c_str(),
                command_topic_.c_str(), reference_topic_.c_str(),
                base_frame_.c_str(), ee_frame_.c_str());
}

void ControllerNode::on_joint_state(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_joint_state_ = *msg;
    last_joint_state_time_ = msg->header.stamp;
}

void ControllerNode::on_reference(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (controller_) {
        controller_->set_reference(*msg);
    }
}

void ControllerNode::on_command_timer() {
    geometry_msgs::msg::TransformStamped transform;
    if (!lookup_end_effector_transform(transform)) {
        publish_zero_velocity();
        return;
    }

    publish_command(transform);
}

bool ControllerNode::lookup_end_effector_transform(
    geometry_msgs::msg::TransformStamped &transform) {
    try {
        transform = tf_buffer_.lookupTransform(base_frame_, ee_frame_,
                                               tf2::TimePointZero);
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), TF_WARN_THROTTLE_MS,
                             "TF lookup failed (%s -> %s): %s",
                             base_frame_.c_str(), ee_frame_.c_str(), ex.what());
        return false;
    }
}

void ControllerNode::publish_command(
    const geometry_msgs::msg::TransformStamped &transform) {
    if (!controller_) {
        publish_zero_velocity();
        return;
    }

    auto command = controller_->compute_command(last_joint_state_, transform);
    if (command.empty()) {
        publish_zero_velocity();
        return;
    }

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = std::move(command);
    command_pub_->publish(cmd);
}

void ControllerNode::publish_zero_velocity() {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(get_joint_count(), 0.0);
    command_pub_->publish(cmd);
}

size_t ControllerNode::get_joint_count() const {
    if (!last_joint_state_.name.empty()) {
        return last_joint_state_.name.size();
    }
    return joint_names_.size();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
