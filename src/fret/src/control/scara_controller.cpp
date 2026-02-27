#include "scara_controller.hpp"

#include <memory>
#include <string>

#include "controller_registry.hpp"

namespace fret::control {

std::vector<double> ScaraController::do_compute_command(
    const sensor_msgs::msg::JointState &joint_state,
    const geometry_msgs::msg::TransformStamped &) {
    const size_t count = joint_state.name.empty() ? joint_names_.size()
                                                  : joint_state.name.size();
    return std::vector<double>(count, 0.0);
}

namespace {

class ScaraRegistrar {
  public:
    ScaraRegistrar() {
        ControllerRegistry::instance().register_controller(
            "scara", []() { return std::make_unique<ScaraController>(); });
    }
};

const ScaraRegistrar scara_registrar;

} // namespace

} // namespace fret::control
