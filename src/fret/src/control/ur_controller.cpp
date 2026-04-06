#include "ur_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_registry.hpp"

namespace fret::control {

std::vector<double> UrController::do_compute_command(
    const sensor_msgs::msg::JointState &joint_state,
    const geometry_msgs::msg::TransformStamped &) {
    const size_t count = joint_state.name.empty() ? joint_names_.size()
                                                  : joint_state.name.size();
    return std::vector<double>(count, 0.0);
}

namespace {

class UrRegistrar {
  public:
    UrRegistrar() {
        constexpr const char *UR_MODELS[] = {
            "ur3",   "ur3e", "ur5",   "ur5e", "ur7e", "ur10",    "ur10e",
            "ur12e", "ur15", "ur16e", "ur20", "ur30", "ur8long",
        };

        for (const char *model : UR_MODELS) {
            ControllerRegistry::instance().register_controller(
                model, []() { return std::make_unique<UrController>(); });
        }
    }
};

const UrRegistrar ur_registrar;

} // namespace

} // namespace fret::control
