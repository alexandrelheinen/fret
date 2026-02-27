#include "controller_registry.hpp"

namespace fret::control {

ControllerRegistry &ControllerRegistry::instance() {
    static ControllerRegistry registry;
    return registry;
}

void ControllerRegistry::register_controller(const std::string &robot_model,
                                             Factory factory) {
    factories_[robot_model] = std::move(factory);
}

std::unique_ptr<ControllerBase>
ControllerRegistry::create(const std::string &robot_model) const {
    auto it = factories_.find(robot_model);
    if (it == factories_.end()) {
        return nullptr;
    }
    return (it->second)();
}

} // namespace fret::control
