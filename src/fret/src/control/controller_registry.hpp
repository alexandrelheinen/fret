/**
 * @file controller_registry.hpp
 * @brief Singleton registry that maps robot model names to controller
 * factories.
 *
 * Private header — not part of the public package API.
 */
#ifndef FRET_CONTROL_CONTROLLER_REGISTRY_HPP
#define FRET_CONTROL_CONTROLLER_REGISTRY_HPP

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "fret/control/controller_base.hpp"

namespace fret::control {

/**
 * @brief Singleton registry that maps robot model names to controller
 * factories.
 *
 * Each concrete controller registers itself at static initialization time by
 * calling register_controller(). The trajectory tracking node then calls
 * create() with the robot_model parameter to obtain the correct controller.
 */
class ControllerRegistry {
  public:
    /** @brief Factory callable that produces a new ControllerBase instance. */
    using Factory = std::function<std::unique_ptr<ControllerBase>()>;

    /**
     * @brief Return the global ControllerRegistry singleton.
     * @return Reference to the singleton instance.
     */
    static ControllerRegistry &instance();

    /**
     * @brief Register a factory for the given robot model name.
     * @param robot_model Unique model identifier (e.g., "scara").
     * @param factory     Callable that constructs a ControllerBase subclass.
     */
    void register_controller(const std::string &robot_model, Factory factory);

    /**
     * @brief Instantiate the controller registered for the given model.
     * @param robot_model Model identifier to look up.
     * @return Owning pointer to the new controller, or nullptr if not found.
     */
    std::unique_ptr<ControllerBase>
    create(const std::string &robot_model) const;

  private:
    ControllerRegistry() = default;

    std::unordered_map<std::string, Factory> factories_;
};

} // namespace fret::control

#endif
