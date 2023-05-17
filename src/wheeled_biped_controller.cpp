#include "wheeled_biped_controller/forward_controllers_base.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "wheeled_biped_controller.hpp"

namespace wheeled_biped_controller
{
  WheeledBipedController::WheeledBipedController()
      : controller_interface::ControllerInterface(),
        rt_command_ptr_(nullptr),
        cmd_subscriber_(nullptr)
  {
  }

  controller_interface::CallbackReturn WheeledBipedController::on_init()
  {
    // TODO: declare parameters

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn WheeledBipedController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO: read parameters from yaml file

    cmd_subscriber_ = get_node()->create_subscription<CmdType>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const CmdType::SharedPtr msg)
        { rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  WheeledBipedController::command_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::ALL};
  }

  controller_interface::InterfaceConfiguration WheeledBipedController::state_interface_configuration()
      const
  {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::ALL};
  }

  controller_interface::CallbackReturn WheeledBipedController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn WheeledBipedController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type WheeledBipedController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    auto command = rt_command_ptr_.readFromRT();

    // TODO: read state from hardware interface

    // TODO: estimate state variables

    // TODO: apply controllers

    // TODO: write commands to hardware interface

    // TODO: publish state

    return controller_interface::return_type::OK;
  }

  double WheeledBipedController::interpolate(double x, std::vector<double> x_vals, std::vector<double> y_vals)
  {
    // Interpolates a value y for a given x using linear interpolation over a set of points
    // x_vals and y_vals must be the same size
    if (x_vals.size() != y_vals.size())
    {
      throw std::invalid_argument("x_vals and y_vals must be the same size");
    }
    // x_vals must be in increasing order
    if (!std::is_sorted(x_vals.begin(), x_vals.end()))
    {
      throw std::invalid_argument("x_vals must be in increasing order");
    }
    // If x is outside of the range of x_vals, the closest y value is returned
    if (x < x_vals.front())
    {
      return y_vals.front();
    }
    if (x > x_vals.back())
    {
      return y_vals.back();
    }
    // If x is between two values in x_vals, the corresponding y value is interpolated
    auto it = std::lower_bound(x_vals.begin(), x_vals.end(), x);
    auto index = std::distance(x_vals.begin(), it);
    if (x == x_vals[index])
    {
      return y_vals[index];
    }
    else
    {
      auto x0 = x_vals[index - 1];
      auto x1 = x_vals[index];
      auto y0 = y_vals[index - 1];
      auto y1 = y_vals[index];
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }
  }

  double WheeledBipedController::constrain(double x, double min, double max)
  {
    // Constrains a value x between min and max
    return fmin(fmax(x, min), max);
  }
} // namespace wheeled_biped_controller