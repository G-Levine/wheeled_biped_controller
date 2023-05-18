#ifndef WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_
#define WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist.hpp"

namespace wheeled_biped_controller
{
  using CmdType = geometry_msgs::msg::Twist;

  class WheeledBipedController : public controller_interface::ControllerInterface
  {
  public:
    WheeledBipedController();

    ~WheeledBipedController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // Utility functions
    double interpolate(double x, const std::vector<double>& x_vals, const std::vector<double>& y_vals);

    template <size_t N>
    double linear_controller(const std::array<double, N> &state,
                             const std::array<double, N> &desired_state,
                             const std::array<double, N> &gains);

    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
    rclcpp::Subscription<CmdType>::SharedPtr cmd_subscriber_;

    double last_publish_time_;

    // State variables
    // - Current state
    // -- Balancing controller
    double pitch_;
    double pitch_vel_;
    double x_;
    double x_vel_;
    // -- Yaw controller
    double yaw_;
    double yaw_vel_;
    // -- Height controller
    double z_;
    double z_vel_;
    // -- Odometry state estimator
    double odom_x_;
    double odom_y_;
    double odom_yaw_;

    // - Desired state
    // -- Balancing controller
    double pitch_des_;
    double pitch_vel_des_;
    double x_des_;
    double x_vel_des_;
    // -- Yaw controller
    double yaw_des_;
    double yaw_vel_des_;
    // -- Height controller
    double z_des_;
    double z_vel_des_;
  };

} // namespace wheeled_biped_controller

#endif // WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_