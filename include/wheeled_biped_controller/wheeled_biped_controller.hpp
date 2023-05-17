#ifndef WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_
#define WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

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
    WHEELED_BIPED_CONTROLLER_PUBLIC
    WheeledBipedController();

    WHEELED_BIPED_CONTROLLER_PUBLIC
    ~WheeledBipedController() = default;

    WHEELED_BIPED_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    WHEELED_BIPED_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    WHEELED_BIPED_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    WHEELED_BIPED_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    WHEELED_BIPED_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    WHEELED_BIPED_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    WHEELED_BIPED_CONTROLLER_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    // Utility functions
    double interpolate(double x, std::vector<double> x_vals, std::vector<double> y_vals);
    double constrain(double x, double min, double max);

    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
    rclcpp::Subscription<CmdType>::SharedPtr cmd_subscriber_;

    double publish_rate_;

    // Parameters
    // - Physical parameters
    double wheel_radius_;
    double wheel_separation_;
    double leg_link_length_;
    double z_min_;
    double z_max_;
    // - Controller parameters
    // -- Balancing controller
    std::vector<double> pitch_kps_;
    std::vector<double> pitch_kds_;
    std::vector<double> x_kps_;
    std::vector<double> x_kds_;
    std::vector<double> balancing_gain_heights;
    // -- Yaw controller
    double yaw_kp_;
    double yaw_kd_;
    // -- Height controller
    double hip_kp_;
    double hip_kd_;
    // -- Contact state estimator
    double contact_force_threshold_;
    double friction_coefficient_;

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
    // -- Contact state estimator
    double contact_force_left_;
    double contact_force_right_;
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
  };

} // namespace wheeled_biped_controller

#endif // WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_