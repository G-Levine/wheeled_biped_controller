#ifndef WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_
#define WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <functional>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// auto-generated by generate_parameter_library
#include "wheeled_biped_controller_parameters.hpp"

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

    // Map from joint names to command types to command interfaces
    std::map<std::string, std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>>
        command_interfaces_map_;

    // Map from joint/sensor names to state types to state interfaces
    std::map<std::string, std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>>
        state_interfaces_map_;

    // Utility functions
    double interpolate(double x, const std::vector<double> &x_vals, const std::vector<double> &y_vals);

    template <size_t N>
    double linear_controller(const std::array<double, N> &state,
                             const std::array<double, N> &desired_state,
                             const std::array<double, N> &gains);

    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
    rclcpp::Subscription<CmdType>::SharedPtr cmd_subscriber_;

    // std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    // std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    //   realtime_odometry_publisher_ = nullptr;

    // std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    //   nullptr;
    // std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    //   realtime_odometry_transform_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> base_link_transform_publisher_ =
      nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
      realtime_base_link_transform_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_ =
      nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
      realtime_joint_state_publisher_ = nullptr;

    rclcpp::Time last_publish_time_;

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
    // -- Roll controller
    double roll_;
    double roll_vel_;
    // -- Height controller
    double z_;
    double z_vel_;
    // -- Contact state estimator
    double left_wheel_normal_force_;
    double right_wheel_normal_force_;
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
    // -- Roll controller
    double roll_des_;
    double roll_vel_des_;
    // -- Height controller
    double z_des_;
    double z_vel_des_;
  };

} // namespace wheeled_biped_controller

#endif // WHEELED_BIPED_CONTROLLER__WHEELED_BIPED_CONTROLLER_HPP_