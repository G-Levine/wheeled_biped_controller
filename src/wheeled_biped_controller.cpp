#include "wheeled_biped_controller/wheeled_biped_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#define EPSILON 1e-6

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
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn WheeledBipedController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO: read parameters from yaml file

    x_des_ = 0.0;
    x_vel_des_ = 0.0;
    pitch_des_ = 0.0;
    pitch_vel_des_ = 0.0;
    yaw_des_ = 0.0;
    yaw_vel_des_ = 0.0;
    z_des_ = params_.z_min;
    z_vel_des_ = 0.0;

    last_publish_time_ = 0.0;

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
    if (!command)
    {
      RCLCPP_WARN(get_node()->get_logger(), "no command received");
      return controller_interface::return_type::OK;
    }
    // TODO: sanitize command

    // update desired velocities
    x_vel_des_ = command.linear.x;
    yaw_vel_des_ = command.angular.z;
    z_vel_des_ = command.linear.z;

    // integrate the desired velocities to obtain desired positions
    x_des_ += x_vel_des_ * period.seconds();
    yaw_des_ += yaw_vel_des_ * period.seconds();
    z_des_ += z_vel_des_ * period.seconds();

    // limit the desired z height
    if (z_des_ < params_.z_min || z_des_ > params_.z_max)
      z_vel_des_ = 0.0;
    z_des_ = std::clamp(z_des_, params_.z_min, params_.z_max);

    // read joint states from hardware interface
    double right_hip_pos = state_interfaces_map_.at(params_.right_hip_name).at("position")->get_value();
    double right_hip_vel = state_interfaces_map_.at(params_.right_hip_name).at("velocity")->get_value();
    double right_hip_torque = state_interfaces_map_.at(params_.right_hip_name).at("effort")->get_value();
    double right_wheel_pos = state_interfaces_map_.at(params_.right_wheel_name).at("position")->get_value();
    double right_wheel_vel = state_interfaces_map_.at(params_.right_wheel_name).at("velocity")->get_value();
    double left_hip_pos = state_interfaces_map_.at(params_.left_hip_name).at("position")->get_value();
    double left_hip_vel = state_interfaces_map_.at(params_.left_hip_name).at("velocity")->get_value();
    double left_hip_torque = state_interfaces_map_.at(params_.left_hip_name).at("effort")->get_value();
    double left_wheel_pos = state_interfaces_map_.at(params_.left_wheel_name).at("position")->get_value();
    double left_wheel_vel = state_interfaces_map_.at(params_.left_wheel_name).at("velocity")->get_value();

    // read IMU states from hardware interface
    pitch_vel_ = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.y")->get_value();
    double orientation_w = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.w")->get_value();
    double orientation_x = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.x")->get_value();
    double orientation_y = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.y")->get_value();
    double orientation_z = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.z")->get_value();

    // compensate for opposite directions on left/right (flip so everything is in the same direction)
    right_hip_pos = -right_hip_pos;
    right_hip_vel = -right_hip_vel;
    right_hip_torque = -right_hip_torque;
    right_wheel_pos = -right_wheel_pos;
    right_wheel_vel = -right_wheel_vel;

    // TODO: convert orientation to Euler angles
    double pitch = 0.0;

    // compensate for base_link and leg orientation/velocity in the wheel position/velocity
    right_wheel_pos = right_wheel_pos + pitch_ + right_hip_pos;
    right_wheel_vel = right_wheel_vel + pitch_vel_ + right_hip_vel;
    left_wheel_pos = left_wheel_pos + pitch_ + left_hip_pos;
    left_wheel_vel = left_wheel_vel + pitch_vel_ + left_hip_vel;

    // calculate tangential position and velocity for each wheel
    double right_wheel_tangential_pos = right_wheel_pos * params_.wheel_radius;
    double right_wheel_tangential_vel = right_wheel_vel * params_.wheel_radius;
    double left_wheel_tangential_pos = left_wheel_pos * params_.wheel_radius;
    double left_wheel_tangential_vel = left_wheel_vel * params_.wheel_radius;

    // TODO: update odometry estimates

    // calculate x_, yaw_, x_vel_, and y_vel_ from tangential wheel positions/velocities
    x_ = 0.5 * (right_wheel_tangential_pos + left_wheel_tangential_pos);
    yaw_ = 0.5 * (right_wheel_tangential_pos - left_wheel_tangential_pos) / params_.wheel_separation;

    // estimate z_ height
    double left_leg_length = 2.0 * cos(left_hip_pos) * params_.leg_link_length;
    double right_leg_length = 2.0 * cos(right_hip_pos) * params_.leg_link_length;
    z_ = 0.5 * (left_leg_length + right_leg_length);

    // estimate contact forces
    double left_wheel_normal_force = -left_hip_torque / (2.0 * sin(left_hip_pos) * params_.leg_link_length + EPSILON);
    double right_wheel_normal_force = -right_hip_torque / (2.0 * sin(right_hip_pos) * params_.leg_link_length + EPSILON);

    double left_wheel_torque_cmd = 0.0;
    double right_wheel_torque_cmd = 0.0;

    // if contact forces are too small, assume the robot is in the air and reset the desired state
    if (left_wheel_normal_force < params_.contact_force_threshold || right_wheel_normal_force < params_.contact_force_threshold)
    {
      x_des_ = x_;
      yaw_des_ = yaw_;
      x_vel_des_ = 0.0;
      yaw_vel_des_ = 0.0;
    }
    // otherwise, the robot is on the ground and we should apply the balancing/yaw controllers
    else
    {
      // get controller gains and pitch offset for the current z height
      double x_kp = interpolate(z_, params_.balancing_gain_heights, params_.x_kps);
      double x_kd = interpolate(z_, params_.balancing_gain_heights, params_.x_kds);
      double pitch_kp = interpolate(z_, params_.balancing_gain_heights, params_.pitch_kps);
      double pitch_kd = interpolate(z_, params_.balancing_gain_heights, params_.pitch_kds);
      pitch_des_ = interpolate(z_, params_.balancing_gain_heights, params_.pitch_offsets);

      // apply balancing/yaw controllers
      double balance_control_force = linear_controller<4>({x_, x_vel_, pitch_, pitch_vel_},
                                                         {x_des_, x_vel_des_, pitch_des_, pitch_vel_des_},
                                                         {x_kp, x_kd, pitch_kp, pitch_kd});
      double yaw_control_force = linear_controller<2>({yaw_, yaw_vel_}, {yaw_des_, yaw_vel_des_}, {params_.yaw_kp, params_.yaw_kd});

      // give the balance control force strict priority over the yaw control force
      double friction_max_force = 2.0 * fmaxf(0.0, fminf(left_wheel_normal_force, right_wheel_normal_force)) * params_.friction_coefficient;
      balance_control_force = std::clamp(balance_control_force, -friction_max_force, friction_max_force);
      double friction_margin_force = abs(friction_max_force - abs(balance_control_force));
      yaw_control_force = std::clamp(yaw_control_force, -friction_margin_force, friction_margin_force);

      // combine balance and yaw control forces into tangential wheel forces
      double left_wheel_tangential_force = 0.5 * (balance_control_force - yaw_control_force);
      double right_wheel_tangential_force = 0.5 * (balance_control_force + yaw_control_force);

      // convert tangential wheel forces to wheel torques
      left_wheel_torque_cmd = left_wheel_tangential_force * params_.wheel_radius;
      right_wheel_torque_cmd = right_wheel_tangential_force * params_.wheel_radius;
    }

    // calculate hip position commands from z_des_ and z_vel_des_
    double right_hip_pos_cmd = acos(z_des_ / (2.0 * params_.leg_link_length));
    double left_hip_pos_cmd = acos(z_des_ / (2.0 * params_.leg_link_length));

    // compensate for opposite directions on left/right (flip so the sides are in opposite directions)
    right_wheel_torque_cmd = -right_wheel_torque_cmd;
    right_hip_pos_cmd = -right_hip_pos_cmd;

    // write commands to hardware interface
    command_interfaces_map_.at(params_.left_hip_name).at("position")->set_value(left_hip_pos_cmd);
    command_interfaces_map_.at(params_.left_hip_name).at("kp")->set_value(params_.hip_kp);
    command_interfaces_map_.at(params_.left_hip_name).at("kd")->set_value(params_.hip_kd);
    command_interfaces_map_.at(params_.right_hip_name).at("position")->set_value(right_hip_pos_cmd);
    command_interfaces_map_.at(params_.right_hip_name).at("kp")->set_value(params_.hip_kp);
    command_interfaces_map_.at(params_.right_hip_name).at("kd")->set_value(params_.hip_kd);
    command_interfaces_map_.at(params_.left_wheel_name).at("effort")->set_value(left_wheel_torque_cmd);
    command_interfaces_map_.at(params_.right_wheel_name).at("effort")->set_value(right_wheel_torque_cmd);

    // TODO: publish state if enough time has passed since the last publish
    // if ((time - last_publish_time_) > 1.0 / params_.publish_rate)
    // {
    //   last_publish_time_ = time;
    // }

    return controller_interface::return_type::OK;
  }

  double WheeledBipedController::interpolate(double x, const std::vector<double>& x_vals, const std::vector<double>& y_vals)
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

  template <size_t N>
  double WheeledBipedController::linear_controller(const std::array<double, N> &state,
                                                   const std::array<double, N> &desired_state,
                                                   const std::array<double, N> &gains)
  {
    double output = 0.0;
    for (size_t i = 0; i < N; ++i)
    {
      double error = desired_state[i] - state[i];
      double control = gains[i] * error;
      output += control;
    }
    return output;
  }
} // namespace wheeled_biped_controller