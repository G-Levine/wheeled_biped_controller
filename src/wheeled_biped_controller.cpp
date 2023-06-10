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

#define EPSILON 1e-3
#define EPSILON_YAW_COMPENSATION 1e-1
#define NUM_JOINTS 6

#define NUM_POLICY_OBS 21
#define NUM_POLICY_ACT 4

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
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn WheeledBipedController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    x_des_ = 0.0;
    x_vel_des_ = 0.0;
    pitch_des_ = 0.0;
    pitch_vel_des_ = 0.0;
    yaw_des_ = 0.0;
    yaw_vel_des_ = 0.0;
    z_des_ = params_.z_min;
    z_vel_des_ = 0.0;

    last_publish_time_ = get_node()->now();
    last_sensor_publish_time_ = get_node()->now();

    // Initialize the command subscriber
    cmd_subscriber_ = get_node()->create_subscription<CmdType>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const CmdType::SharedPtr msg)
        { rt_command_ptr_.writeFromNonRT(msg); });

    // Initialize the action subscriber
    action_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/policy/action", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        { rt_action_ptr_.writeFromNonRT(msg); });

    // Initialize the publishers
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        "~/odom", rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
            odometry_publisher_);

    // odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    //     "~/tf", rclcpp::SystemDefaultsQoS());
    // realtime_odometry_transform_publisher_ =
    //     std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
    //         odometry_transform_publisher_);

    joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS());
    realtime_joint_state_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
            joint_state_publisher_);

    imu_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Imu>(
        "~/imu", rclcpp::SystemDefaultsQoS());
    realtime_imu_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(
            imu_publisher_);

    base_link_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
        "~/tf", rclcpp::SystemDefaultsQoS());
    realtime_base_link_transform_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
            base_link_transform_publisher_);
    
    observation_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/policy/observation", rclcpp::SystemDefaultsQoS());
    realtime_observation_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float32MultiArray>>(
            observation_publisher_);

    auto &odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = "odom";
    odometry_message.child_frame_id = "base_link";
    odometry_message.pose.covariance = {
        1e-3, 0, 0, 0, 0, 0,
        0, 1e-3, 0, 0, 0, 0,
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0,
        0, 0, 0, 0, 1e6, 0,
        0, 0, 0, 0, 0, 1e3};
    odometry_message.twist.covariance = {
        1e-3, 0, 0, 0, 0, 0,
        0, 1e-3, 0, 0, 0, 0,
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0,
        0, 0, 0, 0, 1e6, 0,
        0, 0, 0, 0, 0, 1e3};

    auto & base_link_transform_publisher_message = realtime_base_link_transform_publisher_->msg_;
    base_link_transform_publisher_message.transforms.resize(1);
    base_link_transform_publisher_message.transforms[0].header.frame_id = "odom";
    base_link_transform_publisher_message.transforms[0].child_frame_id = "base_link";

    auto &joint_state_message = realtime_joint_state_publisher_->msg_;
    joint_state_message.name = {params_.right_hip_name, params_.right_knee_name, params_.right_wheel_name, params_.left_hip_name, params_.left_knee_name, params_.left_wheel_name};
    joint_state_message.position.resize(NUM_JOINTS);
    joint_state_message.velocity.resize(NUM_JOINTS);
    joint_state_message.effort.resize(NUM_JOINTS);

    auto &imu_message = realtime_imu_publisher_->msg_;
    imu_message.header.frame_id = "base_link";
    imu_message.orientation_covariance = {
        1e-3, 0, 0,
        0, 1e-3, 0,
        0, 0, 1e-3};
    imu_message.angular_velocity_covariance = {
        1e-3, 0, 0,
        0, 1e-3, 0,
        0, 0, 1e-3};

    auto &observation_message = realtime_observation_publisher_->msg_;
    observation_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    observation_message.layout.dim[0].size = NUM_POLICY_OBS;
    observation_message.layout.dim[0].stride = 1;
    observation_message.layout.dim[0].label = "observation";
    observation_message.data.resize(NUM_POLICY_OBS);

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

    // Populate the command interfaces map
    for (auto &command_interface : command_interfaces_)
    {
      command_interfaces_map_[command_interface.get_prefix_name()].emplace(
          command_interface.get_interface_name(),
          std::ref(command_interface));
    }

    // Populate the state interfaces map
    for (auto &state_interface : state_interfaces_)
    {
      state_interfaces_map_[state_interface.get_prefix_name()].emplace(
          state_interface.get_interface_name(),
          std::ref(state_interface));
    }

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn WheeledBipedController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    for (auto &command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type WheeledBipedController::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    auto command = rt_command_ptr_.readFromRT();
    auto action = rt_action_ptr_.readFromRT();

    double left_wheel_vel_action = 0.0;
    double right_wheel_vel_action = 0.0;
    double left_leg_pos_action = -params_.z_min;
    double right_leg_pos_action = -params_.z_min;

    if (params_.use_policy)
    {
      if (!action)
      {
        RCLCPP_WARN(get_node()->get_logger(), "no action received");
        return controller_interface::return_type::OK;
      }

      // update desired velocities if the command exists
      if (action->get())
      {
        left_leg_pos_action = action->get()->data[0];
        left_wheel_vel_action = action->get()->data[1];
        right_leg_pos_action = action->get()->data[2];
        right_wheel_vel_action = action->get()->data[3];
        // std::cout << left_leg_pos_action << "\t" << left_wheel_vel_action << "\t" << right_leg_pos_action << "\t" << right_wheel_vel_action << "\t";

        // sanitize
        left_leg_pos_action = std::clamp(left_leg_pos_action, -params_.z_max, -params_.z_min);
        left_wheel_vel_action = std::clamp(left_wheel_vel_action, -20.0, 20.0);
        right_leg_pos_action = std::clamp(right_leg_pos_action, -params_.z_max, -params_.z_min);
        right_wheel_vel_action = std::clamp(right_wheel_vel_action, -20.0, 20.0);
      }
    }

    if (!command)
    {
      RCLCPP_WARN(get_node()->get_logger(), "no command received");
      return controller_interface::return_type::OK;
    }

    // update desired velocities if the command exists
    if (command->get())
    {
      x_vel_des_ = std::clamp(command->get()->linear.x, x_vel_ - params_.max_target_velocity, x_vel_ + params_.max_target_velocity);
      yaw_vel_des_ = command->get()->angular.z;
      z_vel_des_ = command->get()->linear.z;
    }

    // integrate the desired velocities to obtain desired positions
    x_des_ += x_vel_des_ * period.seconds();
    yaw_des_ += yaw_vel_des_ * period.seconds();
    z_des_ += z_vel_des_ * period.seconds();

    x_des_ = std::clamp(x_des_, x_ - params_.max_target_distance, x_ + params_.max_target_distance);

    // limit the desired z height
    if (z_des_ < params_.z_min || z_des_ > params_.z_max)
      z_vel_des_ = 0.0;
    z_des_ = std::clamp(z_des_, params_.z_min, params_.z_max);

    double right_hip_pos, right_hip_vel, right_hip_torque;
    double right_wheel_pos, right_wheel_vel;
    double left_hip_pos, left_hip_vel, left_hip_torque;
    double left_wheel_pos, left_wheel_vel;
    double orientation_w, orientation_x, orientation_y, orientation_z;
    double imu_yaw_vel;

    try
    {
      // read joint states from hardware interface
      right_hip_pos = state_interfaces_map_.at(params_.right_hip_name).at("position").get().get_value();
      right_hip_vel = state_interfaces_map_.at(params_.right_hip_name).at("velocity").get().get_value();
      right_hip_torque = state_interfaces_map_.at(params_.right_hip_name).at("effort").get().get_value();
      right_wheel_pos = state_interfaces_map_.at(params_.right_wheel_name).at("position").get().get_value();
      right_wheel_vel = state_interfaces_map_.at(params_.right_wheel_name).at("velocity").get().get_value();
      left_hip_pos = state_interfaces_map_.at(params_.left_hip_name).at("position").get().get_value();
      left_hip_vel = state_interfaces_map_.at(params_.left_hip_name).at("velocity").get().get_value();
      left_hip_torque = state_interfaces_map_.at(params_.left_hip_name).at("effort").get().get_value();
      left_wheel_pos = state_interfaces_map_.at(params_.left_wheel_name).at("position").get().get_value();
      left_wheel_vel = state_interfaces_map_.at(params_.left_wheel_name).at("velocity").get().get_value();

      // read IMU states from hardware interface
      pitch_vel_ = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.y").get().get_value();
      roll_vel_ = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.x").get().get_value();
      imu_yaw_vel = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.z").get().get_value();
      orientation_w = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.w").get().get_value();
      orientation_x = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.x").get().get_value();
      orientation_y = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.y").get().get_value();
      orientation_z = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.z").get().get_value();
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_INFO(get_node()->get_logger(), "failed to read joint states from hardware interface");
      return controller_interface::return_type::OK;
    }

    // compensate for opposite directions on left/right (flip so everything is in the same direction)
    right_hip_pos = -right_hip_pos;
    right_hip_vel = -right_hip_vel;
    right_hip_torque = -right_hip_torque;
    right_wheel_pos = -right_wheel_pos;
    right_wheel_vel = -right_wheel_vel;

    // convert orientation to Euler angles
    tf2::Quaternion q(
        orientation_x,
        orientation_y,
        orientation_z,
        orientation_w);
    tf2::Matrix3x3 m(q);
    double imu_roll, imu_pitch, imu_yaw;
    m.getRPY(imu_roll, imu_pitch, imu_yaw);
    roll_ = imu_roll;
    pitch_ = imu_pitch;

    // compensate for base_link and leg orientation/velocity in the wheel position/velocity
    right_wheel_pos = right_wheel_pos + pitch_ - right_hip_pos;
    right_wheel_vel = right_wheel_vel + pitch_vel_ - right_hip_vel;
    left_wheel_pos = left_wheel_pos + pitch_ - left_hip_pos;
    left_wheel_vel = left_wheel_vel + pitch_vel_ - left_hip_vel;

    // calculate tangential position and velocity for each wheel
    double right_wheel_tangential_pos = right_wheel_pos * params_.wheel_radius;
    double right_wheel_tangential_vel = right_wheel_vel * params_.wheel_radius;
    double left_wheel_tangential_pos = left_wheel_pos * params_.wheel_radius;
    double left_wheel_tangential_vel = left_wheel_vel * params_.wheel_radius;

    double delta_x = 0.5 * (right_wheel_tangential_pos + left_wheel_tangential_pos) - x_;
    double delta_yaw = 0.5 * (right_wheel_tangential_pos - left_wheel_tangential_pos) / (0.5 * params_.wheel_separation) - yaw_;

    // calculate x_, yaw_, x_vel_, and y_vel_ from tangential wheel positions/velocities
    x_ = 0.5 * (right_wheel_tangential_pos + left_wheel_tangential_pos);
    yaw_ = 0.5 * (right_wheel_tangential_pos - left_wheel_tangential_pos) / (0.5 * params_.wheel_separation);
    // yaw_ = imu_yaw;
    x_vel_ = 0.5 * (right_wheel_tangential_vel + left_wheel_tangential_vel);
    yaw_vel_ = 0.5 * (right_wheel_tangential_vel - left_wheel_tangential_vel) / (0.5 * params_.wheel_separation);
    // yaw_vel_ = imu_yaw_vel;

    // use IMU to correct odometry yaw drift
    // odom_yaw_ += delta_yaw * (imu_yaw_vel + EPSILON_YAW_COMPENSATION) / (yaw_vel_ + EPSILON_YAW_COMPENSATION);
    odom_yaw_ = imu_yaw;

    // update odometry estimates
    odom_orientation_.setRPY(roll_, pitch_, odom_yaw_);
    odom_x_ += delta_x * cos(odom_yaw_);
    odom_y_ += delta_x * sin(odom_yaw_);
    odom_x_vel_ = x_vel_ * cos(odom_yaw_);
    odom_y_vel_ = x_vel_ * sin(odom_yaw_);
    odom_yaw_vel_ = imu_yaw_vel;

    // estimate z_ height
    double left_leg_length = 2.0 * cos(left_hip_pos) * params_.leg_link_length;
    double right_leg_length = 2.0 * cos(right_hip_pos) * params_.leg_link_length;
    z_ = 0.5 * (left_leg_length + right_leg_length);

    // estimate contact forces
    double left_leg_lever_arm = 2.0 * sin(left_hip_pos) * params_.leg_link_length;
    double right_leg_lever_arm = 2.0 * sin(right_hip_pos) * params_.leg_link_length;

    left_wheel_normal_force_ = (1 - params_.normal_force_filter_alpha) * left_wheel_normal_force_ + params_.normal_force_filter_alpha * (-left_hip_torque / left_leg_lever_arm + EPSILON);
    right_wheel_normal_force_ = (1 - params_.normal_force_filter_alpha) * right_wheel_normal_force_ + params_.normal_force_filter_alpha * (-right_hip_torque / right_leg_lever_arm + EPSILON);

    double left_wheel_torque_cmd = 0.0;
    double right_wheel_torque_cmd = 0.0;

    double left_hip_torque_cmd = 0.0;
    double right_hip_torque_cmd = 0.0;


    // get controller gains and pitch offset for the current z height
    double x_kp = interpolate(z_, params_.balancing_gain_heights, params_.x_kps);
    double x_kd = interpolate(z_, params_.balancing_gain_heights, params_.x_kds);
    double pitch_kp = interpolate(z_, params_.balancing_gain_heights, params_.pitch_kps);
    double pitch_kd = interpolate(z_, params_.balancing_gain_heights, params_.pitch_kds);
    pitch_des_ = interpolate(z_, params_.balancing_gain_heights, params_.pitch_offsets);

    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(0, -pitch_des_, 0);

    tf2::Vector3 world_gravity_vector(0, 0, -1);
    tf2::Vector3 projected_gravity_vector = m.inverse() * world_gravity_vector;
    projected_gravity_vector = tf2::quatRotate(rotation_quat, projected_gravity_vector);

    // if contact forces are too small, assume the robot is in the air and reset the desired state
    if (left_wheel_normal_force_ < params_.contact_force_threshold || right_wheel_normal_force_ < params_.contact_force_threshold)
    {
      x_des_ = x_;
      yaw_des_ = yaw_;
      x_vel_des_ = 0.0;
      yaw_vel_des_ = 0.0;
    }
    // otherwise, the robot is on the ground and we should apply the balancing/yaw/roll controllers
    else
    {
      // apply balancing/yaw controllers
      double balance_control_force = linear_controller<4>({x_, x_vel_, pitch_, pitch_vel_},
                                                          {x_des_, x_vel_des_, pitch_des_, pitch_vel_des_},
                                                          {x_kp, x_kd, pitch_kp, pitch_kd});
      double yaw_control_force = linear_controller<2>({yaw_, yaw_vel_}, {yaw_des_, yaw_vel_des_}, {params_.yaw_kp, params_.yaw_kd});

      // give the balance control force strict priority over the yaw control force
      double friction_max_force = abs(2.0 * fmaxf(0.0, fminf(left_wheel_normal_force_, right_wheel_normal_force_)) * params_.friction_coefficient);
      balance_control_force = std::clamp(balance_control_force, -friction_max_force, friction_max_force);
      double friction_margin_force = abs(friction_max_force - abs(balance_control_force));
      yaw_control_force = std::clamp(yaw_control_force, -friction_margin_force, friction_margin_force);

      // combine balance and yaw control forces into tangential wheel forces
      double left_wheel_tangential_force = 0.5 * (balance_control_force - yaw_control_force);
      double right_wheel_tangential_force = 0.5 * (balance_control_force + yaw_control_force);

      // convert tangential wheel forces to wheel torques
      left_wheel_torque_cmd = left_wheel_tangential_force * params_.wheel_radius;
      right_wheel_torque_cmd = right_wheel_tangential_force * params_.wheel_radius;

      // calculate feedforward torques for the hip actuators using the roll controller
      // first, calculate the torque around the roll axis
      double roll_control_torque = linear_controller<2>({roll_, roll_vel_}, {roll_des_, roll_vel_des_}, {params_.roll_kp, params_.roll_kd});

      // then, convert to linear forces for the legs
      double left_leg_roll_control_force = 0.5 * roll_control_torque / (0.5 * params_.wheel_separation);
      double right_leg_roll_control_force = -0.5 * roll_control_torque / (0.5 * params_.wheel_separation);

      // then, convert to torques for the hip actuators by multiplying by the lever arm
      left_hip_torque_cmd = -left_leg_roll_control_force * left_leg_lever_arm;
      right_hip_torque_cmd = -right_leg_roll_control_force * right_leg_lever_arm;
    }

    // calculate hip position commands from z_des_ and z_vel_des_
    double right_hip_pos_cmd = acos(z_des_ / (2.0 * params_.leg_link_length));
    double left_hip_pos_cmd = acos(z_des_ / (2.0 * params_.leg_link_length));

    // compensate for opposite directions on left/right (flip so the sides are in opposite directions)
    right_wheel_torque_cmd = -right_wheel_torque_cmd;
    right_hip_pos_cmd = -right_hip_pos_cmd;
    right_hip_torque_cmd = -right_hip_torque_cmd;

    double left_leg_linear_pos = -2.0 * cos(left_hip_pos) * params_.leg_link_length;
    double right_leg_linear_pos = -2.0 * cos(right_hip_pos) * params_.leg_link_length;
    double left_leg_linear_vel = -left_hip_vel * left_leg_lever_arm;
    double right_leg_linear_vel = -right_hip_vel * right_leg_lever_arm;

    double left_hip_pos_action = acos(-left_leg_pos_action / (2.0 * params_.leg_link_length));
    double right_hip_pos_action = -acos(-right_leg_pos_action / (2.0 * params_.leg_link_length));

    if (params_.use_policy)
    {
      command_interfaces_map_.at(params_.left_hip_name).at("kp").get().set_value(left_leg_lever_arm * 30.0 / 0.16);
      command_interfaces_map_.at(params_.left_hip_name).at("kd").get().set_value(left_leg_lever_arm * 6.0 / 0.16);
      command_interfaces_map_.at(params_.right_hip_name).at("kp").get().set_value(right_leg_lever_arm * 30.0 / 0.16);
      command_interfaces_map_.at(params_.right_hip_name).at("kd").get().set_value(right_leg_lever_arm * 6.0 / 0.16);

      command_interfaces_map_.at(params_.left_wheel_name).at("kp").get().set_value(params_.wheel_kp);
      command_interfaces_map_.at(params_.left_wheel_name).at("kd").get().set_value(params_.wheel_kd);
      command_interfaces_map_.at(params_.right_wheel_name).at("kp").get().set_value(params_.wheel_kp);
      command_interfaces_map_.at(params_.right_wheel_name).at("kd").get().set_value(params_.wheel_kd);

      command_interfaces_map_.at(params_.left_wheel_name).at("velocity").get().set_value(left_wheel_vel_action);
      command_interfaces_map_.at(params_.right_wheel_name).at("velocity").get().set_value(right_wheel_vel_action);

      command_interfaces_map_.at(params_.left_hip_name).at("position").get().set_value(left_hip_pos_action);
      command_interfaces_map_.at(params_.right_hip_name).at("position").get().set_value(right_hip_pos_action);
    }
    else
    {
      // write commands to hardware interface
      command_interfaces_map_.at(params_.left_hip_name).at("position").get().set_value(left_hip_pos_cmd);
      command_interfaces_map_.at(params_.left_hip_name).at("kp").get().set_value(params_.hip_kp);
      command_interfaces_map_.at(params_.left_hip_name).at("kd").get().set_value(params_.hip_kd);
      command_interfaces_map_.at(params_.left_hip_name).at("effort").get().set_value(left_hip_torque_cmd);
      command_interfaces_map_.at(params_.right_hip_name).at("position").get().set_value(right_hip_pos_cmd);
      command_interfaces_map_.at(params_.right_hip_name).at("kp").get().set_value(params_.hip_kp);
      command_interfaces_map_.at(params_.right_hip_name).at("kd").get().set_value(params_.hip_kd);
      command_interfaces_map_.at(params_.right_hip_name).at("effort").get().set_value(right_hip_torque_cmd);
      command_interfaces_map_.at(params_.left_wheel_name).at("effort").get().set_value(left_wheel_torque_cmd);
      command_interfaces_map_.at(params_.right_wheel_name).at("effort").get().set_value(right_wheel_torque_cmd);
    }

    // publish state if enough time has passed since the last publish
    if (params_.sensor_publish_rate && (time - last_sensor_publish_time_).seconds() > 1.0 / params_.sensor_publish_rate)
    {
      last_sensor_publish_time_ = time;
      if (realtime_odometry_publisher_->trylock())
      {
        auto &odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.stamp = time;
        odometry_message.pose.pose.position.x = odom_x_;
        odometry_message.pose.pose.position.y = odom_y_;
        odometry_message.pose.pose.position.z = z_ + params_.wheel_radius;
        odometry_message.pose.pose.orientation.x = odom_orientation_.x();
        odometry_message.pose.pose.orientation.y = odom_orientation_.y();
        odometry_message.pose.pose.orientation.z = odom_orientation_.z();
        odometry_message.pose.pose.orientation.w = odom_orientation_.w();
        odometry_message.twist.twist.linear.x = x_vel_;
        odometry_message.twist.twist.angular.z = yaw_vel_;
        realtime_odometry_publisher_->unlockAndPublish();
      }
      if (realtime_imu_publisher_->trylock())
      {
        auto &imu_message = realtime_imu_publisher_->msg_;
        imu_message.header.stamp = time;
        imu_message.orientation.x = orientation_x;
        imu_message.orientation.y = orientation_y;
        imu_message.orientation.z = orientation_z;
        imu_message.orientation.w = orientation_w;
        imu_message.angular_velocity.x = roll_vel_;
        imu_message.angular_velocity.y = pitch_vel_;
        imu_message.angular_velocity.z = imu_yaw_vel;
        realtime_imu_publisher_->unlockAndPublish();
      }
    }

    if (params_.publish_rate && (time - last_publish_time_).seconds() > 1.0 / params_.publish_rate)
    {
      last_publish_time_ = time;
      // Log the state
      // RCLCPP_INFO(get_node()->get_logger(), "rate: %f, x: %f, x_vel: %f, pitch: %f, pitch_vel: %f, yaw: %f, yaw_vel: %f, z: %f, z_vel: %f, left_wheel_normal_force: %f, right_wheel_normal_force: %f",
      // 1.0 / period.seconds(), x_, x_vel_, pitch_, pitch_vel_, yaw_, yaw_vel_, z_, z_vel_, left_wheel_normal_force, right_wheel_normal_force);
      // std::cout << "z_des: " << z_des_ << "z:" << z_ << "right_hip_pos_cmd: " << right_hip_pos_cmd << "right_hip_pos: " << state_interfaces_map_.at(params_.right_hip_name).at("position").get().get_value() << "left_hip_pos_cmd: " << left_hip_pos_cmd << "left_hip_pos: " << state_interfaces_map_.at(params_.left_hip_name).at("position").get().get_value() << std::endl;
      if (realtime_observation_publisher_->trylock())
      {
        auto &observation_message = realtime_observation_publisher_->msg_;
        // Base angular velocity
        observation_message.data[0] = roll_vel_;
        observation_message.data[1] = pitch_vel_;
        observation_message.data[2] = imu_yaw_vel;

        // World down vector in the base link frame
        observation_message.data[3] = projected_gravity_vector[0];
        observation_message.data[4] = projected_gravity_vector[1];
        observation_message.data[5] = projected_gravity_vector[2];

        // Commands
        observation_message.data[6] = x_vel_des_;
        observation_message.data[7] = 0.0;
        observation_message.data[8] = yaw_vel_des_;

        // Joint positions
        observation_message.data[9] = left_leg_linear_pos;
        observation_message.data[10] = state_interfaces_map_.at(params_.left_wheel_name).at("position").get().get_value();
        observation_message.data[11] = right_leg_linear_pos;
        observation_message.data[12] = state_interfaces_map_.at(params_.right_wheel_name).at("position").get().get_value();

        // Joint velocities
        observation_message.data[13] = left_leg_linear_vel;
        observation_message.data[14] = state_interfaces_map_.at(params_.left_wheel_name).at("velocity").get().get_value();
        observation_message.data[15] = right_leg_linear_vel;
        observation_message.data[16] = state_interfaces_map_.at(params_.right_wheel_name).at("velocity").get().get_value();

        // Actions
        // observation_message.data[17] = 0.0;
        // observation_message.data[18] = 0.0;
        // observation_message.data[19] = 0.0;
        // observation_message.data[20] = 0.0;
        realtime_observation_publisher_->unlockAndPublish();
      }

      if (realtime_base_link_transform_publisher_->trylock())
      {
        auto &transform = realtime_base_link_transform_publisher_->msg_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odom_x_;
        transform.transform.translation.y = odom_y_;
        transform.transform.translation.z = z_ + params_.wheel_radius;
        transform.transform.rotation.x = odom_orientation_.x();
        transform.transform.rotation.y = odom_orientation_.y();
        transform.transform.rotation.z = odom_orientation_.z();
        transform.transform.rotation.w = odom_orientation_.w();
        realtime_base_link_transform_publisher_->unlockAndPublish();
      }

      if (realtime_joint_state_publisher_->trylock())
      {
        auto &joint_state_message = realtime_joint_state_publisher_->msg_;
        joint_state_message.header.stamp = time;
        joint_state_message.position[0] = state_interfaces_map_.at(params_.right_hip_name).at("position").get().get_value();
        joint_state_message.position[1] = -2.0 * state_interfaces_map_.at(params_.right_hip_name).at("position").get().get_value();
        joint_state_message.position[2] = state_interfaces_map_.at(params_.right_wheel_name).at("position").get().get_value();
        joint_state_message.position[3] = state_interfaces_map_.at(params_.left_hip_name).at("position").get().get_value();
        joint_state_message.position[4] = -2.0 * state_interfaces_map_.at(params_.left_hip_name).at("position").get().get_value();
        joint_state_message.position[5] = state_interfaces_map_.at(params_.left_wheel_name).at("position").get().get_value();
        realtime_joint_state_publisher_->unlockAndPublish();
      }
    }

    return controller_interface::return_type::OK;
  }

  double WheeledBipedController::interpolate(double x, const std::vector<double> &x_vals, const std::vector<double> &y_vals)
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

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    wheeled_biped_controller::WheeledBipedController,
    controller_interface::ControllerInterface)