# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
#                    FindPackageShare("pi3hat_hardware_interface"),
                    "/home/pi/ros2_ws/src/rhea_description",
                    "description",
                    "rhea.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
#            FindPackageShare("pi3hat_hardware_interface"),
            "/home/pi/ros2_ws/src/wheeled_biped_controller",
            "test",
            "config.yaml",
        ]
    )
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("ros2_control_demo_example_1"), "rviz", "rrbot.rviz"]
    # )
    ekf_yaml = "/home/pi/ros2_ws/src/wheeled_biped_controller/config/ekf.yaml"
    teleop_config_filepath = "/home/pi/ros2_ws/src/wheeled_biped_controller/config/teleop.config.yaml"

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }])
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node', parameters=[teleop_config_filepath],
        remappings={('/cmd_vel', 'cmd_vel')},
        )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_yaml],
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # imu_sensor_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheeled_biped_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        joy_node,
        teleop_twist_joy_node,
        # ekf_node,
        # joint_state_broadcaster_spawner,
        # imu_sensor_broadcaster_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
