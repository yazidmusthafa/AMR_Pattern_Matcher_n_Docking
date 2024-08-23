
# Copyright 2020 ros2_control Development Team
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    gazebo_params_file = os.path.join(get_package_share_directory('diff_drive'),'config','gazebo_params.yaml')

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diff_drive"), "config", "diff_drive.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content, 'use_sim_time': True}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diff_drive"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diff_drive"), "config", "diffbot.rviz"]
    )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_controllers],
    #     output="both",
    #     remappings=[
    #         ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
    #     ],
    # )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            # ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),                      ## The unstamped cmd_vel is mapped to normal due to some issues.
            # ("/diffbot_base_controller/odom", "/odom"),
            # ("/diffbot_base_controller/transition_event", "/transition_event"),
            ("/controller_manager/robot_description", "/robot_description")
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["diffbot_base_controller", "--param-file", robot_controllers],
        arguments=["diffbot_base_controller"],
    )

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'humanoid_double_arm',
                                   '-topic', '/robot_description',
                                  ],
                        output='screen',
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        gazebo,
        spawn_robot,
        # control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
