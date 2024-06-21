
import os
from os import pathsep

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, GroupAction, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import PathJoinSubstitution, FindExecutable, LaunchConfiguration, Command

import xacro 
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Directories for included launch files
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    package_path = os.path.join(
        get_package_share_directory('pattern_matcher'))
    
    xacro_file = os.path.join(package_path,'urdf','pattern.urdf')

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rviz_config = (
            get_package_share_directory("pattern_matcher") + "/config/rviz_1.rviz")
    
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'])
        ),
        
        # Pattern model description parameter
        # DeclareLaunchArgument(
        #     'pattern_description',
        #     default_value = pattern_description,
        #     description='Path to pattern URDF file'
        # ),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description':  LaunchConfiguration('pattern_description'),
        #     'use_sim_time': True}],
        #     remappings=[
        #         ('/joint_states', '/pattern_joint_states')
        #     ]
        # ),

        GroupAction([
            PushRosNamespace('pattern'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description_raw
                }],
            ),

            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'pattern',
                    '-topic', 'robot_description',
                    '-x', '-0.5',
                    '-y', '-0.5',
                    '-z', '0.25',
                    '-Y', '-1.5707'
                ],
                output='screen'
            )
        ]),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],

        ),

        Node(
            name="scan_to_pointcloud_node",
            package="pattern_matcher",
            executable="scan_to_pointcloud_node",
            output="screen",
            parameters=[
                {"use_sim_time": True}
                ],
        ),
    ])
