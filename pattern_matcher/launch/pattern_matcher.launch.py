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
    config_dir = os.path.join(get_package_share_directory('pattern_matcher'),'config')
    map_file = os.path.join(config_dir,'room_map.yaml')
    param_file = os.path.join(config_dir,'tb3_nav2_params.yaml')

    package_path = os.path.join(
        get_package_share_directory('pattern_matcher'))
    
    xacro_file = os.path.join(package_path,'urdf','pattern.urdf')

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rviz_config = (
            get_package_share_directory("pattern_matcher") + "/config/rviz.rviz")
    
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('diff_drive'), 'launch'), '/ros2_control.launch.py'])
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/turtlebot3_room.launch.py'])
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': param_file}.items(),
        ),

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
                    '-x', '2.023530',
                    '-y', '0.009256',
                    '-z', '0.25',
                    '-Y', '-1.5707'
                ],
                output='screen'
            )
        ]),

        # RViz
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     output="log",
        #     arguments=["-d", rviz_config],

        # ),

        Node(
            package="pattern_matcher",
            executable="robot_control",
            output="screen",
            parameters=[
                {"use_sim_time": True}
                ],
        ),
    ])
