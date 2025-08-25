#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_go2_gazebo = FindPackageShare('go2_gazebo')


    desc_prefix = get_package_prefix('go2_description')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        value=f"{desc_prefix}/share:" + "$(env GAZEBO_MODEL_PATH)"
    )




    world_file = LaunchConfiguration('world_file', default='test.world')
    declare_world = DeclareLaunchArgument(
        'world_file',
        default_value='lab_emp.world',
        description='World file inside go2_gazebo/worlds'
    )

    world_path = PathJoinSubstitution([pkg_go2_gazebo, 'worlds', world_file])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'verbose': 'true',
                        'gui': 'true',
                        'world': world_path}.items()
    )

    return LaunchDescription([set_model_path, declare_world, gazebo])
