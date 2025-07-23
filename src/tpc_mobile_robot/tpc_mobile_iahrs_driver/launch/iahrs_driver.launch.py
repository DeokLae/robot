#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    iahrs_driver_node = Node(
        package='tpc_mobile_iahrs_driver',
        executable='tpc_mobile_iahrs_driver',
        output='screen',
        parameters=[
            {"m_single_TF_option_": True}
        ]
    )

    # create and return launch description object
    return LaunchDescription(
        [iahrs_driver_node]
    )
