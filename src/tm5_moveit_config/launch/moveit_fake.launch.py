#!/usr/bin/env python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

  launch_dir = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'launch',
  )

  bringup_launch_dir = os.path.join(
    get_package_share_directory('tm5_bringup'),
    'launch',
  )
  ld = LaunchDescription()

  rviz_lauch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py']),
    launch_arguments={'start_rviz': 'false'}.items(),
  )
  ld.add_action(rviz_lauch)

  move_group_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py']),
    launch_arguments={'use_sim': 'true',}.items(),
  )
  ld.add_action(move_group_launch)

  rviz_arg = DeclareLaunchArgument(
    'start_rviz',
    default_value='false',
    description='Whether execute rviz2',
  )
  ld.add_action(rviz_arg)

  fake_ros2_control_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([bringup_launch_dir, '/fake.launch.py'])
  )
  ld.add_action(fake_ros2_control_launch)

  return ld
