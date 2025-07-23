#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
  launch_dir = os.path.join(
    get_package_share_directory('tm5_moveit_config'),'launch'
  )

  bringup_dir = os.path.join(
    get_package_share_directory('tm5_bringup'),'launch'
  )

  gui_dir = os.path.join(
    get_package_share_directory('tm5_gui'), 'launch'
  )

  ld = LaunchDescription()

  rviz_lauch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py']),
    launch_arguments={'start_rviz': 'false'}.items(),
  )
  #ld.add_action(rviz_lauch)

  move_group_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py']),
    launch_arguments={'use_sim': 'false',}.items(),
  )
  ld.add_action(move_group_launch)

  base_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([bringup_dir,'/base.launch.py'])
  )
  ld.add_action(base_launch)

  servo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/servo.launch.py'])
  )
  ld.add_action(servo_launch)

  teleop_node = Node(
    package='tm5_teleop',
    executable='tm5_teleop',
    output='screen',
  )
  #ld.add_action(teleop_node)

  gui_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([gui_dir, '/tm5_gui.launch.py'])
  )
  ld.add_action(gui_node)

  return ld
