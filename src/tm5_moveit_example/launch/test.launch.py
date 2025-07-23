#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  ld = LaunchDescription()
  launch_dir = os.path.join(
    get_package_share_directory('tm5_moveit_example'),
  'launch',
  )

  rivz_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py'])
  )

  move_group_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py'])
  )

  rsp_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/rsp.launch.py'])
  )

  spawn_controllers_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/spawn_controllers.launch.py'])
  )

  static_virtual_joint_tfs_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([launch_dir, '/static_virtual_joint_tfs.launch.py'])
  )

  ld.add_action(rivz_launch)
  ld.add_action(move_group_launch)
  ld.add_action(rsp_launch)
  ld.add_action(spawn_controllers_launch)
  ld.add_action(static_virtual_joint_tfs_launch)


  return ld
