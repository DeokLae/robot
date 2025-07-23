#!/usr/bin/env python3

import os
import xacro
import yaml
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  teleop_config = os.path.join(get_package_share_directory('tm5_joy_teleop'), 'config', 'teleop_config.yaml')

  ld = LaunchDescription()

  gui_node =  Node(
    package="tm5_gui",
    executable="tm5_gui",
    name="tm5_gui",
  )
  ld.add_action(gui_node)

  teleop_node =  Node(
    package="tm5_joy_teleop",
    executable="tm5_joy_teleop",
    parameters=[teleop_config]
  )
  #ld.add_action(teleop_node)
  return ld
