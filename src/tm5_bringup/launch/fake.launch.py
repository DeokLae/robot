#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

  bringup_launch_dir = os.path.join(
    get_package_share_directory('tm5_bringup'),
    'launch',
  )
  start_rviz = LaunchConfiguration('start_rviz')
  prefix = LaunchConfiguration('prefix')
  use_fake_hardware = LaunchConfiguration('use_fake_hardware')
  fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
  use_sim = LaunchConfiguration('use_sim')

  ld = LaunchDescription()
  use_sim_arg = DeclareLaunchArgument(
    'use_sim',
    default_value='true',
    description='use sim time',
  )
  ld.add_action(use_sim_arg)
  start_rviz_arg = DeclareLaunchArgument(
      'start_rviz',
      default_value='true',
      description='Whether execute rviz2',
    )
  ld.add_action(start_rviz_arg)

  prefix_arg = DeclareLaunchArgument(
      'prefix',
      default_value='""',
      description='Prefix of the joint and link names',
    )
  ld.add_action(prefix_arg)
  use_fake_hardware_arg = DeclareLaunchArgument(
      'use_fake_hardware',
      default_value='true',
      description='Start robot with fake hardware mirronig command to tis states'
    )
  ld.add_action(use_fake_hardware_arg)
  fake_sensor_commands_arg = DeclareLaunchArgument(
      'fake_sensor_commands',
      default_value='true',
      description='Enable fake command interfaces for sensors used for simple simulations. Used only if use_fale_hardware parameter is ture'
    )
  ld.add_action(fake_sensor_commands_arg)

  base_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([bringup_launch_dir, '/base.launch.py']),
    launch_arguments={
      'start_rviz': start_rviz,
      'prefix': prefix,
      'use_fake_hardware': use_fake_hardware,
      'fake_sensor_commands': fake_sensor_commands,
      'use_sim': use_sim,
    }.items(),
  )

  ld.add_action(base_launch)
  return ld

