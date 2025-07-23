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

  declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true'
  )
  ld.add_action(declare_use_sim_time)

  rviz_lauch = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py']),
    PythonLaunchDescriptionSource(
      os.path.join(launch_dir, 'moveit_rviz.launch.py')
    ),
    launch_arguments={'start_rviz': 'false', 'use_sim': 'true'}.items(),
  )
  ld.add_action(rviz_lauch)

  move_group_launch = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py']),
    PythonLaunchDescriptionSource(
      os.path.join(launch_dir, 'move_group.launch.py')
    ),
    launch_arguments={'use_sim': 'true'}.items(),
  )
  ld.add_action(move_group_launch)

  #world_path = PathJoinSubstitution(
  #  [FindPackageShare('turtlebot3_manipulation_bringup'), 'worlds', 'empty_world.model']
  #)
  world_path = os.path.join(
    get_package_share_directory('turtlebot3_manipulation_bringup'),
    'worlds',
    'empty_world.model'
  )

  gazebo_launch = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([bringup_dir, '/gazebo.launch.py']),
    PythonLaunchDescriptionSource(
      os.path.join(bringup_dir, 'gazebo.launch.py')
    ),
    launch_arguments={
      'world': world_path,
      'x_pose': '0.0',
      'y_pose': '0.0',
      'z_pose': '0.0',
      'roll': '0.0',
      'pitch': '0.0',
      'yaw': '0.0',
      'use_sim_time': 'true'}.items(),
  )
  ld.add_action(gazebo_launch)

  servo_node = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([launch_dir, '/servo.launch.py']),
    PythonLaunchDescriptionSource(
      os.path.join(launch_dir, 'servo.launch.py')
    ),
    launch_arguments={'use_sim': 'true'}.items(),
  )

  ld.add_action(servo_node)

  gui_node = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([gui_dir, '/tm5_gui.launch.py']),
    PythonLaunchDescriptionSource(
      os.path.join(gui_dir, 'tm5_gui.launch.py')
    )
  )

  #ld.add_action(gui_node)
  return ld
