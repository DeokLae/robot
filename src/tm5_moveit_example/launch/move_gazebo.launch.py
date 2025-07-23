#!/usr/bin/env python3

import os
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  ld = LaunchDescription()
  launch_dir = os.path.join(
    get_package_share_directory('tm5_moveit_example'), 'launch'
  )

  urdf_file_config = xacro.process_file(
    os.path.join(
      get_package_share_directory("tm5_moveit_example"),
      "config",
      "tm5-700.urdf.xacro",
    )
  )
  urdf_file = {"robot_description": urdf_file_config.toxml()}

  pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00'),
            'CONNECT' : LaunchConfiguration('connected_to', default='world')}


  demo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([launch_dir, '/demo.launch.py'])
  )
  ld.add_action(demo_launch)

  #publisher_node = Node(
  #  package='robot_state_publisher',
  #  executable='robot_state_publisher',
  #  parameters=[{'robot_description': urdf_file, 'use_sim_time': 'true'}],
  #  remappings=[],
  #  output='screen',
  #)
  #ld.add_action(publisher_node)

  gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([PathJoinSubstitution([
      FindPackageShare("gazebo_ros"),
      "launch",
      "gazebo.launch.py"
    ])]),
    launch_arguments={}.items(),
  )
  ld.add_action(gazebo_launch)

  gazebo_node = Node(
    package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=[
        '-topic', 'robot_description',
        '-entity', 'tm5_moveiticonfig_system',
        '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
        '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
      ],
      output='screen',
  )



  return ld

