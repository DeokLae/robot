import os
import yaml

from launch import LaunchDescription

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():
  ld = LaunchDescription()

  use_sim = LaunchConfiguration('use_sim')
  declare_use_sim = DeclareLaunchArgument(
    'use_sim',
    default_value='false',
    description='Satart robot in Gazebo simulation',
  )
  ld.add_action(declare_use_sim)

  servo_params_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'moveit_servo.yml'
  )
  with open(servo_params_path, 'r') as file:
    servo_params = {'moveit_servo': yaml.safe_load(file)}

  robot_description_config = xacro.process_file(
    os.path.join(
      get_package_share_directory('tm5_moveit_config'),
      'config',
      'tm5-700.urdf.xacro',
    )
  )
  robot_description = {'robot_description':robot_description_config.toxml()}

  robot_description_sementic_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'tm5-700.srdf',
  )

  with open(robot_description_sementic_path, 'r') as file:
    robot_description_sementic_config = file.read()
  robot_description_sementic = {'robot_description_semantic':robot_description_sementic_config}

  kinemetic_yaml_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'kinematics.yaml',
  )
  with open(kinemetic_yaml_path, 'r') as file:
    kinemetic_yaml = yaml.safe_load(file)

  servo_node = Node(
    package='moveit_servo',
    executable='servo_node_main',
    parameters=[
      {'use_gazebo':use_sim,},
      servo_params,
      robot_description,
      robot_description_sementic,
      kinemetic_yaml,
    ],
  )
  ld.add_action(servo_node)
  return ld
