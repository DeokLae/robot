#!/usr/bin/env python3
# Copyright 2022 ROBOTIS CO., LTD.
#
# Copyright 2024 TPC Mechatronics Crop.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Original Author: Hye-jong KIM
#
# Modified by: DeokLae Kim (kdl79@tanhay.com)
# Description of modifications : This source code is a modification of the
# turtlebot3_manipulation_bringup package from ROBOTIS, adapted to implement
# tpc_mobile_arm_robot bringup package.


import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from lxml import etree

def generate_launch_description():
  declare_argument=[]
  declare_argument.append(
    DeclareLaunchArgument(
      'start_rviz',
      default_value='false',
      description='whether execute rviz'
    )
  )
  declare_argument.append(
    DeclareLaunchArgument(
      'prefix',
      default_value='""',
      description='prefix of the joint and link names'
    )
  )
  declare_argument.append(
    DeclareLaunchArgument(
      'use_sim',
      default_value='false',
      description='start robot in gazebo'
    )
  )
  declare_argument.append(
    DeclareLaunchArgument(
      'use_fake_hardware',
      default_value='false',
      description='start robot with fake hardware'
    )
  )

  declare_argument.append(
    DeclareLaunchArgument(
      'fake_sensor_commands',
      default_value='false',
      description='enable fake command interfaces for sensors'
    )
  )
  start_rviz = LaunchConfiguration('start_rviz')
  prefix = LaunchConfiguration('prefix')
  use_sim = LaunchConfiguration('use_sim')
  use_fake_hardware = LaunchConfiguration('use_fake_hardware')
  fake_sensor_command = LaunchConfiguration('fake_sensor_commands')

  robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('tpc_mobile_arm_description'),
                    'urdf',
                    'tpc_mobile_arm.urdf.xacro'
                ]
            ),
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'fake_sensor_commands:=',
            fake_sensor_command,
        ]
  )

  mecanum_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('tpc_mobile_arm_description'),
                    'urdf',
                    'tpc_mobile_arm_mecanum.urdf.xacro'
                ]
            ),
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'fake_sensor_commands:=',
            fake_sensor_command,
        ]
    )

  mecanum_controller_manager_config = PathJoinSubstitution(
    [
        FindPackageShare('tpc_mobile_arm_bringup'),
        'config',
        'mecanum_controller_manager.yaml'
    ]
  )

  mobile_control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
      {'robot_description': mecanum_description},
      mecanum_controller_manager_config
    ],
    remappings=[
      ('~/cmd_vel_unstamped', 'cmd_vel'),
      #('~/odom', 'odom')
    ],
    output='both',
    condition=UnlessCondition(use_sim),
    name='mecanumRobot_controller_manager'
  )

  robot_state_pub_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'use_sim_time': use_sim}],
    output='log',
  )

  imu_broadcaster_spawner=Node(
    package='controller_manager',
    executable='spawner',
    arguments=['imu_broadcaster'],
    output='screen'
  )

  joint_state_broadcaster_spawner1 = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster',
               '-c',
               '/controller_manager',
               '--controller-manager',
               '/mecanumRobot_controller_manager'],
    output='log',
  )

  mecanum_drive_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['mecanum_drive_controller',
               '-c',
               '/controller_manager',
               '--controller-manager',
               '/mecanumRobot_controller_manager'],
    output='log',
    condition=UnlessCondition(use_sim),
  )

  odom_filter_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory('tpc_robot_localization'),
                    'launch',),
       '/ekf.launch.py']
    )
  )

  delay_mecanum_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=joint_state_broadcaster_spawner1,
      on_exit=[mecanum_drive_controller_spawner]
    )
  )
  delay_odom_filter_launch_after_mecanum_drive_controller_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=joint_state_broadcaster_spawner1,
      on_exit=[odom_filter_launch]
    )
  )


  nodes = [
    mobile_control_node,
    #robot_state_pub_node,
    joint_state_broadcaster_spawner1,
    delay_mecanum_drive_controller_spawner_after_joint_state_broadcaster_spawner,
    delay_odom_filter_launch_after_mecanum_drive_controller_spawner,
  ]

  return LaunchDescription(declare_argument + nodes)

