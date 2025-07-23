#!/usr/bin/env python3
import os
import xacro
import yaml

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
  declare_argument = []
  declare_argument.append(
    DeclareLaunchArgument(
      'start_rviz',
      default_value='false',
      description='whether execute rviz2'
    )
  )

  declare_argument.append(
    DeclareLaunchArgument(
      'prefix',
      default_value='""',
      description='Prefix of the joint and link names'
    )
  )
  declare_argument.append(
    DeclareLaunchArgument(
      'use_sim',
      default_value='false',
      description='Start robot in Gazebo simulaution'
    )
  )
  declare_argument.append(
    DeclareLaunchArgument(
      'use_fake_hardware',
      default_value='false',
      description='Start robot with fake hardware mirronig command to tis states'
    )
  )
  declare_argument.append(
    DeclareLaunchArgument(
      'fake_sensor_commands',
      default_value='false',
      description='Enable fake command interfaces for sensors used for simple simulations. Used only if use_fale_hardware parameter is ture'
    )
  )

  start_rviz = LaunchConfiguration('start_rviz')
  prefix = LaunchConfiguration('prefix')
  use_sim = LaunchConfiguration('use_sim')
  use_fake_hardware = LaunchConfiguration('use_fake_hardware')
  fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

  urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('tm5_moveit_config'),
                    'config',
                    'tm5-700.urdf.xacro'
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
            fake_sensor_commands,
        ]
    )

  controller_manager_config = PathJoinSubstitution([
    FindPackageShare("tm5_bringup"),
    'config',
    'ethercat_controller_manager.yaml',
  ])

  rviz_config_file = PathJoinSubstitution([
    FindPackageShare('tm5_bringup'),
    'rviz',
    'moveit.rviz',
  ])

#{'robot_description': urdf_file},
  control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
      {'robot_description': urdf_file},
      controller_manager_config
    ],
    remappings=[
      ('~/cmd_Vel_unstamped', 'cmd_vel'),
      ('~/odom', 'odom')
    ],
    output="both",
    condition=UnlessCondition(use_sim)
  )

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description':urdf_file, 'use_sim_time': use_sim}],
    output='screen'
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen',
    condition=IfCondition(start_rviz)
  )

  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    output='screen',
  )

  arm_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['tm_arm_controller'],
    output='screen',
  )

  delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=joint_state_broadcaster_spawner,
      on_exit=[rviz_node],
    )
  )

  delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=joint_state_broadcaster_spawner,
      on_exit=[arm_controller_spawner],
    )
  )
  joint_state_pub_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    #namespace='dumy',
    #, 'use_sim_time': use_sim
    parameters=[{'robot_description':urdf_file,'use_sim_time': use_sim}],
    output='screen',
  )

   # Static TF
  static_tf_node = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="static_transform_publisher",
      output="log",
      arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
        "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
        "--frame-id", "world", "--child-frame-id", "link_0"]
  )

  nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        #joint_state_pub_node,
        #static_tf_node,
    ]
  return LaunchDescription(declare_argument + nodes)


