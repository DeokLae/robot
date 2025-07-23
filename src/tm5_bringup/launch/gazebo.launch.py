#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [
                FindPackageShare('tm5_bringup'),
                'worlds',
                'empty_world.model'
            ]
        )
    )
  start_rviz = LaunchConfiguration('start_rviz')
  prefix = LaunchConfiguration('prefix')
  use_sim = LaunchConfiguration('use_sim')

  pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
          'y': LaunchConfiguration('y_pose', default='-0.50'),
          'z': LaunchConfiguration('z_pose', default='0.00'),
          'R': LaunchConfiguration('roll', default='0.00'),
          'P': LaunchConfiguration('pitch', default='0.00'),
          'Y': LaunchConfiguration('yaw', default='0.00')}

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim',
      default_value='true',
      description='Start tobot in Gazebo simulation'),
    DeclareLaunchArgument(
      'prefix',
      default_value='""',
      description='Previx of the joint and link names'),
    DeclareLaunchArgument(
      'start_rviz',
      default_value='false',
      description='whether execute rviz2'),
    DeclareLaunchArgument(
      'x_pose',
      default_value=pose['x'],
      description='position of turtlebot3'),

    DeclareLaunchArgument(
      'y_pose',
      default_value=pose['y'],
      description='position of turtlebot3'),

    DeclareLaunchArgument(
      'z_pose',
      default_value=pose['z'],
      description='position of turtlebot3'),

    DeclareLaunchArgument(
      'roll',
      default_value=pose['R'],
      description='orientation of turtlebot3'),

    DeclareLaunchArgument(
      'pitch',
      default_value=pose['P'],
      description='orientation of turtlebot3'),

    DeclareLaunchArgument(
      'yaw',
      default_value=pose['Y'],
      description='orientation of turtlebot3'),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/base.launch.py']),
      launch_arguments={
        'start_rviz': start_rviz,
        'prefix': prefix,
        'use_sim': use_sim,
      }.items(),
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [PathJoinSubstitution(
          [FindPackageShare('gazebo_ros'),'launch','gazebo.launch.py']
        )]
      ),
      launch_arguments={
        'verbose': 'false',
        'world': world,
      }.items(),
    ),

    Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=[
        '-topic', 'robot_description',
        '-entity', 'myposition',
        '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
        '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
      ],
      output='screen'
    )
  ])
