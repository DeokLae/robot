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

  rivz_config = os.path.join(
    get_package_share_directory("tm5_moveit_config"),
    "config",
    "moveit.rviz"
  )

  urdf_file_config = xacro.process_file(
    os.path.join(
      get_package_share_directory("tm5_moveit_config"),
      "config",
      "tm5-700.urdf.xacro",
    )
  )
  robot_description =  {"robot_description": urdf_file_config.toxml()}

  robot_description_semantic_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'tm5-700.srdf',
  )
  with open(robot_description_semantic_path, "r") as file:
    robot_description_semantic_config = file.read()
  robot_description_semantic = {"robot_description_semantic" : robot_description_semantic_config}
#default_planner_request_adapters/ResolveConstraintFrames \
  ompl_planning_pipeline_config = {
    "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization \
            default_planner_request_adapters/FixWorkspaceBounds \
            default_planner_request_adapters/FixStartStateBounds \
            default_planner_request_adapters/FixStartStateCollision \
            default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
  }
  ompl_planning_yaml_path = os.path.join(
    get_package_share_directory("tm5_moveit_config"),
    "config",
    "ompl_planning.yaml",
  )
  with open(ompl_planning_yaml_path, "r") as file:
    ompl_planning_yaml = yaml.safe_load(file)
  ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

  kinematics_yaml_path = os.path.join(
    get_package_share_directory("tm5_moveit_config"),
    "config",
    "kinematics.yaml",
  )
  with open(kinematics_yaml_path, "r") as file:
    kinematics_yaml = yaml.safe_load(file)

  ld = LaunchDescription()

  use_sim =LaunchConfiguration('use_sim')
  declare_use_sim = DeclareLaunchArgument(
    'use_sim',
    default_value='false',
    description='Start robot in Gazebo simulation'
  )
  ld.add_action(declare_use_sim)

  rviz_node =  Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rivz_config],
    parameters=[
      robot_description,
      robot_description_semantic,
      ompl_planning_pipeline_config,
      kinematics_yaml,
      {'use_sim_time': use_sim},
    ]
  )
  ld.add_action(rviz_node)
  return ld
