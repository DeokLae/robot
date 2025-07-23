#!/usr/bin/env python3
import xacro
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

#from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

  #mvoeit_configall = (
  #  MoveItConfigsBuilder('tm5-700')
  #  .planning_pipelines(
  #    pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
  #  )
  #  .to_moveit_configs()
  #)

  robot_description_config = xacro.process_file(
    os.path.join(
      get_package_share_directory("tm5_moveit_config"),
      'config',
      'tm5-700.urdf.xacro',
    )
  )
  robot_description = {"robot_description": robot_description_config.toxml()}

  robot_description_semantic_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'tm5-700.srdf'
  )
  with open(robot_description_semantic_path, "r") as file:
    robot_description_semantic_config = file.read()
  robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

  kinematics_yaml_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'kinematics.yaml',
  )
  with open(kinematics_yaml_path, "r") as file:
    kinematics_yaml = yaml.safe_load(file)
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
  ompl_planning_pipeline_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'ompl_planning.yaml',
  )
  with open(ompl_planning_pipeline_path, "r") as file:
    ompl_planning_yaml = yaml.safe_load(file)
  ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

  trajectory_execution = {
    "moveit_manage_controllers": True,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
  }

  moveit_simple_controller_yaml_path = os.path.join(
    get_package_share_directory('tm5_moveit_config'),
    'config',
    'moveit_controllers.yaml',
  )
  with open(moveit_simple_controller_yaml_path, "r") as file:
    moveit_simple_controller_yaml = yaml.safe_load(file)
  moveit_controllers = {
    "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    "moveit_simple_controller_manager" : moveit_simple_controller_yaml,
  }

  planning_scene_monitor_parameters = {
    "publish_planning_scene": True,
    "publish_geometry_updates": True,
    "publish_state_updates": True,
    "publish_transforms_updates": True,
    "publish_robot_description":True,
    "publish_robot_description_semantic":True,
  }

  ld = LaunchDescription()
  use_sim =LaunchConfiguration('use_sim')
  declare_use_sim = DeclareLaunchArgument(
    'use_sim',
    default_value='false',
    description='Start robot in Gazebo simulation'
  )
  ld.add_action(declare_use_sim)

  move_group_node = Node(
    package='moveit_ros_move_group',
    executable='move_group',
    output='screen',
    parameters=[
      robot_description,
      robot_description_semantic,
      kinematics_yaml,
      ompl_planning_pipeline_config,
      trajectory_execution,
      moveit_controllers,
      planning_scene_monitor_parameters,
      {'use_sim_time': use_sim},
    ],
  )
  ld.add_action(move_group_node)

  return ld
