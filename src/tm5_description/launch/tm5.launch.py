import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_display_config_file = os.path.join(
        get_package_share_directory('tm5_description'),
        'rviz',
        'tm5.rviz')
    urdf_file = os.path.join(
        get_package_share_directory('tm5_description'),
        'urdf',
        'tm5-700-nominal.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_file = infp.read()

    #urdf_file = xacro.process_file(
    #    os.path.join(
    #        get_package_share_directory("tm5_description"),
    #        "xacro",
    #        "tm5-700.urdf.xacro"
    #    )
    #)
    #robot_description_file = {"robot_description": urdf_file.toxml()}

    prefix = LaunchConfiguration('prefix')
    #robot_description_file = Command(
    #    [
    #        PathJoinSubstitution([FindExecutable(name='xacro')]),
    #        ' ',
    #        PathJoinSubstitution(
    #            [
    #                FindPackageShare('tm5_moveit_config'),
    #                'config',
    #                'tm5-700.urdf.xacro'
    #            ]
    #        ),
    #        ' ',
    #        'prefix:=',
    #        ' ',
    #        ' ',
    #        'use_fake_hardware:=',
    #        'True',
    #    ]
    #)

    ld = LaunchDescription()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_file}
        ],
        output='screen')

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_display_config_file],
        output='screen')

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)

    return ld
