import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  teleop_config = os.path.join(get_package_share_directory('tpc_mobile_joy_teleop'), 'config', 'teleop_config.yaml')

  ld = LaunchDescription()
  teleop_node = Node(
    package='tpc_mobile_joy_teleop',
    executable = 'tpc_mobile_joy_teleop',
    output='screen',
    parameters=[teleop_config]
  )
  ld.add_action(teleop_node)
  return ld
