import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('tpc_mobile_robot_slam'),
        'config', 'slam_config.yaml'
    )

    return LaunchDescription([
        # SLAM Toolbox 노드 실행
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file]
        ),

        # RViz2 노드 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('tpc_mobile_robot_slam'),
                'rviz', 'slam_toolbox.rviz'
            )]
        )
    ])
