"""S4：里程计 + 2D scan + slam_toolbox 建图

ros2 launch go2w_bringup mapping.launch.py
建完后保存：bash /go2w/scripts/save_map.sh <地图名>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2w_bringup')
    config = os.path.join(share, 'config')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'odometry.launch.py'))),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pcl2scan_slam',
            parameters=[os.path.join(config, 'pcl2scan_slam.yaml')],
            remappings=[('cloud_in', '/cloud_registered_body'), ('scan', '/scan')],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(config, 'slam_toolbox_mapping.yaml')],
        ),
    ])
