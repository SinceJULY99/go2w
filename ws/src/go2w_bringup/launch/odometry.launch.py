"""S3：雷达驱动 + FAST-LIO + 平面里程计（odom → base_footprint）

ros2 launch go2w_bringup odometry.launch.py
启动时机器人应静止放在平地上：FAST-LIO 用最初的 IMU 数据初始化，odom 原点就是此刻的位置。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2w_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'lidar.launch.py')),
            launch_arguments={'xfer_format': '1'}.items(),
        ),

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',
            output='screen',
            parameters=[os.path.join(share, 'config', 'fast_lio_mid360.yaml')],
            # FAST-LIO 固定发布 camera_init → body；移出全局 TF，避免出现第二棵树
            remappings=[('/tf', '/fast_lio/tf')],
        ),

        Node(
            package='go2w_bringup',
            executable='planar_odom.py',
            name='planar_odom',
            output='screen',
        ),
    ])
