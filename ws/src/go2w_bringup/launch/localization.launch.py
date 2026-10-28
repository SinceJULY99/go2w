"""S5：里程计 + 2D scan + 已保存地图 + AMCL 定位

ros2 launch go2w_bringup localization.launch.py map:=/go2w/data/maps/lab_20261021.yaml
启动后在 RViz 里用 2D Pose Estimate 给出机器人在地图上的位置和朝向。
不要和 mapping.launch.py 同时运行（slam_toolbox 和 AMCL 都会发布 map → odom）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2w_bringup')
    config = os.path.join(share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/go2w/data/maps/lab_20261021.yaml'),

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
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(config, 'amcl.yaml')],
        ),

        # 自动 configure + activate map_server 和 AMCL，不用再手动敲 lifecycle 命令
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}],
        ),
    ])
