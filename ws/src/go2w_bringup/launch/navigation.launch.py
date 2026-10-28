"""S7：完整导航 = 定位（里程计 + 地图 + AMCL）+ 速度桥 + Nav2

ros2 launch go2w_bringup navigation.launch.py map:=/go2w/data/maps/lab_20261021.yaml
启动后：
  1. RViz 里 2D Pose Estimate 设置初始位姿
  2. 解锁速度桥：ros2 service call /go2w_cmd_bridge/arm std_srvs/srv/SetBool "{data: true}"
  3. RViz 里 2D Goal Pose 发目标
速度指令链：controller_server → cmd_vel_nav → velocity_smoother → /cmd_vel → go2w_cmd_bridge
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
    params = os.path.join(share, 'config', 'nav2.yaml')
    lifecycle_nodes = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                       'bt_navigator', 'waypoint_follower', 'velocity_smoother']

    def nav2(package, executable, name=None, remappings=None):
        return Node(package=package, executable=executable, name=name or executable, output='screen',
                    parameters=[params], remappings=remappings or [])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/go2w/data/maps/lab_20261021.yaml'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization.launch.py')),
            launch_arguments={'map': LaunchConfiguration('map')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'control.launch.py'))),

        nav2('nav2_controller', 'controller_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_smoother', 'smoother_server'),
        nav2('nav2_planner', 'planner_server'),
        nav2('nav2_behaviors', 'behavior_server'),
        nav2('nav2_bt_navigator', 'bt_navigator'),
        nav2('nav2_waypoint_follower', 'waypoint_follower'),
        nav2('nav2_velocity_smoother', 'velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True, 'node_names': lifecycle_nodes}],
        ),
    ])
