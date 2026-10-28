"""S6：速度桥 /cmd_vel → Go2W（启动后为上锁状态）

ros2 launch go2w_bringup control.launch.py
解锁：ros2 service call /go2w_cmd_bridge/arm std_srvs/srv/SetBool "{data: true}"
上锁：ros2 service call /go2w_cmd_bridge/arm std_srvs/srv/SetBool "{data: false}"
需要里程计在运行（/odom_healthy 为 true）才能解锁。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2w_bringup')
    return LaunchDescription([
        Node(
            package='go2w_bringup',
            executable='go2w_cmd_bridge.py',
            name='go2w_cmd_bridge',
            output='screen',
            parameters=[os.path.join(share, 'config', 'cmd_bridge.yaml')],
            sigterm_timeout='5',
        ),
    ])
