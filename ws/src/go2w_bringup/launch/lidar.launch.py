"""S2：MID360 驱动 + 机器人描述（base_footprint → base_link → livox_frame）

ros2 launch go2w_bringup lidar.launch.py
注意：同一时间只能有一台机器运行雷达驱动（Orin、扩展坞、PC2 上的 ~/start-mid360.sh 只能开一个）。
雷达把数据发给 host IP：Orin 是 .51、扩展坞是 .50，由 ros_env.sh 的 GO2W_LIDAR_HOST_IP 决定，
启动时以 config/MID360_config.json 为模板替换后写到 /tmp。
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def livox_driver(context, share):
    host_ip = LaunchConfiguration('lidar_host_ip').perform(context)
    with open(os.path.join(share, 'config', 'MID360_config.json')) as f:
        cfg = json.load(f)
    host = cfg['MID360']['host_net_info']
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        host[key] = host_ip
    config_path = '/tmp/go2w_MID360_config.json'
    with open(config_path, 'w') as f:
        json.dump(cfg, f, indent=2)

    return [Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': LaunchConfiguration('xfer_format'),
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': config_path,
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory('go2w_bringup')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'go2w.urdf.xacro')]), value_type=str)

    return LaunchDescription([
        # 0 = PointCloud2（RViz、调试直接可看），1 = Livox CustomMsg（FAST-LIO 需要）
        DeclareLaunchArgument('xfer_format', default_value='1'),
        DeclareLaunchArgument(
            'lidar_host_ip',
            default_value=EnvironmentVariable('GO2W_LIDAR_HOST_IP', default_value='192.168.1.51')),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        OpaqueFunction(function=livox_driver, args=[share]),
    ])
