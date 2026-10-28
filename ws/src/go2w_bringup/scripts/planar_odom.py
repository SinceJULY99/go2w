#!/usr/bin/env python3
"""把 FAST-LIO 的 3D 位姿换算成机器人底盘的平面里程计。

输入  /Odometry（FAST-LIO：camera_init → body，body 为 MID360 内置 IMU）
输出  TF odom → base_footprint（平面：x、y、yaw）
      TF base_footprint → base_link（机身实时的 roll、pitch）
      /odom（nav_msgs/Odometry，平面）

FAST-LIO 启动时 body 位姿为单位阵，所以
    T_odom_base(t) = T_base_body · T_fastlio(t) · T_base_body⁻¹
odom 原点即启动时机身所在位置；base_link 已按地面校平（静止站立时），odom 因此是水平的。
旋转分解为 Rz(yaw) · R_tilt：yaw 和 x、y 给 base_footprint（2D 建图、定位、Nav2 用），
R_tilt（机身相对水平面的俯仰/横滚）放到 base_footprint → base_link。
这样点云按机身真实姿态变换：机身加减速时前后晃动，地面也不会被"抬高"成障碍物。
z 漂移不发布（平面导航用不到）。

R_tilt 的来源（参数 tilt_source）：
  robot   （默认）Go2W 主控 IMU：域 0 /lowstate 的 imu_state.rpy（约 500 Hz，best-effort），
          以启动后第一秒的站姿为水平基准（base_link 就是按站立时的地面标定的）。
          数据超过 robot_imu_timeout 秒没更新时按水平处理并告警。
  fastlio FAST-LIO 估计的姿态。10-28 实测：快速转向后俯仰/横滚漂到 5–10°，而机身实际是平的
          （主控 IMU 与地面拟合一致，偏差 < 0.5°），校平反而把地面"抬"成障碍物，规划失败。
          见 scripts/calib/tilt_compare.py。

健康状态 /odom_healthy（std_msgs/Bool，10 Hz）：
FAST-LIO 输出中断超过 max_gap 秒（例如网线断开、雷达没数据）后变为 false 并保持，
因为中断期间 FAST-LIO 只靠 IMU 推算，恢复后会发散（实测 25 s 中断后跑到数千公里外）。
不健康后不再发布 /odom 和 TF：下游（slam_toolbox、AMCL、Nav2）拿不到新 TF 就会停下，
而不是吃进错误位姿（曾导致 slam_toolbox 内存涨到 10 GB 被 OOM 杀掉）。需重启里程计恢复。
速度控制和导航应在它为 false 时停车。
"""
import math
import signal
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from unitree_go.msg import LowState


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    """旋转矩阵 → (x, y, z, w)，用于小角度的俯仰/横滚，w 始终为正。"""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)


def to_T(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tilt_R(roll, pitch):
    """去掉偏航后的机身姿态 Ry(pitch) · Rx(roll)。"""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])


class RobotImu(Node):
    """域 0：读主控 IMU 的 roll/pitch，前 ref_samples 帧的平均作为水平基准。"""

    def __init__(self, context, ref_samples=500):
        super().__init__('planar_odom_robot', context=context)
        self.lock = threading.Lock()
        self.ref_samples = ref_samples
        self.acc = []
        self.R_ref = None
        self.rp = None            # 最新 (roll, pitch)
        self.stamp = None         # 收到最新一帧时的本机时间（秒）
        self.create_subscription(LowState, 'lowstate', self.on_state, qos_profile_sensor_data)

    def on_state(self, m):
        rp = (float(m.imu_state.rpy[0]), float(m.imu_state.rpy[1]))
        with self.lock:
            self.rp, self.stamp = rp, time.monotonic()
            if self.R_ref is None:
                self.acc.append(rp)
                if len(self.acc) >= self.ref_samples:
                    r0, p0 = np.mean(self.acc, axis=0)
                    self.R_ref = tilt_R(r0, p0)
                    self.get_logger().info('主控 IMU 水平基准：roll %.2f°  pitch %.2f°' % (
                        math.degrees(r0), math.degrees(p0)))

    def tilt(self, timeout):
        """相对基准的 R_tilt；还没定好基准或数据过期时返回 None。"""
        with self.lock:
            if self.R_ref is None or time.monotonic() - self.stamp > timeout:
                return None
            return tilt_R(*self.rp) @ self.R_ref.T


class PlanarOdom(Node):
    def __init__(self, context, robot_imu):
        super().__init__('planar_odom', context=context)
        self.robot_imu = robot_imu
        self.tilt_source = self.declare_parameter('tilt_source', 'robot').value
        self.robot_imu_timeout = self.declare_parameter('robot_imu_timeout', 0.2).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.body_frame = self.declare_parameter('body_frame', 'base_link').value
        self.imu_frame = self.declare_parameter('imu_frame', 'body').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        # 速度的一阶低通系数（0–1，越大越跟手、越抖）
        self.alpha = self.declare_parameter('velocity_alpha', 0.5).value
        self.max_gap = self.declare_parameter('max_gap', 0.5).value

        self.healthy = True
        self.last_rx = None       # 收到上一帧 /Odometry 时的本机时间（秒）
        self.health_pub = self.create_publisher(Bool, 'odom_healthy', 10)
        self.create_timer(0.1, self.check_health)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.create_subscription(Odometry, 'Odometry', self.on_odom, 20)

        self.T_bi = None          # base_link ← body（静态，来自 URDF）
        self.T_ib = None
        self.prev = None          # (t, x, y, yaw)
        self.vel = np.zeros(3)    # vx, vy（底盘系）, wz
        self.max_z = 0.0
        self.create_timer(10.0, self.report)

    def lookup_extrinsic(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.body_frame, self.imu_frame, Time(),
                                                 timeout=Duration(seconds=0.0))
        except Exception as e:  # noqa: BLE001  静态 TF 还没收到时重试
            self.get_logger().info('等待静态 TF %s → %s：%s' % (self.body_frame, self.imu_frame, e),
                                   throttle_duration_sec=2.0)
            return False
        self.T_bi = to_T(tf.transform.translation, tf.transform.rotation)
        self.T_ib = np.linalg.inv(self.T_bi)
        self.get_logger().info('外参 %s → %s：t=(%.3f, %.3f, %.3f)' % (
            self.body_frame, self.imu_frame, *self.T_bi[:3, 3]))
        return True

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_health(self):
        if self.healthy and self.last_rx is not None and self.now() - self.last_rx > self.max_gap:
            self.healthy = False
            self.get_logger().error('FAST-LIO 输出中断 %.1f s（超过 %.1f s）：odom 已不可信，需重启里程计。'
                                    '请检查雷达网线和驱动。' % (self.now() - self.last_rx, self.max_gap))
        self.health_pub.publish(Bool(data=self.healthy and self.last_rx is not None))

    def on_odom(self, msg):
        self.last_rx = self.now()
        if not self.healthy:
            return
        if self.T_bi is None and not self.lookup_extrinsic():
            return
        T = self.T_bi @ to_T(msg.pose.pose.position, msg.pose.pose.orientation) @ self.T_ib
        x, y, z = T[:3, 3]
        yaw = math.atan2(T[1, 0], T[0, 0])
        self.max_z = max(self.max_z, abs(z))
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        if self.prev is not None:
            dt = t - self.prev[0]
            if 1e-3 < dt < 1.0:
                c, s = math.cos(yaw), math.sin(yaw)
                dx, dy = (x - self.prev[1]) / dt, (y - self.prev[2]) / dt
                raw = np.array([c * dx + s * dy, -s * dx + c * dy, wrap(yaw - self.prev[3]) / dt])
                self.vel = self.alpha * raw + (1 - self.alpha) * self.vel
        self.prev = (t, x, y, yaw)

        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw

            R_tilt = self.current_tilt(yaw, T)
            tilt = TransformStamped()
            tilt.header.stamp = msg.header.stamp
            tilt.header.frame_id = self.base_frame
            tilt.child_frame_id = self.body_frame
            (tilt.transform.rotation.x, tilt.transform.rotation.y,
             tilt.transform.rotation.z, tilt.transform.rotation.w) = R_to_quat(R_tilt)
            self.tf_pub.sendTransform([tf, tilt])

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.linear.y = self.vel[0], self.vel[1]
        od.twist.twist.angular.z = self.vel[2]
        self.odom_pub.publish(od)

    def current_tilt(self, yaw, T):
        """base_footprint → base_link 的旋转（机身俯仰/横滚），来源见模块说明。"""
        if self.tilt_source == 'fastlio':
            # R_tilt = Rz(-yaw) · R
            c, s = math.cos(yaw), math.sin(yaw)
            return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ T[:3, :3]
        R = self.robot_imu.tilt(self.robot_imu_timeout)
        if R is None:
            self.get_logger().warn('主控 IMU（域 0 /lowstate）没有数据或还没定好水平基准，机身倾斜按水平处理',
                                   throttle_duration_sec=2.0)
            return np.eye(3)
        return R

    def report(self):
        if self.prev is not None:
            _, x, y, yaw = self.prev
            self.get_logger().info('odom x=%.3f y=%.3f yaw=%.1f°  |z|max=%.3f m（3D 高度漂移，平面化前）  %s'
                                   % (x, y, math.degrees(yaw), self.max_z,
                                      'healthy' if self.healthy else 'UNHEALTHY'))


def main():
    # 两个域各一个 context：域 0 读主控 IMU，导航核心（ROS_DOMAIN_ID，默认 43）发布里程计
    stop_event = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_event.set())

    ctx_robot, ctx_core = Context(), Context()
    rclpy.init(context=ctx_robot, domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
    rclpy.init(context=ctx_core, signal_handler_options=SignalHandlerOptions.NO)
    robot_imu = RobotImu(ctx_robot)
    node = PlanarOdom(ctx_core, robot_imu)
    ex_robot = SingleThreadedExecutor(context=ctx_robot)
    ex_core = SingleThreadedExecutor(context=ctx_core)
    ex_robot.add_node(robot_imu)
    ex_core.add_node(node)

    def spin_robot():
        while not stop_event.is_set():
            ex_robot.spin_once(timeout_sec=0.05)

    t = threading.Thread(target=spin_robot, daemon=True)
    t.start()
    try:
        while not stop_event.is_set():
            ex_core.spin_once(timeout_sec=0.05)
    finally:
        stop_event.set()
        t.join(timeout=1.0)
        node.destroy_node()
        robot_imu.destroy_node()
        rclpy.shutdown(context=ctx_core)
        rclpy.shutdown(context=ctx_robot)


if __name__ == '__main__':
    main()
