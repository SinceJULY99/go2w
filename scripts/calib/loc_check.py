#!/usr/bin/env python3
"""S5 定位检查：每秒打印 AMCL 位姿、不确定度，以及 scan 与地图的贴合率。

贴合率 = 有效激光点中，落在地图障碍物 tol 米以内的比例。定位准确时应明显偏高；
家具挪动、有人走动会让它下降一些，但不应该在行走中突然掉很多。
只订阅，不发布。用法（容器内）：python3 loc_check.py [--tol 0.15] [--seconds 600]
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class LocCheck(Node):
    def __init__(self, tol):
        super().__init__('loc_check')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.tol = tol
        self.near = None
        self.pose = None
        self.scan = None

    def on_map(self, m):
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        occ = g > 50
        r = int(math.ceil(self.tol / m.info.resolution))
        near = occ.copy()
        for dy in range(-r, r + 1):        # 方形膨胀：障碍物 tol 范围内都算"贴合"
            for dx in range(-r, r + 1):
                near |= np.roll(np.roll(occ, dy, 0), dx, 1)
        self.near, self.info = near, m.info
        self.get_logger().info('地图 %d×%d，分辨率 %.2f m' % (m.info.width, m.info.height, m.info.resolution))

    def on_pose(self, m):
        self.pose = m

    def on_scan(self, m):
        self.scan = m

    def report(self, t0):
        if self.near is None or self.scan is None or self.pose is None:
            return
        s = self.scan
        try:
            tf = self.tf.lookup_transform('map', s.header.frame_id, Time())
        except Exception:  # noqa: BLE001  AMCL 还没发布 map → odom
            return
        q = tf.transform.rotation
        yaw = 2 * math.atan2(q.z, q.w)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        r = np.array(s.ranges)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
        x = tx + r[ok] * np.cos(a[ok] + yaw)
        y = ty + r[ok] * np.sin(a[ok] + yaw)
        i = ((x - self.info.origin.position.x) / self.info.resolution).astype(int)
        j = ((y - self.info.origin.position.y) / self.info.resolution).astype(int)
        inside = (i >= 0) & (j >= 0) & (i < self.info.width) & (j < self.info.height)
        match = self.near[j[inside], i[inside]].mean() if inside.any() else float('nan')
        c = self.pose.pose.covariance
        p = self.pose.pose.pose
        print('%6.1fs  x=%+6.2f y=%+6.2f yaw=%+6.1f°  std=(%.3f, %.3f m, %.1f°)  贴合率 %.0f%%（%d 点）'
              % (time.time() - t0, p.position.x, p.position.y,
                 math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w)),
                 math.sqrt(c[0]), math.sqrt(c[7]), math.degrees(math.sqrt(c[35])),
                 100 * match, inside.sum()), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--seconds', type=float, default=600)
    args = ap.parse_args()
    rclpy.init()
    node = LocCheck(args.tol)
    t0 = last = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - last >= 1.0:
            last = time.time()
            node.report(t0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
