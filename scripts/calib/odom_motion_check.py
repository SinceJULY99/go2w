#!/usr/bin/env python3
"""S3 行驶验收：分析录下的 /odom（mcap 包）。

自动按"静止段"把轨迹切成若干运动段，并给出：
  - 直线段：前进方向与 base x 轴的夹角（检查雷达 yaw）
  - 原地转圈段：轨迹圆的圆心与半径 → 雷达相对旋转中心的偏移（lidar_x / lidar_y 的修正量）
  - 全程：终点相对起点的位置和航向误差（回原点误差）

用法（容器内）：python3 odom_motion_check.py /go2w/data/bags/<包目录>
"""
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import rosbag2_py


def load(bag, topic='/odom'):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    rows = []
    while r.has_next():
        name, data, _ = r.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, Odometry)
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rows.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    a = np.array(rows)
    a[:, 3] = np.unwrap(a[:, 3])
    return a


def segments(a, v_still=0.03, w_still=math.radians(3), min_still=2.0):
    """按速度阈值找出运动段（两端由 ≥min_still 秒的静止隔开）。"""
    t = a[:, 0]
    dt = np.gradient(t)
    v = np.hypot(np.gradient(a[:, 1]), np.gradient(a[:, 2])) / dt
    w = np.abs(np.gradient(a[:, 3])) / dt
    k = 5
    kern = np.ones(k) / k
    moving = (np.convolve(v, kern, 'same') > v_still) | (np.convolve(w, kern, 'same') > w_still)
    segs, start = [], None
    for i, mv in enumerate(moving):
        if mv and start is None:
            start = i
        if not mv and start is not None:
            # 只有静止持续够长才算段结束
            j = i
            while j < len(moving) and not moving[j] and t[j] - t[i] < min_still:
                j += 1
            if j >= len(moving) or not moving[j]:
                segs.append((start, i))
                start = None
    if start is not None:
        segs.append((start, len(a) - 1))
    return segs


def fit_circle(x, y):
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    b = x ** 2 + y ** 2
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return cx, cy, math.sqrt(c + cx ** 2 + cy ** 2)


def main():
    a = load(sys.argv[1])
    print('共 %d 帧，%.1f 秒' % (len(a), a[-1, 0] - a[0, 0]))
    for n, (i, j) in enumerate(segments(a), 1):
        s = a[i:j + 1]
        dist = np.sum(np.hypot(np.diff(s[:, 1]), np.diff(s[:, 2])))
        dyaw = math.degrees(s[-1, 3] - s[0, 3])
        disp = np.hypot(s[-1, 1] - s[0, 1], s[-1, 2] - s[0, 2])
        print('\n段 %d：%.1f–%.1f s，路程 %.2f m，位移 %.2f m，转角 %+.1f°'
              % (n, s[0, 0] - a[0, 0], s[-1, 0] - a[0, 0], dist, disp, dyaw))
        if abs(dyaw) < 15 and disp > 0.5:
            # 直线：位移方向在起始底盘系中的角度
            dx, dy = s[-1, 1] - s[0, 1], s[-1, 2] - s[0, 2]
            ang = math.degrees(math.atan2(dy, dx) - s[0, 3])
            ang = (ang + 180) % 360 - 180
            print('  → 直线段：运动方向与底盘 x 轴夹角 %+.1f°（前进应接近 0°，后退接近 ±180°）' % ang)
        elif abs(dyaw) > 270 and disp < 0.5 and fit_circle(s[:, 1], s[:, 2])[2] < 0.5:
            # 轨迹圆半径 < 0.5 m 才算原地转；绕场地一圈回原点也会"转 360° 且位移小"
            cx, cy, r = fit_circle(s[:, 1], s[:, 2])
            # 把圆心（真实旋转中心）换算到每一帧的底盘系，取平均；纯原地转时这个向量应恒定
            c, sn = np.cos(s[:, 3]), np.sin(s[:, 3])
            ex, ey = cx - s[:, 1], cy - s[:, 2]
            vb = np.c_[c * ex + sn * ey, -sn * ex + c * ey]
            bx, by = vb.mean(axis=0)
            print('  → 原地转圈：轨迹圆半径 %.3f m；旋转中心在 base_footprint 的 (%+.3f, %+.3f) m，'
                  '逐帧标准差 (%.3f, %.3f) m（越小说明越接近纯原地转）'
                  % (r, bx, by, *vb.std(axis=0)))
            print('    建议：URDF 中 lidar_x、lidar_y 各减去 (%.3f, %.3f)，即雷达位于旋转中心 (%+.3f, %+.3f) m 处'
                  % (bx, by, -bx, -by))
    end, start = a[-1], a[0]
    ex, ey = end[1] - start[1], end[2] - start[2]
    eyaw = math.degrees((end[3] - start[3] + math.pi) % (2 * math.pi) - math.pi)
    print('\n[全程] 终点相对起点：位置误差 %.3f m（x %+.3f, y %+.3f），航向误差 %+.1f°'
          % (math.hypot(ex, ey), ex, ey, eyaw))
    print('[S3 验收] 回到起点标记时位置误差应 < 0.2 m（含摆放误差）')


if __name__ == '__main__':
    main()
