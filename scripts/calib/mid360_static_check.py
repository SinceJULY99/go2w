#!/usr/bin/env python3
"""MID360 静止标定检查：录几秒 IMU 和点云，比较重力方向与地面法向量。

在机器人静止、停在平地上时运行（Foxy/Humble 均可）：
    python3 mid360_static_check.py --seconds 5
只订阅话题，不发布任何东西。
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg


class Recorder(Node):
    def __init__(self, lidar_topic, imu_topic):
        super().__init__('mid360_static_check')
        self.acc, self.gyr, self.pts, self.frames = [], [], [], set()
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(CustomMsg, lidar_topic, self.on_lidar, qos_profile_sensor_data)
        self.n_scans = 0

    def on_imu(self, m):
        a, w = m.linear_acceleration, m.angular_velocity
        self.acc.append((a.x, a.y, a.z))
        self.gyr.append((w.x, w.y, w.z))
        self.frames.add('imu:' + m.header.frame_id)

    def on_lidar(self, m):
        self.n_scans += 1
        self.frames.add('lidar:' + m.header.frame_id)
        self.pts.extend((p.x, p.y, p.z) for p in m.points)


def deg(rad):
    return float(np.degrees(rad))


def angle(u, v):
    u, v = u / np.linalg.norm(u), v / np.linalg.norm(v)
    return deg(np.arccos(np.clip(abs(u @ v), -1.0, 1.0)))


def ransac_plane(P, iters=400, thresh=0.03):
    best_n, best_d, best_in = None, None, None
    rs = np.random.RandomState(0)
    for _ in range(iters):
        s = P[rs.choice(len(P), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        d = -n @ s[0]
        inl = np.abs(P @ n + d) < thresh
        if best_in is None or inl.sum() > best_in.sum():
            best_n, best_d, best_in = n, d, inl
    # 用内点做最小二乘精修
    Q = P[best_in]
    c = Q.mean(axis=0)
    n = np.linalg.svd(Q - c)[2][-1]
    d = -n @ c
    inl = np.abs(P @ n + d) < thresh
    if d < 0:          # 统一成"法向量指向传感器一侧"，d = 传感器到平面的距离
        n, d = -n, -d
    return n, d, inl


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF rpy 约定


def base_check(ext, acc, P):
    """S2 验收：数据变换到 base_link 后，地面应在 z≈0 且水平，IMU 重力应沿 z。"""
    x, y, z, r, p, yw = ext
    R, t = rpy_to_R(r, p, yw), np.array([x, y, z])
    up = R @ (acc.mean(axis=0) / np.linalg.norm(acc.mean(axis=0)))
    print('\n[base_link] IMU "上"方向 = (%+.3f, %+.3f, %+.3f)，与 z 轴夹角 %.2f°'
          % (*up, angle(up, np.array([0, 0, 1.0]))))
    rng = np.linalg.norm(P, axis=1)
    Q = (R @ P[(rng > 0.6) & (rng < 8.0)].T).T + t
    if len(Q) > 60000:
        Q = Q[np.random.RandomState(1).choice(len(Q), 60000, replace=False)]
    floor = Q[np.abs(Q[:, 2]) < 0.15]          # 只在 z≈0 附近找地面
    if len(floor) < 300:
        print('[base_link] z≈0 附近点太少（%d），外参的高度可能不对' % len(floor))
        return
    n, d, inl = ransac_plane(floor)
    if n[2] < 0:
        n, d = -n, -d
    zf = floor[inl][:, 2]
    print('[base_link] 地面：%d 个点，法向量与 z 轴夹角 %.2f°，z 均值 %+.3f m，标准差 %.3f m'
          % (inl.sum(), angle(n, np.array([0, 0, 1.0])), zf.mean(), zf.std()))
    above = Q[Q[:, 2] > 0.3]
    for i in range(3):
        if len(above) < 500:
            break
        wn, wd, winl = ransac_plane(above)
        a = angle(wn, np.array([0, 0, 1.0]))
        kind = '竖直面（墙），应接近 90°' if a > 45 else '水平面（桌面/天花板）'
        print('[base_link] 平面 %d：%d 个点，法向量与 z 轴夹角 %.1f°，%s' % (i + 1, winl.sum(), a, kind))
        above = above[~winl]
    tilt = angle(n, np.array([0, 0, 1.0]))
    ok = tilt < 1.0 and abs(zf.mean()) < 0.03
    print('\n[S2 验收] %s：地面水平误差 %.2f°（< 1°）、高度误差 %+.3f m（< 3 cm）'
          % ('通过' if ok else '未通过', tilt, zf.mean()))

    # 以地面为基准反推外参（保持 yaw 不变）：地面法向量在雷达系下为 up_l = (-sin p, sin r cos p, cos r cos p)
    up_l = R.T @ n
    p_new = np.arcsin(np.clip(-up_l[0], -1, 1))
    r_new = np.arctan2(up_l[1], up_l[2])
    h_new = n @ t + d          # 雷达原点到地面平面的距离
    print('[建议外参，以地面为基准] lidar_roll=%.5f (%.2f°)  lidar_pitch=%.5f (%.2f°)  lidar_z=%.3f'
          % (r_new, deg(r_new), p_new, deg(p_new), h_new))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--lidar', default='/livox/lidar')
    ap.add_argument('--imu', default='/livox/imu')
    ap.add_argument('--save', default='')
    ap.add_argument('--to-base', nargs=6, type=float, metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                    help='先按 URDF 中 base_link→livox_frame 的外参（米、弧度）把数据变换到 base_link，再做验收检查')
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.lidar, args.imu)
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    acc, gyr, P = np.array(node.acc), np.array(node.gyr), np.array(node.pts, dtype=float)
    print('frames:', sorted(node.frames))
    print('IMU msgs: %d (%.0f Hz)   lidar scans: %d (%.1f Hz)   points: %d'
          % (len(acc), len(acc) / args.seconds, node.n_scans, node.n_scans / args.seconds, len(P)))
    if len(acc) == 0 or len(P) == 0:
        print('没有收到数据，检查驱动和话题')
        return
    if args.save:
        np.savez_compressed(args.save, acc=acc, gyr=gyr, pts=P)
    if args.to_base:
        base_check(args.to_base, acc, P)
        return

    # ---- IMU：静止时加速度计读数指向"上"（与重力相反）
    a = acc.mean(axis=0)
    up_imu = a / np.linalg.norm(a)
    print('\n[IMU] 平均加速度 = (%.3f, %.3f, %.3f)，模长 %.3f（MID360 单位为 g 时应约 1.0）'
          % (a[0], a[1], a[2], np.linalg.norm(a)))
    print('[IMU] 陀螺均值 = (%.4f, %.4f, %.4f) rad/s，标准差 %.4f（静止时应接近 0）'
          % (*gyr.mean(axis=0), gyr.std(axis=0).max()))
    print('[IMU] "上"方向与 IMU z 轴夹角 = %.1f°' % angle(up_imu, np.array([0, 0, 1.0])))
    # 按 x 前 y 左 z 上的约定，给出俯仰和横滚（使 up 旋到 +z 所需的倾斜）
    pitch = deg(np.arctan2(-up_imu[0], up_imu[2]))
    roll = deg(np.arctan2(up_imu[1], up_imu[2]))
    print('[IMU] 若以 IMU 坐标系为准：pitch ≈ %.1f°（正值=前端朝下），roll ≈ %.1f°' % (pitch, roll))

    # ---- 点云：近距离点（保护架、线缆、机身）
    r = np.linalg.norm(P, axis=1)
    valid = r > 0.01
    P, r = P[valid], r[valid]
    print('\n[点云] 距离分布：<0.2m %d，0.2–0.5m %d，0.5–1m %d，>1m %d'
          % ((r < 0.2).sum(), ((r >= 0.2) & (r < 0.5)).sum(), ((r >= 0.5) & (r < 1)).sum(), (r >= 1).sum()))
    near = P[(r >= 0.05) & (r < 0.6)]
    if len(near):
        lo, hi = near.min(axis=0), near.max(axis=0)
        print('[点云] 0.05–0.6m 内的点包围盒：x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
              % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))

    # ---- 点云：依次找最大的几个平面
    Q = P[(r > 0.6) & (r < 8.0)]
    if len(Q) > 60000:
        Q = Q[np.random.RandomState(1).choice(len(Q), 60000, replace=False)]
    print('\n[平面] 在 0.6–8m 范围内用 RANSAC 依次提取最大平面（阈值 3cm）：')
    print('  #  内点占比  法向量(点云系)              距传感器   与IMU"上"夹角')
    planes = []
    rest = Q
    for i in range(4):
        if len(rest) < 500:
            break
        n, d, inl = ransac_plane(rest)
        ang = angle(n, up_imu)
        planes.append((n, d, inl.sum(), ang))
        print('  %d  %5.1f%%   (%+.3f, %+.3f, %+.3f)   %5.2f m    %5.1f°'
              % (i + 1, 100.0 * inl.sum() / len(Q), n[0], n[1], n[2], d, ang))
        rest = rest[~inl]

    # 水平面：法向量与 IMU "上" 接近。n 指向传感器一侧，n·up > 0 表示平面在传感器下方（地面），
    # n·up < 0 表示在上方（天花板）
    flat = [p for p in planes if p[3] < 30]
    floors = [p for p in flat if p[0] @ up_imu > 0]
    for n, d, cnt, ang in flat:
        if n @ up_imu < 0:
            print('[天花板] 在雷达上方 %.2f m，与 IMU 重力夹角 %.1f°' % (d, ang))
    print()
    if floors:
        n, d, cnt, ang = max(floors, key=lambda p: p[2])
        print('[结论] 地面：法向量与 IMU 重力夹角 %.1f°，雷达离地 %.2f m' % (ang, d))
        print('       点云系中地面法向量与 z 轴夹角 %.1f°' % angle(n, np.array([0, 0, 1.0])))
        print('       → 夹角接近 0° 说明点云和 IMU 在同一坐标系；接近 90° 说明点云被额外旋转过')
    else:
        print('[结论] 没有平面与 IMU 重力方向接近（<30°）。点云与 IMU 坐标系不一致，'
              '或地面被遮挡。请对照上表中各平面与重力的夹角。')


if __name__ == '__main__':
    main()
