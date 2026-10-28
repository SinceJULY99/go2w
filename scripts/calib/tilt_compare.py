#!/usr/bin/env python3
"""对比三种机身倾斜（俯仰/横滚）的来源，判断点云校平该用哪一个。

在容器里运行（需要导航或 odometry.launch.py 正在跑）：
  python3 /go2w/scripts/calib/tilt_compare.py [秒数=60] [输出 csv]

  robot   Go2W 主控 IMU（域 0 /lowstate 的 imu_state.rpy，best-effort；/sportmodestate 实测没有发布者），以启动时为 0
  fastlio planar_odom 发布的 base_footprint → base_link，即点云校平实际用的倾斜
          （planar_odom 的 tilt_source=fastlio 时是 FAST-LIO 的估计，=robot 时是主控 IMU）
  ground  只用静态外参把点云转到 base_link，拟合 0.8–3 m 内的地面平面：机身相对地面的真实倾斜
          （假设地面是平的；以启动时为 0，消掉外参本身的残差）
俯仰正值 = 低头（绕 y 轴右手），横滚正值 = 右侧（-y）低（绕 x 轴右手）；主控 rpy 按同样的约定（未经实测确认）。机器人站在平地上启动，测量期间可以用遥控器原地转圈、前后走。
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowState


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_of_up(up):
    """base 系下的"上"方向 → (俯仰, 横滚)，单位度；与绕 y / x 轴的右手转角同号。"""
    return math.degrees(math.atan2(-up[0], up[2])), math.degrees(math.atan2(up[1], up[2]))


class RobotSide(Node):
    def __init__(self, context):
        super().__init__('tilt_compare_robot', context=context)
        self.rpy = None
        self.count = 0
        self.create_subscription(LowState, 'lowstate', self.on_state, qos_profile_sensor_data)

    def on_state(self, m):
        self.rpy = tuple(m.imu_state.rpy)
        self.count += 1


class CoreSide(Node):
    def __init__(self, context):
        super().__init__('tilt_compare', context=context)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.cloud = None
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, 2)

    def on_cloud(self, m):
        self.cloud = m

    def fastlio_tilt(self):
        t = self.tf.lookup_transform('base_footprint', 'base_link', Time())
        q = t.transform.rotation
        return tilt_of_up(quat_to_R(q.x, q.y, q.z, q.w).T @ np.array([0, 0, 1.0]))

    def ground_tilt(self, rng):
        m = self.cloud
        t = self.tf.lookup_transform('base_link', m.header.frame_id, Time())
        q, p = t.transform.rotation, t.transform.translation
        pts = point_cloud2.read_points_numpy(m, ('x', 'y', 'z'), skip_nans=True)
        P = pts @ quat_to_R(q.x, q.y, q.z, q.w).T + [p.x, p.y, p.z]
        r = np.hypot(P[:, 0], P[:, 1])
        g = P[(r > 0.8) & (r < 3.0) & (np.abs(P[:, 2]) < 0.5)]
        if len(g) < 100:
            return None
        best = None
        for _ in range(150):
            s = g[rng.choice(len(g), 3, replace=False)]
            n = np.cross(s[1] - s[0], s[2] - s[0])
            if np.linalg.norm(n) < 1e-6:
                continue
            n /= np.linalg.norm(n)
            if abs(n[2]) < 0.8:       # 只要接近水平的面（地面），排除墙
                continue
            k = int((np.abs((g - s[0]) @ n) < 0.02).sum())
            if best is None or k > best[1]:
                best = (n if n[2] > 0 else -n, k)
        if best is None or best[1] < 50:
            return None
        # 地面法向就是 base 系下的"上"
        return tilt_of_up(best[0])


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    out = sys.argv[2] if len(sys.argv) > 2 else '/go2w/data/logs/tilt_compare.csv'
    ctx_robot, ctx_core = Context(), Context()
    rclpy.init(context=ctx_robot, domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
    rclpy.init(context=ctx_core, domain_id=43, signal_handler_options=SignalHandlerOptions.NO)
    robot, core = RobotSide(ctx_robot), CoreSide(ctx_core)
    ex_r, ex_c = SingleThreadedExecutor(context=ctx_robot), SingleThreadedExecutor(context=ctx_core)
    ex_r.add_node(robot)
    ex_c.add_node(core)
    stop = threading.Event()

    def spin_robot():
        while not stop.is_set():
            ex_r.spin_once(timeout_sec=0.05)

    threading.Thread(target=spin_robot, daemon=True).start()

    rng = np.random.default_rng(0)
    t0 = time.time()
    while (robot.rpy is None or core.cloud is None) and time.time() - t0 < 10:
        ex_c.spin_once(timeout_sec=0.1)
    if robot.rpy is None:
        print('收不到 /lowstate（域 0）')
        return
    if core.cloud is None:
        print('收不到 /cloud_registered_body（域 43），导航或里程计没在跑？')
        return

    rows, ref = [], None
    c0 = robot.count
    t0 = nxt = time.time()
    while time.time() - t0 < dur:
        ex_c.spin_once(timeout_sec=0.02)
        if time.time() < nxt:
            continue
        nxt += 0.5
        try:
            fl = core.fastlio_tilt()
            gr = core.ground_tilt(rng)
        except Exception as e:  # noqa: BLE001  TF 暂时不可用就跳过这一拍
            print('跳过：%s' % e)
            continue
        rp = (math.degrees(robot.rpy[1]), math.degrees(robot.rpy[0]))   # (俯仰, 横滚)
        if ref is None:
            if gr is None:
                continue
            ref = (rp, gr)
        rp = (rp[0] - ref[0][0], rp[1] - ref[0][1])
        gr = None if gr is None else (gr[0] - ref[1][0], gr[1] - ref[1][1])
        rows.append((time.time() - t0, math.degrees(robot.rpy[2]), *rp, *fl, *(gr or (float('nan'),) * 2)))

    stop.set()
    print('主控 IMU 频率 %.0f Hz' % ((robot.count - c0) / dur))
    with open(out, 'w') as f:
        f.write('t,yaw_deg,robot_pitch,robot_roll,fastlio_pitch,fastlio_roll,ground_pitch,ground_roll\n')
        for r in rows:
            f.write(','.join('%.2f' % v for v in r) + '\n')
    a = np.array(rows)
    print('样本 %d（%.0f s），已保存 %s' % (len(a), dur, out))
    print('              俯仰范围(°)        横滚范围(°)')
    for name, i in (('主控 IMU', 2), ('FAST-LIO', 4), ('地面拟合', 6)):
        v = a[:, i:i + 2]
        v = v[~np.isnan(v).any(axis=1)]
        print('%-10s  %6.1f ~ %5.1f      %6.1f ~ %5.1f' % (name, v[:, 0].min(), v[:, 0].max(), v[:, 1].min(), v[:, 1].max()))
    ok = ~np.isnan(a[:, 6])
    for name, i in (('主控 IMU', 2), ('FAST-LIO', 4)):
        e = a[ok, i:i + 2] - a[ok, 6:8]
        print('%-10s 与地面拟合的偏差：俯仰 平均 %.2f° 最大 %.2f°，横滚 平均 %.2f° 最大 %.2f°' % (
            name, np.abs(e[:, 0]).mean(), np.abs(e[:, 0]).max(), np.abs(e[:, 1]).mean(), np.abs(e[:, 1]).max()))


if __name__ == '__main__':
    main()
