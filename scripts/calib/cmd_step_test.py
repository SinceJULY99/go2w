#!/usr/bin/env python3
"""速度桥分步测试：发一小段 /cmd_vel，然后停止，用 /odom 核对实际运动和停车延迟。

python3 cmd_step_test.py --vx 0.1 --duration 1.0
python3 cmd_step_test.py --wz 0.26 --duration 1.0
流程：确认已解锁 → 以 20 Hz 发指令 duration 秒 → 停止发送 → 观察 3 s → 上锁 → 报告
脚本自带上限（比速度桥更严）：|vx|,|vy| ≤ 0.15 m/s，|wz| ≤ 20°/s，duration ≤ 2 s。
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

LIM_V, LIM_W, LIM_T = 0.15, math.radians(20), 2.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vx', type=float, default=0.0)
    ap.add_argument('--vy', type=float, default=0.0)
    ap.add_argument('--wz', type=float, default=0.0)
    ap.add_argument('--duration', type=float, default=1.0)
    a = ap.parse_args()
    if abs(a.vx) > LIM_V or abs(a.vy) > LIM_V or abs(a.wz) > LIM_W or not 0 < a.duration <= LIM_T:
        raise SystemExit('超出测试上限：|v| ≤ %.2f m/s，|wz| ≤ %.0f°/s，0 < duration ≤ %.0f s'
                         % (LIM_V, math.degrees(LIM_W), LIM_T))

    rclpy.init()
    n = rclpy.create_node('cmd_step_test')
    odom, armed = [], [None]
    n.create_subscription(Odometry, '/odom', lambda m: odom.append((time.time(), m)), 50)
    n.create_subscription(Bool, '/go2w_cmd_bridge/armed', lambda m: armed.__setitem__(0, m.data), 10)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    arm = n.create_client(SetBool, '/go2w_cmd_bridge/arm')

    def spin(sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(n, timeout_sec=0.02)

    def set_arm(v):
        if not arm.wait_for_service(timeout_sec=3.0):
            raise SystemExit('找不到 /go2w_cmd_bridge/arm，速度桥没运行？')
        f = arm.call_async(SetBool.Request(data=v))
        rclpy.spin_until_future_complete(n, f, timeout_sec=3.0)
        r = f.result()
        print('%s：%s' % ('解锁' if v else '上锁', r.message if r else '无响应'))
        return r is not None and r.success

    spin(1.0)
    if not odom:
        raise SystemExit('没有 /odom 数据')
    try:
        if not set_arm(True):
            raise SystemExit('解锁失败，测试取消')
        spin(0.3)
        p0 = odom[-1][1].pose.pose
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = a.vx, a.vy, a.wz
        print('发送 vx=%.2f vy=%.2f wz=%.0f°/s，持续 %.1f s' % (a.vx, a.vy, math.degrees(a.wz), a.duration))
        t_start = time.time()
        while time.time() - t_start < a.duration:
            pub.publish(cmd)
            spin(0.05)
        t_last = time.time()
        print('停止发送指令')
        spin(3.0)
    finally:
        set_arm(False)

    after = [(t, m) for t, m in odom if t >= t_last]
    t_stop = None
    for t, m in after:
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        if v < 0.02 and abs(m.twist.twist.angular.z) < math.radians(2):
            t_stop = t
            break
    p1 = odom[-1][1].pose.pose
    dx, dy = p1.position.x - p0.position.x, p1.position.y - p0.position.y
    y0 = yaw_of(p0.orientation)
    fwd = math.cos(y0) * dx + math.sin(y0) * dy
    lat = -math.sin(y0) * dx + math.cos(y0) * dy
    dyaw = math.degrees((yaw_of(p1.orientation) - y0 + math.pi) % (2 * math.pi) - math.pi)
    vmax = max((math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) for t, m in odom if t >= t_start), default=0)
    wmax = max((abs(m.twist.twist.angular.z) for t, m in odom if t >= t_start), default=0)
    print('\n结果：前进 %+.3f m，横移 %+.3f m，转角 %+.1f°；最大速度 %.2f m/s、%.0f°/s'
          % (fwd, lat, dyaw, vmax, math.degrees(wmax)))
    print('期望：前进 %+.3f m，横移 %+.3f m，转角 %+.1f°（不计加减速）'
          % (a.vx * a.duration, a.vy * a.duration, math.degrees(a.wz * a.duration)))
    print('停车延迟（停止发送 → odom 速度归零）：%s'
          % ('%.2f s' % (t_stop - t_last) if t_stop else '3 s 内未归零！'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
