#!/usr/bin/env python3
"""/cmd_vel → Go2W 高层运动接口（sport Move / StopMove），带多层安全保护。

两个 DDS 域在同一进程里：
  域 43（导航核心，本机回环）：订阅 /cmd_vel、/odom_healthy，提供 ~/arm 服务
  域 0 （Go2W 内网，有线口）： 发布 /api/sport/request，订阅 /api/sport/response

安全设计（由内到外）：
  1. 默认上锁：启动后不发任何运动指令，必须调用 ~/arm(true) 解锁
  2. 限速：vx、vy、wz 分别限幅
  3. 指令超时：cmd_timeout 内没有新的 /cmd_vel → 发 StopMove
  4. 里程计不健康（/odom_healthy 为 false 或超过 0.5 s 没收到）→ StopMove 并上锁
  5. 节点退出（Ctrl+C、SIGTERM）→ 先发 StopMove 再退出
  6. Go2W 自身：1 s 内收不到 Move 会自动停下
"""
import json
import math
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from unitree_api.msg import Request, Response

API_STOPMOVE = 1003
API_MOVE = 1008


class RobotSide(Node):
    """域 0：与 Go2W 主控通信。"""

    def __init__(self, context):
        # 不接受 launch 传入的 __node 重命名，否则会和域 43 的节点同名
        super().__init__('go2w_cmd_bridge_robot', context=context, use_global_arguments=False)
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Response, '/api/sport/response', self.on_response, 10)
        self.seq = 0
        self.sent = set()     # 本节点发出的、需要回复的请求 id（主控也会回复其他程序的请求）

    def send(self, api_id, parameter='', noreply=False):
        self.seq += 1
        r = Request()
        r.header.identity.id = int(time.time() * 1e6) * 100 + self.seq % 100
        if not noreply:
            self.sent.add(r.header.identity.id)
        r.header.identity.api_id = api_id
        r.header.policy.noreply = noreply
        r.parameter = parameter
        self.pub.publish(r)

    def on_response(self, m):
        if m.header.identity.id not in self.sent:
            return
        self.sent.discard(m.header.identity.id)
        api, code = m.header.identity.api_id, m.header.status.code
        log = self.get_logger().info if code == 0 else self.get_logger().error
        log('主控回复 api_id=%d code=%d' % (api, code))


class CoreSide(Node):
    """域 43：接收导航指令，做安全检查，决定发什么。"""

    def __init__(self, context, robot):
        super().__init__('go2w_cmd_bridge', context=context)
        self.robot = robot
        p = self.declare_parameter
        self.max_vx = p('max_vx', 0.3).value
        self.max_vy = p('max_vy', 0.2).value
        self.max_wz = p('max_wz', math.radians(30)).value
        self.timeout = p('cmd_timeout', 0.3).value
        self.require_odom = p('require_odom_healthy', True).value
        self.deadband = p('deadband', 0.01).value
        rate = p('rate', 20.0).value

        self.armed = False
        self.moving = False
        self.cmd = None
        self.cmd_t = 0.0
        self.healthy = False
        self.healthy_t = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, '/odom_healthy', self.on_health, 10)
        self.create_service(SetBool, '~/arm', self.on_arm)
        self.armed_pub = self.create_publisher(Bool, '~/armed', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.add_on_set_parameters_callback(self.on_params)
        self.get_logger().info('已启动（上锁状态）。限速 vx %.2f m/s、vy %.2f m/s、wz %.0f°/s，指令超时 %.2f s'
                               % (self.max_vx, self.max_vy, math.degrees(self.max_wz), self.timeout))

    def on_params(self, params):
        """运行中调整限速和超时：ros2 param set /go2w_cmd_bridge max_vx 0.45"""
        from rcl_interfaces.msg import SetParametersResult
        names = {'max_vx': 'max_vx', 'max_vy': 'max_vy', 'max_wz': 'max_wz', 'cmd_timeout': 'timeout'}
        for p in params:
            if p.name in names:
                if not isinstance(p.value, (int, float)) or p.value < 0:
                    return SetParametersResult(successful=False, reason='%s 必须是非负数' % p.name)
        for p in params:
            if p.name in names:
                setattr(self, names[p.name], float(p.value))
        self.get_logger().warn('限速已更新：vx %.2f m/s、vy %.2f m/s、wz %.0f°/s，指令超时 %.2f s'
                               % (self.max_vx, self.max_vy, math.degrees(self.max_wz), self.timeout))
        return SetParametersResult(successful=True)

    def on_cmd(self, m):
        self.cmd, self.cmd_t = m, time.time()

    def on_health(self, m):
        self.healthy, self.healthy_t = m.data, time.time()

    def odom_ok(self):
        return (not self.require_odom) or (self.healthy and time.time() - self.healthy_t < 0.5)

    def on_arm(self, req, res):
        if req.data:
            if not self.odom_ok():
                res.success, res.message = False, '拒绝解锁：里程计不健康或没有数据'
            else:
                self.armed, self.cmd = True, None   # 解锁前收到的旧指令作废
                res.success, res.message = True, '已解锁'
        else:
            self.disarm('手动上锁')
            res.success, res.message = True, '已上锁'
        self.get_logger().warn(res.message)
        return res

    def stop(self, reason):
        self.robot.send(API_STOPMOVE)
        if self.moving:
            self.get_logger().warn('停车：%s' % reason)
        self.moving = False

    def disarm(self, reason):
        self.stop(reason)
        self.armed = False

    def tick(self):
        self.armed_pub.publish(Bool(data=self.armed))
        if not self.armed:
            return
        if not self.odom_ok():
            self.disarm('里程计不健康')
            self.get_logger().error('里程计不健康，已停车并上锁；恢复后需重新解锁')
            return
        if self.cmd is None or time.time() - self.cmd_t > self.timeout:
            if self.moving:
                self.stop('%.2f s 内没有新的 /cmd_vel' % self.timeout)
            return
        c = self.cmd
        vx = max(-self.max_vx, min(self.max_vx, c.linear.x))
        vy = max(-self.max_vy, min(self.max_vy, c.linear.y))
        wz = max(-self.max_wz, min(self.max_wz, c.angular.z))
        if abs(vx) < self.deadband and abs(vy) < self.deadband and abs(wz) < self.deadband:
            if self.moving:
                self.stop('速度指令为零')
            return
        self.robot.send(API_MOVE, json.dumps({'x': vx, 'y': vy, 'z': wz}), noreply=True)
        self.moving = True


def main():
    stop_event = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_event.set())

    ctx_robot, ctx_core = Context(), Context()
    rclpy.init(context=ctx_robot, domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
    rclpy.init(context=ctx_core, domain_id=43, signal_handler_options=SignalHandlerOptions.NO)
    robot = RobotSide(ctx_robot)
    core = CoreSide(ctx_core, robot)
    ex_robot = SingleThreadedExecutor(context=ctx_robot)
    ex_core = SingleThreadedExecutor(context=ctx_core)
    ex_robot.add_node(robot)
    ex_core.add_node(core)

    def spin_robot():
        while not stop_event.is_set():
            ex_robot.spin_once(timeout_sec=0.05)

    t = threading.Thread(target=spin_robot, daemon=True)
    t.start()
    try:
        while not stop_event.is_set():
            ex_core.spin_once(timeout_sec=0.05)
    finally:
        # 退出前确保停车：连发几次 StopMove
        for _ in range(3):
            robot.send(API_STOPMOVE)
            time.sleep(0.02)
        core.get_logger().warn('退出：已发送 StopMove')
        stop_event.set()
        t.join(timeout=1.0)
        core.destroy_node()
        robot.destroy_node()
        rclpy.shutdown(context=ctx_core)
        rclpy.shutdown(context=ctx_robot)


if __name__ == '__main__':
    main()
