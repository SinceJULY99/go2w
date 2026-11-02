#!/usr/bin/env python3
"""A↔B 往返巡逻：在 RViz 里设两个点，机器人在两点之间来回导航。

前提：导航已启动、初始位姿已设好、速度桥已解锁（nav.sh arm）
  ros2 run go2w_bringup patrol.py [--ros-args -p loops:=3 -p pause:=3.0]
  1. RViz 工具栏选 Patrol Pose（发 /patrol_pose），像 2D Goal Pose 一样按下定位置、拖动定朝向，依次设 A、B
     也可以用 Publish Point（/clicked_point）只点位置，这时到点朝向 = 本段行进方向
  2. 机器人先去 A，再去 B，再回 A……
停止：Ctrl+C（先取消当前导航目标，机器人停下）
速度桥被上锁（手动 disarm 或里程计不健康）→ 取消目标并结束巡逻
某段导航失败会重试 retries 次，仍失败就结束巡逻
巡逻结束时清除 RViz 上的 A、B 标记

参数：loops 往返次数（A→B→A 算一次，0 = 一直跑）；pause 每到一个点停几秒；retries 失败重试次数
"""
import math
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

NAMES = 'AB'


class Stop(Exception):
    """巡逻需要结束（原因在 args[0]）。"""


class Patrol(Node):

    def __init__(self):
        super().__init__('go2w_patrol')
        p = self.declare_parameter
        self.loops = p('loops', 0).value
        self.pause = p('pause', 3.0).value
        self.retries = p('retries', 2).value

        self.points = []      # [(x, y, yaw)]，map 坐标系；yaw 为 None = 朝行进方向
        self.armed = None     # None = 还没收到速度桥状态
        self.goal_handle = None    # 当前导航目标

        self.create_subscription(PoseStamped, '/patrol_pose', self.on_pose, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        self.create_subscription(Bool, '/go2w_cmd_bridge/armed', self.on_armed, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.add_point(m.header.frame_id, m.pose.position.x, m.pose.position.y, yaw)

    def on_click(self, m):
        self.add_point(m.header.frame_id, m.point.x, m.point.y, None)

    def add_point(self, frame, x, y, yaw):
        if frame != 'map':
            self.get_logger().error('点的坐标系是 %s，不是 map，忽略（RViz Fixed Frame 要设成 map）' % frame)
            return
        if len(self.points) >= 2:
            self.get_logger().warn('A、B 已经设好，忽略新点；要换点请 Ctrl+C 后重新运行')
            return
        self.points.append((x, y, yaw))
        self.get_logger().info('%s 点：(%.2f, %.2f)，朝向 %s' % (
            NAMES[len(self.points) - 1], x, y, '行进方向' if yaw is None else '%.0f°' % math.degrees(yaw)))
        self.publish_markers()

    def on_armed(self, m):
        self.armed = m.data

    def publish_markers(self):
        arr = MarkerArray()
        for i, (x, y, yaw) in enumerate(self.points):
            kinds = (Marker.CYLINDER, Marker.TEXT_VIEW_FACING) + (() if yaw is None else (Marker.ARROW,))
            for k, kind in enumerate(kinds):
                mk = Marker()
                mk.header.frame_id = 'map'
                mk.ns, mk.id, mk.type = 'patrol', 3 * i + k, kind
                mk.pose.position.x, mk.pose.position.y = x, y
                mk.pose.orientation.w = 1.0
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = (1.0, 0.6, 0.0, 0.8) if i == 0 else (0.2, 0.6, 1.0, 0.8)
                if kind == Marker.CYLINDER:
                    mk.scale.x, mk.scale.y, mk.scale.z = 0.3, 0.3, 0.05
                    mk.pose.position.z = 0.025
                elif kind == Marker.ARROW:
                    mk.scale.x, mk.scale.y, mk.scale.z = 0.5, 0.08, 0.08
                    mk.pose.position.z = 0.1
                    mk.pose.orientation.z, mk.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                else:
                    mk.text, mk.scale.z, mk.pose.position.z = NAMES[i], 0.4, 0.5
                    mk.color.r = mk.color.g = mk.color.b = 1.0
                arr.markers.append(mk)
        self.marker_pub.publish(arr)

    def clear_markers(self):
        mk = Marker(action=Marker.DELETEALL)
        mk.header.frame_id = 'map'
        self.marker_pub.publish(MarkerArray(markers=[mk]))
        self.spin_for(0.5, check=False)   # 给桥一点时间转发出去

    def goal(self, i):
        """第 i 个点的目标位姿；没设朝向时朝向 = 从另一个点过来的方向。"""
        (x, y, yaw), (ox, oy, _) = self.points[i], self.points[1 - i]
        if yaw is None:
            yaw = math.atan2(y - oy, x - ox)
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return g

    def spin_for(self, seconds, check=True):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if check:
                self.check_armed()

    def check_armed(self):
        if self.armed is False:
            raise Stop('速度桥已上锁')

    def go(self, i):
        """导航到第 i 个点，成功返回 True。"""
        goal = NavigateToPose.Goal(pose=self.goal(i))
        future = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 拒绝了目标')
            return False
        self.goal_handle = handle
        result = handle.get_result_async()
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.check_armed()
        self.goal_handle = None
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('去 %s 点失败（状态 %d）' % (NAMES[i], status))
        return status == GoalStatus.STATUS_SUCCEEDED

    def cancel(self):
        if self.goal_handle is None:
            return
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.goal_handle = None
        self.get_logger().warn('已取消当前导航目标' if future.done() else
                               '取消导航目标没有回应！请执行 nav.sh disarm')

    def run(self):
        log = self.get_logger()
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise Stop('找不到 navigate_to_pose，导航没有启动？')
        self.spin_for(1.0, check=False)
        if self.armed is not True:
            log.warn('速度桥还没解锁，等待解锁（另开终端执行 nav.sh arm）……')
            while self.armed is not True:
                rclpy.spin_once(self, timeout_sec=0.1)
        log.info('在 RViz 里用 Patrol Pose 依次设 A、B（按下定位置、拖动定朝向）')
        while len(self.points) < 2:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.check_armed()
        log.info('开始巡逻：%s，每个点停 %.1f s' % ('一直往返' if self.loops == 0 else '往返 %d 次' % self.loops, self.pause))

        i, legs = 0, 0       # 第一段先去 A
        while self.loops == 0 or legs < 2 * self.loops + 1:
            t0 = time.time()
            log.info('→ 去 %s 点' % NAMES[i])
            for attempt in range(self.retries + 1):
                if self.go(i):
                    break
                if attempt < self.retries:
                    log.warn('%.0f s 后重试（%d/%d）' % (self.pause, attempt + 1, self.retries))
                    self.spin_for(self.pause)
            else:
                raise Stop('去 %s 点连续失败 %d 次' % (NAMES[i], self.retries + 1))
            legs += 1
            log.info('到达 %s 点，用时 %.1f s（第 %d 段）' % (NAMES[i], time.time() - t0, legs))
            i = 1 - i
            self.spin_for(self.pause)
        log.info('巡逻完成')


def main():
    # 不用 rclpy 的信号处理，否则 Ctrl+C 时上下文先被关掉，没法取消导航目标
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.default_int_handler)
    node = Patrol()
    try:
        node.run()
    except Stop as e:
        node.get_logger().error('结束巡逻：%s' % e.args[0])
    except KeyboardInterrupt:
        node.get_logger().warn('收到停止信号')
    finally:
        node.cancel()
        node.clear_markers()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
