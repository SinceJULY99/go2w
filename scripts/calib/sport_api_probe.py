#!/usr/bin/env python3
"""Go2W sport 接口连通性测试：在域 0 发几次 StopMove（不会让机器人动），打印主控的回复。

python3 sport_api_probe.py [--api 1003] [--count 3]
只允许发不产生运动的接口（StopMove 1003、GetState 1034 等），Move 被拒绝。
"""
import argparse
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request, Response

SAFE_APIS = {1003, 1034}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--api', type=int, default=1003)
    ap.add_argument('--param', default='')
    ap.add_argument('--count', type=int, default=3)
    args = ap.parse_args()
    if args.api not in SAFE_APIS:
        raise SystemExit('只允许测试不产生运动的接口：%s' % sorted(SAFE_APIS))

    rclpy.init(domain_id=0)
    n = rclpy.create_node('sport_api_probe')
    got = []
    n.create_subscription(Response, '/api/sport/response', got.append,
                          QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    pub = n.create_publisher(Request, '/api/sport/request', 10)

    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    print('request 订阅者 %d 个，等待 %.1f s' % (pub.get_subscription_count(), time.time() - t0))
    for _ in range(10):                       # 给发现过程一点时间
        rclpy.spin_once(n, timeout_sec=0.1)

    ids = []
    for k in range(args.count):
        r = Request()
        r.header.identity.id = int(time.time() * 1000) * 10 + k
        r.header.identity.api_id = args.api
        r.header.policy.noreply = False
        r.parameter = args.param
        pub.publish(r)
        ids.append(r.header.identity.id)
        print('已发送 api_id=%d id=%d' % (args.api, r.header.identity.id))
        t1 = time.time()
        while time.time() - t1 < 0.5:
            rclpy.spin_once(n, timeout_sec=0.05)
    t1 = time.time()
    while time.time() - t1 < 2.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    mine = [m for m in got if m.header.identity.id in ids]
    print('共收到回复 %d 条，其中对应本次请求 %d 条' % (len(got), len(mine)))
    for m in mine:
        print('  id=%d api_id=%d code=%d data=%s' % (m.header.identity.id, m.header.identity.api_id,
                                                    m.header.status.code, m.data[:120]))
    others = sorted({m.header.identity.api_id for m in got if m.header.identity.id not in ids})
    print('其他程序的回复 api_id：%s' % others)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
