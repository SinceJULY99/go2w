#!/usr/bin/env python3
"""导航平顺性分析：把"规划的路径本身"和"机器人跟踪路径的动作"分开评估。

python3 nav_smooth_check.py /go2w/data/bags/<包目录>

路径（/plan）：
  - 折角：相邻 0.2 m 线段的航向变化，超过 20° 记为一个折点（锯齿感）
  - 重规划跳变：新路径离上一条路径在机器人前方 1 m 内的最大横向偏差（路线突然变了）
跟踪（/cmd_vel_nav 与 /odom）：
  - 转向来回摆：运动中角速度符号翻转的频率（次/秒）
  - 速度抖动：前进速度相邻两帧差值的标准差
  - 中途停顿：导航进行中速度接近 0 的时间占比
  - 横向偏差：机器人位置到当时路径的距离
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage

TYPES = {'/plan': Path, '/cmd_vel_nav': Twist, '/odom': Odometry, '/goal_pose': PoseStamped, '/tf': TFMessage}


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    out = {k: [] for k in TYPES}
    while r.has_next():
        name, data, t = r.read_next()
        if name in TYPES:
            out[name].append((t * 1e-9, deserialize_message(data, TYPES[name])))
    return out


def path_xy(p):
    return np.array([[q.pose.position.x, q.pose.position.y] for q in p.poses])


def resample(xy, step=0.2):
    if len(xy) < 2:
        return xy
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    if d[-1] < step:
        return xy
    s = np.arange(0, d[-1], step)
    return np.c_[np.interp(s, d, xy[:, 0]), np.interp(s, d, xy[:, 1])]


def kinks(xy, thr=20):
    r = resample(xy)
    if len(r) < 3:
        return 0, 0.0
    h = np.arctan2(*np.diff(r, axis=0)[:, ::-1].T)
    dh = np.degrees(np.abs((np.diff(h) + np.pi) % (2 * np.pi) - np.pi))
    return int((dh > thr).sum()), float(dh.max())


def map_to_odom_series(tf_msgs):
    """map → odom 随时间的变换（AMCL 发布），用来把 /odom 位置换到 map 系"""
    s = []
    for t, m in tf_msgs:
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                q = tr.transform.rotation
                s.append((t, tr.transform.translation.x, tr.transform.translation.y,
                          2 * math.atan2(q.z, q.w)))
    return np.array(s)


def main():
    d = read(sys.argv[1])
    plans, cmds, odom = d['/plan'], d['/cmd_vel_nav'], d['/odom']
    print('录包：/plan %d 条，/cmd_vel_nav %d 条，/odom %d 条，/goal_pose %d 条'
          % (len(plans), len(cmds), len(odom), len(d['/goal_pose'])))

    # ---- 路径本身
    ks, kmax, lens = [], [], []
    for _, p in plans:
        xy = path_xy(p)
        if len(xy) < 3:
            continue
        k, m = kinks(xy)
        ks.append(k)
        kmax.append(m)
        lens.append(np.hypot(*np.diff(xy, axis=0).T).sum())
    if ks:
        print('\n[路径] %d 条有效路径，平均长度 %.2f m' % (len(ks), np.mean(lens)))
        print('  折点（0.2 m 内转向 > 20°）：平均每条 %.1f 个，最多 %d 个；最大单处折角 中位数 %.0f°，最大 %.0f°'
              % (np.mean(ks), max(ks), np.median(kmax), max(kmax)))

    # 重规划跳变：相邻两条路径，在新路径前 1 m 内，到旧路径的最大距离
    jumps = []
    for (t0, a), (t1, b) in zip(plans, plans[1:]):
        A, B = resample(path_xy(a), 0.05), resample(path_xy(b), 0.05)
        if len(A) < 2 or len(B) < 2 or t1 - t0 > 3:
            continue
        Bn = B[:21]
        dist = np.min(np.hypot(Bn[:, None, 0] - A[None, :, 0], Bn[:, None, 1] - A[None, :, 1]), axis=1)
        jumps.append(dist.max())
    if jumps:
        j = np.array(jumps)
        print('  重规划（约 1 Hz）前方 1 m 内的路线变化：中位数 %.2f m，90%% 分位 %.2f m，> 0.2 m 的有 %d 次 / %d'
              % (np.median(j), np.percentile(j, 90), (j > 0.2).sum(), len(j)))

    # ---- 跟踪动作
    if cmds:
        t = np.array([c[0] for c in cmds])
        v = np.array([[c[1].linear.x, c[1].linear.y, c[1].angular.z] for c in cmds])
        moving = np.abs(v[:, 0]) + np.abs(v[:, 1]) > 0.02
        vm = v[moving]
        dur = moving.sum() / 20.0
        wz = vm[:, 2]
        flips = np.sum((np.sign(wz[1:]) != np.sign(wz[:-1])) & (np.abs(wz[1:]) > 0.05) & (np.abs(wz[:-1]) > 0.05))
        print('\n[跟踪] 运动指令约 %.0f s' % dur)
        print('  转向来回摆：角速度符号翻转 %.2f 次/秒（< 0.3 比较平稳）' % (flips / max(dur, 1e-3)))
        print('  前进速度抖动：相邻帧差值标准差 %.3f m/s；角速度抖动 %.3f rad/s'
              % (np.std(np.diff(vm[:, 0])), np.std(np.diff(vm[:, 2]))))
        print('  速度：|vx| 中位数 %.2f，90%% 分位 %.2f m/s；|wz| 中位数 %.2f rad/s'
              % (np.median(np.abs(vm[:, 0])), np.percentile(np.abs(vm[:, 0]), 90), np.median(np.abs(vm[:, 2]))))
        # 导航进行中（有指令流）但几乎没速度
        active = np.r_[False, np.diff(t) < 0.2]
        stalls = active & ~moving
        print('  导航进行中速度接近 0 的时间占比：%.0f%%' % (100 * stalls.sum() / max(active.sum(), 1)))

    # ---- 横向偏差：机器人（map 系）到当时路径的距离
    m2o = map_to_odom_series(d['/tf'])
    if len(m2o) and odom and plans:
        errs = []
        pi = 0
        for to, o in odom[::5]:
            while pi + 1 < len(plans) and plans[pi + 1][0] <= to:
                pi += 1
            if plans[pi][0] > to or to - plans[pi][0] > 2:
                continue
            k = np.searchsorted(m2o[:, 0], to) - 1
            if k < 0:
                continue
            _, mx, my, myaw = m2o[k]
            ox, oy = o.pose.pose.position.x, o.pose.pose.position.y
            x = mx + math.cos(myaw) * ox - math.sin(myaw) * oy
            y = my + math.sin(myaw) * ox + math.cos(myaw) * oy
            P = resample(path_xy(plans[pi][1]), 0.05)
            if len(P):
                errs.append(np.min(np.hypot(P[:, 0] - x, P[:, 1] - y)))
        if errs:
            e = np.array(errs)
            print('\n[路径跟踪] 机器人到路径的横向偏差：中位数 %.2f m，90%% 分位 %.2f m，最大 %.2f m'
                  % (np.median(e), np.percentile(e, 90), e.max()))


if __name__ == '__main__':
    main()
