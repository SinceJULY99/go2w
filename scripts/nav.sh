#!/usr/bin/env bash
# 导航一键操作（在 Orin 宿主机上运行，不需要进容器，也不需要 tmux）
#
#   ~/go2w/scripts/nav.sh start [地图.yaml]   启动导航（后台运行，SSH 断开也不影响）
#   ~/go2w/scripts/nav.sh status             查看状态
#   ~/go2w/scripts/nav.sh arm                解锁（机器人可以动）
#   ~/go2w/scripts/nav.sh disarm             上锁（机器人不会动）
#   ~/go2w/scripts/nav.sh stop               上锁并停止导航
#   ~/go2w/scripts/nav.sh log                看最近的日志
set -uo pipefail

C=go2w_humble
LOG=/go2w/data/logs/navigation.log
DEFAULT_MAP=/go2w/data/maps/lab_20261021.yaml

ros() {  # 在容器里执行一条 ROS 命令
  docker exec "$C" bash -c "source /go2w/docker/ros_env.sh >/dev/null 2>&1; $1"
}

running() {
  docker exec "$C" pgrep -f "[r]os2 launch go2w_bringup" >/dev/null 2>&1
}

healthy() {
  ros "timeout 5 ros2 topic echo --once /odom_healthy 2>/dev/null | grep -q 'data: true'"
}

armed() {
  ros "timeout 5 ros2 topic echo --once /go2w_cmd_bridge/armed 2>/dev/null | grep -q 'data: true'"
}

case "${1:-}" in
  start)
    map=${2:-$DEFAULT_MAP}
    if running; then
      echo "导航已经在运行了。要重启请先执行：$0 stop"
      exit 1
    fi
    echo "启动导航，地图：$map"
    echo "（机器人要站好、停在平地上不要动，FAST-LIO 正在初始化）"
    docker exec "$C" bash -c "mkdir -p /go2w/data/logs; [ -f $LOG ] && mv $LOG /go2w/data/logs/navigation_\$(date +%Y%m%d_%H%M%S).log; chown -R 1000:1000 /go2w/data"
    docker exec -d "$C" bash -c "source /go2w/docker/ros_env.sh && exec ros2 launch go2w_bringup navigation.launch.py map:=$map > $LOG 2>&1"
    for i in $(seq 1 30); do
      sleep 2
      if healthy; then
        echo "✅ 导航已启动，里程计正常。"
        echo "下一步：在 RViz 里用 2D Pose Estimate 设置初始位姿，然后执行：$0 arm"
        exit 0
      fi
    done
    echo "❌ 60 秒内里程计没有正常输出。看日志：$0 log"
    exit 1
    ;;

  status)
    if ! running; then echo "导航：没有运行"; exit 0; fi
    echo "导航：运行中"
    healthy && echo "里程计：正常" || echo "里程计：❌ 不健康（多半是网线断过，需要 $0 stop 后重新 start）"
    armed && echo "速度桥：🔓 已解锁（机器人可以动）" || echo "速度桥：🔒 已上锁"
    docker exec "$C" grep -q initialPoseReceived "$LOG" 2>/dev/null && echo "初始位姿：已设置" || echo "初始位姿：还没设置（在 RViz 里用 2D Pose Estimate）"
    ;;

  arm)
    if ! running; then echo "导航没有运行，先执行：$0 start"; exit 1; fi
    if ! healthy; then echo "❌ 里程计不健康，不能解锁。执行 $0 stop 后重新 start"; exit 1; fi
    docker exec "$C" grep -q initialPoseReceived "$LOG" 2>/dev/null || echo "⚠️  还没设置初始位姿，导航目标会规划失败"
    ros "ros2 service call /go2w_cmd_bridge/arm std_srvs/srv/SetBool '{data: true}'" | grep -q "success=True" \
      && echo "🔓 已解锁。现在可以在 RViz 里用 2D Goal Pose 发目标（手里拿着遥控器）" \
      || echo "❌ 解锁失败，执行 $0 log 查看原因"
    ;;

  disarm)
    ros "ros2 service call /go2w_cmd_bridge/arm std_srvs/srv/SetBool '{data: false}'" | grep -q "success=True" \
      && echo "🔒 已上锁，机器人不会动" || echo "上锁失败（速度桥可能没在运行）"
    ;;

  stop)
    running || { echo "导航没有运行"; exit 0; }
    ros "ros2 service call /go2w_cmd_bridge/arm std_srvs/srv/SetBool '{data: false}'" >/dev/null 2>&1 && echo "🔒 已上锁"
    docker exec "$C" pkill -INT -f "[r]os2 launch go2w_bringup"
    docker exec "$C" pkill -INT -f "[r]os2 bag record" 2>/dev/null
    echo "正在停止……"
    sleep 10
    docker exec "$C" pkill -9 -f "[f]astlio_mapping|[l]ivox_ros_driver2_node|[g]o2w_cmd_bridge.py" 2>/dev/null
    running && echo "⚠️  还有进程没退出，再执行一次 $0 stop" || echo "✅ 导航已停止"
    ;;

  log)
    docker exec "$C" tail -n 40 "$LOG"
    ;;

  *)
    sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
