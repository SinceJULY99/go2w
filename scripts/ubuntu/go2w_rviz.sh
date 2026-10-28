#!/usr/bin/env bash
# 在外部 Ubuntu 上运行：bash ~/go2w_viz/go2w_rviz.sh
# 连接 Orin 上的可视化桥（域 44），用项目的 RViz 配置打开 RViz
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=44
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$here/cyclonedds_viz.xml"

ros2 daemon stop >/dev/null 2>&1 || true
exec rviz2 -d "$here/go2w.rviz"
