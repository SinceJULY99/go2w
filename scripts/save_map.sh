#!/usr/bin/env bash
# 在容器内保存当前 slam_toolbox 地图：bash /go2w/scripts/save_map.sh <地图名>
# 输出 /go2w/data/maps/<地图名>.{pgm,yaml}（给 map_server/AMCL）和 .posegraph/.data（slam_toolbox 续建用）
set -eo pipefail
name=${1:?用法：save_map.sh <地图名>}
dir=/go2w/data/maps
mkdir -p "$dir"
source /go2w/docker/ros_env.sh

ros2 run nav2_map_server map_saver_cli -f "$dir/$name" --ros-args -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$dir/$name'}"
chown -R 1000:1000 "$dir"
ls -l "$dir/$name".*
