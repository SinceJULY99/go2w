#!/usr/bin/env bash
# 在容器内编译工作空间：bash /go2w/scripts/build.sh [colcon 额外参数]
# 第三方包直接从 /go2w/third_party 编译，不复制进 ws/src
set -eo pipefail
source /opt/ros/humble/setup.bash
# livox_ros_driver2 上游只带 package_ROS1/2.xml，它自己的 build.sh 会先复制成 package.xml
LIVOX=/go2w/third_party/livox_ros_driver2
[ -f $LIVOX/package.xml ] || cp $LIVOX/package_ROS2.xml $LIVOX/package.xml
cd /go2w/ws
colcon build --symlink-install \
  --base-paths src /go2w/third_party/livox_ros_driver2 /go2w/third_party/FAST_LIO_ROS2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  "$@"
