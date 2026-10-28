# 容器内的 ROS 环境，交互 shell 自动加载；脚本里用 `source /go2w/docker/ros_env.sh`
source /opt/ros/humble/setup.bash
[ -f /go2w/ws/install/setup.bash ] && source /go2w/ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 容器以 root 运行，避免在挂载的仓库里留下宿主机用户删不掉的 __pycache__
export PYTHONDONTWRITEBYTECODE=1

# 42 是旧系统（go2w_foxy）用的，新系统用 43，两套同时运行也互不干扰
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
# 网卡绑定交给 cyclonedds.xml：域 43 只走本机回环，域 44 走 Wi-Fi（仅可视化桥使用）
# ROS_LOCALHOST_ONLY=1 会强制所有域都只用回环，必须关掉
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///go2w/docker/cyclonedds.xml}

# 同一套代码跑在两台机器上，按网卡识别（容器用 host 网络，看得到宿主机网卡）：
#   Orin（reComputer）：有线 enP7p1s0、Wi-Fi wlP1p1s0、雷达 host IP 192.168.1.51
#   扩展坞（PC2）     ：有线 eth0、Wi-Fi wlan0（USB 网卡）、雷达 host IP 192.168.1.50
# cyclonedds.xml 和 lidar.launch.py 读这几个变量；需要时可在外面预先设置来覆盖
if [ -e /sys/class/net/enP7p1s0 ]; then
  export GO2W_HOST=${GO2W_HOST:-orin}
  export GO2W_WIRED_IF=${GO2W_WIRED_IF:-enP7p1s0}
  export GO2W_WIFI_IF=${GO2W_WIFI_IF:-wlP1p1s0}
  export GO2W_LIDAR_HOST_IP=${GO2W_LIDAR_HOST_IP:-192.168.1.51}
else
  export GO2W_HOST=${GO2W_HOST:-dock}
  export GO2W_WIRED_IF=${GO2W_WIRED_IF:-eth0}
  export GO2W_WIFI_IF=${GO2W_WIFI_IF:-wlan0}
  export GO2W_LIDAR_HOST_IP=${GO2W_LIDAR_HOST_IP:-192.168.1.50}
fi
