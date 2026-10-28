# Go2W 导航运行环境：ROS 2 Humble（arm64，跑在 Orin 上）
# 构建上下文是仓库根目录，见 docker/compose.yaml
ARG BASE_IMAGE=docker.m.daocloud.io/library/ros:humble-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Orin 访问不到官方源时走清华镜像
ARG APT_MIRROR=mirrors.tuna.tsinghua.edu.cn
RUN sed -i "s|http://ports.ubuntu.com|http://${APT_MIRROR}|g" /etc/apt/sources.list \
 && find /etc/apt/sources.list.d -type f -exec sed -i "s|http://packages.ros.org|http://${APT_MIRROR}|g" {} +

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      python3-colcon-common-extensions python3-numpy python3-scipy \
      libpcl-dev libeigen3-dev \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-pcl-conversions ros-humble-pcl-ros \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-robot-state-publisher ros-humble-xacro \
      ros-humble-tf2-ros ros-humble-tf2-tools \
      ros-humble-slam-toolbox \
      ros-humble-navigation2 ros-humble-nav2-bringup \
      ros-humble-foxglove-bridge \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-teleop-twist-keyboard \
      iproute2 iputils-ping tmux less nano \
 && rm -rf /var/lib/apt/lists/*

# Livox-SDK2：源码来自 third_party/（Orin 上无法直接访问 GitHub）
COPY third_party/Livox-SDK2 /tmp/Livox-SDK2
RUN cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" \
 && cmake --install /tmp/Livox-SDK2/build \
 && ldconfig && rm -rf /tmp/Livox-SDK2

# 可视化桥：把导航核心（域 43，仅本机）的部分话题转发到域 44（Wi-Fi，给 Ubuntu 的 RViz）
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-domain-bridge \
 && rm -rf /var/lib/apt/lists/*

RUN echo 'source /go2w/docker/ros_env.sh' >> /root/.bashrc

WORKDIR /go2w/ws
CMD ["sleep", "infinity"]
