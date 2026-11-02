# Go2W 导航

Go2W + Livox MID360 的二维几何导航系统，运行在 ROS 2 Humble 容器 `go2w_humble` 里（ROS_DOMAIN_ID=43）。

> **2026-10-28 起导航跑在 Go2W 扩展坞（`unitree@192.168.1.142`）上，Orin 只作备用**，同一套代码按网卡自动识别机器。
> 先读 `docs/扩展坞部署说明.md`；下文和其他文档里以 Orin 为主的命令仍适用于 Orin，扩展坞上把 `scripts/sync.sh` 换成 `scripts/sync.sh dock`。

旧系统（Foxy 容器 `go2w_foxy`，ROS_DOMAIN_ID=42）保留在 Orin 上作回退。
项目背景、问题分析和阶段计划见 `docs/Go2W导航项目交接说明.md`。

## 目录

```
docker/            Dockerfile、compose.yaml、容器内 ROS 环境 ros_env.sh
ws/src/            自己写的 ROS 包（go2w_bringup：launch、参数、机器人描述）
scripts/           sync.sh（Mac→Orin 同步）、build.sh（容器内编译）
scripts/orin/      Orin 宿主机配置脚本（fix-time.sh）
scripts/calib/     标定工具（mid360_static_check.py）
third_party.lock   第三方源码版本；源码本身只在 Orin 的 ~/go2w/third_party/
```

## 日常流程

```bash
# Mac：改完代码同步到 Orin（Wi-Fi 不通时自动经 PC2 走有线）
scripts/sync.sh

# Orin：首次或 Dockerfile 改动后构建并启动容器（之后开机自启）
cd ~/go2w/docker && docker compose up -d --build

# Orin：编译
docker exec go2w_humble bash /go2w/scripts/build.sh

# Orin：进入容器
docker exec -it go2w_humble bash
```

## 登录方式

| 路径 | 命令 |
|---|---|
| Wi-Fi | `ssh w@192.168.1.111` |
| 经 PC2 走有线（Wi-Fi 断了时） | `ssh -o HostKeyAlias=192.168.1.111 -J unitree@192.168.1.142 w@192.168.123.222` |
| USB-C 设备口 | `ssh w@192.168.55.1` |
