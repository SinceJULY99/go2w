# Go2W + MID360 几何导航：项目交接说明

> 更新日期：2026-10-21
> 项目：四足机器人具身智能导航技术研发
> 当前阶段：重建二维几何导航（ROS 2 Humble；旧系统中包含 Foxy）
> 依据：两份历史文档（《完整启动与恢复手册》10-13 版、《Agent 交接说明》），以及 2026-10-16 对 Orin 的实测。标为"待确认"的项目需要在设备开机后核实。

## 一句话现状

已经跑通**雷达采集、FAST-LIO 里程计、点云转二维激光、slam_toolbox 建图、静态地图上的 AMCL 初定位**。还没做成的是可靠的定位、Nav2 规划和机器人自主行走的闭环。

旧系统的地图会偏斜，重启恢复也很繁琐，所以决定**在 Orin 上重建一套干净的系统**，旧系统保留作回退。具身导航（语义地图、VLN）放到几何导航稳定之后再做。

---

## 1. 硬件与网络

| 设备 | 型号 / 系统 | 地址（上次实测） | 角色 |
|---|---|---|---|
| Go2W 内置 PC2 | Unitree 扩展坞主机；ROS 版本**待确认** | Wi-Fi `192.168.1.142`；内网 `192.168.123.18`；用户 `unitree` | 接 MID360，运行 `livox_ros_driver2` |
| Jetson Orin | Seeed reComputer Robo，Orin NX 16GB（J401）；Ubuntu 24.04，L4T R39.2 | Wi-Fi `192.168.1.111`；有线 `192.168.123.222`（网口 `enP7p1s0`）；用户 `w` | 核心计算：FAST-LIO、点云处理；新方案中承担全部导航计算 |
| 外部 Ubuntu | Ubuntu + ROS 2 Humble | `192.168.1.167`；用户 `dx` | RViz 可视化；旧系统中还运行 map_server、AMCL、slam_toolbox |
| Mac | 开发机 | — | SSH 运维、写代码；已配好到 Orin 的免密登录 |
| Livox MID360 | 激光雷达，带内置 IMU | IP **待确认** | 倾斜安装在 Go2W 上，历史上试过 10° / 11.5° / 13° |

说明：

- 以上 IP 都是 DHCP 动态分配的，重启后可能变化。10-21 重启后 Orin 仍为 `192.168.1.111`。
- Orin 有线口的 `192.168.123.222` 已确认是 NetworkManager 里的永久配置（连接名 `go2w-wired`）。它只在 Go2W 开机、网线有链路时才会出现，平时看不到是正常的。

---

## 2. 之前的系统是怎么做的

旧系统由三台机器协作，具体以 2026-10-12 实测的《完整启动与恢复手册》为准。

### 2.1 数据流

```text
MID360 ──▶ PC2：~/start-mid360.sh → livox_ros_driver2
             话题 /livox/lidar（livox_ros_driver2/msg/CustomMsg）
   │ 有线 192.168.123.x
   ▼
Orin：Docker 容器 go2w_foxy（ROS 2 Foxy）
   /ws/start-fastlio.sh → laser_mapping（FAST-LIO）
       输出 /cloud_registered_body、/Odometry、TF camera_init→body（约 8–9 Hz）
   pointcloud_to_laserscan：target_frame=body，高度 ±0.15 m → /scan_body
   4 个自写 TCP 转发脚本（两套 CycloneDDS 配置，内网和 Wi-Fi 各一套）
       scan_tcp_sender / receiver（端口 19001）→ /scan_body_wifi
       tf_tcp_sender / receiver（端口 19002）→ /tf
   │ Wi-Fi
   ▼
外部 Ubuntu（Humble）
   map_server：/home/dx/maps/go2w_lab.yaml（435×347，0.05 m/px）
   AMCL：/home/dx/go2w_amcl.yaml
       odom_frame=camera_init，base_frame=body，scan=/scan_body_wifi，OmniMotionModel
   或者 slam_toolbox 建图：/home/dx/go2w_slam_body.yaml（与 AMCL 二选一）
   RViz2：Fixed Frame=map，TF 链 map→camera_init→body
```

### 2.2 启动方式

需要手动开十几个终端，按以下顺序执行：

1. 核对 IP 和 DDS 配置
2. PC2 启动雷达驱动
3. Orin 进入容器，启动 FAST-LIO
4. 启动 pcl2scan
5. 重建 `/tmp` 下的 Wi-Fi DDS 配置，启动 4 个转发进程
6. Ubuntu 启动 map_server，用 lifecycle 激活
7. 启动并激活 AMCL
8. 在 RViz 中手动设置初始位姿

所有节点统一使用 `ROS_DOMAIN_ID=42` 和 `rmw_cyclonedds_cpp`。

### 2.3 完成度

| 模块 | 状态 | 说明 |
|---|---|---|
| MID360 驱动 | ✅ 已跑通 | 只能有 1 个发布者，曾因重复启动驱动导致不稳定 |
| PC2 ↔ Orin 有线链路 | ✅ 已跑通 | 是从 Wi-Fi 卡顿（1–2 Hz）的教训中改过来的 |
| FAST-LIO | ✅ 已跑通 | 点云和里程计正常；外参和长时间漂移没有系统验证 |
| 点云转二维激光 | ⚠️ 有问题 | 在倾斜的坐标系里切片，见第 3 节 |
| 跨机可视化 | ⚠️ 可用但脆弱 | 依赖 4 个 TCP 脚本；出现过话题列表为空、QoS 不兼容 |
| slam_toolbox 建图 | ⚠️ 有问题 | 能出图，但墙体偏斜，有双线和散点 |
| AMCL 定位 | ⚠️ 仅静态对齐 | 协方差较大，动态定位精度没有验证 |
| Nav2 + Go2W 速度控制 | ⬜ 未开始 | 没有 cmd_vel 速度桥，没有自主行走 |
| 语义地图 / VLN | ⬜ 暂缓 | 几何导航稳定后再推进 |

---

## 3. 发现的问题与原因

### 3.1 定位依赖 Wi-Fi，延迟约 2 秒（最严重）

AMCL 跑在外部 Ubuntu 上，scan 要经过 Wi-Fi 才能到达。手册记录的 LaserScan 延迟约 1.9–2 秒，TF 也有波动。AMCL 需要把 scan 和同一时刻的位姿配对，机器人一动，定位就会被拖偏。如果在这个架构上接 Nav2，控制回路里会带着 2 秒延迟，没法安全导航。

后来查到的加重因素：

- Orin 的无线网卡开着省电模式，ping 延迟 80–130 ms（已修复）。
- 信号 −68 dBm，偏弱。
- Orin 的上行速率只有约 29 Mbit/s，而往外发点云和 scan 用的正是上行。

### 3.2 坐标系跟着雷达倾斜，这是地图偏斜的主要嫌疑

FAST-LIO 的 `camera_init` 和 `body` 都是 MID360 内置 IMU 的坐标系，跟着雷达一起倾斜 10–13°，并不对齐重力。

在 `body` 系里切 ±0.15 m，实际切出来的是一个斜面：每往前 1 米，切片高度变化约 0.2 米。结果是：

- 前方一两米处的地板被切进来，当成了障碍物；
- 远处只切到墙的高处；
- 地图因此出现双线、偏斜和散点。

此外：

- AMCL 和 slam_toolbox 直接把倾斜的 `camera_init` 和 `body` 当作 odom 和 base 来用。
- ±0.15 m 的切片太窄，比雷达低 15 cm 以上的障碍物全部看不到，不能用来避障。

> **10-21 实测（`scripts/calib/mid360_static_check.py`，机器人静止）**：雷达实际前倾约 **16°**、横滚约 −3°，雷达原点离地 **0.54 m**；点云中的地面、天花板法向量与 IMU 重力方向一致（误差约 1°），说明点云与 IMU 处于同一坐标系，驱动配置里的 pitch 90°/yaw 180° 没有造成错位。0.5 m 内没有点，保护架在 MID360 盲区内，不影响。雷达 x 轴是否指向机器人正前方尚待行驶验证。
>
> 注意：这个结论是根据坐标系配置、切片参数和地图表现推断的，倾斜部分已由上方实测确认。
> 验证方法：机器人在平地上静止，运行 `ros2 run tf2_ros tf2_echo camera_init body`，看 pitch 是否接近安装倾角。
> 另外，RViz 里的网格画在 Fixed Frame 的 z=0 平面上，`camera_init` 的原点在雷达的高度。所以地面点出现在网格下方是正常的，判断依据应该是地面是否水平、墙是否竖直。

### 3.3 开机时间错误（1970 年）

- Orin 断电后 RTC 时间丢失，开机时系统时间是 1969-12-31。
- chrony 没有启用等待同步的服务（`chrony-wait` 是 disabled），`go2w_foxy` 容器又设置了开机自启（`unless-stopped`）。所以容器在 1970 年就启动了，之后 NTP 同步，时间突然跳到 2026 年。
- 这会让 TF 缓存和时间戳出错，很可能就是手册里"时间问题"的来源。
- 系统时区也被设成了 AKDT（阿拉斯加时间）。

### 3.4 架构复杂，重启恢复脆弱

Foxy 和 Humble 混用、两套 DDS 配置（其中一套放在 `/tmp`，重启就会丢）、4 个转发进程、动态 IP、手动的启动顺序，任何一环漏掉都需要人工排查。

两份历史文档之间也有矛盾，例如交接说明写 Orin 用的是 Humble，实际容器是 Foxy。**以手册为准。**

### 3.5 Orin 磁盘几乎占满

79 GB 只剩 3.3 GB（占用 96%），没法构建新镜像。已清理，见第 4 节。

---

## 4. 本轮已完成的工作（10-16 至 10-19）

| 事项 | 状态 | 具体做法与结果 |
|---|---|---|
| Mac → Orin 免密 SSH | ✅ 完成 | 用 `ssh-copy-id` 安装公钥，用 `ssh-add --apple-use-keychain` 把私钥加入钥匙串 |
| 关闭 Wi-Fi 省电 | ✅ 完成 | `nmcli connection modify YanGu_5G 802-11-wireless.powersave 2`，永久生效。ping 从 80–130 ms 降到平均 12 ms |
| docker 权限 | ✅ 完成 | 用户 `w` 加入 docker 组，docker 命令不再需要 sudo |
| 磁盘清理 | ✅ 完成 | 删除 vscode-cpptools、pip、浏览器缓存，Docker 构建缓存（4.6 GB），conda 包缓存。可用空间从 3.3 GB 增加到 16 GB。旧镜像 `go2w-foxy:base` 和 conda 环境 `unitree_rl_mjlab` 保留 |
| 有线网口配置核查 | ✅ 完成 | `go2w-wired` 是手动配置的 `192.168.123.222/24`，开启自动连接，不需要修改 |
| 时间修复脚本 | ✅ 完成（10-21 验证） | 见下方说明。重启后容器在开机约 30 秒后、时间同步完成时启动，时间正确 |

时间修复脚本：

- 位置：`scripts/orin/fix-time.sh`，已拷到 Orin 的 `~/fix-time.sh`。
- 内容：
  1. 时区改为 Asia/Shanghai；
  2. 加入阿里云和腾讯的 NTP 服务器；
  3. 启用 `chrony-wait`，开机最多等 90 秒，没网时超时后照常开机；
  4. Docker 改为在 `time-sync.target` 之后启动。
- 执行：`sudo bash ~/fix-time.sh`，重启后生效。

---

## 5. 新方案：和之前有什么不同

算法组件基本不换，仍然是 FAST-LIO、slam_toolbox、AMCL、Nav2。变化在于**组件跑在哪里、坐标系怎么定、系统怎么启动**。

### 5.1 架构对比

之前：

```text
PC2     雷达驱动
 │ 有线
Orin    FAST-LIO → 在倾斜的 body 系里切 scan
 │ TCP 转发 + Wi-Fi（约 2 秒延迟）
Ubuntu  AMCL / slam_toolbox + RViz（定位在这里算）
```

新方案：

```text
PC2 或 Orin  雷达驱动（取决于 Orin 能不能直接连到雷达）
 │ 有线
Orin    FAST-LIO → 平面 odom → 在 base_link 系里切 scan
        → AMCL / slam_toolbox → Nav2 → cmd_vel 速度桥 → Go2W
        （整个闭环都在 Orin 上，不经过 Wi-Fi）
 │ Wi-Fi（只传显示数据）
Ubuntu  RViz（看画面，点初始位姿和目标点）
```

### 5.2 逐项对比

| 方面 | 之前 | 新方案 |
|---|---|---|
| 定位和建图在哪算 | Ubuntu，经 Wi-Fi 接收数据 | Orin，和 FAST-LIO 在同一台机器上；Wi-Fi 断了也能照常导航 |
| ROS 版本 | Orin 用 Foxy，Ubuntu 用 Humble | 统一用 Humble。Orin 宿主机是 24.04，没有 Humble 的 apt 包，所以放在容器里 |
| 跨机通信 | 4 个 TCP 脚本 + 两套 DDS 配置 | Humble 之间直接走 DDS；也可以用 foxglove_bridge 做可视化 |
| 坐标系 | `map → camera_init → body`，带倾斜 | 按 REP-105：`map → odom → base_link → livox_frame`，`base_link` 贴地且水平 |
| scan 切片 | `body` 系 ±0.15 m | `base_link` 系按离地高度切；定位用墙体高度那一段，避障用低位那一段 |
| 雷达外参 | 手动试角度 | 静止时用 RANSAC 拟合地面，算出倾角和安装高度 |
| 启动 | 十几个终端手敲命令 | `docker compose` 开机自启，在时间同步之后启动，配健康检查脚本 |
| 配置管理 | 分散在各台机器上，部分放在 `/tmp` | 全部放进一个 git 仓库 |

### 5.3 保持不变的部分

- FAST-LIO 本来就跑在 Orin 上，这一点之前就是对的。
- PC2 和 Orin 之间走有线。
- 仍然在 Ubuntu 上用 RViz 看画面。
- AMCL 等组件的调参经验可以沿用。

---

## 6. 后续任务

每个阶段通过验收后，再进入下一阶段。旧系统 `go2w_foxy` 在新系统跑通之前一直保留，作为回退方案。

### 6.1 第 0 步：收尾和摸底

| # | 任务 | 状态 | 说明 |
|---|---|---|---|
| 0.1 | 执行 `fix-time.sh`，重启后验证 | ✅ 完成（10-21） | 验证方法：`docker inspect -f '{{.State.StartedAt}}' go2w_foxy` 不再是 1970 年 |
| 0.2 | 在路由器上给三台机器做 DHCP 地址保留 | ⬜ 未做 | 否则每次重启都要改 DDS 配置和 SSH 地址 |
| 0.3 | 配置 `~/.ssh/config` 别名；给 PC2 也配免密登录 | 🔶 PC2 免密已完成，别名未配 | `ssh-copy-id unitree@<PC2_IP>` |
| 0.4 | 验证 USB 救命通道 | ⬜ 未做 | 用 USB-C 线连接 Orin 的设备口，`ssh w@192.168.55.1`。改网络配置之前必须先确认这条通道可用 |
| 0.5 | 给 Orin 载板装 RTC 纽扣电池 | ⬜ 未做 | 先确认载板上有没有电池座 |
| 0.6 | **Go2W 开机摸底** | ✅ 完成（10-21） | 确认 PC2 的系统和 ROS 版本；确认 MID360 的 IP（出厂默认 `192.168.1.1xx`，和 Wi-Fi 网段冲突）；确认 Orin 能否直接 ping 到雷达。这一项决定最终是单机还是三机架构 |
| 0.7 | 确认 Go2W 在轮式模式下能否横移 | ✅ 能（10-21 实测横向速度最高 0.52 m/s） | 不能横移的话，AMCL 应改用 `DifferentialMotionModel` |

### 6.2 第 1–7 步：重建几何导航

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| S1 基础环境 ✅（10-21） | Orin 上建 Humble 容器（arm64，host 网络），新建 git 仓库，用 compose 管理。镜像 `go2w-humble:dev`、容器 `go2w_humble`，`ROS_DOMAIN_ID=43`；用法见仓库 README | 一条命令起环境，重启后自动恢复（已验证：开机约 36 秒后、时间同步完成时自动启动） |
| S2 传感器与坐标系 ✅（10-21） | 跑通 `livox_ros_driver2`；定义 `base_link`；用地面拟合标定雷达倾角和高度，写进 URDF 或静态 TF | 平地静止时，地面点在 `base_link` 下 z≈0，墙是竖直的 |
| S3 里程计 ✅（10-21） | Humble 版 FAST-LIO，加一个平面 odom 节点：订阅 `/Odometry`，只保留 x、y、yaw，发布 `odom → base_link` | 绕一圈回到起点，误差小于 0.2 m，静止时不漂 |
| S4 建图 ✅（10-21） | 在 `base_link` 下切 scan，用 slam_toolbox 建图并保存 | 用 TopDownOrtho 俯视，墙线笔直、平行的墙保持平行、没有双线；重新加载后和实时 scan 对齐 |
| S5 定位 ✅（10-21） | map_server + AMCL，跑在 Orin 上 | 走一圈 scan 一直和地图贴合；换一个位置重新给初始位姿后能快速收敛 |
| S6 底盘控制 | cmd_vel 速度桥，调用 unitree_sdk2 的 sport 接口，带限速、看门狗、急停。可以参考开源项目 unitree-go2w-autonomous-carrier 里的 `go2w_cmd_vel_control` | 键盘遥控平稳；断开连接或杀掉节点后 0.5 秒内停车 |
| S7 Nav2 | 先限速 0.3 m/s 以下，再调 costmap 和控制器；障碍层用 3D 点云或低位 scan | 连续导航到多个目标点，能绕开临时放置的障碍物 |


> S2/S3 结果（10-21）：Orin 经有线口 `192.168.1.51/32` 直接接收雷达，PC2 不再参与。外参（地面 + 行驶标定）：z 0.527 m、roll −1.80°、pitch 15.59°、yaw −3.4°、雷达在旋转中心前 0.187 m。静止 2 分钟漂移 < 5 mm；绕场 13 m 回原点误差 1.8 cm、航向 0.2°。第一次绕圈因 Orin 有线网口行驶中断开 3 秒而失败，已加 `/odom_healthy` 检测；网线固定仍需处理。


> S4/S5 结果（10-21）：地图 `lab_20261021`（15.4×21.8 m，墙线笔直、无双线）。定位用 map_server + AMCL（OmniMotionModel），跑在 Orin 上；可视化经 domain_bridge（域 43→44）送到 Ubuntu RViz（`scripts/ubuntu/go2w_rviz.sh`）。差速模型在倒车+侧移时粒子发散，改全向模型后 230 s 测试（含 17 s 倒车、最高 0.52 m/s 横移、60°/s 转向）全程稳定，收敛后不确定度 ≤0.2 m / 7.6°，scan 贴合率 89–96%。首次建图因网线断开 25 s 导致 FAST-LIO 发散、slam_toolbox OOM；现在 odom 不健康时会停发 TF。

### 6.3 贯穿全程

- 每个阶段结束后更新启动手册，替换掉现有两份互相矛盾的文档。
- 写一个健康检查脚本：一条命令检查话题频率、TF、重复节点和时间同步。
- 离线时间同步：让 Orin 作为局域网时间服务器，这样在没有外网的场地，三台机器的时间也能保持一致。

### 6.4 之后：具身导航

1. 加 RGB-D 相机（Orin 载板有 GMSL 接口）。
2. 开放词汇检测 + 语义地图。
3. 用 LLM 或 VLM 把语言指令转成 Nav2 目标点。
4. 视情况研究 NaVILA 一类的 VLN 方法。

原则：大模型只决定"去哪里"，Nav2 负责"怎么去"。

---

## 7. 注意事项

- **不要重复启动节点。** 雷达驱动、`laser_mapping`、AMCL 都出现过重复实例。启动前先用 `ros2 node list` 和 `pgrep` 检查。
- **建图和定位互斥。** slam_toolbox 和 AMCL 都会发布 `map → odom`，不能同时运行。
- **旧系统里禁止在 RViz 中直接点 2D Goal Pose。** 动态定位精度没有验证，也没有速度桥和急停。
- **通过 Wi-Fi SSH 修改网络时，只改有线配置。** 改之前先确认 USB 救命通道可用。
- **Foxy 版 CycloneDDS 的配置格式不一样。** 要用 `<NetworkInterfaceAddress>`，不能用 `<Interfaces>`。
- **任何会让机器人运动的测试**，先在架空或空旷环境下低速进行，并安排人随时急停。
- **不要用 `killall python3`**，在各自的终端里用 Ctrl+C 停止。
- Orin 宿主机的终端默认会激活 conda 的 `(base)` 环境，可能影响 ROS 的 Python 环境。建议执行 `conda config --set auto_activate_base false`。

---

## 8. 资料位置

| 资料 | 位置 |
|---|---|
| **新系统启动手册（10-21）** | 仓库 `docs/启动手册.md` |
| 完整启动与恢复手册（10-13 版，旧系统的实测流程，已被上一行取代） | Mac：`~/Downloads/Go2W_三机导航系统完整启动与恢复手册.docx` |
| 架构与 Agent 交接说明（部分内容已过时，以手册为准） | Mac：`~/Downloads/Go2W_三机导航系统_Agent交接说明.md` |
| 新仓库（建设中）与时间修复脚本 | Mac：`~/Desktop/go2w/`，脚本在 `scripts/orin/fix-time.sh` |
| 旧系统 Orin 端 | 容器 `go2w_foxy` 内：`/ws/start-fastlio.sh`、`/ws/cyclonedds_fastlio.xml`、`/ws/*_tcp_*.py` |
| 旧系统 Ubuntu 端 | `/home/dx/maps/go2w_lab.*`、`/home/dx/go2w_amcl.yaml`、`/home/dx/go2w_slam_body.yaml`、`/home/dx/mid360-wifi.xml` |
| 旧系统 PC2 端 | `/home/unitree/start-mid360.sh` |
| 参考开源项目 | https://github.com/tzf230201/unitree-go2w-autonomous-carrier （主要参考它的速度桥和里程计投影） |
