# go2w_deploy

Go2-W 的 RL 策略部署控制器。基于宇树 [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) 的 `deploy/`(C++),只保留 `robots/go2w`。

替代原厂运控:关掉原厂运控 → 本控制器读 `rt/lowstate`、跑 ONNX 策略、发 `rt/lowcmd`。

## 相对官方的改动

- **Go2-W / DreamWaQ 适配**:`deploy.yaml` 顶层 `wheel_ids` 指定哪几个动作是轮子(速度目标);新增观测 `joint_pos_rel_wheel_ids`
- **兼容 `SportClient::Move`**:`include/sport_move_bridge.h` 监听 `rt/api/sport/request`,外部程序(避障等)照常调 `Move` 即可驱动策略
- **关运控工具**:`tools/release_mode`,不再需要手机 App
- **编译适配**:`CMakeLists.txt`(Arch)、`CMakeLists.txt.jetson`(机载 Orin)、`CMakeLists.txt.ubuntu.orig`(官方原版)

## 外部速度指令(给避障等程序)

照常用原厂接口:`SportClient::Move(vx, vy, vyaw)`(机体系,m/s、rad/s)。

- 需持续发送(建议 ≥ 10 Hz),**0.5 s 收不到即自动归零**
- 摇杆有输入时摇杆优先;指令被夹到训练范围(当前 ±1)
- 只支持 `Move`、`StopMove`,其余 sport api 返回 3203
- 控制器监听 `eth0`(123 网段),外部程序需走同一网段

## 用法(机载 Jetson)

```bash
ssh unitree@192.168.123.18          # 或 WiFi 地址
~/go2w_deploy/start.sh flat         # 或 dreamwaq;会先关运控(先让狗趴下,等确认)
~/go2w_deploy/stop.sh               # 先遥控器 LT+B 趴下再停
```

遥控器:`LT+A` 站起 → `Start` 进策略 → `LT+B` 趴下(急停)。

脚本在 `scripts/jetson/`。Jetson 目录布局:`~/go2w_deploy/{deploy,unitree_sdk2,policies}`。

## 编译

依赖 unitree_sdk2(用源码树自带预编译库,`UNITREE_SDK_DIR` 指定)和 onnxruntime 1.22.0(放到 `thirdparty/`,按架构下载 [x64](https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz) / [aarch64](https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz))。

```bash
cd robots/go2w                      # Jetson 上先 cp CMakeLists.txt.jetson CMakeLists.txt
mkdir -p build && cd build && cmake .. && make -j
./go2w_ctrl -n <网卡>
```

注:SDK 若从 GitHub zip 解压,`thirdparty/lib/*/libddsc*.so.0` 符号链接会变成文本文件,需 `ln -sf` 重建。

## 策略

`robots/go2w/config/config.yaml` 的 `policy_dir` 指向策略目录(`exported/policy.onnx` + `params/deploy.yaml`),策略不在本仓库:

| 策略 | 说明 |
|---|---|
| `robotlab_go2w_flat` | robot_lab 纯平地,57 维单帧,站姿 0.8/-1.5 |
| `dreamwaq_go2w` | DreamWaQ 爬台阶/高台,6 帧 342 维,站姿 0.67/-1.3 |

切策略需同时改 `policy_dir` 和 FixStand `qs` 末行(Jetson 上 `start.sh` 已自动处理)。
