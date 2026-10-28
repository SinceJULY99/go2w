# unitree_go

Go2/Go2W 主控状态消息（`/sportmodestate`、`/lowstate` 等），ROS 2 域 0、走有线口。

`msg/` 取自 [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
（commit `668d1ec`）的 `cyclonedds_ws/src/unitree/unitree_go/msg`，未改动，字段与主控的 DDS 类型一一对应。
`CMakeLists.txt` 去掉了上游只给 RTI Connext 用的 `rosidl_generator_dds_idl` 步骤。

导航里用到的：`/sportmodestate`（`SportModeState`）里的 `imu_state.rpy`，机身相对重力的横滚 / 俯仰 / 偏航（rad）。
