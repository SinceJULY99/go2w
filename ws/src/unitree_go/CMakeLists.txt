cmake_minimum_required(VERSION 3.8)
project(unitree_go)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息定义取自 unitreerobotics/unitree_ros2（668d1ec）cyclonedds_ws/src/unitree/unitree_go，未改动；
# 去掉了上游只给 RTI Connext 用的 dds_idl 生成步骤（我们用 CycloneDDS）。见 README
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  "msg/BmsCmd.msg"
  "msg/BmsState.msg"
  "msg/Error.msg"
  "msg/Go2FrontVideoData.msg"
  "msg/HeightMap.msg"
  "msg/IMUState.msg"
  "msg/InterfaceConfig.msg"
  "msg/LidarState.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/MotorCmd.msg"
  "msg/MotorCmds.msg"
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  "msg/PathPoint.msg"
  "msg/Req.msg"
  "msg/Res.msg"
  "msg/SportModeCmd.msg"
  "msg/SportModeState.msg"
  "msg/TimeSpec.msg"
  "msg/UwbState.msg"
  "msg/UwbSwitch.msg"
  "msg/WirelessController.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()
