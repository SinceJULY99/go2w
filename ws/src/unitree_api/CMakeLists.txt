cmake_minimum_required(VERSION 3.8)
project(unitree_api)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 与 unitree_sdk2 的 DDS 类型 unitree_api::msg::dds_::*_ 字段逐一对应（见 README）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RequestIdentity.msg"
  "msg/RequestLease.msg"
  "msg/RequestPolicy.msg"
  "msg/RequestHeader.msg"
  "msg/Request.msg"
  "msg/ResponseStatus.msg"
  "msg/ResponseHeader.msg"
  "msg/Response.msg"
)

ament_package()
