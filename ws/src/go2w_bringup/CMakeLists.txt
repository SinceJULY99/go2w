cmake_minimum_required(VERSION 3.8)
project(go2w_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/planar_odom.py scripts/go2w_cmd_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_package()
