cmake_minimum_required(VERSION 3.12)
project(release_mode)

set(CMAKE_CXX_STANDARD 17)

# 和 go2w_ctrl 一样直接用 unitree_sdk2 源码树里的预编译库。Jetson 默认路径如下,
# 其他机器用 -DUNITREE_SDK_DIR=... 指定
set(UNITREE_SDK_DIR "$ENV{HOME}/go2w_deploy/unitree_sdk2" CACHE PATH "unitree_sdk2 源码目录")
set(SDK_ARCH ${CMAKE_SYSTEM_PROCESSOR})  # aarch64 / x86_64

include_directories(
  ${UNITREE_SDK_DIR}/include
  ${UNITREE_SDK_DIR}/thirdparty/include
  ${UNITREE_SDK_DIR}/thirdparty/include/ddscxx
)
link_directories(${UNITREE_SDK_DIR}/thirdparty/lib/${SDK_ARCH})

add_executable(release_mode main.cpp)
target_link_libraries(release_mode
  ${UNITREE_SDK_DIR}/lib/${SDK_ARCH}/libunitree_sdk2.a
  ddsc ddscxx rt pthread
)
# 机上 /usr/local/lib 有旧 SDK 的另一版 libddsc,写死 rpath 用随附的
set_target_properties(release_mode PROPERTIES
  BUILD_RPATH "${UNITREE_SDK_DIR}/thirdparty/lib/${SDK_ARCH}"
)
