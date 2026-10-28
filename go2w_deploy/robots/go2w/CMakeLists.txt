cmake_minimum_required(VERSION 3.12)
project(go2w_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Arch 适配：不走 sudo make install，直接指向本地 unitree_sdk2 源码树
set(UNITREE_SDK_DIR "/home/barry/Desktop/debug/unitree_sdk2-main" CACHE PATH "unitree_sdk2 源码目录")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

include_directories(
  /usr/include/eigen3
  ${UNITREE_SDK_DIR}/include                    # dds (原为 /usr/local/include)
  ${UNITREE_SDK_DIR}/thirdparty/include
  ${UNITREE_SDK_DIR}/thirdparty/include/ddscxx
  include
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_directories(${UNITREE_SDK_DIR}/thirdparty/lib/x86_64)

# Arch 的 spdlog 用 external fmt 编译，不定义这个宏会和 bundled fmt 冲突
add_compile_definitions(SPDLOG_FMT_EXTERNAL)

link_libraries(
  ${UNITREE_SDK_DIR}/lib/x86_64/libunitree_sdk2.a
  ddsc ddscxx rt pthread # dds
  libboost_program_options.a yaml-cpp fmt spdlog   # Arch 无 libyaml-cpp.a，改用动态库
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so
)

file(GLOB_RECURSE ADD_SRC_LIST
  src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(go2w_ctrl main.cpp)

# onnxruntime / cyclonedds 是本地路径的动态库，写死 rpath 免得跑时找不到
set_target_properties(go2w_ctrl PROPERTIES
  BUILD_RPATH "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib;${UNITREE_SDK_DIR}/thirdparty/lib/x86_64"
)
