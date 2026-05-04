cmake_minimum_required(VERSION 3.15)
project(gimbal_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O3 -march=native")
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)

# ========== 统一路径 ==========
set(TENSORRT_ROOT "/home/thatbbbbbb/TensorRT-10.0.1.6")
set(CUDA_PATH "/usr/local/cuda-11.8")

# ========== 自动查找依赖 ==========
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(CUDAToolkit REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ========== TensorRT 手动配置 ==========
set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)
set(TENSORRT_LIBRARY_DIR ${TENSORRT_ROOT}/lib)

set(TENSORRT_LIBRARIES
    ${TENSORRT_LIBRARY_DIR}/libnvinfer.so
    ${TENSORRT_LIBRARY_DIR}/libnvinfer_plugin.so
)

if(EXISTS ${TENSORRT_LIBRARY_DIR}/libnvonnxparser.so)
    list(APPEND TENSORRT_LIBRARIES ${TENSORRT_LIBRARY_DIR}/libnvonnxparser.so)
endif()

add_definitions(-DMACRO_AIMASSISTNN_FLAG -DDISABLE_ROS2)
add_subdirectory(deploy)

# ========== 头文件路径 ==========
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/tasks
    ${PROJECT_SOURCE_DIR}/tasks/yolos
    ${PROJECT_SOURCE_DIR}/io
    ${PROJECT_SOURCE_DIR}/io/hikrobot/include
    ${PROJECT_SOURCE_DIR}/tools
    ${PROJECT_SOURCE_DIR}/deploy
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_PATH}/include
)

# ========== 源文件 ==========
set(MAIN_SOURCES src/main.cpp src/serial_port.cpp)
file(GLOB TASK_SOURCES tasks/yolos/*.cpp tasks/lightbar_detector.cpp)
file(GLOB IO_SOURCES io/*.cpp io/hikrobot/hikrobot.cpp)
file(GLOB TOOL_SOURCES tools/*.cpp)

set(ALL_SOURCES ${MAIN_SOURCES} ${TASK_SOURCES} ${IO_SOURCES} ${TOOL_SOURCES})

# ========== 可执行文件 ==========
add_executable(gimbal_node ${ALL_SOURCES})

# ========== 库链接（修复版） ==========
target_link_directories(gimbal_node PUBLIC
    ${CUDA_PATH}/lib64
    ${TENSORRT_LIBRARY_DIR}
    ${PROJECT_SOURCE_DIR}/io/hikrobot/lib/amd64
)

target_link_libraries(gimbal_node
    deploy
    ${OpenCV_LIBS}
    yaml-cpp
    spdlog::spdlog
    fmt::fmt
    Threads::Threads
    dl
    rt
    stdc++fs
    CUDA::cudart
    ${TENSORRT_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    MvCameraControl
)
add_executable(camera_test test/camera_test.cpp)

# 链接必要的库
target_link_libraries(camera_test
    ${OpenCV_LIBS}
    yaml-cpp
    spdlog
    fmt
    pthread
    dl
)

# 添加头文件路径
target_include_directories(camera_test PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

set_target_properties(gimbal_node PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

message(STATUS "========================================")
message(STATUS "Build Configuration Complete")
message(STATUS "TensorRT Path: ${TENSORRT_ROOT}")
message(STATUS "========================================")