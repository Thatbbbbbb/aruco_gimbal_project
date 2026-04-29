cmake_minimum_required(VERSION 3.15)
project(gimbal_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O3 -march=native")
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)

set(TENSORRT_PATH "/usr/local/tensorrt")
set(CUDA_PATH "/usr/local/cuda-11.8")

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

add_definitions(-DMACRO_AIMASSISTNN_FLAG -DDISABLE_ROS2)
add_subdirectory(deploy)
link_directories(/usr/local/cuda-11.8/lib64)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/tasks
    ${PROJECT_SOURCE_DIR}/tasks/yolos
    ${PROJECT_SOURCE_DIR}/io
    ${PROJECT_SOURCE_DIR}/io/hikrobot
    ${PROJECT_SOURCE_DIR}/io/hikrobot/include
    ${PROJECT_SOURCE_DIR}/tools
    ${PROJECT_SOURCE_DIR}/deploy
    ${PROJECT_SOURCE_DIR}/deploy/core
    ${PROJECT_SOURCE_DIR}/deploy/infer
    ${PROJECT_SOURCE_DIR}/deploy/utils
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${spdlog_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
    ${TENSORRT_PATH}/include
    ${LIBUSB_INCLUDE_DIRS}
)

set(MAIN_SOURCES src/main.cpp src/serial_port.cpp)
file(GLOB TASK_SOURCES tasks/yolos/*.cpp)
file(GLOB_RECURSE DEPLOY_SOURCES deploy/*.cpp deploy/*.cu)
file(GLOB IO_SOURCES io/*.cpp io/gimbal/*.cpp)
file(GLOB TOOL_SOURCES tools/*.cpp)

if(EXISTS ${PROJECT_SOURCE_DIR}/io/hikrobot/hikrobot.cpp)
    list(APPEND IO_SOURCES ${PROJECT_SOURCE_DIR}/io/hikrobot/hikrobot.cpp)
    link_directories(${PROJECT_SOURCE_DIR}/io/hikrobot/lib/amd64)
endif()

set(ALL_SOURCES ${MAIN_SOURCES} ${TASK_SOURCES} ${IO_SOURCES} ${TOOL_SOURCES}) 
list(FILTER ALL_SOURCES EXCLUDE REGEX "\\.hpp$|\\.h$")
add_executable(gimbal_node ${ALL_SOURCES})

# 使用 CUDAToolkit 找到的库
link_directories(/usr/local/tensorrt/lib)
target_link_libraries(gimbal_node
    deploy
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    spdlog::spdlog
    fmt::fmt
    yaml-cpp
    spdlog
    Threads::Threads
    pthread
    dl
    rt
    stdc++fs
    ${CUDAToolkit_LIBRARIES}
    /usr/local/tensorrt/lib/libnvinfer.so
    /usr/local/tensorrt/lib/libnvinfer_plugin.so
    /usr/local/tensorrt/lib/libnvonnxparser.so
    /usr/local/tensorrt/lib/libnvparsers.so
    ${LIBUSB_LIBRARIES}
)

# 添加 CUDA 运行时库的显式路径
link_directories(/usr/local/tensorrt/lib)
target_link_libraries(gimbal_node /usr/local/cuda-11.8/lib64/libcudart.so)

if(EXISTS ${PROJECT_SOURCE_DIR}/io/hikrobot/lib/amd64/libMvCameraControl.so)
    link_directories(/usr/local/tensorrt/lib)
    target_link_libraries(gimbal_node MvCameraControl)
endif()

set_target_properties(gimbal_node PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

message(STATUS "========================================")
message(STATUS "Build Configuration Complete")
message(STATUS "========================================")
