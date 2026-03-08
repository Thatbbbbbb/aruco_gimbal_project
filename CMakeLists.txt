cmake_minimum_required(VERSION 3.10)
project(GimbalControl)

set(CMAKE_CXX_STANDARD 14)

# 查找依赖
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 收集源文件
set(SOURCES
    src/main.cpp
    src/config_parser.cpp
    src/detector.cpp
    src/serial_port.cpp
)

add_executable(gimbal_node ${SOURCES})

target_include_directories(gimbal_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR} 
    
)

target_link_libraries(gimbal_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
    pthread
)