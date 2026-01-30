cmake_minimum_required(VERSION 3.10)
project(aruco_gimbal_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖库
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED) # 新增：查找yaml-cpp库
find_package(Threads REQUIRED)

# 包含头文件目录
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS} # 新增：添加yaml-cpp头文件目录
)

# 源文件列表（新增config_parser.cpp）
set(SOURCES
    src/aruco_detection.cpp
    src/serial_port.cpp
    src/gimbal_transform.cpp
    src/config_parser.cpp
    src/main.cpp
)

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接依赖库（新增yaml-cpp库）
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

# 安装配置（可选）
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION config/${PROJECT_NAME})