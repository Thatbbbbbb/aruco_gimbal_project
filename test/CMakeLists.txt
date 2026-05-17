find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加 tools 目录下的源文件
set(TOOLS_SOURCES
    ${CMAKE_SOURCE_DIR}/tools/logger.cpp
    ${CMAKE_SOURCE_DIR}/tools/exiter.cpp
    ${CMAKE_SOURCE_DIR}/tools/img_tools.cpp
    ${CMAKE_SOURCE_DIR}/tools/math_tools.cpp
)

# 公共链接库列表（两个 test 都可能需要）
set(COMMON_LIBS
    io
    ${OpenCV_LIBS}
    yaml-cpp
    MvCameraControl
    fmt
    usb-1.0
)

# ========== detect_test ==========
add_executable(detect_test
    detect_test.cpp
    ${TOOLS_SOURCES}
)

target_link_libraries(detect_test PRIVATE ${COMMON_LIBS})

# 海康库路径（根据架构判断，两个 test 都需要）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(detect_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(detect_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/arm64)
endif()

# 包含路径（如果需要）
target_include_directories(detect_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tools
    ${EIGEN3_INCLUDE_DIRS}
)

# ========== lightbar_test ==========
add_executable(lightbar_test
    lightbar_test.cpp
    ${TOOLS_SOURCES}
)

target_link_libraries(lightbar_test PRIVATE ${COMMON_LIBS})

# 海康库路径（同样需要）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(lightbar_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(lightbar_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/arm64)
endif()

target_include_directories(lightbar_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tools
    ${EIGEN3_INCLUDE_DIRS}
)