find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# tools 源文件
set(TOOLS_SOURCES
    ${CMAKE_SOURCE_DIR}/tools/logger.cpp
    ${CMAKE_SOURCE_DIR}/tools/exiter.cpp
    ${CMAKE_SOURCE_DIR}/tools/img_tools.cpp
    ${CMAKE_SOURCE_DIR}/tools/math_tools.cpp
)

# 公共链接库
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

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(detect_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(detect_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/arm64)
endif()

target_include_directories(detect_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/include      # 添加 include 路径，找到 serial_port.h 和 engineer_kinematics.hpp
    ${CMAKE_SOURCE_DIR}/io
    ${EIGEN3_INCLUDE_DIRS}
)

# 可选：如果需要 pthread
target_link_libraries(detect_test PRIVATE pthread)

# ========== lightbar_test ==========
add_executable(lightbar_test
    lightbar_test.cpp
    ${TOOLS_SOURCES}
)

target_link_libraries(lightbar_test PRIVATE ${COMMON_LIBS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(lightbar_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(lightbar_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/arm64)
endif()

target_include_directories(lightbar_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/io
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(lightbar_test PRIVATE pthread)
# ========== kinematic_test ==========
add_executable(kinematic_test
    kinematic_test.cpp
    ${TOOLS_SOURCES}
)

target_link_libraries(kinematic_test PRIVATE ${COMMON_LIBS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(kinematic_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(kinematic_test PRIVATE ${CMAKE_SOURCE_DIR}/io/hikrobot/lib/arm64)
endif()

target_include_directories(kinematic_test PRIVATE
    ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/io
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(kinematic_test PRIVATE pthread)