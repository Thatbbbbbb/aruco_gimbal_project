add_library(io STATIC
    camera.cpp
    hikrobot/hikrobot.cpp
)

# 设置头文件路径等
target_include_directories(io PUBLIC ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include  ${CMAKE_SOURCE_DIR} )

# 链接海康 SDK 库和依赖
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64)
endif()
target_link_libraries(io PUBLIC MvCameraControl yaml-cpp)