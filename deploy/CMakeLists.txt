# 设置项目
cmake_minimum_required(VERSION 3.12)
cmake_policy(SET CMP0091 NEW)
project(deploy LANGUAGES CXX CUDA)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成编译数据库，便于代码分析工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置默认 TensorRT 路径（如果未设置）
if(NOT TENSORRT_PATH)
    # 尝试多个可能的路径
    if(EXISTS "/home/thatbbbbbb/TensorRT-10.0.1.6")
        set(TENSORRT_PATH "/home/thatbbbbbb/TensorRT-10.0.1.6")
    elseif(EXISTS "/usr/local/tensorrt")
        set(TENSORRT_PATH "/usr/local/tensorrt")
    else()
        message(FATAL_ERROR "TensorRT path is not set. Please specify the TensorRT path using -DTENSORRT_PATH=/path/to/TensorRT")
    endif()
    message(STATUS "Auto-detected TensorRT path: ${TENSORRT_PATH}")
endif()

# 检查 TensorRT 路径是否存在
if(NOT EXISTS ${TENSORRT_PATH})
    message(FATAL_ERROR "TensorRT path does not exist: ${TENSORRT_PATH}")
endif()

# 检查必要的库文件是否存在
set(TENSORRT_LIB_DIR ${TENSORRT_PATH}/lib)
set(TENSORRT_INCLUDE_DIR ${TENSORRT_PATH}/include)

message(STATUS "TensorRT libraries path: ${TENSORRT_LIB_DIR}")
message(STATUS "TensorRT include path: ${TENSORRT_INCLUDE_DIR}")

# 配置CUDA和TensorRT的函数
function(configure_cuda_trt target)
    target_compile_definitions(${target} PRIVATE ${CUDA_DEFINITIONS})
    
    # 将 include 路径改为 PUBLIC，这样引用 deploy 的项目也能找到 CUDA 头文件
    target_include_directories(${target} PUBLIC ${CUDA_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${CUDA_cudart_LIBRARY})

    # 将 include 路径改为 PUBLIC，这样引用 deploy 的项目也能找到 TensorRT 头文件
    target_include_directories(${target} PUBLIC ${TENSORRT_INCLUDE_DIR})
    target_include_directories(${target} PUBLIC /usr/local/cuda-11.8/include)
    target_link_directories(${target} PRIVATE ${TENSORRT_LIB_DIR})
    
    # 链接 TensorRT 库（只链接存在的库）
    if(EXISTS ${TENSORRT_LIB_DIR}/libnvinfer.so)
        target_link_libraries(${target} PRIVATE ${TENSORRT_LIB_DIR}/libnvinfer.so)
    else()
        message(WARNING "libnvinfer.so not found in ${TENSORRT_LIB_DIR}")
    endif()
    
    if(EXISTS ${TENSORRT_LIB_DIR}/libnvinfer_plugin.so)
        target_link_libraries(${target} PRIVATE ${TENSORRT_LIB_DIR}/libnvinfer_plugin.so)
    else()
        message(WARNING "libnvinfer_plugin.so not found in ${TENSORRT_LIB_DIR}")
    endif()
    
    # 可选库，存在则链接
    if(EXISTS ${TENSORRT_LIB_DIR}/libnvonnxparser.so)
        target_link_libraries(${target} PRIVATE ${TENSORRT_LIB_DIR}/libnvonnxparser.so)
        message(STATUS "Found libnvonnxparser.so")
    endif()
    
    if(EXISTS ${TENSORRT_LIB_DIR}/libnvparsers.so)
        target_link_libraries(${target} PRIVATE ${TENSORRT_LIB_DIR}/libnvparsers.so)
        message(STATUS "Found libnvparsers.so")
    endif()
endfunction()

# 添加源文件的函数
function(add_compile_files target)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..) 
    file(GLOB_RECURSE SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/infer/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/infer/*.cu
        ${CMAKE_CURRENT_SOURCE_DIR}/model.cpp
    )
    target_sources(${target} PRIVATE ${SOURCES})
endfunction()

# 设置编译选项的函数
function(set_compile_options target)
    target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-O3 -flto=auto>)
    target_link_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-O3 -flto=auto>)
endfunction()

# 定义目标 deploy
add_library(deploy STATIC)
add_compile_files(deploy)
configure_cuda_trt(deploy)
set_compile_options(deploy)
set_target_properties(deploy PROPERTIES OUTPUT_NAME deploy)

set_target_properties(deploy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)