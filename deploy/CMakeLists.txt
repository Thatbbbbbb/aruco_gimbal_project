# 设置项目
cmake_minimum_required(VERSION 3.12)
cmake_policy(SET CMP0091 NEW)
project(deploy LANGUAGES CXX CUDA)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成编译数据库，便于代码分析工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
 

if(NOT TENSORRT_PATH)
    message(FATAL_ERROR "TensorRT path is not set. Please specify the TensorRT path.")
endif()

# 配置CUDA和TensorRT的函数
function(configure_cuda_trt target)
    target_compile_definitions(${target} PRIVATE ${CUDA_DEFINITIONS})
    
    # [修改] 将 include 路径改为 PUBLIC，这样引用 deploy 的项目也能找到 CUDA 头文件
    target_include_directories(${target} PUBLIC ${CUDA_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${CUDA_cudart_LIBRARY})

    # [修改] 将 include 路径改为 PUBLIC，这样引用 deploy 的项目也能找到 TensorRT 头文件 (如 NvInferRuntime.h)
    target_include_directories(${target} PUBLIC ${TENSORRT_PATH}/include)
    # 添加 CUDA 头文件路径
    target_include_directories(${target} PUBLIC /usr/local/cuda-11.8/include)
    target_link_directories(${target} PRIVATE ${TENSORRT_PATH}/lib)
    
    target_link_libraries(${target} PRIVATE 
    /usr/local/tensorrt/lib/libnvinfer.so
    /usr/local/tensorrt/lib/libnvinfer_plugin.so
    /usr/local/tensorrt/lib/libnvonnxparser.so
)
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