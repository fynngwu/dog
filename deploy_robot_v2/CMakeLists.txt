cmake_minimum_required(VERSION 3.14)
project(deploy_robot_v2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

# ============================================================================
# CUDA 路径设置
# ============================================================================
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4" CACHE PATH "CUDA Toolkit root directory")

# 尝试查找 CUDA
find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "CUDA found: ${CUDA_VERSION}")
    message(STATUS "CUDA include: ${CUDA_INCLUDE_DIRS}")
    message(STATUS "CUDA libraries: ${CUDA_LIBRARIES}")
else()
    message(STATUS "CUDA not found, using default paths")
    set(CUDA_INCLUDE_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include")
    set(CUDA_LIBRARIES "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so")
endif()

# ============================================================================
# TensorRT 路径 (TensorRT 8.6.1 tar 包安装)
# ============================================================================
set(TENSORRT_ROOT "/usr/local/TensorRT-8.6.1.6" CACHE PATH "TensorRT root directory")
set(TENSORRT_INCLUDE "${TENSORRT_ROOT}/include" CACHE PATH "TensorRT include directory")
set(TENSORRT_LIB "${TENSORRT_ROOT}/lib" CACHE PATH "TensorRT lib directory")

# ============================================================================
# 源文件
# ============================================================================
set(COMMON_SOURCES
    src/observations.cpp
    src/robstride.cpp
    src/tensorrt_inference.cpp
    src/can/can_interface.cpp
    src/imu/serial.c
    src/imu/wit_c_sdk.c
    input/gamepad_command_source.cpp
    input/keyboard_command_source.cpp
)

set(COMMON_INCLUDES
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/policy
    ${CMAKE_SOURCE_DIR}/logging
    ${CMAKE_SOURCE_DIR}/input
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE}
)

set(COMMON_LIBS
    ${CUDA_LIBRARIES}
    nvinfer
)

# 添加库搜索路径
link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${TENSORRT_LIB}
)

# ============================================================================
# 可执行目标
# ============================================================================

# 主入口：policy_test_main
add_executable(policy_test_main
    app/policy_test_main.cpp
    core/robot_bringup.cpp
    policy/policy_runner.cpp
    logging/session_logger.cpp
    ${COMMON_SOURCES}
)
target_include_directories(policy_test_main PRIVATE ${COMMON_INCLUDES})
target_link_libraries(policy_test_main ${COMMON_LIBS})

# 校零工具：zero_calibration
add_executable(zero_calibration
    app/zero_calibration_main.cpp
    ${COMMON_SOURCES}
)
target_include_directories(zero_calibration PRIVATE ${COMMON_INCLUDES})
target_link_libraries(zero_calibration ${COMMON_LIBS})

# 工具1：inspect_feedback
add_executable(inspect_feedback
    tools/inspect_feedback_main.cpp
    core/robot_bringup.cpp
    ${COMMON_SOURCES}
)
target_include_directories(inspect_feedback PRIVATE ${COMMON_INCLUDES})
target_link_libraries(inspect_feedback ${COMMON_LIBS})

# 工具2：ramp_to_offset
add_executable(ramp_to_offset
    tools/ramp_to_offset_main.cpp
    core/robot_bringup.cpp
    ${COMMON_SOURCES}
)
target_include_directories(ramp_to_offset PRIVATE ${COMMON_INCLUDES})
target_link_libraries(ramp_to_offset ${COMMON_LIBS})

# 工具3：shadow_policy
add_executable(shadow_policy
    tools/shadow_policy_main.cpp
    core/robot_bringup.cpp
    policy/policy_runner.cpp
    logging/session_logger.cpp
    ${COMMON_SOURCES}
)
target_include_directories(shadow_policy PRIVATE ${COMMON_INCLUDES})
target_link_libraries(shadow_policy ${COMMON_LIBS})

# ============================================================================
# 安装
# ============================================================================
install(TARGETS policy_test_main zero_calibration inspect_feedback ramp_to_offset shadow_policy
    RUNTIME DESTINATION bin
)

# ============================================================================
# 打印配置信息
# ============================================================================
message(STATUS "=== Deploy Robot V2 Configuration ===")
message(STATUS "CUDA toolkit: ${CUDA_TOOLKIT_ROOT_DIR}")
message(STATUS "TensorRT: ${TENSORRT_ROOT}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")