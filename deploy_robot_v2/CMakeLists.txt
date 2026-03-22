cmake_minimum_required(VERSION 3.10)
project(deploy_robot_v2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

# ============================================================================
# CUDA (可选)
# ============================================================================
find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "CUDA found: ${CUDA_TOOLKIT_ROOT_DIR}")
else()
    message(WARNING "CUDA not found, TensorRT inference will not be available")
endif()

# ============================================================================
# TensorRT
# ============================================================================
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS /usr/include/x86_64-linux-gnu /usr/include/aarch64-linux-gnu
    NO_DEFAULT_PATH
)
find_library(NVINFER_LIB nvinfer
    PATHS /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)
find_library(NVONNXPARSER_LIB nvonnxparser
    PATHS /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)

if(TENSORRT_INCLUDE_DIR AND NVINFER_LIB AND NVONNXPARSER_LIB)
    set(TENSORRT_FOUND TRUE)
    message(STATUS "TensorRT found")
    message(STATUS "  Include: ${TENSORRT_INCLUDE_DIR}")
    message(STATUS "  nvinfer: ${NVINFER_LIB}")
    message(STATUS "  nvonnxparser: ${NVONNXPARSER_LIB}")
else()
    set(TENSORRT_FOUND FALSE)
    message(WARNING "TensorRT not found")
endif()

# ============================================================================
# Threads
# ============================================================================
find_package(Threads REQUIRED)

# ============================================================================
# ROS2 (可选)
# ============================================================================
find_package(rclcpp QUIET)
find_package(geometry_msgs QUIET)

if(rclcpp_FOUND AND geometry_msgs_FOUND)
    set(ROS2_FOUND TRUE)
    message(STATUS "ROS2 found")
    message(STATUS "  rclcpp: ${rclcpp_VERSION}")
else()
    set(ROS2_FOUND FALSE)
    message(STATUS "ROS2 not found, ros2 input source will not be available")
endif()

# ============================================================================
# 源文件
# ============================================================================
set(COMMON_SOURCES
    src/observations.cpp
    src/robstride.cpp
    src/can/can_interface.cpp
    src/imu/serial.c
    src/imu/wit_c_sdk.c
    input/gamepad_command_source.cpp
    input/keyboard_command_source.cpp
    input/input_source.cpp
)

set(COMMON_INCLUDES
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/policy
    ${CMAKE_SOURCE_DIR}/logging
    ${CMAKE_SOURCE_DIR}/input
)

if(CUDA_FOUND)
    list(APPEND COMMON_INCLUDES ${CUDA_INCLUDE_DIRS})
endif()

if(TENSORRT_FOUND)
    list(APPEND COMMON_INCLUDES ${TENSORRT_INCLUDE_DIR})
endif()

# ROS2 include 路径
if(ROS2_FOUND)
    list(APPEND COMMON_INCLUDES ${rclcpp_INCLUDE_DIRS})
    list(APPEND COMMON_INCLUDES /opt/ros/humble/include)
    list(APPEND COMMON_INCLUDES /opt/ros/humble/include/geometry_msgs)
endif()

# ============================================================================
# 可执行目标 (需要 TensorRT)
# ============================================================================
if(TENSORRT_FOUND)
    # 主入口：policy_test_main
    add_executable(policy_test_main
        app/policy_test_main.cpp
        core/robot_bringup.cpp
        policy/policy_runner.cpp
        logging/session_logger.cpp
        src/tensorrt_inference.cpp
        ${COMMON_SOURCES}
    )
    target_include_directories(policy_test_main PRIVATE ${COMMON_INCLUDES})
    target_link_libraries(policy_test_main
        Threads::Threads
        ${CUDA_LIBRARIES}
        ${NVINFER_LIB}
        ${NVONNXPARSER_LIB}
    )
    if(ROS2_FOUND)
        target_sources(policy_test_main PRIVATE input/ros2_command_source.cpp)
        target_compile_definitions(policy_test_main PRIVATE ROS2_FOUND)
        target_link_libraries(policy_test_main
            rclcpp::rclcpp
            /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
        )
    endif()

    # 工具3：shadow_policy
    add_executable(shadow_policy
        tools/shadow_policy_main.cpp
        core/robot_bringup.cpp
        policy/policy_runner.cpp
        logging/session_logger.cpp
        src/tensorrt_inference.cpp
        ${COMMON_SOURCES}
    )
    target_include_directories(shadow_policy PRIVATE ${COMMON_INCLUDES})
    target_link_libraries(shadow_policy
        Threads::Threads
        ${CUDA_LIBRARIES}
        ${NVINFER_LIB}
        ${NVONNXPARSER_LIB}
    )
    if(ROS2_FOUND)
        target_sources(shadow_policy PRIVATE input/ros2_command_source.cpp)
        target_compile_definitions(shadow_policy PRIVATE ROS2_FOUND)
        target_link_libraries(shadow_policy
            rclcpp::rclcpp
            /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
        )
    endif()
else()
    message(WARNING "policy_test_main and shadow_policy not built: TensorRT not found")
endif()

# ============================================================================
# 可执行目标 (不需要 TensorRT)
# ============================================================================

# 校零工具：zero_calibration
add_executable(zero_calibration
    app/zero_calibration_main.cpp
    ${COMMON_SOURCES}
)
target_include_directories(zero_calibration PRIVATE ${COMMON_INCLUDES})
target_link_libraries(zero_calibration Threads::Threads)

# 工具1：inspect_feedback
add_executable(inspect_feedback
    tools/inspect_feedback_main.cpp
    core/robot_bringup.cpp
    ${COMMON_SOURCES}
)
target_include_directories(inspect_feedback PRIVATE ${COMMON_INCLUDES})
target_link_libraries(inspect_feedback Threads::Threads)

# 工具2：ramp_to_offset
add_executable(ramp_to_offset
    tools/ramp_to_offset_main.cpp
    core/robot_bringup.cpp
    ${COMMON_SOURCES}
)
target_include_directories(ramp_to_offset PRIVATE ${COMMON_INCLUDES})
target_link_libraries(ramp_to_offset Threads::Threads)

# ============================================================================
# 安装
# ============================================================================
if(TENSORRT_FOUND)
    install(TARGETS policy_test_main shadow_policy
        RUNTIME DESTINATION bin
    )
endif()
install(TARGETS zero_calibration inspect_feedback ramp_to_offset
    RUNTIME DESTINATION bin
)

# ============================================================================
# 打印配置信息
# ============================================================================
message(STATUS "")
message(STATUS "=== Deploy Robot V2 Build Summary ===")
message(STATUS "zero_calibration: YES")
message(STATUS "inspect_feedback: YES")
message(STATUS "ramp_to_offset: YES")
if(TENSORRT_FOUND)
    message(STATUS "policy_test_main: YES")
    message(STATUS "shadow_policy: YES")
else()
    message(STATUS "policy_test_main: NO (TensorRT not found)")
    message(STATUS "shadow_policy: NO (TensorRT not found)")
endif()
if(ROS2_FOUND)
    message(STATUS "ros2 input: YES")
else()
    message(STATUS "ros2 input: NO (ROS2 not found)")
endif()
message(STATUS "")