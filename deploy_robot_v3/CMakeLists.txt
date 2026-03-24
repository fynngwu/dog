cmake_minimum_required(VERSION 3.18)
project(minimal_dog LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# ============================================================================
# Configurable paths
# ============================================================================
set(POLICY_ENGINE_PATH "/home/ares/pure_cpp/policy.engine" CACHE PATH "Path to policy.engine file")
set(V2_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../deploy_robot_v2" CACHE PATH "Path to deploy_robot_v2 source")

# ============================================================================
# Find CUDA
# ============================================================================
find_program(CUDA_NVCC nvcc)
if(CUDA_NVCC)
    get_filename_component(CUDA_TOOLKIT_ROOT_DIR "${CUDA_NVCC}" DIRECTORY)
    get_filename_component(CUDA_TOOLKIT_ROOT_DIR "${CUDA_TOOLKIT_ROOT_DIR}" DIRECTORY)
    message(STATUS "CUDA nvcc found: ${CUDA_NVCC}")
    message(STATUS "CUDA root: ${CUDA_TOOLKIT_ROOT_DIR}")

    # Set CUDA include dirs
    set(CUDA_INCLUDE_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include")
    # Find CUDA libraries
    find_library(CUDA_CUDART_LIB cudart PATHS ${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib)
    find_library(CUDA_CUBLAS_LIB cublas PATHS ${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib)
    set(CUDA_LIBRARIES ${CUDA_CUDART_LIB} ${CUDA_CUBLAS_LIB})
    set(CUDA_FOUND TRUE)
else()
    set(CUDA_FOUND FALSE)
endif()

if(NOT CUDA_FOUND)
    message(FATAL_ERROR "CUDA not found (nvcc not in PATH)")
endif()

# ============================================================================
# Find TensorRT
# ============================================================================
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS
        /usr/include/x86_64-linux-gnu
        /usr/include/aarch64-linux-gnu
        /usr/include
        /usr/local/TensorRT/include
        /usr/local/cuda/include
)

find_library(NVINFER_LIB nvinfer
    PATHS
        /usr/lib/x86_64-linux-gnu
        /usr/lib/aarch64-linux-gnu
        /usr/lib
        /usr/local/TensorRT/lib
        /usr/local/cuda/lib64
)

find_library(NVONNXPARSER_LIB nvonnxparser
    PATHS
        /usr/lib/x86_64-linux-gnu
        /usr/lib/aarch64-linux-gnu
        /usr/lib
        /usr/local/TensorRT/lib
        /usr/local/cuda/lib64
)

if(TENSORRT_INCLUDE_DIR AND NVINFER_LIB AND NVONNXPARSER_LIB)
    set(TENSORRT_FOUND TRUE)
    message(STATUS "TensorRT found")
    message(STATUS "  Include: ${TENSORRT_INCLUDE_DIR}")
    message(STATUS "  nvinfer: ${NVINFER_LIB}")
    message(STATUS "  nvonnxparser: ${NVONNXPARSER_LIB}")
else()
    message(FATAL_ERROR "TensorRT not found")
endif()

# ============================================================================
# Threads
# ============================================================================
find_package(Threads REQUIRED)

# ============================================================================
# Find ROS2 packages (optional)
# ============================================================================
if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO_VAL "$ENV{ROS_DISTRO}")
else()
    set(ROS_DISTRO_VAL "humble")  # Default to humble
endif()

# Find ament_cmake first (needed for ament_target_dependencies)
find_package(ament_cmake QUIET)

# Find ROS2 packages
find_package(rclcpp QUIET)
find_package(rclcpp_action QUIET)
find_package(geometry_msgs QUIET)
find_package(action_msgs QUIET)

if(rclcpp_FOUND AND geometry_msgs_FOUND)
    set(ROS2_FOUND TRUE)
    message(STATUS "ROS2 found - cmd_vel support enabled")
else()
    set(ROS2_FOUND FALSE)
    message(STATUS "ROS2 not found - cmd_vel support disabled")
endif()

# ============================================================================
# v2 source files to reuse
# ============================================================================
set(V2_SOURCES
    ${V2_SOURCE_DIR}/src/can/can_interface.cpp
    ${V2_SOURCE_DIR}/src/robstride.cpp
    ${V2_SOURCE_DIR}/src/tensorrt_inference.cpp
    ${V2_SOURCE_DIR}/src/imu/serial.c
    ${V2_SOURCE_DIR}/src/imu/wit_c_sdk.c
)

set(V2_INCLUDE_DIRS
    ${V2_SOURCE_DIR}/include
)

# ============================================================================
# Project sources
# ============================================================================
set(PROJECT_SOURCES
    src/main.cpp
    src/minimal_dog.cpp
    src/obs_builder.cpp
    tool/imu_reader.cpp
    tool/motor_io.cpp
    tool/motor_math.cpp
    tool/policy.cpp
)

# Add ROS2 command source if available
if(ROS2_FOUND)
    list(APPEND PROJECT_SOURCES tool/ros2_command_source.cpp)
endif()

set(PROJECT_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tool
)

# ============================================================================
# Combine include directories
# ============================================================================
set(ALL_INCLUDE_DIRS
    ${PROJECT_INCLUDE_DIRS}
    ${V2_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
)

# ============================================================================
# Create library for reused v2 components
# ============================================================================
add_library(v2_components STATIC ${V2_SOURCES})
target_include_directories(v2_components PUBLIC ${ALL_INCLUDE_DIRS})

# ============================================================================
# Main executable
# ============================================================================
add_executable(minimal_dog ${PROJECT_SOURCES})
target_include_directories(minimal_dog PRIVATE ${ALL_INCLUDE_DIRS})

# Link libraries
target_link_libraries(minimal_dog
    v2_components
    Threads::Threads
    ${CUDA_LIBRARIES}
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB}
)

# Add ROS2 support if available
if(ROS2_FOUND)
    # Add ROS2 include directories (typically /opt/ros/<distro>/include)
    # Add ROS2 include directories (all subdirectories under /opt/ros/<distro>/include)
    file(GLOB ROS2_INCLUDE_DIRS "/opt/ros/${ROS_DISTRO_VAL}/include/*")
    target_include_directories(minimal_dog PRIVATE
        "/opt/ros/${ROS_DISTRO_VAL}/include"
        ${ROS2_INCLUDE_DIRS}
    )
    message(STATUS "ROS2 include dir: /opt/ros/${ROS_DISTRO_VAL}/include")

    target_compile_definitions(minimal_dog PRIVATE ROS2_FOUND)

    # Use ament_target_dependencies to properly link ROS2 libraries
    # This handles all transitive dependencies automatically
    ament_target_dependencies(minimal_dog
        rclcpp
        rclcpp_action
        geometry_msgs
        action_msgs
    )

    add_executable(ros2_command_test
        tests/ros2_command_test.cpp
        tool/ros2_command_source.cpp
    )
    target_include_directories(ros2_command_test PRIVATE ${ALL_INCLUDE_DIRS})
    target_compile_definitions(ros2_command_test PRIVATE ROS2_FOUND)
    target_link_libraries(ros2_command_test Threads::Threads)
    target_include_directories(ros2_command_test PRIVATE
        "/opt/ros/${ROS_DISTRO_VAL}/include"
        ${ROS2_INCLUDE_DIRS}
    )
    ament_target_dependencies(ros2_command_test
        rclcpp
        rclcpp_action
        geometry_msgs
        action_msgs
    )
endif()

# Pass policy engine path as compile definition
target_compile_definitions(minimal_dog PRIVATE POLICY_ENGINE_PATH="${POLICY_ENGINE_PATH}")


add_executable(imu_tool_test
    tests/imu_tool_test.cpp
    tool/imu_reader.cpp
)
target_include_directories(imu_tool_test PRIVATE ${ALL_INCLUDE_DIRS})
target_link_libraries(imu_tool_test v2_components Threads::Threads)

add_executable(motor_tool_test
    tests/motor_tool_test.cpp
    tool/motor_io.cpp
    tool/motor_math.cpp
)
target_include_directories(motor_tool_test PRIVATE ${ALL_INCLUDE_DIRS})
target_link_libraries(motor_tool_test v2_components Threads::Threads)


# ============================================================================
# Build summary
# ============================================================================
message(STATUS "")
message(STATUS "=== Minimal Dog Build Summary ===")
message(STATUS "CUDA: YES (${CUDA_TOOLKIT_ROOT_DIR})")
message(STATUS "TensorRT: YES (${TENSORRT_INCLUDE_DIR})")
message(STATUS "")
