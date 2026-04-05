cmake_minimum_required(VERSION 3.10)
project(dog_debug_daemon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

find_package(Threads REQUIRED)

add_executable(dog_debug_daemon
    daemon/main.cpp
    daemon/can_interface.cpp
    daemon/robstride.cpp
    daemon/motor_io.cpp
    daemon/twin_agent.cpp
)

target_include_directories(dog_debug_daemon PRIVATE daemon)
target_link_libraries(dog_debug_daemon Threads::Threads)
