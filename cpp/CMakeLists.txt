cmake_minimum_required(VERSION 3.16)
project(aidin_gripper_sdk
        VERSION 1.2.0
        DESCRIPTION "C++ SDK for Aidin BLDC Gripper (DH-Robotics Modbus RTU, self-contained framing)"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---------- SDK library ----------
# Modbus RTU framing is implemented in-tree (src/modbus_rtu.cpp) —
# no external Modbus library, no FetchContent, no network at configure time.
add_library(aidin_gripper STATIC
    src/gripper.cpp
    src/modbus_rtu.cpp
)
target_include_directories(aidin_gripper
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
    # Sources are UTF-8: /utf-8 avoids C4819 on non-UTF-8 code pages (e.g. 949, Korean Windows)
    target_compile_options(aidin_gripper PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(aidin_gripper PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(aidin::gripper ALIAS aidin_gripper)

# ---------- Examples ----------
option(AIDIN_BUILD_EXAMPLES "Build example programs" ON)
if(AIDIN_BUILD_EXAMPLES)
    foreach(ex
        01_activate_and_home
        02_open_close_loop
        03_status_monitor
        04_tactile_monitor
        05_fast_poll)
        add_executable(${ex} examples/${ex}.cpp)
        target_link_libraries(${ex} PRIVATE aidin_gripper)
        if(MSVC)
            target_compile_options(${ex} PRIVATE /utf-8)
        endif()
    endforeach()
endif()

message(STATUS "")
message(STATUS "Aidin Gripper SDK  v${PROJECT_VERSION}")
message(STATUS "  Build type   : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Examples     : ${AIDIN_BUILD_EXAMPLES}")
message(STATUS "  Modbus       : self-contained RTU framing (no external library)")
message(STATUS "")
