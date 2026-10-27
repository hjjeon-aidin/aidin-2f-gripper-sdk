cmake_minimum_required(VERSION 3.16)
project(aidin_gripper_driver CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(aidin_gripper_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Pull in the Aidin C++ SDK (sibling directory in this monorepo).
# The SDK is fully self-contained (in-tree Modbus RTU framing) — no external
# Modbus library, no network access at configure time.
# Override AIDIN_SDK_DIR if your workspace layout differs.
# ---------------------------------------------------------------------------
set(AIDIN_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../cpp" CACHE PATH
    "Path to aidin C++ SDK (cpp/ at the repo root)")
if(NOT EXISTS "${AIDIN_SDK_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "Cannot find aidin C++ SDK at '${AIDIN_SDK_DIR}'.\n"
        "Either clone the full aidin-2f-gripper-sdk repo so that cpp/ sits next to ros2/,\n"
        "or pass -DAIDIN_SDK_DIR=/abs/path/to/cpp to colcon.")
endif()

set(AIDIN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(${AIDIN_SDK_DIR} aidin_sdk_build)

# ---------------------------------------------------------------------------
# Driver node
# ---------------------------------------------------------------------------
# OFF compiles the developer-command service (~/dev_cmd, Protocol Manual §3)
# out of the node entirely. ~/fault_clear stays available either way.
option(AIDIN_ENABLE_DEV_CMD "Expose the ~/dev_cmd service" ON)

add_executable(gripper_node src/gripper_node.cpp)
target_link_libraries(gripper_node aidin::gripper)
if(NOT AIDIN_ENABLE_DEV_CMD)
    target_compile_definitions(gripper_node PRIVATE AIDIN_ENABLE_DEV_CMD=0)
endif()
ament_target_dependencies(gripper_node
    rclcpp std_msgs std_srvs aidin_gripper_msgs)

install(TARGETS gripper_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_package()
