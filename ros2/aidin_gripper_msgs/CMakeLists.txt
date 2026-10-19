cmake_minimum_required(VERSION 3.16)
project(aidin_gripper_msgs)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    msg/GripperState.msg
    msg/TactileTip.msg
    msg/TactileState.msg
)

set(srv_files
    srv/MoveTo.srv
    srv/EmergencyRelease.srv
    srv/DevCmd.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
