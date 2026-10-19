"""Launch the AIDIN BLDC gripper ROS2 driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("aidin_gripper_driver")
    default_cfg = PathJoinSubstitution([pkg, "config", "default.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_cfg,
            description="Path to parameter YAML."),
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="Serial port (overrides config if set)."),
        DeclareLaunchArgument(
            "publish_tactile",
            default_value="false",
            description="Publish ~/tactile from the fingertip tactile sensors (overrides config)."),
        Node(
            package="aidin_gripper_driver",
            executable="gripper_node",
            name="aidin_gripper_driver",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {"port": LaunchConfiguration("port")},
                {"publish_tactile": LaunchConfiguration("publish_tactile")},
            ],
        ),
    ])
