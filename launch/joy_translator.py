"""Launch a joy_translator node."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Create a joy_translator node with a launch description."""
    device_argument = DeclareLaunchArgument(
        "device",
        default_value="/dev/input/js0",
        description="joystick device name",
    )
    controller_argument = DeclareLaunchArgument(
        "controller",
        default_value="xbox-360-controller.yaml",
        description="desired controller configuration",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"dev": LaunchConfiguration("device")}],
        output={"both": "screen"},
    )
    joy_translator_node = Node(
        package="ezrassor_joy_translator",
        executable="joy_translator",
        parameters=[
            PathJoinSubstitution(
                (
                    get_package_share_directory("ezrassor_joy_translator"),
                    "config",
                    LaunchConfiguration("controller"),
                )
            ),
        ],
        output={"both": "screen"},
    )

    return LaunchDescription(
        [
            device_argument,
            controller_argument,
            joy_node,
            joy_translator_node,
        ]
    )
