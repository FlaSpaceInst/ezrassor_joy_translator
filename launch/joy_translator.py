"""Launch a joy_translator node."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Create a joy_translator node with a launch description."""
    device_argument = DeclareLaunchArgument(
        "device",
        default_value="/dev/input/js0",
        description="joystick device name",
    )
    controller_argument = DeclareLaunchArgument(
        "controller",
        default_value="xbox-360",
        description="pre-loaded controller configuration",
    )
    controller_path_argument = DeclareLaunchArgument(
        "controller_path",
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    get_package_share_directory("ezrassor_joy_translator"),
                    "config",
                    "",
                ),
            ),
            LaunchConfiguration("controller"),
            TextSubstitution(text="-controller.yaml"),
        ],
        description="path to desired controller configuration",
    )

    joy_node = Node(
        package="joy",
        node_executable="joy_node",
        parameters=[{"dev": LaunchConfiguration("device")}],
        output={"both": "screen"},
    )
    joy_translator_node = Node(
        package="ezrassor_joy_translator",
        node_executable="joy_translator",
        parameters=[LaunchConfiguration("controller_path")],
        output={"both": "screen"},
    )

    return LaunchDescription(
        [
            device_argument,
            controller_argument,
            controller_path_argument,
            joy_node,
            joy_translator_node,
        ]
    )
