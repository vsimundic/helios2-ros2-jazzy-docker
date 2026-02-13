import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tree_capture")

    default_params = os.path.join(pkg_share, "params", "capture_params.yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the YAML file with capture parameters",
        ),

        Node(
            package="tree_capture",
            executable="capture_node",
            name="tree_capture",
            output="screen",
            parameters=[params_file],
        ),
    ])
