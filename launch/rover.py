from launch import LaunchDescription
from launch_ros.actions import Node

nodes = [
    Node(package="rover_driver", executable="base", namespace="rover"),
    Node(
        package="spinnaker_camera",
        executable="spinnaker_camera",
        namespace="rover",
    ),
    Node(
        package="perception",
        executable="ros2",
        namespace="rover",
    ),
]


def generate_launch_description():
    return LaunchDescription(nodes)
