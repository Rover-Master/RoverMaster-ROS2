from launch import LaunchDescription
from launch_ros.actions import Node

nodes = [
    Node(package="rover", executable="base", namespace="rover"),
    Node(
        package="socket_agent",
        executable="socket_agent",
        remappings=[
            ("velocity/get", "/rover/base/velocity/get"),
            ("velocity/set", "/rover/base/velocity/set"),
            ("imu", "/rover/base/imu"),
            ("halt", "/rover/base/halt"),
        ],
    ),
]


def generate_launch_description():
    return LaunchDescription(nodes)
