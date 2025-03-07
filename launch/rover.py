from launch import LaunchDescription
from launch_ros.actions import Node

nodes = [
    Node(
        package="rover",
        executable="base",
        namespace="rover",
        parameters=[
            {"vid": "x0483"},
            {"pid": "x5740"},
        ],
    ),
    Node(
        package="socket_agent",
        executable="agent",
        remappings=[
            ("vel/get", "/rover/base/velocity/get"),
            ("vel/set", "/rover/base/velocity/set"),
            ("imu", "/rover/base/imu"),
            ("halt", "/rover/base/halt"),
        ],
    ),
]

# return nodes
def generate_launch_description():
    return LaunchDescription(nodes)
