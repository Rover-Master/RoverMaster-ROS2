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
        package="spinnaker_camera",
        executable="capture",
        namespace="spinnaker",
    ),
    Node(
        package="perception",
        executable="ros2",
        namespace="perception",
        remappings=[
            ("image_in", "/spinnaker/camera_0/img"),
            ("motion", "/rover/base/velocity/set"),
        ],
    ),
    Node(
        package="playback",
        executable="recorder",
        parameters=[
            {"src": "/perception/image_out"},
            {"dst": "var/perception"},
        ],
    ),
]


def generate_launch_description():
    return LaunchDescription(nodes)
