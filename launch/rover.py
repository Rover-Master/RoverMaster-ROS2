from launch import LaunchDescription
from launch_ros.actions import Node

nodes = [
    Node(package="rover_driver", executable="base", namespace="rover"),
    Node(
        package="spinnaker_camera",
        executable="spinnaker_camera",
        namespace="spinnaker",
    ),
    Node(
        package="perception",
        executable="ros2",
        namespace="perception",
        remappings=[
            ("image_in", "/spinnaker/camera_0/img"),
        ],
    ),
    Node(
        package="playback",
        executable="recorder",
        parameters=[
            {"src": "/perception/image_out"},
            {"dst": "var/perception.mp4"},
        ],
    ),
]


def generate_launch_description():
    return LaunchDescription(nodes)
