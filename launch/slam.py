import os
from launch import LaunchDescription
from launch_ros.actions import Node

nodes = [
    Node(
        package="rover_driver",
        executable="base",
        # parameters=[{"vid": "x0483", "pid": "x5740"}],
        remappings=[("base/imu/get", "/slam/imu/get")],
    ),
    Node(
        package="spinnaker_camera",
        executable="spinnaker_camera",
        remappings=[("/camera_0/img", "/slam/img/get")],
    ),
]

SLAM = Node(
    package="orb_slam3",
    executable="monocular",
    namespace="slam",
    output="screen",
    emulate_tty=True,
    parameters=[
        {
            "voc_file": "assets/ORBvoc.txt",
            "settings_file": "assets/TestDevice.yaml",
            "use_imu": True,
        }
    ],
)

if "SENSOR_ONLY" not in os.environ:
    nodes.append(SLAM)
else:
    print("[INFO] Launching in >>> sensor only mode <<<")


def generate_launch_description():
    return LaunchDescription(nodes)
