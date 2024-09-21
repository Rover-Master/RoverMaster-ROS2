from launch import LaunchDescription
from launch_ros.actions import Node
import os

if "DATASET" in os.environ:
    DATASET = os.environ["DATASET"]
else:
    DATASET = "MH_01"

nodes = [
    Node(
        package="playback",
        executable="player",
        parameters=[{"src": f"var/{DATASET}", "dst": "/player"}],
        remappings=[
            ("/player/cam0", "/slam/img/get"),
            ("/player/imu", "/slam/imu/get"),
        ],
    ),
    Node(
        package="playback",
        executable="recorder",
        parameters=[{"src": "/slam/tracking_image", "dst": f"var/{DATASET}_SLAM.mp4"}],
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
            "settings_file": "assets/RoverMaster.yaml",
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
