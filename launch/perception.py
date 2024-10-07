from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from os import environ as env
from pathlib import Path
run_id = "TODO"
PWD = Path(env['PWD'])
RUN_VAR = Path(PWD) / 'var' / run_id
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
            {"dst": RUN_VAR},
        ],
    ),
    Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        parameters=[{
            'channel_type':'serial',
            'serial_port': '/dev/rplidar', 
            'serial_baudrate': 115200, 
            'frame_id': 'laser',
            'inverted': False, 
            'angle_compensate': True,
        }],
        output='screen'
    ),
    Node(
        package='lidar_toolbox',
        executable='scan_transformer',
        output='screen'
    ),
    Node(
        package='lidar_toolbox',
        executable='proximity',
        output='screen'
    ),
    ExecuteProcess(
        cmd=[
            *['ros2', 'bag', 'record', '-o', RUN_VAR],
            '/scan_transformed',
            '/rover/base/velocity/get',
            '/rover/base/halt'
        ],
        cwd=str(PWD / 'var'),
        output='screen'
    )
    # TODO: Run FFMPEG video encoder upon completion
]


def generate_launch_description():
    return LaunchDescription(nodes)
