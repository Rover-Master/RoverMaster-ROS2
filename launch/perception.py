from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node
from os import environ as env
from pathlib import Path
from datetime import datetime


run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
PWD = Path(env['PWD'])
RUN_VAR = str(PWD / 'var' / run_id)
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
            {"dst": RUN_VAR + "/perception_images"},
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
    ),

    # Define an event handler to run the encode-video script on shutdown
    RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[str(PWD / 'scripts/runtime-bin/encode-video'), RUN_VAR+"/perception_images"],
                    cwd=str(PWD),
                    shell=True,
                    output='screen'
                )
            ]
        )
    ),

]

def generate_launch_description():
    return LaunchDescription(nodes)
