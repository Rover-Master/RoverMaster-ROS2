from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from pathlib import Path
HOME = Path(__file__).parent.parent

VAR = HOME / "var"

from shutil import rmtree
rmtree(VAR)
VAR.mkdir()

channel_type =  LaunchConfiguration('channel_type', default='serial')
serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
frame_id = LaunchConfiguration('frame_id', default='laser')
inverted = LaunchConfiguration('inverted', default='false')
angle_compensate = LaunchConfiguration('angle_compensate', default='true')
scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

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
        namespace="capture",
    ),
    Node(
        package="camera_detection",
        executable="perception",
        namespace="capture",
        remappings=[
            ("img", "/capture/camera_0/img")
        ],
    ),
    Node(
        package="camera_detection",
        executable="operation",
    ),
    Node(
        package="camera_detection",
        executable="scanner",
    ),
    DeclareLaunchArgument(
        'channel_type',
        default_value=channel_type,
        description='Specifying channel type of lidar'),

    DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar'),

    DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar'),
    
    DeclareLaunchArgument(
        'frame_id',
        default_value=frame_id,
        description='Specifying frame_id of lidar'),

    DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data'),

    DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle_compensate of scan data'),
    DeclareLaunchArgument(
        'scan_mode',
        default_value=scan_mode,
        description='Specifying scan mode of lidar'),
    Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        parameters=[{'channel_type':channel_type,
                        'serial_port': serial_port, 
                        'serial_baudrate': serial_baudrate, 
                        'frame_id': frame_id,
                        'inverted': inverted, 
                        'angle_compensate': angle_compensate}],
        output='screen'),
]

def generate_launch_description():
    return LaunchDescription(nodes)
