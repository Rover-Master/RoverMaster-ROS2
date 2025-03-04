import cv2
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException  # Fix: Import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from time import strftime, time as now
import subprocess  # For running FFmpeg as a subprocess

class CameraDetection(Node):
    def __init__(self):
        super().__init__('camera_detection_node')
        self.br = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'img',
            self.listenerCallback,
            10
        )
        self.declare_parameter("FPS", 10.0)
        self.declare_parameter("address", "10.138.231.149")
        self.declare_parameter("port", "1234")
        # Start FFmpeg process for streaming
        self.ffmpeg_process = self.start_ffmpeg_stream()
        self.create_timer(0.1, self.timerCallback)

    @property
    def fps(self) -> float:
        return self.get_parameter('FPS').get_parameter_value().double_value

    @property
    def address(self) -> str:
        return self.get_parameter('address').get_parameter_value().string_value

    @property
    def port(self) -> str:
        return self.get_parameter('port').get_parameter_value().string_value

    def start_ffmpeg_stream(self):
        # FFmpeg command to stream frames over HTTP
        ffmpeg_command = [
            'ffmpeg',
            '-y',  # Overwrite output files
            '-f', 'rawvideo',  # Input format
            '-pix_fmt', 'bgr24',  # Pixel format
            '-s', '720x540',  # Frame size
            '-r', str(self.fps),  # Reduce frame rate to 10 FPS for smoother streaming
            '-i', '-',  # Read input from stdin
            '-c:v', 'libx264',  # Use H.264 for better compression
            '-preset', 'ultrafast',  # Fast encoding
            '-tune', 'zerolatency',  # Reduce latency for real-time streaming
            '-b:v', '1M',  # Set constant bitrate to 700 kbps
            # '-minrate', '2M',
            # '-maxrate', '2M',
            '-bufsize', '4M',  # Increase buffer size to prevent underflow
            '-f', 'mpegts',  # Output format
            f'udp://{self.address}:{self.port}'  # Stream to your laptop's IP address
        ]
        return subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)

    flag_send = False
    def timerCallback(self):
        self.flag_send = True

    def listenerCallback(self, msg):
        if self.flag_send:
            # Convert the ROS image message to a CV2 image
            self.flag_send = False
            self.sendFrame(self.br.imgmsg_to_cv2(msg))

    def sendFrame(self, frame):
        # Write the annotated frame to FFmpeg's stdin
        try:
            out_frame = cv2.resize(frame, (720, 540))
            self.ffmpeg_process.stdin.write(out_frame.tobytes())
        except BrokenPipeError:
            self.get_logger().error("FFmpeg process crashed. Restarting...")
            self.ffmpeg_process = self.start_ffmpeg_stream()  # Restart FFmpeg

def main():
    global node
    # Initialize ROS 2
    rclpy.init()
    node = CameraDetection()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Cleanup
        node.ffmpeg_process.stdin.close()
        node.ffmpeg_process.wait()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()