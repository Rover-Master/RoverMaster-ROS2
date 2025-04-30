import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Bool
from io import BytesIO
import base64
import matplotlib.pyplot as plt
from json import dumps
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

from .socket import SocketClient

def points_to_base64(points: list[tuple[float, float]]) -> str:
        """Convert list of points to base64 string"""
        arr = np.array(points, dtype=np.float32)
        buf = arr.tobytes()
        return base64.b64encode(buf).decode('ascii')

class Scanner(Node):

    def __init__(self):
        super().__init__("scanner")
        self.socket = SocketClient("/tmp/omni-control.sock")

        # Subscribe to the /scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.process_laser_scan, 10
        )

        # Subscribe to the /scan topic
        self.operation_override_subscription = self.create_subscription(
            Bool, "/rover/operation_override", self.update_override, 10
        )

        # Subscribe to /rover/base/odometry
        self.odometry_subscription = self.create_subscription(
            Odometry, "/rover/base/odometry", self.sendOrientation, 10
        )

        # Translation parameter (20 cm in x-direction)
        self.translation_x = 0.2  # 20 cm in meters

    override: bool = False

    def sendOrientation(self, pose_msg: Odometry):
        x = pose_msg.pose.pose.orientation.x
        y = pose_msg.pose.pose.orientation.y
        z = pose_msg.pose.pose.orientation.z
        w = pose_msg.pose.pose.orientation.w
        # self._logger.info(f"Orientation (x,y,z,w): ({x}, {y}, {z}, {w})")
        # Convert to yaw (radians)
        (roll, pitch, yaw) = quat2euler([x, y, z, w])

        self.socket.send_all(dumps(["rotation", {"x": roll, "y": pitch, "z": yaw}]))

    def update_override(self, override: Bool):
        self.override = override.data

    def process_laser_scan(self, scan_msg: LaserScan):
        # Get the number of laser scan points
        num_points = len(scan_msg.ranges)

        # Calculate the circular shift amount
        shift_amount = num_points // 2  # Halfway shift, e.g., 180 degree rotation

        # Circularly shift ranges and intensities by 180 indices
        ranges_rotated = np.roll(scan_msg.ranges, shift_amount)

        front_ranges = list[float]()

        # fig, ax = plt.subplots(1, 1)
        # ax.set_xlim(-5, 5)
        # ax.set_ylim(-5, 5)
        # ax.axis('equal')

        scan_points: list[tuple[float, float]] = []
        front_points: list[tuple[float, float]] = []

        for i, r in enumerate(ranges_rotated):
            if not np.isfinite(r) or r <= 0:
                continue

            # Calculate the angle for the current point in the rotated array
            angle: float = scan_msg.angle_min + i * scan_msg.angle_increment

            # Convert polar to Cartesian coordinates
            x = r * np.cos(angle) + self.translation_x
            y = r * np.sin(angle)
            scan_points.append((x, y))

            # Filter out structure onboard the robot
            if max(abs(x), abs(y)) < 0.2:
                continue

            # Filter out points in front of the robot
            if abs(y) < 0.1 and x > 0:
                front_ranges.append(x)
                front_points.append((x, y))
    
        # ax.scatter(*zip(*scan_points), s=1, c='black')
        # ax.scatter(*zip(*front_points), s=1, c='red')
        # fig.savefig('scan.jpg')

        # if len(front_ranges) > 0:
        #     self.get_logger().info(f"Object Distance: {min(front_ranges):.2f}m")
        # else:
        #     self.get_logger().info(f"No object detected in front of robot.")

        string_scan_points = points_to_base64(scan_points)
        self.socket.send_all(dumps(["laser", string_scan_points]) + "\n")


def main(args=None):
    rclpy.init(args=args)
    scanner = Scanner()
    try:
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        pass
    scanner.destroy_node()


if __name__ == "__main__":
    main()
