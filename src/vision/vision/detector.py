#!/usr/bin/env python3
import cv2, numpy as np
from .lib.ArUco import aruco, use_dict, detect
from .calibrate import init_undistort
from .lib.estimate import estimate
from .lib.env import video
from .lib.transforms import X, Y, Z, rotate_around
from .lib.utils import radians, degrees


use_dict(aruco.DICT_4X4_100)

# Open camera
ret, frame = video.read()
h, w, _ = frame.shape
mtx, dist, new_mtx = init_undistort((w, h))


# ROS2 Related
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class ArUcoDetector(Node):

    def __init__(self):
        super().__init__("vision")
        self.marker_pos = self.create_publisher(Twist, "vision/marker/pos", 10)
        self.ITER = 4
        # Connect to camera platform
        self.platform_attitude_get = self.create_subscription(
            Twist, "platform/attitude/get", self.update_attitude, 10
        )
        self.platform_attitude_set = self.create_publisher(
            Twist, "platform/attitude/set", 10
        )
        self.platform_speed = self.create_publisher(Float64, "platform/speed/set", 10)
        # Call task in an infinite loop
        self.create_timer(0.01, self.task)

    rz = 0
    ry = 0
    T_base: np.ndarray | None = None
    # Destination point is 1 meter away from the marker
    # in the direction of the marker's surface normal
    p_dst = np.array([[0, 0, 1000, 1]], np.float32).T

    def update_attitude(self, attitude: Twist):
        if self.T_base is None:
            self.get_logger().info("Platform attitude initialized.")
            self.platform_speed.publish(Float64(data=float(10)))
        self.rz = attitude.angular.z
        self.ry = attitude.angular.y
        self.T_base = np.identity(4, np.float64)
        Rz = rotate_around(X, radians(self.rz - 90))
        Ry = rotate_around(Y, radians(self.ry + 90))
        self.T_base[:3, :3] = Rz @ Ry

    prev_flag_found = False

    def task(self, frame: np.ndarray):
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, None, fx=0.5, fy=0.5)
            # Detect ArUco markers
            corners, ids, _ = detect(gray)
            flag_found = False
            if corners is not None and ids is not None:
                for loc, id in zip(corners, ids):
                    if len(id) != 1:
                        print("Invalid ID:", id)
                        continue
                    (id,) = id
                    if id != 0:
                        continue
                    flag_found = True
                    # Propotional control
                    h, w = gray.shape
                    c = np.array([w, h], np.float64) / 2
                    l = np.average(loc.reshape((-1, 2)), 0)
                    offset = l - c
                    # self.get_logger().info(f"Offset: {offset}")
                    # continue
                    kx = 0.08  # deg/px
                    ky = 0.08  # deg/px
                    msg = Twist()
                    msg.angular.y = float(self.ry + offset[Y] * ky)
                    msg.angular.z = float(self.rz - offset[X] * kx)
                    self.platform_attitude_set.publish(msg)
                    continue
                    T = estimate(gray, id, loc, mtx, dist)
                    # Set found to true
                    # Project destination point
                    dst = self.T_base @ T @ self.p_dst
                    dst = dst[:3].reshape((-1))
                    # Publish position of marker destination
                    msg = Twist()
                    msg.linear.x = float(dst[X])
                    msg.linear.y = float(dst[Y])
                    msg.linear.z = float(dst[Z])
                    self.marker_pos.publish(msg)
                    # Move platform to follow the marker
                    target = T[:, 3]
                    ry = degrees(np.arctan2(target[Y], target[Z]))
                    rz = degrees(np.arctan2(target[X], target[Z]))
                    msg = Twist()
                    msg.angular.y = float(self.ry + 2 * ry)
                    msg.angular.z = float(self.rz - rz)
                    self.platform_attitude_set.publish(msg)
            if not flag_found and self.prev_flag_found:
                msg = Twist()
                self.marker_pos.publish(msg)
            self.prev_flag_found = flag_found

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetector()
    while rclpy.ok():
        # Wait until platform attitude is available
        if node.T_base is None:
            return
        ret, frame = video.read()
        if not ret:
            node.get_logger().error("Failed to read frame from camera, aborting")
            node.destroy_node()
        rclpy.spin_once(node)
        try:
            node.task(frame)
        except Exception as e:
            print(e)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
