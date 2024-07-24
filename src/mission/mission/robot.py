#!/usr/bin/env python3
import time, math, cv2, numpy as np
from .transforms import X, Y, Z, rotate_around
from .utils import radians, degrees


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


class Position:
    x: float = 0 # X offset in millimeters
    y: float = 0 # Y offset in millimeters
    r: float = 0 # Heading in degrees, zero at north, range (-180, 180)


class Velocity:
    linear: float = 0 # Forward/Backward transitional velocity in mm/s
    angular: float = 0 # Rotational velocity in deg/s

    def __init__(self, linear = 0, angular = 0):
        self.linear = linear
        self.angular = angular


class Robot(Node):
    heading_initialized = False
    initial_heading: float = 0

    # Latest estimated position, in millimeters
    # Origin is at [TODO]
    position = Position()

    # Latest velocity vector in mm/s
    # [vel-fwd/back, rot]
    velocity = Velocity()

    # Planned velocities to be executed in an interval of 100Hz
    motion: List[Velocity] = []

    def update_attitude(self, next_attitude: Twist):
        """
        Handles the incoming attitude message, offsets the value according to initial heading.
        Adjusts the range from 0~360 to -180~180.
        """
        if (self.heading_initialized):
            # TODO Normal logic
            pass
        else:
            # Record initial heading
            # TODO Initial heading is 90 deg right of 0 degrees
            pass
    
    def update_velocity(self, next_velocity: Twist):
        pass
        self.odm_integrator()

    last_odm_update_time = None # Milliseconds
    def odm_integrator(self):
        """Accumulate transient positional offset to the estimated position"""
        now = time.time()
        if self.last_odm_update_time is not None:
            dt = now - self.last_odm_update_time
            dist = self.velocity.linear * dt / 1000.0
            self.position.x += dist * math.cos(radians(self.position.r))
            self.position.y += dist * math.sin(radians(self.position.r))
        self.last_odm_update_time = now


    def move_to(self, dst: Position):
        """
        Calcuates a viable strategy to move from current position to the destination
        Both translation and heading need to be satisfied
        """
        # Convert the destination location from world coordinate to robot coordinate
        dx = dst.x - self.position.x
        dy = dst.y - self.position.y
        R1 = rotate_around(Z, radians(self.position.r))[:2, :2]
        dx, dy = (R1 @ np.ndarray([[dx, dy]]).T)[:, 0]
        dr = dst.r - self.position.r


    def timed_velocity_update(self):
        if len(self.motion):
            next_velocity, self.motion = self.motion[0], self.motion[1:]
        else:
            self.base_velocity.publish(Twist())
            

    def __init__(self):
        super().__init__("mission")
        self.marker_pos = self.create_publisher(Twist, "mission/marker/pos", 10)
        self.ITER = 4
        # Connect to robot drive train
        self.base_attitude_get = self.create_subscription(
            Twist, "base/velocity/get", self.update_velocity, 10
        )
        self.base_attitude_get = self.create_subscription(
            Twist, "base/attitude/get", self.update_attitude, 10
        )
        self.base_velocity = self.create_publisher(Twist, "base/velocity/set", 10)
        # Call task in an infinite loop
        self.create_timer(0.01, self.timed_velocity_update)

    def wait(self, condition: Callable):
        while rclpy.ok():
            rclpy.spin_once(self)
            if condition():
                return

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    robot.wait(lambda _: robot.heading_initialized)
    # Initialize the arm
    # First: Move & Turn to estimated initial position
    robot.motion = [Velocity(linear = 1.0, angular = 0.5)]
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
