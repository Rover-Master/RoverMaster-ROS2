#!/usr/bin/env python3
import time, math, cv2, sys, numpy as np
from typing import Callable
from .transforms import X, Y, Z, rotate_around
from math import radians, degrees
from .arm import Arm
from .io import OutFile
from .kinematics import Kinematics

# ROS2 Related
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Detection
from .actions.detection import NavAlignment, Detection, Mapping

initial_mapping = OutFile("/mount/disk/Initial_Mapping_ABE_Gator.txt")
final_mapping = OutFile("/mount/disk/Final_Mapping_ABE_Gator.txt")


def normalize_angle(angle: float) -> float:
    return (angle + 180) % 360 - 180


def sign(x: float) -> int:
    return 1 if float(x) >= 0 else -1


class Position:
    x: float = 0  # X offset in millimeters
    y: float = 0  # Y offset in millimeters
    r: float = 0  # Heading in degrees, zero at north, range (-180, 180)

    def __init__(self, x=0, y=0, r=0):
        self.x = x
        self.y = y
        self.r = r


class Velocity:
    linear: float = 0  # Forward/Backward transitional velocity in mm/s
    angular: float = 0  # Rotational velocity in deg/s

    def __init__(self, linear=0, angular=0):
        self.linear = linear
        self.angular = angular


class Robot(Node):
    # Heading initialization
    initial_heading = -90
    heading_initialized = False
    zero_heading: float = 0
    # Drift compensation
    delta_angle: float = 0
    drift_rate: float = None  # deg/s
    drift_lock: bool = False

    # Multiplier constant for the velocities
    K_VEL_LINEAR = 1280.0  # mm/s
    K_VEL_ANGULAR = 150.0  # deg/s

    # Latest estimated position, in millimeters
    # Origin is at [TODO]
    position = Position()

    # Latest velocity vector in mm/s
    # [vel-fwd/back, rot]
    velocity = Velocity()

    MOTION_FREQ = 100.0  # Hz
    # Planned velocities to be executed in an interval of MOTION_FREQ Hertz
    motion: list[Velocity] = []

    # Robot arm driver
    arm = Arm(0x2341, 0x0070)
    # Arm inverse kinematics
    kinematics = Kinematics(L1=170, L2=180, L3=-70)

    # Detection models
    models = {
        "detection": Detection("pre-trained_weights/detection_harvest.pt"),
        "navigation": NavAlignment("pre-trained_weights/navigation.pt"),
        "mapping": Mapping("pre-trained_weights/mapping.pt"),
    }

    last_updated_angle = None
    last_updated_time: float = None  # Seconds

    def update_attitude(self, attitude: Twist):
        """
        Handles the incoming attitude message, offsets the value according to initial heading.
        Adjusts the range from 0~360 to -180~180.
        """

        def normalize_angle(angle: float):
            return (angle + 180) % 360 - 180

        now = time.time()
        angle = attitude.angular.z
        if self.last_updated_time is not None:
            dt = now - self.last_updated_time
        else:
            dt = 0
        # Sensor drift compensation
        if (
            self.prev_velocity is None
            and self.last_updated_angle is not None
            and not self.drift_lock
        ):
            delta_angle = normalize_angle(angle - self.last_updated_angle)
            drift_rate = delta_angle / dt
            if self.drift_rate is not None:
                self.drift_rate = 0.9 * self.drift_rate + 0.1 * drift_rate
            else:
                self.drift_rate = drift_rate
        elif self.drift_rate is not None:
            delta_angle = self.drift_rate * dt
        else:
            delta_angle = 0
        self.delta_angle = normalize_angle(self.delta_angle + delta_angle)
        self.last_updated_angle = angle
        self.last_updated_time = now
        # Initialize the heading
        if not self.heading_initialized:
            # Record initial heading
            self.zero_heading = normalize_angle(angle - self.initial_heading)
            self.heading_initialized = True
            self.delta_angle = 0
            self.position.r = normalize_angle(angle - self.zero_heading)
        elif self.prev_velocity is not None or self.drift_lock:
            self.position.r = normalize_angle(
                angle - self.zero_heading - self.delta_angle
            )
        # self.get_logger().info(
        #     f"Heading: {self.position.r}, delta_angle: {self.delta_angle}"
        # )

    def update_velocity(self, next_velocity: Twist):
        self.velocity.linear = next_velocity.linear.x * self.K_VEL_LINEAR
        self.velocity.angular = next_velocity.angular.z * self.K_VEL_ANGULAR

    last_odm_update_time = None  # Milliseconds

    def odm_integrator(self):
        """Accumulate transient positional offset to the estimated position"""
        now = time.time()
        if self.last_odm_update_time is not None and self.prev_velocity is not None:
            dt = now - self.last_odm_update_time
            dist = self.prev_velocity.linear * dt
            self.position.x += dist * math.cos(radians(self.position.r))
            self.position.y += dist * math.sin(radians(self.position.r))
        self.last_odm_update_time = now

    def turn_to(
        self,
        heading: float,
        angular_vel: float = 20.0,
        linear_vel: float = 0.0,
        tolerance: float = 2.0,
        timeout: float = 10.0,
    ):
        """aw
        Turn to the desired heading
        """
        dir = sign(normalize_angle(heading - self.position.r))
        for _ in range(int(round(timeout * self.MOTION_FREQ))):
            delta = normalize_angle(heading - self.position.r)
            if abs(delta) < tolerance or sign(delta) != dir:
                break
            self.motion = [Velocity(linear=linear_vel, angular=dir * angular_vel)]
            self.wait(lambda: len(self.motion) == 0)

    def move_to(self, dst: Position, speed: float = 200.0, kr: float = 6.0):
        """
        Calculates a viable strategy to move from current position to the destination
        Both translation and heading need to be satisfied
        """
        dx = dst.x - self.position.x
        dy = dst.y - self.position.y
        distance = sqrt(dx**2 + dy**2)

        heading = degrees(atan2(dy, dx))

        if abs(normalize_angle(heading - self.position.r)) > 90:
            # Back off
            heading = normalize_angle(heading + 180)
            speed = -speed
        # Step 1: turn to desired heading
        self.turn_to(heading)
        # Step 2: move given distance, lock the heading
        for _ in range(int(distance * self.MOTION_FREQ / abs(speed))):
            self.motion = [
                Velocity(
                    linear=speed,
                    angular=kr * normalize_angle(heading - self.position.r),
                )
            ]
            self.wait(lambda: len(self.motion) == 0)
        # Step 3: turn to the final heading
        self.turn_to(dst.r)
        self.wait(lambda: self.prev_velocity is None)
        return

    prev_velocity: Velocity | None = None

    def timed_velocity_update(self):
        self.odm_integrator()
        vel_msg = Twist()
        if len(self.motion):
            velocity = self.motion[0]
            self.motion = self.motion[1:]
            vel_msg.linear.x = float(velocity.linear / self.K_VEL_LINEAR)
            vel_msg.angular.z = float(velocity.angular / self.K_VEL_ANGULAR)
            self.prev_velocity = velocity
        elif self.prev_velocity is not None:
            vel_msg.linear.x = float(0.0)
            vel_msg.angular.z = float(0.0)
            self.prev_velocity = None
        else:
            return
        self.base_velocity.publish(vel_msg)

    def __init__(self):
        super().__init__("mission")
        self.declare_parameter("start_pos", "L")
        # Connect to robot drive train
        self.base_attitude_get = self.create_subscription(
            Twist, "base/velocity/get", self.update_velocity, 10
        )
        self.base_attitude_get = self.create_subscription(
            Twist, "base/imu/att", self.update_attitude, 10
        )
        self.base_velocity = self.create_publisher(Twist, "base/velocity/set", 10)
        # Call task in an infinite loop
        self.create_timer(1.0 / self.MOTION_FREQ, self.timed_velocity_update)
        self.create_timer(0.5, self.report_position)
        # initialize the arm
        self.arm.init()
        self.arm.tick = lambda: rclpy.spin_once(self)
        # Prepare the output file for writing
        line = "Plant Number Healthy Unhealthy Stems Flower"
        initial_mapping.writeln(line)
        final_mapping.writeln(line)
        return

    def report_position(self):
        self.get_logger().info(
            f"POS X={self.position.x} Y={self.position.y} HDG={self.position.r}"
        )

    def wait(self, condition: Callable):
        while rclpy.ok():
            rclpy.spin_once(self)
            try:
                if condition():
                    return
            except Exception as e:
                self.get_logger().error(f"Robot.wait(): error in condition: {e}")
                return

    def delay(self, seconds: float):
        now = time.time()
        self.wait(lambda: time.time() - now > seconds)

    def take_picture(self) -> np.ndarray:
        # TODO check the index
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if not ret:
            self.get_logger.info("Capture ret error " + ret)
        cap.release()
        return frame

    def update_arm_camera_position(
        acc_angle: float, stemp_center: Position
    ) -> Position:
        # TODO test logic
        # assuming home is 0,0 and starting pos of camera
        # assume dis is distance to stemp center on x? plane
        dis = (
            5  # TODO: when robot is center distance should be measurable and a constant
        )
        x = dis * math.cos(radians(acc_angle)) + stemp_center.x
        y = dis * math.sin(radians(acc_angle)) + stemp_center.y
        return Position(x=x, y=y, r=0)
