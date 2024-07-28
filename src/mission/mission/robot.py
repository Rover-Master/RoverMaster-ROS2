#!/usr/bin/env python3
import time, math, cv2, sys, numpy as np
from typing import Callable
from .transforms import X, Y, Z, rotate_around
from math import radians, degrees, atan2, sqrt
from .arm import Arm
from .io import OutFile
from .kinematics import Kinematics


def normalize_angle(angle: float) -> float:
    return (angle + 180) % 360 - 180


def sign(x: float) -> int:
    return 1 if float(x) >= 0 else -1


# ROS2 Related
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Detection
from .actions.detection import NavAlignment, Detection


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
    initial_heading_offset = -90
    heading_initialized = False
    zero_heading: float = 0
    # Drift compensation
    delta_angle: float = 0
    drift_rate: float = None  # deg/s

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
    # arm = Arm(0x2341, 0x0070)
    # Arm inverse kinematics
    kinematics = Kinematics(L1=170, L2=180, L3=-70)

    # Detection models
    models = {
        # Initiate detection
        "detection": Detection(""),
        # Navigation alignment
        "navigation": NavAlignment(""),
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
        if self.prev_velocity is None and self.last_updated_angle is not None:
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
            self.zero_heading = normalize_angle(angle + self.initial_heading_offset)
            self.heading_initialized = True
            self.delta_angle = 0
            self.position.r = normalize_angle(angle - self.zero_heading)
        elif self.prev_velocity is not None:
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
        angular_vel: float = 45.0,
        tolerance: float = 1.0,
        timeout: float = 10.0,
    ):
        """
        Turn to the desired heading
        """
        for _ in range(int(round(timeout * self.MOTION_FREQ))):
            delta = normalize_angle(heading - self.position.r)
            if abs(delta) < tolerance:
                break
            self.motion = [Velocity(linear=0, angular=sign(delta) * angular_vel)]
            self.wait(lambda: len(self.motion) == 0)

    def move_to(self, dst: Position, speed: float = 200.0, kr: float = 2.0):
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
        for _ in range(int(distance / abs(speed))):
            self.motion = [
                Velocity(
                    linear=speed,
                    angular=kr * normalize_angle(heading - self.position.r),
                )
            ]
            self.wait(lambda: len(self.motion) == 0)
        # Step 3: turn to the final heading
        self.turn_to(dst.r)
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
        self.declare_parameter("nav_model_path", "")
        self.declare_parameter("seg_model_path", "")
        self.declare_parameter("map_model_path", "")
        self.declare_parameter("starting_pos", "L")
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
        # self.arm.init()
        return
        # Prepare the output file for writing
        self.initial_mapping = OutFile("/mount/disk/Initial_Mapping_ABE_Gator.txt")
        self.final_mapping = OutFile("/mount/disk/Final_Mapping_ABE_Gator.txt")
        line = "Plant Number Healthy Unhealthy Stems Flower\n"
        self.initial_mapping.write(line)
        self.final_mapping.write(line)

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


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    starting_pos = robot.get_parameter("starting_pos").value
    starting_pos = "R"
    if starting_pos == "L":
        robot.initial_heading_offset = 90.0
    else:
        robot.initial_heading_offset = -90.0
    # Init the robot
    robot.get_logger().info("Waiting for heading to be initialized ...")
    robot.wait(lambda: robot.heading_initialized)
    # Wait 5 seconds for precise measurement of sensor drift
    robot.get_logger().info("Measuring sensor drift ...")
    robot.delay(1)
    # First: Move & Turn to initial position
    robot.get_logger().info(f"Moving out of parking position <{starting_pos}> ...")
    now = time.time()
    turn = robot.initial_heading_offset
    robot.motion = list(
        [Velocity(linear=float(v), angular=0) for v in range(100, 300, 10)]
    )
    robot.motion += [Velocity(linear=450, angular=turn / 1.85)] * 1000
    threshold = -turn / 30
    if turn > 0:
        robot.wait(lambda: robot.position.r >= threshold or len(robot.motion) == 0)
    else:
        robot.wait(lambda: robot.position.r <= threshold or len(robot.motion) == 0)
    robot.motion = []
    robot.get_logger().info(f"Turn completed in {time.time() - now} seconds ...")

    # Second: Move to the starting point
    robot.move_to(
        Position(x=robot.position.x + 120, y=robot.position.y, r=0), speed=300
    )
    robot.wait(lambda: len(robot.motion) == 0)
    robot.move_to(Position(x=robot.position.x + 120, y=robot.position.y, r=0))
    robot.wait(lambda: len(robot.motion) == 0)
    robot.position.x = 0
    robot.position.y = 0

    robot.delay(1)
    for _ in [None] * 12:
        robot.move_to(Position(x=robot.position.x + 304.8, y=robot.position.y, r=0))
        robot.wait(lambda: len(robot.motion) == 0)
        robot.delay(1)

    robot.get_logger().info(f"Harvesting completed, backing off")
    robot.move_to(Position(x=-280, y=robot.position.y, r=0))
    robot.wait(lambda: len(robot.motion) == 0)

    # Turn to parking spot
    now = time.time()
    robot.motion = [Velocity(linear=-320, angular=turn / 2.8)] * 1000
    threshold = turn + threshold
    if turn > 0:
        robot.wait(lambda: robot.position.r >= threshold or len(robot.motion) == 0)
    else:
        robot.wait(lambda: robot.position.r <= threshold or len(robot.motion) == 0)
    robot.motion = []
    robot.get_logger().info(f"Turn completed in {time.time() - now} seconds ...")

    # Mission completed, wait for exit signal
    robot.wait(lambda: False)
    return
    # ROW A harvesting: Move to each stop points
    for x in [100, 120, 140, 160]:  # TODO unit and measurement
        robot.move_to(Position(x=x, y=0, r=0))
        robot.wait(lambda: len(robot.motion) == 0)

        # Detect right side stem/find center
        frame_first = robot.take_picture()
        nav_box = robot.navigation.process_image(frame_first)
        if nav_box is not None:
            # TODO calculate the 3D position
            robot.arm.move(J3=45)
            frame_sec = robot.take_picture()
            good_matches, kp1, kp2 = robot.navigation.detect_and_match_features(
                frame_first, frame_sec
            )

            # Visualization confirmation
            # img_matches = cv2.drawMatches(robot.navigation.prepare_roi(frame_first, robot.navigation.process_image(frame_first)), kp1, robot.navigation.prepare_roi(frame_second, robot.navigation.process_image(frame_second)), kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            # cv2.imshow("Matches", img_matches)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            offset_left: Position = robot.detect(Position(x=x, y=-120, r=0))

        # Harvest
        robot.harvest(Position(x=x_box_nav, y=y_box_nav, r=0))

        # Mapping
        # TODO Make an association between specific plant and its corresponding mapping please!
        robot.navigation.mapping(frame)
        # Same for right side
        offset_right: Position = robot.detect(Position(x=x, y=120, r=0))
        # Correct robot location
        robot.position = (offset_left + offset_right) / 2

    # Rotate 180 degrees

    # ROW B harvesting: Move to each stop points
    for x in [160, 140, 120, 100]:  # TODO unit and measurement
        # TODO: when done ROW A copy behavior/encapsulate in method and call here
        pass

    # Return to the stop location
    # TODO

    # write file and move to usb
    robot.write_solution_file()
    # TODO move file to USB stick


if __name__ == "__main__":
    main()
