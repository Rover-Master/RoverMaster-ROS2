#!/usr/bin/env python3
import time, math, cv2, sys, numpy as np
from typing import Callable
from .transforms import X, Y, Z, rotate_around
from math import radians, degrees, atan2, sqrt
from .arm import Arm
from .io import OutFile
from .kinematics import Kinematics

# ROS2 Related
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Detection
from .actions.detection import NavAlignment, Detection, Mapping


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
        tolerance: float = 0.6,
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
    start_pos = robot.get_parameter("start_pos").value
    if start_pos == "L":
        robot.initial_heading = -90.0
    else:
        robot.initial_heading = 90.0
    # Init the robot
    robot.get_logger().info("Waiting for heading to be initialized ...")
    robot.wait(lambda: robot.heading_initialized)
    # Wait 5 seconds for precise measurement of sensor drift
    robot.get_logger().info("Measuring sensor drift ...")
    robot.delay(1)
    robot.drift_lock = True
    # First: Move & Turn to initial position
    robot.get_logger().info(f"Moving out of parking position <{start_pos}> ...")
    now = time.time()
    robot.turn_to(0, angular_vel=90 / 1.85, linear_vel=450, tolerance=3.0)
    robot.get_logger().info(f"Turn completed in {time.time() - now} seconds ...")

    # Second: Move to the starting point
    robot.position.x = -180
    if start_pos == "L":
        robot.position.y = -0
    else:
        robot.position.y = 0
    robot.move_to(Position(x=0, y=0, r=0), speed=200)
    robot.wait(lambda: len(robot.motion) == 0)

    robot.delay(1)
    for row in range(1, 13):
        robot.move_to(Position(x=robot.position.x + 304.8, y=0, r=0))
        y_offset = 90  # millimeters
        robot.arm.speed()
        robot.arm.move(Z=0)
        # Sweep left side to locate the plant
        model_nav: NavAlignment = robot.models["navigation"]

        def find_plant(x_range: list[float], y_offset: float, camera_angle: float):
            detections = []
            for x in x_range:
                solutions = robot.kinematics.backward(x, y_offset, camera_angle)
                angles, speeds, _ = robot.arm.plan(solutions)
                robot.arm.speed(**speeds)
                robot.arm.move(**angles)
                # Camera is now at (x, y_offset, 90 deg)
                R = rotate_around(Y, 30) @ rotate_around(Z, camera_angle)
                t = np.array([[x, y_offset, 220]]).T
                image = robot.take_picture()
                result = model_nav.process_image(image)
                if result is not None:
                    detections.append([result, R, t])
                if len(detections) >= 2:
                    break
            if len(detections) >= 2:
                (d1, R1, t1), (d2, R2, t2) = detections[:2]
                # Get R and t from camera 1 to camera 2
                t = R1.T @ (t2 - t1)
                R = R2 @ R1.T
                # Get the position of the plant relative to camera 1
                relative_position = model_nav.point_reconstruction(d1, d2, R, t)
                # Get the position of the plant relative to the robot
                return list((R1 @ relative_position + t1).reshape(-1, 1))
            else:
                return None

        offset_detection = 90

        position_left_plant = find_plant(range(-100, 101, 10), y_offset, 90)
        if position_left_plant is None:
            robot.get_logger().info(
                f"Left side plant not found, moving to next position ..."
            )
        else:
            cx, cy, _ = position_left_plant
            mapping = None
            for angle in range(0, 361, 45):
                dx = offset_detection * math.cos(radians(angle))
                dy = offset_detection * math.sin(radians(angle))
                solutions = robot.kinematics.backward(cx + dx, cy + dy, angle)
                angles, speeds, _ = robot.arm.plan(solutions)
                if angles is not None and speeds is not None:
                    robot.arm.speed(**speeds)
                    robot.arm.move(**angles)
                    image = robot.take_picture()
                    if mapping is None:
                        mapping = robot.models["mapping"].mapping(image)
                        robot.initial_mapping.write(f"Plant {row}L {mapping}")
                    boxes = robot.models["detection"].process_image(image)
                    if boxes is not None:
                        box_interest = robot.models["detection"].det_target(boxes, Position(cx, cy, angle))s
                        

        position_right_plant = find_plant(range(-100, 101, 10), -y_offset, -90)

    robot.get_logger().info(f"Harvesting completed, backing off")
    robot.move_to(Position(x=0, y=robot.position.y, r=0), speed=300)
    robot.move_to(Position(x=-500, y=robot.position.y, r=0), speed=400)
    robot.wait(lambda: len(robot.motion) == 0)

    # Turn to parking spot
    robot.get_logger().info(f"Parking into end position ...")
    now = time.time()
    robot.turn_to(
        -robot.initial_heading, angular_vel=90 / 2.2, linear_vel=-400, tolerance=3.0
    )
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
