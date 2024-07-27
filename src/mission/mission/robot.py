#!/usr/bin/env python3
import time, math, cv2, sys, numpy as np
from typing import Callable
from .transforms import X, Y, Z, rotate_around
from .utils import radians, degrees
from .arm import Arm

# ROS2 Related
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

# Detection
# from model import NavAlignment, Detection


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
    initial_heading_offset = -90
    heading_initialized = False
    zero_heading: float = 0
    # Drift compensation
    delta_angle: float = 0
    drift_rate: float = None  # deg/s

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
    models = {
        # Initiate detection
        # "detection": Detection(""),
        # Navigation alignment
        # "navigation": NavAlignment("")
    }

    # logger of mapped leaves
    logger = []

    def write_solution_file(self):
        # Generate Plant Numbers
        plant_numbers = [
            f"Plant {letter}{number}" for letter in "AB" for number in range(1, 13)
        ]

        # Initial file writing
        with open("Initial_Mapping_ABE_Gator", "w") as file:
            # Write header
            file.write("Plant Number Healthy Unhealthy Stems Flower\n")

            for entry in self.logger:
                file.write(
                    f"{plant_numbers[i]} {entry['Healthy']} {entry['Unhealthy']} {entry['Flower']}\n"
                )
            file.close()

        # Final file writing
        with open("Final_Mapping_ABE_Gator", "w") as file:
            # Write header
            file.write("Plant Number Healthy Unhealthy Stems Flower\n")

            for entry, i in zip(self.logger, range(24)):
                flowers = 0
                if entry["Flower"] >= 1:
                    flowers = 1
                    file.write(f"{plant_numbers[i]} {entry['Healthy']} {0} {flowers}\n")
            file.close()

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
        angle = 360.0 - attitude.angular.z
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
        pass
        self.odm_integrator()

    last_odm_update_time = None  # Milliseconds

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
        Calculates a viable strategy to move from current position to the destination
        Both translation and heading need to be satisfied
        """
        velocity = 100.0  # mm/s
        distance = dst.x - self.position.x
        duration = distance / velocity
        self.motion = [Velocity(linear=velocity, angular=0)] * int(
            duration * self.MOTION_FREQ
        )
        return
        # Convert the destination location from world coordinate to robot coordinate
        dx = dst.x - self.position.x
        dy = dst.y - self.position.y
        R1 = rotate_around(Z, radians(self.position.r))[:2, :2]
        dx, dy = (R1 @ np.ndarray([[dx, dy]]).T)[:, 0]
        dr = dst.r - self.position.r

    prev_velocity: Velocity | None = None

    def timed_velocity_update(self):
        vel_msg = Twist()
        if len(self.motion):
            velocity = self.motion[0]
            self.motion = self.motion[1:]
            self.get_logger().info(
                "Setting speed to %f %f" % (velocity.linear, velocity.angular)
            )
            vel_msg.linear.x = velocity.linear / 1000.0
            vel_msg.angular.z = velocity.angular / 360.0
            self.prev_velocity = velocity
        elif self.prev_velocity is not None:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.prev_velocity = None
        else:
            return
        self.base_velocity.publish(vel_msg)
        self.get_logger().info("Setting speed to %f %f" % (vel_msg.linear.x, vel_msg.angular.z))

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
        # initialize the arm
        # self.arm.init()

    def wait(self, condition: Callable):
        while rclpy.ok():
            rclpy.spin_once(self)
            if condition():
                return

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

    def harvest(self, stemp_center):
        #
        self.arm.move(J1=90, J2=0, J3=J3)
        # init camera position as (0,0)
        camera_position = Position(x=0, y=0, r=0)
        # rotate around stemp, do 8 stops
        for J3 in range(0, 361, 45):
            # take picture
            frame = self.take_picture()
            # Determine all bboxes of images in frame (only unhealthy, flowers)
            det_boxes = self.detection.process_image(frame)

            # Update camera position
            camera_position = self.update_arm_camera_position(J3, stemp_center)
            # Get closest target
            target_plant = self.detection.get_closest_target(det_boxes, camera_position)
            time.sleep(1)  # TODO Determine if necessary, and how long

            # Harvest if not None and move j3 if is None
            if target_plant is not None:
                # move to z = min height
                self.arm.move(Z=100)
                # move endeffector to trimming position (further inward)
                self.arm.move(E=10)
                self.arm.move(Z=0)
                self.arm.move(E=180)
                # move endeffector back out (all the way to home of endeffector)
                self.arm.move(E=0)


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    starting_pos = robot.get_parameter("starting_pos").value
    if starting_pos == "L":
        robot.initial_heading_offset = -90
    else:
        robot.initial_heading_offset = 90
    # Init the robot
    robot.get_logger().info("Waiting for heading to be initialized ...")
    robot.wait(lambda: robot.heading_initialized)
    # Wait 5 seconds for precise measurement of sensor drift
    robot.get_logger().info("Measuring sensor drift ...")
    now = time.time()
    robot.wait(lambda: time.time() - now > 5.0)
    # First: Move & Turn to estimated initial positionift
    robot.get_logger().info(f"Moving out of parking slot <{starting_pos}> ...")
    turn = 90 if starting_pos == "L" else -90
    robot.motion = [Velocity(linear=100, angular=turn / 5.0)] * 10000
    robot.get_logger().info("Length of motion: %d" % len(robot.motion))
    robot.wait(lambda: robot.position.r >= 0 or len(robot.motion) == 0)
    robot.get_logger().info("Move completed")
    robot.wait(lambda: False)
    return
    # ROW A harvesting: Move to each stop points
    for x in [100, 120, 140, 160]:  # TODO unit and measurement
        robot.move_to(Position(x=x, y=0, r=0))
        robot.wait(lambda: len(robot.motion) == 0)

        # Detect right side stem/find center
        frame = robot.take_picture()
        nav_box = robot.navigation.process_image(frame)
        if nav_box is not None:
            x_box_nav, y_box_nav, _, _ = nav_box # Pixel unit
            #TODO calculate the 3D position
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
