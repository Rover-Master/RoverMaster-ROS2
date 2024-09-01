#!/usr/bin/env python3
import time, math, numpy as np, rclpy
from .robot import (
    Robot,
    Position,
    Velocity,
    initial_mapping,
    final_mapping,
    normalize_angle,
)
from .transforms import X, Y, Z, rotate_around
from math import radians, degrees, atan2, sqrt

rclpy.init()
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
# robot.turn_to(0, angular_vel=90 / 1.85, linear_vel=450, tolerance=3.0)
robot.get_logger().info(f"Turn completed in {time.time() - now} seconds ...")

# Second: Move to the starting point
robot.position.x = -0
if start_pos == "L":
    robot.position.y = -0
else:
    robot.position.y = 0
# robot.move_to(Position(x=0, y=0, r=0), speed=200)


def move_arm(x, y, r):
    solutions = robot.kinematics.backward(x, y, r)
    angles, speeds, _ = robot.arm.plan(solutions)
    if angles is not None and speeds is not None:
        robot.arm.speed(**speeds)
        robot.arm.move(**angles)
        return True
    return False


def find_plant(x_range: list[float], y_offset: float, camera_angle: float):
    try:
        detections = []
        for x in x_range:
            if not move_arm(x, y_offset, camera_angle):
                continue
            robot.arm.send("MOVE E=180\n")
            robot.delay(0.2)
            # Camera is now at (x, y_offset, 90 deg)
            R = rotate_around(Y, 30) @ rotate_around(Z, camera_angle)
            t = np.array([[x, y_offset, 220]]).T
            image = robot.take_picture()
            result = model_nav.process_image(image, print=robot.get_logger().info)
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
            return list((R1 @ relative_position + t1).reshape(-1, 1))[:2]
    except Exception as e:
        robot.get_logger().warn(f"Error: {e}")
    return None


def process_plant(center, mapping, offset=90):
    # Knock off unhealthy leaves and extra flowers
    # 0:Flower, 1:Healthy, 2:Stem_Top, 3:Unhealthy
    for angle in range(0, 361, 45):
        try:
            dx = offset * math.cos(radians(angle + 90))
            dy = offset * math.sin(radians(angle + 90))
            if move_arm(center.x + dx, center.y + dy, angle):
                robot.delay(0.2)
                image = robot.take_picture()
                label = robot.models["detection"].label_closest(image)
                if label is not None and (
                    label == 3 or (label == 0 and mapping[0] > 1)
                ):
                    robot.arm.move(z=100)
                    robot.send("MOVE E=100\n")
                    robot.arm.move(z=0)
                    robot.send("MOVE E=0\n")
                    robot.delay(1)
                    robot.send("MOVE E=180\n")
                    mapping[label] -= 1
        except Exception as e:
            robot.get_logger().warn(f"Error: {e}")


model_nav = robot.models["navigation"]
robot.delay(1)


def format_report(mapping: dict[int, int]):
    return f"{mapping[1]}, {mapping[3]}, {mapping[0]}"


for row in range(1, 13):
    try:
        robot.arm.send("MOVE E=0\n")
        # Center the arm before driving
        robot.arm.move(J1=30, J2=135, J3=0)
        robot.move_to(Position(x=robot.position.x + 354.8, y=0, r=0))
        y_offset = 90  # millimeters
        robot.arm.speed()
        robot.arm.move(Z=0)

        # Process right side plant
        plant = f"Plant A{row}"
        robot.get_logger().info(f"Processing {plant}...")
        robot.arm.send("MOVE E=0\n")
        position_right_plant = find_plant(range(-150, 151, 30), -y_offset, -90)
        if position_right_plant is None:
            position_right_plant = [0, -300]
        cx, cy = position_right_plant
        if move_arm(cx, cy - 90, -90):
            image = robot.take_picture()
            mapping = robot.models["mapping"].mapping(
                image, print=robot.get_logger().info
            )
            initial_mapping.writeln(f"{plant}, {format_report(mapping)}")
            process_plant(Position(cx, cy, 0), mapping)
            final_mapping.writeln(f"{plant} {format_report(mapping)}")
        else:
            initial_mapping.writeln(f"{plant} FAILED")
            final_mapping.writeln(f"{plant} FAILED")

        # Process left side plant
        plant = f"Plant B{row}"
        robot.get_logger().info(f"Processing {plant}...")
        robot.arm.send("MOVE E=0\n")
        position_left_plant = find_plant(range(-150, 151, 30), y_offset, 90)
        if position_left_plant is None:
            position_right_plant = [0, 300]
        cx, cy = position_left_plant
        if move_arm(cx, cy + 90, 90):
            image = robot.take_picture()
            mapping = robot.models["mapping"].mapping(
                image, print=robot.get_logger().info
            )
            initial_mapping.writeln(f"{plant}, {format_report(mapping)}")
            process_plant(Position(cx, cy, 0), mapping)
            final_mapping.writeln(f"{plant} {format_report(mapping)}")
        else:
            initial_mapping.writeln(f"{plant} FAILED")
            final_mapping.writeln(f"{plant} FAILED")
        continue
        if position_left_plant is not None and position_right_plant is not None:
            center = (position_left_plant + position_right_plant) / 2
            robot.position.x -= center[0]
            robot.position.y -= center[1]
            vec = position_left_plant - position_right_plant
            true_zero_heading = normalize_angle(degrees(atan2(vec[1], vec[0])) - 90)
            robot.delta_angle -= true_zero_heading - robot.position.r
    except Exception as e:
        robot.get_logger().info(f"{e}")
        raise e

robot.arm.move(J1=30, J2=135, J3=0)
robot.move_to(Position(x=robot.position.x + 500, y=0, r=0))
robot.move_to(Position(x=0, y=robot.position.y, r=0), speed=300)
robot.move_to(Position(x=-600, y=robot.position.y, r=0), speed=400)

# Turn to parking spot
robot.get_logger().info(f"Parking into end position ...")
now = time.time()
robot.turn_to(
    -robot.initial_heading, angular_vel=90 / 2.4, linear_vel=-400, tolerance=3.0
)
robot.get_logger().info(f"Turn completed in {time.time() - now} seconds ...")

# Mission completed, wait for exit signal
robot.wait(lambda: False)
