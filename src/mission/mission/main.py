#!/usr/bin/env python3
import time, math, numpy as np, rclpy
from .robot import Robot, Position, Velocity, initial_mapping, final_mapping
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
robot.position.x = -180
if start_pos == "L":
    robot.position.y = -0
else:
    robot.position.y = 0
# robot.move_to(Position(x=0, y=0, r=0), speed=200)

robot.delay(1)
for row in range(1, 13):
    try:
        robot.arm.send("MOVE E=0\n")
        # robot.move_to(Position(x=robot.position.x + 304.8, y=0, r=0))
        y_offset = 90  # millimeters
        robot.arm.speed()
        robot.arm.move(Z=0)
        # Sweep left side to locate the plant
        model_nav = robot.models["navigation"]

        def find_plant(x_range: list[float], y_offset: float, camera_angle: float):
            detections = []
            for x in x_range:
                solutions = robot.kinematics.backward(x, y_offset, camera_angle)
                if len(solutions) == 0:
                    continue
                # angles, speeds, _ = robot.arm.plan(solutions)
                # if angles is None or speeds is None:
                #     continue
                # robot.arm.speed(**speeds)

                robot.get_logger().info(f"S1={solutions[0]}, S2={solutions[1]}")
                robot.arm.move(
                    J1=solutions[1][0], J2=solutions[1][1], J3=solutions[1][2]
                )
                robot.arm.send("MOVE E=180\n")
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

        def format_report(mapping: dict[int, int]):
            return f", {mapping[1]}, {mapping[3]}, {mapping[0]}"

        plant = f"Plant B{row}"
        robot.arm.send("MOVE E=0\n")
        position_left_plant = find_plant(range(-150, 151, 30), y_offset, 90)
        if position_left_plant is None:
            robot.get_logger().info(
                f"Left side plant not found, moving to next position ..."
            )
            initial_mapping.writeln(f"{plant} MISSING")
        else:
            cx, cy, _ = position_left_plant
            mapping = None
            for angle in range(0, 361, 45):
                dx = offset_detection * math.cos(radians(angle + 90))
                dy = offset_detection * math.sin(radians(angle + 90))
                solutions = robot.kinematics.backward(cx + dx, cy + dy, angle)
                angles, speeds, _ = robot.arm.plan(solutions)
                if angles is not None and speeds is not None:
                    robot.arm.speed(**speeds)
                    robot.arm.move(**angles)
                    image = robot.take_picture()
                    print("image shape:", image.shape)
                    if mapping is None:
                        mapping = robot.models["mapping"].mapping(image)
                        initial_mapping.writeln(f"{plant} {format_report(mapping)}")
                        final_mapping.writeln(f"{plant} {format_report(mapping)}")
                    break
                    boxes = robot.models["detection"].process_image(image)
                    if boxes is not None:
                        box_interest = robot.models["detection"].det_target(
                            boxes, Position(cx, cy, angle)
                        )
        robot.arm.send("MOVE E=0\n")
        position_right_plant = find_plant(range(-150, 151, 30), -y_offset, -90)
        plant = f"Plant A{row}"
        if position_right_plant is None:
            robot.get_logger().info(
                f"Right side plant not found, moving to next position ..."
            )
            initial_mapping.writeln(f"{plant} MISSING")
        else:
            cx, cy, _ = position_right_plant
            mapping = None
            for angle in range(0, 361, 45):
                dx = offset_detection * math.cos(radians(angle - 90))
                dy = offset_detection * math.sin(radians(angle - 90))
                solutions = robot.kinematics.backward(cx + dx, cy + dy, angle)
                angles, speeds, _ = robot.arm.plan(solutions)
                if angles is not None and speeds is not None:
                    robot.arm.speed(**speeds)
                    robot.arm.move(**angles)
                    image = robot.take_picture()
                    if mapping is None:
                        mapping = robot.models["mapping"].mapping(image)
                        initial_mapping.writeln(f"{plant} {format_report(mapping)}")
                        final_mapping.writeln(f"{plant} {format_report(mapping)}")
                    break
                    boxes = robot.models["detection"].process_image(image)
                    if boxes is not None:
                        box_interest = robot.models["detection"].det_target(
                            boxes, Position(cx, cy, angle)
                        )
    except Exception as e:
        robot.get_logger().info(f"{e}")
        continue
# robot.move_to(Position(x=robot.position.x + 500, y=0, r=0))
robot.get_logger().info(f"Harvesting completed, backing off")
# robot.move_to(Position(x=0, y=robot.position.y, r=0), speed=300)
# robot.move_to(Position(x=-500, y=robot.position.y, r=0), speed=400)

# Turn to parking spot
robot.get_logger().info(f"Parking into end position ...")
now = time.time()
robot.turn_to(
    -robot.initial_heading, angular_vel=90 / 2.2, linear_vel=-400, tolerance=3.0
)
robot.get_logger().info(f"Turn completed in {time.time() - now} seconds ...")

# Mission completed, wait for exit signal
robot.wait(lambda: False)
