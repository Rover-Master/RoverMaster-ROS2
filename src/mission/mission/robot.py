#!/usr/bin/env python3
import time, math, cv2, sys, numpy as np
from typing import Callable
from .transforms import X, Y, Z, rotate_around
from .utils import radians, degrees

# ROS2 Related
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

# Detection
from model import NavAlignment, Detection
from control import init, move
import subprocess


class Position:
    x: float = 0 # X offset in millimeters
    y: float = 0 # Y offset in millimeters
    r: float = 0 # Heading in degrees, zero at north, range (-180, 180)

    def __init__(self, x = 0, y = 0, r = 0):
        self.x = x
        self.y = y
        self.r = r
    


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
    motion: list[Velocity] = []
    
    # Initiate detection
    detection = Detection("")
    
    
    

    def update_attitude(self, next_attitude: Twist):
        """
        Handles the incoming attitude message, offsets the value according to initial heading.
        Adjusts the range from 0~360 to -180~180.
        """
        if self.heading_initialized:
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
        Calculates a viable strategy to move from current position to the destination
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
            linear = Vector3(X=next_velocity.linear / 1000.0)
            angular = Vector3(Z=next_velocity.angular / 360.0)
        else:
            linear = Vector3(X=0)
            angular = Vector3(Z=0)
        self.base_velocity.publish(Twist(linear=linear, angular=angular))


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
    
    def collection(self):
        # Initialize camera
        #TODO check the index
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        #TODO Do I need to save all images for mapping?
        cap.release()
        return frame

    
    def harvest(self):
        # TODO Home position of the camera
        camera_position = self.home()
    
        for J3 in range(0, 361, 45):
            move(J1=90, J2=-2, J3=J3)
            time.sleep(1) #TODO Config
            frame = self.collection()
            
            # Do target detection
            det_boxes = self.detection.process_image(frame)
            # Get the closest plant to harverst
            target_plant = self.detection.det_target(det_boxes, camera_position)
            
            # Harvest if not None and move j3 if is None
            if target_plant is not None:
                # Harvest
                pass
            else:
                continue
        
        
        
        
    
    

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    robot.wait(lambda _: robot.heading_initialized)
    # Initialize the arm
    
    
    
    # Navigation alignment
    navigation = NavAlignment("")
    
    
    
    # First: Move & Turn to estimated initial position
    robot.motion = [Velocity(linear = 100, angular = 90 / 5.0)] * 10000
    robot.wait(lambda _: robot.position.r >= 0 or len(robot.motion) == 0)
    # Move to each stop points
    for x in [100, 120, 140, 160]: #TODO unit and measurement
        robot.move_to(Position(x=x, y=0, r=0))
        robot.wait(lambda _: len(robot.motion) == 0)
        
        
        # Detect right side stem
        frame = robot.collection()
        nav_box = navigation.process_image(frame)
        x_box_nav, y_box_nav, w_box_nav, h_box_nav = nav_box
        offset_left: Position = robot.detect(Position(x=x, y=-120, r=0))
        
        # Harvest
        robot.harvest()
        
        # Mapping
        
        # Same for right side
        offset_right: Position = robot.detect(Position(x=x, y=120, r=0))
        # Correct robot location
        robot.position = (offset_left + offset_right) / 2
    # Return to the stop location
    # TODO

if __name__ == "__main__":
    main()
