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
from control import init, move, is_moving
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


    # Navigation alignment
    navigation = NavAlignment("")
    
    #logger of mapped leaves
    logger = []
    
    
    def write_solution_file(self):
        # Generate Plant Numbers
        plant_numbers = [f"Plant {letter}{number}" for letter in 'AB' for number in range(1, 13)]

        # Initial file writing
        with open("Initial_Mapping_ABE_Gator", 'w') as file:
            # Write header
            file.write("Plant Number Healthy Unhealthy Stems Flower\n")
            
            for entry in self.logger:
                    file.write(f"{plant_numbers[i]} {entry['Healthy']} {entry['Unhealthy']} {entry['Flower']}\n")
            file.close()
            
        
        # Final file writing
        with open("Final_Mapping_ABE_Gator", 'w') as file:
            # Write header
            file.write("Plant Number Healthy Unhealthy Stems Flower\n")
            
            for entry,i in zip(self.logger,range(24)):
                flowers = 0
                if entry["Flower"] >= 1:
                    flowers = 1
                    file.write(f"{plant_numbers[i]} {entry['Healthy']} {0} {flowers}\n")
            file.close()
            

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

    def update_arm_camera_position(acc_angle:float, stemp_center:Position) -> Position:
    #TODO test logic
    #assuming home is 0,0 and starting pos of camera
    #assume dis is distance to stemp center on x? plane
        dis = 5 #TODO: when robot is center distance should be measurable and a constant
        x = dis*math.cos(radians(acc_angle)) + stemp_center.x
        y = dis*math.sin(radians(acc_angle)) + stemp_center.y
        return Position(x=x, y=y, r=0)
    
    
    def harvest(self, stemp_center):
        #drive to HOME
        move(J1=90, J2=-2, J3=J3) 
        #init camera position as (0,0)
        camera_position = Position(x=0, y=0,r=0)
        
        #rotate around stemp, do 8 stops
        for J3 in range(0, 361, 45):
            #take picture
            frame = self.collection() 
            # Determine all bboxes of images in frame (only unhealthy, flowers)
            det_boxes = self.detection.process_image(frame)
            
            # Update camera position
            camera_position = self.update_arm_camera_position(J3, stemp_center)
            # Get closest target
            target_plant = self.detection.get_closest_target(det_boxes, camera_position)
            time.sleep(1) #TODO Determine if necessary, and how long 

            # Harvest if not None and move j3 if is None
            if target_plant is not None:
                #TODO whats the value for each of these

                #move to z = min height 
                move(Z=lowest_point) 
                while is_moving(Z):
                    pass 
                #move endeffector to trimming position (further inward)
                move(endeffector=10)
                while is_moving(endeffector):
                    pass 
                #move to z = max height
                move(Z=heighest_point)
                while is_moving(Z):
                    pass 
                # move endeffector back out (all the way to home of endeffector)
                move(endeffector=0) 
                while is_moving(endeffector):
                    pass 
    

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    #Init the robot 
    robot.wait(lambda _: robot.heading_initialized)
    # TODO: Initialize the arm

    #TODO figure out in which corner we started! -> determine origin position
    #idea: move arm to home position and check similarilz to detecting a stemp, if there is one
    #if it is -> we are starting on the B position
    #if not -> we are starting on the A position (or messed up our scan)
    
    # First: Move & Turn to estimated initial position
    robot.motion = [Velocity(linear = 100, angular = 90 / 5.0)] * 10000
    robot.wait(lambda _: robot.position.r >= 0 or len(robot.motion) == 0)
    # ROW A harvesting: Move to each stop points 
    for x in [100, 120, 140, 160]: #TODO unit and measurement
        robot.move_to(Position(x=x, y=0, r=0))
        robot.wait(lambda _: len(robot.motion) == 0)
        
        
        # Detect right side stem/find center
        frame = robot.collection()
        nav_box = robot.navigation.process_image(frame)
        x_box_nav, y_box_nav, w_box_nav, h_box_nav = nav_box
        offset_left: Position = robot.detect(Position(x=x, y=-120, r=0))
        
        
        
        # Harvest
        robot.harvest(Position(x=x_box_nav, y=y_box_nav, r=0)) 
        
        # Mapping
        #TODO Make an association between specific plant and its corresponding mapping please!
        robot.navigation.mapping(frame)
        # Same for right side
        offset_right: Position = robot.detect(Position(x=x, y=120, r=0))
        # Correct robot location
        robot.position = (offset_left + offset_right) / 2
    
    # Rotate 180 degrees

    # ROW B harvesting: Move to each stop points 
    for x in [160, 140, 120, 100]: #TODO unit and measurement
        #TODO: when done ROW A copy behavior/encapsulate in method and call here

    # Return to the stop location
    # TODO
    
    #write file and move to usb
    robot.write_solution_file()
    #TODO move file to USB stick

if __name__ == "__main__":
    main()