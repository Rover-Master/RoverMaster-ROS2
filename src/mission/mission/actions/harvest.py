import time

def harvest(self, stemp_center):
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