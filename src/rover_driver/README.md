1. Send velocity commands to the motor.
    + `/motion/vel_x` <!-- forward/back -->
    + `/motion/vel_y` <!-- left/right -->
    + `/motion/vel_r` <!-- clockwise/counterclockwise -->
2. Get IMU data from the flight controller and pulish into corresponding topic.
    + `/imu/accX`
    + `/imu/accY`
    + `/imu/accZ`
    + `/imu/heading`
    + `/imu/rotX`
    + `/imu/rotY`
3. Manage Camera Platform