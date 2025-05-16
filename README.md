Support package needed for Segway RMP220 control.

Part of the CPS Segway RMP220 ROS2 meta-package:
- ros2_rmp
- ros2_rmp_middleware
- ros2_rmp_teleop
- ros2_rmp_support

## Robot Support Package for Segway RMP220

This package should be placed into your workspace src/ folder.
Alongside you must clone https://github.com/bjoernellens1/segwayrmp and https://github.com/bjoernellens1/segway_msgs to control the robot.
Additional packages needed are:
- https://github.com/bjoernellens1/rmp220_teleop
- https://github.com/bjoernellens1/rmp220_middleware

Go back to ws folder and colcon build --symlink-install && source install/setup.bash

### Launch files:

#### Launch robot controller
ros2 launch cps_rmp_220_support robot_controller.launch.py

#### Launch robot state publisher
ros2 launch cps_rmp_220_support rsp.launch.py

#### Launch rplidar
ros2 launch cps_rmp_220_support robot_lidar.launch.py

#### Launch teleop - from other package
ros2 launch rmp220_teleop robot_joystick.launch.py 
