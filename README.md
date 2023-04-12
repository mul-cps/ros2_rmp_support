## Supplementary Robot PAckage for Segway RMP220 lite

This package should be placed into your workspace src/ folder.
Alongside you must clone https://github.com/bjoernellens1/segwayrmp and https://github.com/bjoernellens1/segway_msgs to control the robot.
Go back to ws folder and colcon build --symlink-install && source install/setup.bash

### Launch files:
#### Launch Gazebo
ros2 launch cps_rmp220_support launch_sim.launch.py

#### Launch Robot operator (Only neeed for Camera and additional devices for now)
ros2 launch cps_rmp220_support launch_robot.launch.py