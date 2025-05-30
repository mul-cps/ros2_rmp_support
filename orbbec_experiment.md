download this repo (ros2_rmp_support)

build the package.
source the package

install christians orbbec package

run
```bash
ros2 launch ros2_rmp_support rsp.launch.py
ros2 launch orbbec_camera femto_mega.launch.py enable_colored_point_cloud:=True enumerate_net_device:=True camera_name:=femto_mega
```

download ros2_rmp repo
cd into ros2_rmp repo
run
```bash
./run_robot.sh
```
to start all the robot hardware services.

--> ready to rosbag!

Important: orbbec topics will be prefixed with "femto_mega"