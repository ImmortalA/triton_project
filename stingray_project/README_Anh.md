0. Copy files

scp -r ~/stingray_docker/catkin_ws/src/stingray_project triton@192.168.0.123:~/catkin_ws/src/

chmod +x *.py

1. Setup

- Robot: Open terminal

ssh triton@192.168.0.123

export ROS_IP=192.168.0.123

- Laptop:

export ROS_MASTER_URI=http://192.168.0.123:11311

export ROS_IP=192.168.0.13

2. Run

- Robot:

roslaunch stingray_project triton.launch

- Laptop:

source ~/stingray_docker/catkin_ws/devel/setup.bash

rosrun stingray_project gesture_controller.py