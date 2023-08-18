#!/usr/bin/env bash

# Launch the robot
source /opt/ros/melodic/setup.bash 
source /home/michelle/catkin_ws/devel/setup.bash 

echo "Launching application, please wait!"

./wifi_docker_proxy/go.py setup
./wifi_docker_proxy/go.py connect

roslaunch yolov4_trt_ros $your_launch.launch 
