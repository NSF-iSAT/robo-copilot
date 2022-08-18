#!/bin/bash
source ~/ros_ws/devel/setup.bash
roslaunch misty_wrapper misty.launch robot_ip:=$1