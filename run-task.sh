#!/bin/bash
source ~/ros_ws/devel/setup.bash
cd assets && cp task_current.cpp /data/task_copy.cpp
featherpad /data/task_copy.cpp &
roslaunch robo_copilot copilot_setup.launch use_robot:=true robot_ip:=$2 use_tts:=true