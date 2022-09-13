#!/bin/bash
source ~/ros_ws/devel/setup.bash
cd assets && cp task_current.cpp /data/task_copy.cpp
featherpad /data/task_copy.cpp &
roslaunch robo_copilot copilot_setup.launch use_robot:=true audio_device_idx:=$1 use_tts:=true