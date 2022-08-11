#!/bin/bash

cd assets && cp simple_game_task.cpp /data/task_$1.cpp
# roslaunch robo_copilot copilot_demo.launch audio_device_idx:=$2 &
featherpad /data/task_$1.cpp &
rosbag record -a