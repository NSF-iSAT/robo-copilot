#!/bin/bash

cd robo_copilot/assets
cp simple_game_task.cpp ./task_$1.cpp
roslaunch robo_copilot copilot_demo.launch audio_device_idx:=$2 &
featherpad task_$1.cpp
