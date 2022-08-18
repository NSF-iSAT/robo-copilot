#!/bin/bash
source ~/ros_ws/devel/setup.bash
cd assets && cp simple_game_task.cpp /data/task_$1.cpp
featherpad /data/task_$1.cpp &