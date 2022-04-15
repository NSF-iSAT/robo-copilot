#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# source the catkin setup bash script
if [ -e ~/ros_ws/devel/setup.bash ]; then
    echo "Sourcing ~/ros_ws/devel/setup.bash.."
    source ~/ros_ws/devel/setup.bash
fi
exec $@ 