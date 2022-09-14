source /home/ros/ros_ws/devel/setup.bash
cd /data
rosbag record \
    /cpp_editor_node/test \
    /cpp_editor_node/text \
    /misty/id_0/action \
    /misty/id_0/speech \
    /rosout