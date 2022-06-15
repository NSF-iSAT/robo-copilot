import rospy
from std_msgs.msg import String
import re
import ast

class CopilotFeedback:
    def __init__(self):

        rospy.init_node("feedback_gen", anonymous=True)
        # subscribers
        rospy.Subscriber("/cpp_editor_node/test", String, self.test_cb)

        rospy.spin()

    def test_cb(self, msg):
        # for now, assume it's a compilation error

        # print(msg.data)
        p = re.compile(".*simple_game.cpp:([0-9]*):[0-9]*: error: (.*)")
        it = p.finditer(msg.data)
        for match in it:
            line_num = match.group(1)
            error_msg = match.group(2)
            print(line_num, error_msg)

if __name__ == "__main__":
    CopilotFeedback()