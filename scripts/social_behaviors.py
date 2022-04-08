import rospy
from misty_wrapper.msg import DetectedFace, MoveHead

class IdleListeningModule:
    def __init__(self):
        self.misty_id = rospy.get_param("/robot_id", "0")
