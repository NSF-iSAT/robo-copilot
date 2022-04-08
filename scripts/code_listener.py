#!/usr/bin/env python
from random import random

import rospy
from makecode_ros_msgs.msg import BlockEvent
from misty_wrapper.msg import MoveArm, MoveArms, MoveHead
from std_msgs.msg import String

class CoPilotListener(object):
    def __init__(self):
        self.misty_id = 0
        self.event_sub = rospy.Subscriber('/makecode_event', BlockEvent, callback=self.event_cb)
        self.speech_pub = rospy.Publisher('/misty/id_0/speech', String, queue_size=10)
        self.expression_pub = rospy.Publisher('/misty/id_0/expression', String, queue_size=10)
        self.arms_pub = rospy.Publisher('/misty/id_0/arms', String, queue_size=10)
        rospy.init_node('code_listener', anonymous=True)
        self.for_count = 0

    def generic_thinkaloud_prompt(self):
        prompt_strings = [
            "Can you tell me what you're thinking?",
            "What are you thinking about right now?",
            "I'm lost, can you explain what you're doing right now?"
        ]

        msg = String(random.choice(prompt_strings))

        self.speech_pub(msg)

    def event_cb(self, msg):
        # print(msg)
        if msg.type == "create":
            if msg.blockId in ["forever", "pxt_controls_for", "pxt_controls_for_of"]:
                self.for_count += 1
                if self.for_count >= 3:
                    self.expression_pub.publish(String("e_Surprise.jpg"))
                    self.speech_pub.publish(String("I'm sorry, I'm afraid that's just too many loops!"))
                    arms_up = MoveArms({"left": MoveArm(10, 100), "right": MoveArm(10, 100)})
                    self.arms_pub.publish(arms_up)
                    self.for_count -= 1
                    

if __name__ == "__main__":
    listener = CoPilotListener()
    rospy.spin()

        