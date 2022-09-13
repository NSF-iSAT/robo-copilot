import rospy
from std_msgs.msg import String
from robo_copilot.msg import Debug, Speech
from .function_text import *

import random
import numpy as np
import random
import re
import json

class BaseSupportBot:
    def __init__(self):
        self.thinkaloud_timeout_mean    = 6.0
        self.thinkalout_timeout_sd      = 2.0

        self.backchannel_trigger_mean   = 2.5
        self.backchannel_trigger_sd     = 1.0

        self.is_listening           = False
        self.last_utterance_start   = None
        self.last_utterance_end     = None

        self.setup()
        self.startup()
        self.spin()

    def setup(self):
        self.speech_pub = rospy.Publisher("/misty/id_0/speech",  String, queue_size=2)
        self.face_pub   = rospy.Publisher("/misty/id_0/face_img", String, queue_size=1)
        self.action_pub = rospy.Publisher("/misty/id_0/action", String, queue_size=1)
        
        rospy.Subscriber("vad/utterance", Speech, self.speech_cb)

    def startup(self):
        pass

    def spin(self):
        while not rospy.is_shutdown():
            if self.is_listening and (rospy.Time.now() - self.last_utterance_start) > rospy.Duration(np.random.normal(self.backchannel_trigger_mean, self.backchannel_trigger_sd)):
                self.backchannel()
                self.last_utterance_start = rospy.Time.now()

            elif not self.is_listening and (rospy.Time.now() - self.last_utterance_end) > rospy.Duration(np.random.normal(self.thinkaloud_timeout_mean, self.thinkalout_timeout_sd)):
                self.prompt()
                rospy.loginfo("Thinkaloud timeout triggered")
                self.last_utterance_end = rospy.Time.now()

            rospy.sleep(0.25)

    def speech_cb(self, msg):
        if msg.type == msg.STARTED:
            self.last_utterance_start = rospy.Time.now()
            self.is_listening = True

        elif msg.type == msg.STOPPED:
            self.last_utterance_end = rospy.Time.now()
            self.is_listening = False

    def prompt(self):
        msg = String(random.choice(THINKALOUD_PROMPTS_GENERIC))
        self.speech_pub.publish(msg)

    def backchannel(self):
        # TODO tweak
        self.action_pub.publish("nod")

if __name__ == "__main__":
    rospy.init_node("feedback_gen", anonymous=False)

    condition = rospy.get_param("~condition", "copilot")

    if condition == "base":
        BaseSupportBot()
