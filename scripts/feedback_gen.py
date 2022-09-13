import rospy
from std_msgs.msg import String
from robo_copilot.msg import Debug, Speech
# from ros_speech2text.msg import Event
from .function_text import *

# import random
import numpy as np
import random
import re
import json

class CopilotFeedback:
    def __init__(self):
        rospy.init_node("feedback_gen", anonymous=True)
        # subscribers
        self.speech_pub = rospy.Publisher("/misty/id_0/speech",  String, queue_size=2)
        self.face_pub   = rospy.Publisher("/misty/id_0/face_img", String, queue_size=1)
        self.action_pub = rospy.Publisher("/misty/id_0/action", String, queue_size=1)
        
        self.CONDITION = rospy.get_param("~condition")
        # TODO uncommment when running for real
        # self.startup()

        self.is_listening = False
        self.code_latest = None
        self.prompts_on_err = 0

        self.err_timeout = rospy.Duration(10.0)
        self.last_msg_time = rospy.Time(0.0)
        self.last_err   = None

        self.speaking_timeout = rospy.Duration(20.0)
        self.last_speech_time = rospy.Time.now()

        rospy.Subscriber("/cpp_editor_node/test", Debug, self.test_cb)
        rospy.Subscriber("/cpp_editor_node/text", String, self.code_cb)
        # rospy.Subscriber("/speech_to_text/log", Event, self.speaking_cb)
        rospy.Subscriber("/vad/utterance", Speech, self.speaking_cb)

        # TODO uncomment
        self.startup()

        while not rospy.is_shutdown():
            if not self.is_listening and rospy.Time.now() - self.last_speech_time > self.speaking_timeout:
                rospy.loginfo("thinkaloud timeout triggered")
                self.thinkaloud_prompt()
                self.last_speech_time = rospy.Time.now()

            rospy.sleep(2.0)

    def startup(self):
        startup_msg = """
            Hi there! I'm Misty. <s>I'm trying to debug this C plus plus program I found to make
            a tic tac toe game.</s> <s>But, it has a lot of problems and I don't know that much about programming.</s>
            <s>Could you help me fix the bugs, and explain how you do it?</s>
            """
        rospy.sleep(6.0)
        self.face_pub.publish(String("e_Joy.jpg"))
        self.speech_pub.publish(startup_msg)
        rospy.sleep(6.0)
        self.action_pub.publish("unsure")

    def code_cb(self, msg):
        self.code_latest = msg.data

    def speaking_cb(self, msg):
        if msg.type == msg.STARTED:
            self.is_listening = True
        elif msg.type == msg.STOPPED:
            self.is_listening = False
            self.last_speech_time = rospy.Time.now()

    def thinkaloud_prompt(self):
        if self.last_err is None:
            return
        elif self.CONDITION != "copilot":
            self.speech_pub.publish(String(random.choice(THINKALOUD_PROMPTS_GENERIC)))
        else:
            # # base prompt on latest error
            if self.last_err["type"] == self.last_err["COMPILE"]:
                if self.prompts_on_err == 0:
                    speech = "What do you think caused that error on line {}?".format(self.last_err["line"])
                else:
                    speech = "What do you think your next steps are for solving that first error?"

            if self.last_err["type"] == self.last_err["RUNTIME"]:
                if self.prompts_on_err == 0:
                    speech = "What is the {} function supposed to do? And what input do you think triggered the {}?".format(
                        self.last_err["function"], self.last_err["signal"])
                else:
                    speech = "What do you think your next steps are for solving that first error?"

            elif self.last_err["type"] == self.last_err["OUTPUT"]:
                if self.prompts_on_err == 0:
                    speech = CODE_KEYWORD_DICT[self.last_err["function"]]
                else:
                    speech = "What do you think the {} function should have produced for that test case?".format(
                        self.last_err["function"]
                    )
            else:
                return
            self.prompts_on_err += 1

            self.action_pub.publish("unsure")
            self.speech_pub.publish(String(speech))

    def test_cb(self, msg):
        self.last_speech_time = rospy.Time.now()
        self.prompts_on_err = 0
        self.last_err = {}
        self.last_err["type"] = msg.type
        self.last_err["payload"] = msg.payload

        self.last_err["SUCCESS"] = msg.SUCCESS
        self.last_err["RUNTIME"] = msg.RUNTIME
        self.last_err["COMPILE"] = msg.COMPILE
        self.last_err["OUTPUT"]  = msg.OUTPUT

        if self.CONDITION == "control":
            return

        if msg.type == msg.COMPILE:
            error_msg = random.choice(COMPILATION_ERROR_POOL)

            p = re.compile(".cpp:(\d*):\d*: error: ([^\n]*)\n\s*\d*\s*\|\s*([^\n]*)")
            it = p.finditer(msg.stderr)
            self.speech_pub.publish(String(error_msg))
            
            for match in it:
                first_err_msg = match.group(2)
                first_line_no = match.group(1)
                first_line_of_err = match.group(3)
                break
                    
            rospy.sleep(5.0)
            # rospy.loginfo(first_err_msg)
            self.last_err["line"] = first_line_no
            self.last_err["message"] = first_err_msg
            self.action_pub.publish(String("unsure"))

            if self.CONDITION == "copilot":
                error_msg = "<s>It looks like on line %s there's an error: %s. Do you know how we can fix that?</s>" % (first_line_no, first_err_msg)
                self.speech_pub.publish(String(error_msg))
                rospy.sleep(2.0)
            self.action_pub.publish(String("unsure"))
            
        elif msg.type == msg.SUCCESS:
            self.action_pub.publish(String("celebrate"))
            speech = random.choice(SUCCESS_POOL)
            self.speech_pub.publish(String(speech))

        elif msg.type == msg.RUNTIME:
            speech = random.choice(RUNTIME_ERROR_POOL)
            self.action_pub.publish(String("unsure"))
            # print(msg.payload)

            if self.CONDITION == "copilot":
                k = msg.payload.replace("'", '"')
                payload_as_dict = json.loads(k)
                line = payload_as_dict['frame']['line']
                fn   = payload_as_dict['frame']['func']
                sig  = payload_as_dict['signal-meaning'] 

                self.last_err["line"] = line
                self.last_err["function"] = fn
                self.last_err["signal"] = sig

                rospy.sleep(5.0)

                speech += " On line %s in function %s, we got a %s." % (line, fn, sig)
                speech += " I'm not sure why. What do you think?"
                self.speech_pub.publish(String(speech))
    
        elif msg.type == msg.OUTPUT:
            self.action_pub.publish(String("unsure"))
            speech = random.choice(OUTPUT_ERROR_POOL)

            if self.CONDITION == "copilot":
                speech += (" The test reads: " + msg.payload + ". ")
                speech += "Maybe we can look in the code for that test and see what should have happened."
                
                fn_capture = re.compile("ERROR: (\S*)")
                fn_name = re.search(fn_capture, msg.payload).group(1)
                
                self.last_err["function"] = fn_name
                # rospy.sleep(2.0)
                # self.action_pub.publish(String("unsure"))

            self.speech_pub.publish(String(speech))
            self.face_pub.publish(String("e_SystemGearPrompt.jpg"))
        self.last_speech_time = rospy.Time.now()

if __name__ == "__main__":
    CopilotFeedback()