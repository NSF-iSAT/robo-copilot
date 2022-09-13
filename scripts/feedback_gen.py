from doctest import OutputChecker
import rospy
from std_msgs.msg import String
from robo_copilot.msg import Debug, Speech
# from function_text import *

import random
import numpy as np
import random
import re
import json


CODE_KEYWORD_DICT = {
    "getPlayerName" : "It looks like there's a switch statement in that get player name function. Can you explain how that should work?",
    "placeChar"     : "Looks like that place char function has just a single if statement, then does something in the body. What do you think it's doing?",
    "checkWin"      : "Looks like the check Win function has a for loop, and lots of conditionals. What are all those for?",
    "checkFull"     : "Hm, how does the checkFull function decide whether the board is full?",
    "checkEmptySquare" : "How is the check empty square function supposed to work?"
}

SINGLE_SUCCESS_POOL = [
    "Great, it looks like you fixed an error! Make sure to think aloud as you move on to the next one."
]

FUNCTION_DICT = {
    "getPlayerName" : """
            string name;
            switch (player_num) {
                case 1:
                    name = player1_name;
                    
                case 2:
                    name = player2_name;
                    break;
            }
            return name;""",
    "placeChar" : """
            if (row >= 3 || col >= 3) {
                cout << "placeChar: invalid" << endl;
            } else if (board[row][col] != ' ') {
                cout << "placeChar: spot not empty" << endl;
            } else {
                board[row][col] = c;
                
            }
        """,
    "printBoard" : """
            cout << "   0 | 1 | 2" << endl;
            cout << "   _________" << endl;
            for (int i = 0; i < 3; i++) {
                cout << i;
                for (int i = 0; i < 3; i++) {
                    cout << "| " << board[i] << " ";
                }
                cout << endl << "   _________" << endl;

            }
    """,
    "checkWin" : """
                // check for 3 in a row horizontally and vertically
            for (int i = 0; i <= 3; i++) {
                if (board[i][0] == c || board[i][1] == c || board[i][2] == c) {
                    return true;
                } else if (board[0][i] == c && board[1][i] == c && board[2][i] == c) {
                }
                    return true;
            }

            // check for 3 in a row diagonally
            if (board[0][0] == c && board[1][1] == c && board[2][2] == c) {
                return true;
            } else if (board[0][2] == c && board[1][1] == c && board[2][0] == c) {
                return true;
            }
            return false;
    """,
    "checkFull" : "return (filled_squares >= 8);",
    "checkEmptySquare" : "return (board[row][col] = ' ');"
}
COMPILATION_ERROR_POOL = [
    "We got a compilation error. Can we take a look at the line that caused the error?",
    "Looks like it didn't compile. Let's see what the error message says.",
    "Oh, it looks like it didn't compile that time. Try to think aloud as you trace the cause of the error."
]

RUNTIME_ERROR_POOL = [
    "It compiled right, but we got an error at runtime. Let's see the debugger output.",
    "Hmmm... it looks like the code had an error when it ran. Let's see."
]

OUTPUT_ERROR_POOL = [
    "Aw, our code compiled okay but ran into a test error. Do you know what output caused the error?",
    "It looks like we might have failed one of the built-in tests. Try to think aloud as you trace the cause of the error.",
    "We failed an output test. Can you think aloud as you look at the first test error?"
]

SUCCESS_POOL = [
    "You passed all the tests, that's amazing! Great work!"
]

THINKALOUD_PROMPTS_GENERIC = [
    "Can you tell me what you're thinking?",
    "What do you think the next step is?",
    "Can you explain your current process to me?",
    "Remember to think aloud as you work.",
    "Try to remember to think aloud as you work."
]

class BaseSupportBot:
    def __init__(self):
        self.backchannel_timeout = 2.5
        self.thinkaloud_timeout  = 15.0

        self.is_listening           = False
        self.last_utterance_start   = rospy.Time.now()
        self.last_utterance_end     = rospy.Time.now()

        self.last_errors = None

        self.setup()
        self.startup()
        self.spin()

    def setup(self):
        self.speech_pub = rospy.Publisher("/misty/id_0/speech",  String, queue_size=2)
        self.face_pub   = rospy.Publisher("/misty/id_0/face_img", String, queue_size=1)
        self.action_pub = rospy.Publisher("/misty/id_0/action", String, queue_size=3)
        
        rospy.Subscriber("cpp_editor_node/test", Debug, self.code_test_cb)
        rospy.Subscriber("vad/utterance", Speech, self.speech_cb)

    def code_test_cb(self, msg):
        # reset speech times
        self.last_utterance_end = rospy.Time.now()

        if msg.type == msg.SUCCESS:
            speech = random.choice(SUCCESS_POOL)
            self.action_pub.publish("celebrate")
            self.speech_pub.publish(speech)
            return

        elif msg.type == msg.OUTPUT:
            errors = []
            # print(msg)
            for i in range(7):
                if "TEST " + str(i) in msg.payload:
                    errors.append(i)
            print(errors)
            if self.last_errors is not None:
                if len(errors) < len(self.last_errors):
                    speech = random.choice(SINGLE_SUCCESS_POOL)
                    self.action_pub.publish("waggle")
                    self.action_pub.publish("tilt")
                    self.speech_pub.publish(String(speech))
                    self.last_errors = errors

                    return

            self.last_errors = errors

        if msg.type == msg.COMPILE:
            error_msg = random.choice(COMPILATION_ERROR_POOL)
            self.action_pub.publish("unsure")
            self.speech_pub.publish(String(error_msg))

        elif msg.type == msg.RUNTIME:
            error_msg = random.choice(RUNTIME_ERROR_POOL)
            self.action_pub.publish("nod")
            self.speech_pub.publish(String(error_msg))

        elif msg.type == msg.OUTPUT:
            error_msg = random.choice(OUTPUT_ERROR_POOL)
            self.action_pub.publish("nod")
            self.speech_pub.publish(String(error_msg))
            

    def startup(self):
        msg = "Hi there! My name is Misty. I'm trying to learn more about how programmers find and fix bugs in C plus plus code.\
                Can we complete this debugging task together so I can learn?"

    def spin(self):
        while not rospy.is_shutdown():
            if self.is_listening and (rospy.Time.now() - self.last_utterance_start) > rospy.Duration(self.backchannel_timeout):
                self.backchannel()
                self.last_utterance_start = rospy.Time.now()

            elif not self.is_listening and (rospy.Time.now() - self.last_utterance_end) > rospy.Duration(self.thinkaloud_timeout):
                self.prompt()
                rospy.loginfo("Thinkaloud timeout triggered: " + str((rospy.Time.now() - self.last_utterance_end).to_sec()) + " > " + str(self.thinkaloud_timeout))
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
        self.action_pub.publish(String("tilt"))
        # self.action_pub.publish(String("waggle"))

    def backchannel(self):
        # TODO tweak
        bc = random.choice(["nod", "check"])
        if bc == "nod":
            self.action_pub.publish("nod")
        elif bc == "check":
            self.action_pub.publish("look")

if __name__ == "__main__":
    rospy.init_node("feedback_gen", anonymous=False)

    condition = rospy.get_param("~condition", "copilot")

    # if condition == "base":
    BaseSupportBot()
