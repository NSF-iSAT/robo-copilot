import rospy
from std_msgs.msg import String
from robo_copilot.msg import Debug
from ros_speech2text.msg import Event

import random
import re
import json

CODE_KEYWORD_DICT = {
    "getPlayerName" : "It looks like there's a switch statement in that get player name function. Can you explain how that should work?",
    "placeChar"     : "Looks like that place char function has a lot of conditionals. What do those do?",
    "checkWin"      : "Looks like the check Win function has a lot of conditionals. What is that function trying to do?",
    "checkFull"     : "Hm, can you explain how the checkFull function works?",
    "checkEmptySquare" : "How is the check empty square function supposed to work?"
}


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
    "We got a compilation error. Can we take a look?",
    "Looks like it didn't compile. Let's see what it says.",
    "Oh, it looks like it didn't compile that time. Let's see what the error was."
]

RUNTIME_ERROR_POOL = [
    "It compiled right, but we got an error at runtime. Let's see the debugger output.",
    "Hmmm... it looks like the code had an error when it ran. Let's see."
]

OUTPUT_ERROR_POOL = [
    "Aw, it compiled okay but ran into a test error. ",
    "It looks like the code ran but might have failed one of the built-in tests. ",
    "The code compiled and ran, but it looks like the output it produced wasn't quite right. "
]

SUCCESS_POOL = [
    "It compiled and ran, that's great! Is the output what you were expecting? It looks good to me!"
]

THINKALOUD_PROMPTS_GENERIC = [
    "Can you tell me what you're thinking?",
    "What do you think the next step is?",
    "Can you explain your current process to me?"
]

class CopilotFeedback:
    def __init__(self):
        rospy.init_node("feedback_gen", anonymous=True)
        # subscribers
        self.speech_pub = rospy.Publisher("/misty/id_0/speech",  String, queue_size=2)
        self.face_pub   = rospy.Publisher("/misty/id_0/face_img", String, queue_size=1)
        self.action_pub = rospy.Publisher("/misty/id_0/action", String, queue_size=1)
        
        self.CONDITION = "copilot"
        # TODO uncommment when running for real
        # self.CONDITION = rospy.get_param("~condition")
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
        rospy.Subscriber("/speech_to_text/log", Event, self.speaking_cb)

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
            <s>Could you help me fix the bugs?</s>
            """
        rospy.sleep(6.0)
        self.face_pub.publish(String("e_Joy.jpg"))
        self.speech_pub.publish(startup_msg)
        rospy.sleep(6.0)
        self.action_pub.publish("unsure")

    def code_cb(self, msg):
        self.code_latest = msg.data

    def speaking_cb(self, msg):
        if msg.event == msg.STARTED:
            self.is_listening = True
        elif msg.event == msg.STOPPED:
            self.is_listening = False
            self.last_speech_time = rospy.Time.now()

    def thinkaloud_prompt(self):
        if self.last_err is None:
            return
        elif self.CONDITION != "copilot" or self.code_latest is None:
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
                rospy.sleep(2.0)
                self.action_pub.publish(String("unsure"))

            self.speech_pub.publish(String(speech))
        self.last_speech_time = rospy.Time.now()

if __name__ == "__main__":
    CopilotFeedback()