import rospy
from std_msgs.msg import String
from robo_copilot.msg import Debug

import random
import re
import json

SPOKEN_BACKCHANNELS = [
    "I see.",
    "Hmmmm.",
    "Oh.",
    "I think I follow."
]

COMPILATION_ERROR_POOL = [
    "We got a compilation error. Can we take a look?",
    "Looks like it didn't compile. Let's see what it says.",
    "Oh, it looks like it didn't compile that time. Let's see what the error was."
]

RUNTIME_ERROR_POOL = [
    "It compiled right, but we got an error at runtime. Let's see the debugger output.",
    "Hmmm... it looks like the code had an error when it ran. Let's see."
]

SUCCESS_POOL = [
    "It compiled and ran okay, that's great!"
]

CHARACTER_DICT = {
    ';' : 'semicolon',
    ':' : 'colon',
    ',' : 'comma',
    '.' : 'period',
    '!' : 'exclamation',
    '?' : 'question mark',
    '"' : 'quote',
    '\'' : 'apostrophe',
    # '(' : 'left parenthesis',
    # ')' : 'right parenthesis',
    ')' : "",
    '(' : "",
    '[' : 'left bracket',
    ']' : 'right bracket',
    '{' : 'left brace',
    '}' : 'right brace',
    '<' : 'less than sign',
    '>' : 'greater than sign',
    '-' : 'hyphen',
    '_' : 'underscore',
    '+' : 'plus',
    '=' : 'equals',
    '*' : 'asterisk',
    '/' : 'slash',
    '%' : 'percent',
    '$' : 'dollar',
    '#' : 'hash',
    '@' : 'at',
    '&' : 'ampersand',
    '|' : 'pipe',
    '~' : 'tilde',
    '`' : 'backtick'
}

keyword_dict = {
    "operator" : "<s>What is that operator supposed to do here?</s>",
    "==" : "<s>What things are you comparing?</s> <s>And what types are they? </s>"
}

class CopilotFeedback:
    def __init__(self):

        rospy.init_node("feedback_gen", anonymous=True)
        # subscribers
        self.speech_pub = rospy.Publisher("/misty/id_0/gcloud_speech",  String, queue_size=2)
        self.face_pub   = rospy.Publisher("/misty/id_0/face_img", String, queue_size=1)
        self.action_pub = rospy.Publisher("/misty/id_0/action", String, queue_size=1)

        startup_msg = """
            Hi there! I'm Misty. <s>I'm trying to debug this C plus plus program I found to make
            an interactive game.</s> <s>It has a lot of problems and I'm not very good at programming.</s>
            <s>Could we work on it together?</s>
            """
        rospy.sleep(6.0)
        self.face_pub.publish(String("e_Joy.jpg"))
        self.speech_pub.publish(startup_msg)
        rospy.sleep(6.0)
        self.action_pub.publish("unsure")

        self.last_state = None

        rospy.Subscriber("/cpp_editor_node/test", Debug, self.test_cb)
        rospy.spin()

    def test_cb(self, msg):
        if msg.type == msg.COMPILE:
            # print(msg.data)
            error_msg = random.choice(COMPILATION_ERROR_POOL)

            p = re.compile(".cpp:(\d*):\d*: error: ([^(]*).*\s*\d*\s\|\s*(.*)")
            it = p.finditer(msg.stderr)
            self.speech_pub.publish(String(error_msg))
            
            for match in it:
                first_err_msg = match.group(2)
                first_line_no = match.group(1)
                first_line_of_err = match.group(3)
                break
            
            for char in CHARACTER_DICT.keys():
                if char in first_err_msg:
                    first_err_msg = first_err_msg.replace(char, " " + CHARACTER_DICT[char] + " ")
                if char in first_line_of_err:
                    first_err_msg = first_err_msg.replace(char, " ")
                    
            rospy.sleep(5.0)
            rospy.loginfo(first_err_msg)
            self.action_pub.publish(String("unsure"))
            error_msg = "<s>It looks like on line %s there's an error: %s. Do you know how we can fix that?</s>" % (first_line_no, first_err_msg)
            # error_msg += ("<s>It's the line that says %s</s>" % first_line_of_err)
            self.speech_pub.publish(String(error_msg))
            rospy.sleep(2.0)
            self.action_pub.publish(String("unsure"))
            
        elif msg.type == msg.SUCCESS:
            self.action_pub.publish(String("celebrate"))
            speech = random.choice(SUCCESS_POOL)
            # speech += " The output is: %s" % msg.stdout[:msg.stdout.index('~')]
            self.speech_pub.publish(String(speech))

        elif msg.type == msg.RUNTIME:
            # TODO
            speech = random.choice(RUNTIME_ERROR_POOL)
            # print(msg.payload)
            k = msg.payload.replace("'", '"')
            payload_as_dict = json.loads(k)
            line = payload_as_dict['frame']['line']
            fn   = payload_as_dict['frame']['func']
            sig  = payload_as_dict['signal-meaning'] 

            rospy.sleep(5.0)

            self.action_pub.publish(String("unsure"))
            speech += " On line %s in function %s, we got a %s." % (line, fn, sig)
            speech += " I'm not sure why. What do you think?"
            self.speech_pub.publish(String(speech))

        elif msg.type == msg.ONGOING:
            if self.last_state == msg.ONGOING:
                return
            self.face_pub.publish(String("e_Joy.jpg"))
            speech = "Awesome, it compiled ok! Now we can see how it runs."
            self.speech_pub.publish(String(speech))
            rospy.sleep(3.0)
            self.face_pub.publish(String("e_DefaultContent.jpg"))

        self.last_state = msg.type

if __name__ == "__main__":
    CopilotFeedback()