import rospy
from std_msgs.msg import String
from robo_copilot.msg import Error

import random
import re
import json

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
    '(' : 'left parenthesis',
    ')' : 'right parenthesis',
    '[' : 'left bracket',
    ']' : 'right bracket',
    '{' : 'left brace',
    '}' : 'right brace',
    '<' : 'less than',
    '>' : 'greater than',
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

class CopilotFeedback:
    def __init__(self):

        rospy.init_node("feedback_gen", anonymous=True)
        # subscribers
        rospy.Subscriber("/cpp_editor_node/test", Error, self.test_cb)
        self.speech_pub = rospy.Publisher("/text_to_speech", String, queue_size=2)
        rospy.spin()

    def test_cb(self, msg):

        if msg.type == msg.COMPILE:
            # print(msg.data)
            error_msg = random.choice(COMPILATION_ERROR_POOL)

            p = re.compile(".*simple_game.cpp:(\d*):\d*: error: (.*)")
            it = p.finditer(msg.stderr)
            self.speech_pub.publish(String(error_msg))
            
            for match in it:
                first_err_msg = match.group(2)
                first_line_no = match.group(1)
                break
            
            for char in CHARACTER_DICT.keys():
                if char in first_err_msg:
                    first_err_msg = first_err_msg.replace(char, CHARACTER_DICT[char])
                    
            rospy.sleep(5.0)
            error_msg = "It looks like on line %s there's an error: %s. Do you know how we can fix that?" % (first_line_no, first_err_msg)
            self.speech_pub.publish(String(error_msg))
            
        elif msg.type == msg.SUCCESS:
            speech = random.choice(SUCCESS_POOL)
            speech += " The output is: %s" % msg.stdout[:msg.stdout.index('~')]
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
            speech += " On line %s in function %s, we got a %s." % (line, fn, sig)
            speech += " I'm not sure why. What do you think?"
            self.speech_pub.publish(String(speech))

if __name__ == "__main__":
    CopilotFeedback()