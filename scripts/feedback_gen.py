import rospy
from std_msgs.msg import String
from robo_copilot.msg import Error

import re

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
            error_msg = "Oh, it looks like it didn't compile yet. Let's take a look at the code."

            p = re.compile(".*simple_game.cpp:(\d*):\d*: error: (.*)")
            it = p.finditer(msg.stderr)
            self.speech_pub.publish(String(error_msg))
            rospy.sleep(5.0)
            
            for match in it:
                first_err_msg = match.group(2)
                first_line_no = match.group(1)
                break
            
            for char in CHARACTER_DICT.keys():
                if char in first_err_msg:
                    first_err_msg = first_err_msg.replace(char, CHARACTER_DICT[char])
                    
            error_msg = "It looks like on line %s there's an error: %s. Do you know how we can fix that" % (first_line_no, first_err_msg)
            print(error_msg)
            self.speech_pub.publish(String(error_msg))
            
        elif msg.type == msg.SUCCESS:
            speech = "Looks like it compiled and ran okay! Let's take a look at the output."
            self.speech_pub.publish(String(speech))
            speech = "The output is: %s" % msg.stdout
            self.speech_pub.publish(String(speech))

        elif msg.type == msg.RUNTIME:
            # TODO
            pass

if __name__ == "__main__":
    CopilotFeedback()