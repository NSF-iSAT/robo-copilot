import rospy
from std_msgs.msg import String
from robo_copilot.msg import Debug
from ros_speech2text.msg import Event

import random
import re
import json

import pygccxml as pg

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
    # '_' : 'underscore',
    "_" : " ",
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
    "operator" : "What is that operator supposed to do here?",
    "==" : "What things are you comparing? And what types are they?"
}

class CopilotFeedback:
    def __init__(self):
        gen_path, gen_name = pg.utils.find_xml_generator()
        self.xml_gen_config = pg.parser.xml_generator_configuration_t(
            xml_generator_path = gen_path,
            xml_generator = gen_name
        )

        rospy.init_node("feedback_gen", anonymous=True)
        # subscribers
        self.speech_pub = rospy.Publisher("/misty/id_0/speech",  String, queue_size=2)
        self.face_pub   = rospy.Publisher("/misty/id_0/face_img", String, queue_size=1)
        self.action_pub = rospy.Publisher("/misty/id_0/action", String, queue_size=1)
        
        # self.CONDITION = rospy.get_param("~condition")
        self.CONDITION = "copilot"

        # self.startup()
        self.is_listening = False
        self.code_latest = None

        self.err_timeout = rospy.Duration(10.0)
        self.last_msg_time = rospy.Time(0.0)
        self.last_state = None
        self.last_err   = None

        self.speaking_timeout = rospy.Duration(20.0)
        self.last_speech_time = rospy.Time.now()

        rospy.Subscriber("/cpp_editor_node/test", Debug, self.test_cb)
        rospy.Subscriber("/cpp_editor_node/text", String, self.code_cb)
        rospy.Subscriber("/speech_to_text/log", Event, self.speaking_cb)

        # rospy.spin()
        if self.CONDITION == "control":
            rospy.spin()
        else:
            while not rospy.is_shutdown():
                if rospy.Time.now() - self.last_speech_time > self.speaking_timeout:
                    print("timeout triggered")
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
        if self.CONDITION != "copilot" or self.code_latest is None:
            self.speech_pub.publish(String(random.choice(THINKALOUD_PROMPTS_GENERIC)))
        else:
            # base prompt on latest error
            if self.last_err.type == self.last_err.COMPILE or self.last_err.type == self.last_err.RUNTIME:
                for keyword in keyword_dict.keys():
                    if keyword in self.last_err.body:
                        self.speech_pub.publish(self.keyword_dict[keyword])
                        return
                self.speech_pub.publish(String("Do you think you are able to fix that error?"))

            elif self.last_err.type == self.last_err.OUTPUT:
                # parse error message for function information
                fn_capture = re.compile("ERROR: (\S*)")
                fn_name = re.search(fn_capture, self.last_err.payload).group(1)
                
                # parse latest code
                latest_code_parsed = pg.parser.parse_string(self.code_latest, self.xml_gen_config)
                fn_decl_object = self.function_lookup(latest_code_parsed, fn_name)
                if fn_decl_object is not None:
                    args = fn_decl_object.arguments
                    if(len(args) == 0):
                        args_lst = "nothing"
                    else:
                        args_lst = " and ".join([a.name for a in args])
                    speech = "Seems like the function {} takes {} as arguments. ".format(fn_name, args_lst)    
                    speech += ("And it looks like it returns {}. ".format(str(fn_decl_object.return_type)))
                    speech += "What steps does that function take?"
                    self.speech_pub.publish(String(speech))


    def test_cb(self, msg):
        if rospy.Time.now() - self.last_msg_time >= self.err_timeout:
            self.last_state = None

        if msg.type == msg.COMPILE:
            self.last_err = msg
            error_msg = random.choice(COMPILATION_ERROR_POOL)

            p = re.compile(".cpp:(\d*):\d*: error: ([^\n]*)\n\s*\d*\s*\|\s*([^\n]*)")
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

            if self.CONDITION == "copilot":
                error_msg = "<s>It looks like on line %s there's an error: %s. Do you know how we can fix that?</s>" % (first_line_no, first_err_msg)
                self.speech_pub.publish(String(error_msg))
                rospy.sleep(2.0)
                for key in keyword_dict.keys():
                    if key in first_err_msg:
                        speech = "<s>" + keyword_dict[key] + "</s>"
                        self.speech_pub.publish(speech)
                        break
                self.action_pub.publish(String("unsure"))
            
        elif msg.type == msg.SUCCESS:
            self.last_err = msg
            self.action_pub.publish(String("celebrate"))
            speech = random.choice(SUCCESS_POOL)
            self.speech_pub.publish(String(speech))

        elif msg.type == msg.RUNTIME:
            self.last_err = msg
            speech = random.choice(RUNTIME_ERROR_POOL)
            # print(msg.payload)
            k = msg.payload.replace("'", '"')
            payload_as_dict = json.loads(k)
            line = payload_as_dict['frame']['line']
            fn   = payload_as_dict['frame']['func']
            sig  = payload_as_dict['signal-meaning'] 

            rospy.sleep(5.0)

            if self.CONDITION == "copilot":
                speech += " On line %s in function %s, we got a %s." % (line, fn, sig)
                speech += " I'm not sure why. What do you think?"
            self.action_pub.publish(String("unsure"))
            self.speech_pub.publish(String(speech))
            
            # function lookup
            if fn != "main":
                namespace = pg.declarations.get_global_namespace(self.latest_code_parsed)
                for decl in namespace.declarations:
                    if decl.name == fn and isinstance(decl, pg.declarations.free_function_t):
                        args = decl.arguments
                        if len(args) > 0:
                            arg_names = " ".join([str(a.decl_type) + " " + a.name + "," for a in args])
                            print(arg_names)
                            speech = "Looks like the function takes " + arg_names + "  as arguments"

                            self.speech_pub.publish(String(speech)) 
                            rospy.sleep(3.0)

        elif msg.type == msg.ONGOING:
            if self.last_state != msg.ONGOING:
                self.face_pub.publish(String("e_Joy.jpg"))
                speech = "Awesome, it compiled ok! Now we can see how it runs."
                self.speech_pub.publish(String(speech))
                rospy.sleep(3.0)
                self.face_pub.publish(String("e_DefaultContent.jpg"))

        elif msg.type == msg.OUTPUT:
            if self.last_state != msg.OUTPUT:
                self.action_pub.publish(String("unsure"))
                speech = random.choice(OUTPUT_ERROR_POOL)

                if self.CONDITION == "copilot":
                    speech += (" The test reads: " + msg.payload + ". ")
                    speech += "Maybe we can look in the code for that test and see what should have happened."
                self.speech_pub.publish(String(speech))
                self.last_err = msg

        self.last_state = msg.type
        
    def function_lookup(self, parsed_code, fn):
        namespace = pg.declarations.get_global_namespace(parsed_code)
        for decl in namespace.declarations:
            if fn in decl.name and isinstance(decl, pg.declarations.free_function_t):
                return decl

if __name__ == "__main__":
    CopilotFeedback()