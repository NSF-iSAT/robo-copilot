#!/usr/bin/env python
import random

import rospy
from misty_wrapper.msg import MoveArm, MoveArms, MoveHead
from ros_speech2text.msg import StartUtterance, Transcript
from std_msgs.msg import String
from robo_copilot.msg import BlockEvent

import xml.etree.ElementTree as ET
import json
from collections import defaultdict

NAMING_DICT = { 
    # 'pxt-on-start': 'start',
    'controls_repeat_ext' : 'repeat some times',
    'device_while' : 'while-loop',
    'pxt_controls_for' : 'for-loop',
    # 'variables_get': 'get-variable',
    'pxt_controls_for_of' :  'for-loop',
    'every_interval' : "every-interval",
    # 'device_forever' : 'forever',
    'math_op3' : "math operation",
    # 'break_keyword' : "break statement",
    'playMelody' : "play melody",
    # 'continue_keyword' : "continue statement",
    'radio_datagram_send_string': "radio recieve",
    'controls_if' : "if-statement",
    'logic_compare' : "logical comparison",
    'math_op2': "math operation",
    'math_js_round': "rounding",
    'logic_operation' : "logical operator",
    'device_show_leds' : "device L E D ",
    'device_clear_display' : "clear display"
}

class CoPilotListener(object):
    def __init__(self):
        self.misty_id = 0

        # TODO: add dictionary for block names (from blockly to common)
        self.naming_dict = NAMING_DICT
        self.known_blocks = []
        
        # publishers
        self.speech_pub = rospy.Publisher('/misty/id_0/speech', String, queue_size=10)
        self.expression_pub = rospy.Publisher('/misty/id_0/expression', String, queue_size=10)

        rospy.init_node('code_listener', anonymous=True)

        self.last_event_time  = rospy.Time.now()
        self.last_speech_time = rospy.Time.now()

        self.event_timeout = rospy.Duration(20)
        self.speech_timeout = rospy.Duration(30)

        self.stats_by_block = defaultdict(lambda: 0)
        self.types_by_block = {}
        self.parents_by_block = {}

        self.xml_sub   = rospy.Subscriber('/makecode_xml', String, callback=self.xml_cb)        
        self.event_sub = rospy.Subscriber('/makecode_event', BlockEvent, callback=self.event_cb)
        self.s2t_sub = rospy.Subscriber('/speech_to_text/transcript', Transcript, callback=self.s2t_cb)
        self.utterance_start_sub = rospy.Subscriber('/speech_to_text/utterance_start', StartUtterance, callback=self.utterance_start_cb)

        while not rospy.is_shutdown():
            if rospy.Time.now() - self.last_speech_time > self.speech_timeout:
                if rospy.Time.now() - self.last_event_time > self.event_timeout and self.stats_by_block:
                    self.review_edits()
                else:
                    self.generic_thinkaloud_prompt()
                self.last_speech_time = rospy.Time.now()
            rospy.sleep(0.5)
        
    def generic_thinkaloud_prompt(self):
        prompt_strings = [
            "Can you tell me what you're thinking?",
            "What are you thinking about right now?",
            "I'm lost, can you explain what you're doing right now?"
        ]

        msg = String(random.choice(prompt_strings))
        self.last_speech_time = rospy.Time.now()
        self.speech_pub.publish(msg)

    def parse_blockly_xml(self, root, governing_block=None):
        # import pdb; pdb.set_trace()
        new_governing_block = governing_block
        for child in root:
            if child.tag=='{https://developers.google.com/blockly/xml}block':
                self.types_by_block[child.attrib['id']] = child.attrib['type']
                # self.parents_by_block[child.attrib['id']] = governing_block
                new_governing_block = child.attrib['id']
                # if child.attrib['type'] not in self.known_blocks:
                #     self.known_blocks.append(child.attrib['type'])
            elif child.tag=='{https://developers.google.com/blockly/xml}value':
                # want to treat values as blocks, except they have different properties in the Blockly xml
                # so we'll just treat them as a block with a type of "value"
                shadow = child.find('{https://developers.google.com/blockly/xml}shadow')
                if shadow is not None:
                    try:
                        id = shadow.attrib['id']
                        self.types_by_block[id] = 'value'
                        self.parents_by_block[id] = governing_block
                    except KeyError:
                        pass
            self.parse_blockly_xml(child, new_governing_block)
    

    def xml_cb(self, msg):
        self.latest_xml = msg.data
        root = ET.fromstring(self.latest_xml)
        self.parse_blockly_xml(root)

    def review_edits(self):
        print(self.stats_by_block)
        max_edit_block_id = max(self.stats_by_block.keys(), key=lambda k: self.stats_by_block[k])
        min_edit_block_id = min(self.stats_by_block.keys(), key=lambda k: self.stats_by_block[k])
        
        print(max_edit_block_id, min_edit_block_id)
        print(self.stats_by_block)
        print(self.types_by_block)

        max_edit_block_name = self.types_by_block[max_edit_block_id]
        min_edit_block_name = self.types_by_block[min_edit_block_id]

        if random.random() < 0.5:
            # comment on high-edit block
            print("Commenting on high-edit block: " + max_edit_block_name)
            block_name = self.naming_dict[max_edit_block_name]

            str_to_utter = "It seems like you're editing that %s block a lot. \
                What is that block supposed to do?" % block_name
            self.speech_pub.publish(String(str_to_utter))
            self.stats_by_block[max_edit_block_id] = 0

        else:
            # comment on low-edit block
            print("Commenting on low-edit block: " + min_edit_block_name)
            try:
                block_name = self.naming_dict[min_edit_block_name]
            except KeyError:
                return
            str_to_utter = "Have you looked at that %s block at all? Maybe the problem is there." % block_name
            self.speech_pub.publish(String(str_to_utter))
            self.stats_by_block[min_edit_block_id] = 0

    def event_cb(self, msg):
        # import pdb; pdb.set_trace()
        self.latest_event = msg.data
        self.last_event_time = rospy.Time.now()

        event_type = msg.type
        event_json = json.loads(msg.data)

        if not (event_type in ["selected", "end_drag", "viewport_change", "theme_change"]):
            try:
                event_block_id = event_json['blockId']
                if self.types_by_block[event_block_id] == 'value':
                    event_block_id = self.parents_by_block[event_block_id]
                if self.types_by_block[event_block_id] in self.naming_dict.keys():
                    # only update ones we care about
                    self.stats_by_block[event_block_id] += 1
            except KeyError:
                pass

    def s2t_cb(self, msg):
        self.latest_s2t = msg.data
    
    def utterance_start_cb(self, msg):
        self.last_speech_time = rospy.Time.now()

if __name__ == "__main__":
    listener = CoPilotListener()
    rospy.spin()