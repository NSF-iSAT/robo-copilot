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

class CoPilotListener(object):
    def __init__(self):
        self.misty_id = 0

        # TODO: add dictionary for block names (from blockly to common)
        self.naming_dict = {}
        
        # publishers
        self.speech_pub = rospy.Publisher('/misty/id_0/speech', String, queue_size=10)
        self.expression_pub = rospy.Publisher('/misty/id_0/expression', String, queue_size=10)

        rospy.init_node('code_listener', anonymous=True)

        self.last_event_time  = rospy.Time.now()
        self.last_speech_time = rospy.Time.now()

        self.event_timeout = rospy.Duration(100)
        self.speech_timeout = rospy.Duration(100)

        self.stats_by_block = defaultdict(lambda: 0)
        self.types_by_block = {}

        self.xml_sub   = rospy.Subscriber('/makecode_xml', String, callback=self.xml_cb)        
        self.s2t_sub = rospy.Subscriber('/speech_to_text/transcript', Transcript, callback=self.s2t_cb)
        self.utterance_start_sub = rospy.Subscriber('/speech_to_text/utterance_start', StartUtterance, callback=self.utterance_start_cb)

        # while not rospy.is_shutdown():
        #     if rospy.Time.now() - self.last_speech_time > self.speech_timeout:
        #         self.generic_thinkaloud_prompt()
        #         self.last_speech_time = rospy.Time.now()

        #     rospy.sleep(0.5)

    def generic_thinkaloud_prompt(self):
        prompt_strings = [
            "Can you tell me what you're thinking?",
            "What are you thinking about right now?",
            "I'm lost, can you explain what you're doing right now?"
        ]

        msg = String(random.choice(prompt_strings))
        self.last_speech_time = rospy.Time.now()
        self.speech_pub.publish(msg)

    def parse_blockly_xml(self, root):
        for child in root:
            if child.name=='block':
                if self.stats_by_block[child.attrib['id']] == 0:
                    self.stats_by_block[child.attrib['id']] = 1
                    self.types_by_block[child.attrib['id']] = child.attrib['type']
            self.parse_blockly_xml(child)
    
    def xml_cb(self, msg):
        # self.latest_xml = msg.data
        # root = ET.fromstring(self.latest_xml)

    def review_edits(self):
        max_edit_block_id = max(self.stats_by_block.keys(), key=lambda k: self.stats_by_block[k])
        min_edit_block_id = min(self.stats_by_block.keys(), key=lambda k: self.stats_by_block[k])
        
        max_edit_block_name = self.types_by_block[max_edit_block_id]
        min_edit_block_name = self.types_by_block[min_edit_block_id]

        if random.random() < 0.5:
            # comment on high-edit block
            block_name = naming_dict[max_edit_block_name]
            str_to_utter = "I think you're editing that %s block a lot. \
                What is that block supposed to do?" % block_name
            self.speech_pub.publish(String(str_to_utter))
            self.stats_by_block[max_edit_block_id] = 0

        else:
            # comment on low-edit block
            block_name = naming_dict[min_edit_block_name]
            str_to_utter = "Have you looked at that %s block at all? Maybe the problem is there." % block_name
            self.speech_pub.publish(String(str_to_utter))
            self.stats_by_block[min_edit_block_id] = 0

    def event_cb(self, msg):
        self.latest_event = msg.data
        self.last_event_time = rospy.Time.now()

        event_type = msg.type
        event_json = json.loads(msg.data)

        if event_type != "selected" and event_type != "end_drag":
            try:
                event_block_id = event_json['block_id']
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