import subprocess
import os
from tkinter import *
import tkinter.filedialog as fd
import tkinter.messagebox as mb

from pygdbmi.gdbcontroller import GdbController

import rospy
from std_msgs.msg import String
from robo_copilot.msg import Debug

class OutputWindow:
    """ Module to draw an output display window, plus text entry and a test button.
        Inputs:
            root : the tkinter root (a Tk() instance)
            test_fn  : a funciton to be triggered when user presses the "Test" button.
    """
    def __init__(self, root, test_fn):
        root.title("Output")
        root.resizable(True, True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        frame = Frame(root, bd=2, relief=SUNKEN)
        bottomframe = Frame(root, relief=SUNKEN)

        self.text = Text(frame, font=("Courier New", 12), state=DISABLED)
        self.text.pack(side=LEFT, expand=True, fill=BOTH)

        scrollbar = Scrollbar(frame)
        scrollbar.pack(side=RIGHT, fill=Y)
        scrollbar.config(command=self.text.yview)
        self.text.config(yscrollcommand=scrollbar.set)
        self.scrollbar=scrollbar

        test_button = Button(bottomframe, text="Test", command=test_fn)
        test_button.pack(side=RIGHT)

        frame.pack(side=TOP, fill=BOTH, expand=True)
        bottomframe.pack(side=BOTTOM, fill=X)

    def place_text(self, new_text):
        # self.test_count += 1
        prev_yview = self.text.yview()

        self.text["state"] = NORMAL
        self.text.insert(END, new_text + "\n")
        self.text["state"] = DISABLED

        self.text.yview_moveto(prev_yview[1])


class CppEditorNode:
    def __init__(self):            
        # set up ros bindings
        rospy.init_node('cpp_editor_node')
        self.code_pub = rospy.Publisher('cpp_editor_node/text', String, queue_size=1)
        self.test_pub = rospy.Publisher('cpp_editor_node/test', Debug, queue_size=1)

        self.tk     = Tk()
        self.output = OutputWindow(self.tk, self.run_test)
        self.test_count = 0

        self.filename = "/home/kaleb/code/ros_ws/src/robo_copilot/assets/simple_game_task.cpp"

        self.in_debug = False
        self.gdbmi    = None
        self.test_result = None

        while not rospy.is_shutdown():
            self.tk.update()
        self.tk.quit()

    def compile(self, cpp_file, binary_file):
        res = subprocess.run(["g++", "-g3", cpp_file, "-o", binary_file], check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(res.stdout.decode('utf-8'))
        print(res.stderr.decode('utf-8'))
        if res.returncode != 0:
            return (False, res.stderr.decode('utf-8'))
        return (True, res.stderr.decode('utf-8'))

    def run_test(self):
        self.test_result = None
        self.test_count += 1
        
        cpp_file = self.filename
        binary_file = os.path.join(
            os.path.dirname(self.filename), "tmp.out"
        )

        # publish cpp file contents for logging purposes
        with open(cpp_file, 'r') as code_file:
            code = code_file.read()
            self.code_pub.publish(String(code))

        
        self.output.place_text("\n----------------\nTest #{}: testing {}".format(  self.test_count,
                os.path.basename(self.filename)))

        result, err = self.compile(cpp_file, binary_file)
        if not result:
            msg = Debug()
            # failed to compile
            msg.type = msg.COMPILE
            msg.stderr = err
            msg.stdout = ""
            self.test_pub.publish(msg)
            self.output.place_text(err)
            return
        if (err.strip()):
            self.output.place_text("\nCOMPILER: " + err)
        # compiled successfully, now try to run via gdb
        self.gdbmi = gdbmi = GdbController()
        response = gdbmi.write('-file-exec-and-symbols ' + binary_file)
        response = gdbmi.write('-exec-run')
        
        if self.process_gdb_response(response):
            self.in_debug = True
        else:
            self.in_debug = False
            self.gdbmi = None
            gdbmi.exit()

    def process_gdb_response(self, response):
        msg = Debug()
        program_output = ""
        gdb_output = ""
        msg.type = msg.SUCCESS
        for item in response:
            if item["type"] == "output":
                program_output += item["payload"]
                self.output.place_text(item["payload"])

                if "ERROR" in item["payload"] and self.test_result is None:
                    self.test_result = msg.OUTPUT
                    msg.type = msg.OUTPUT
                    # msg.payload = item["payload"]
                    # self.test_pub.publish(msg)
                msg.payload += str(item["payload"])

            elif item["type"] == "console":
                gdb_output += item["payload"]
                self.output.place_text("\nDEBUG: " + item["payload"])

            elif item["message"] == "stopped":
                if self.test_result is None:
                    if item["payload"]["reason"] != "exited-normally":
                        self.test_result = msg.RUNTIME
                        msg.type = msg.RUNTIME
                        msg.payload += str(item["payload"])

                self.test_pub.publish(msg)
                return False

        # msg.type = msg.ONGOING
        msg.payload = ""
        msg.stdout = program_output
        msg.stderr = gdb_output
        self.test_pub.publish(msg)
        return True

    
if __name__ == "__main__":
    CppEditorNode()