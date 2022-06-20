import subprocess
import os
from pprint import pprint 
from tkinter import *
import tkinter.filedialog as fd
import tkinter.messagebox as mb

from pygdbmi.gdbcontroller import GdbController

import rospy
from std_msgs.msg import String
from robo_copilot.msg import Error

class TextLineNumbers(Canvas):
    # module to add line numbers
    def __init__(self, *args, **kwargs):
        Canvas.__init__(self, *args, **kwargs)
        self.textwidget = None

    def attach(self, text_widget):
        self.textwidget = text_widget
        
    def redraw(self, *args):
        '''redraw line numbers'''
        self.delete("all")

        i = self.textwidget.index("@0,0")
        while True :
            dline= self.textwidget.dlineinfo(i)
            if dline is None: break
            y = dline[1]
            linenum = str(i).split(".")[0]
            self.create_text(2,y,anchor="nw", text=linenum, font=("Courier New", 12))
            i = self.textwidget.index("%s+1line" % i)

class CustomText(Text):
    # text widget with event proxy
    def __init__(self, *args, **kwargs):
        Text.__init__(self, *args, **kwargs)

        # create a proxy for the underlying widget
        self._orig = self._w + "_orig"
        self.tk.call("rename", self._w, self._orig)
        self.tk.createcommand(self._w, self._proxy)

    def _proxy(self, *args):
        # let the actual widget perform the requested action
        cmd = (self._orig,) + args
        result = self.tk.call(cmd)

        # generate an event if something was added or deleted,
        # or the cursor position changed
        if (args[0] in ("insert", "replace", "delete") or 
            args[0:3] == ("mark", "set", "insert") or
            args[0:2] == ("xview", "moveto") or
            args[0:2] == ("xview", "scroll") or
            args[0:2] == ("yview", "moveto") or
            args[0:2] == ("yview", "scroll")
        ):
            self.event_generate("<<Change>>", when="tail")

        # return what the actual widget returned
        return result      

class tkCppEditorNode:
    def __init__(self):
        self.root = Tk()
        self.root.title("TK Editor")
        self.root.geometry("1200x800")
        self.root.resizable(True, True)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.frame = Frame(self.root, bd=2, relief=SUNKEN)
        menu_bar = Menu(self.frame)

        self.file_name = None

        # Adding the File Menu and its components to create Python Text Editor
        file_menu = Menu(menu_bar, tearoff=False, activebackground='DodgerBlue')

        file_menu.add_command(label="New", command=self.open_new_file)
        file_menu.add_command(label="Open File", command=self.open_file)
        file_menu.add_command(label="Save As", command=self.save_file)
        file_menu.add_command(label='Test', command=self.run_test)
        file_menu.add_separator()
        file_menu.add_command(label="Close File", command=self.exit_application)

        menu_bar.add_cascade(label="File", menu=file_menu)

        # Adding the Edit Menu and its components
        edit_menu = Menu(menu_bar, tearoff=False, activebackground='DodgerBlue')

        edit_menu.add_command(label='Copy', command=self.copy_text)
        edit_menu.add_command(label='Cut', command=self.cut_text)
        edit_menu.add_command(label='Paste', command=self.paste_text)
        edit_menu.add_separator()
        edit_menu.add_command(label='Select All', command=self.select_all)
        edit_menu.add_command(label='Delete', command=self.delete_last_char)

        menu_bar.add_cascade(label="Edit", menu=edit_menu)

        # Adding the Help Menu and its components
        help_menu = Menu(menu_bar, tearoff=False, activebackground='DodgerBlue')

        help_menu.add_command(label='About Notepad', command=self.about_notepad)
        help_menu.add_command(label='About Commands', command=self.about_commands)

        menu_bar.add_cascade(label="Help", menu=help_menu)

        self.root.config(menu=menu_bar)

        # Setting the basic components of the window
        # self.text_area.grid(sticky=NSEW)

        scroller = Scrollbar(self.frame, orient=VERTICAL)
        scroller.pack(side=RIGHT, fill=Y)

        self.text_area = CustomText(self.frame, font=("Courier New", 12), undo=True)
        # self.text_area.bind("<KeyPress>", self.keypress_cb)
        self.linenumbers = TextLineNumbers(self.frame, width=60)
        self.linenumbers.attach(self.text_area)
        self.linenumbers.pack(side=LEFT, fill=Y)
        self.text_area.pack(side=RIGHT, fill=BOTH, expand=1)

        scroller.config(command=self.text_area.yview)
        self.text_area.config(yscrollcommand=scroller.set)

        self.text_area.bind("<<Change>>", self._on_change)
        self.text_area.bind("<Configure>", self._on_change)

        self.frame.pack(side=TOP, fill=BOTH, expand=True)

        # set up ros bindings
        rospy.init_node('cpp_editor_node')
        self.code_pub = rospy.Publisher('cpp_editor_node/text', String, queue_size=1)
        self.test_pub = rospy.Publisher('cpp_editor_node/test', Error, queue_size=1)

        self.run()

    def _on_change(self, event=None):
        self.linenumbers.redraw()
        if event is not None:
            self.keypress_cb(event)

    def run(self):
        self._open_file("/home/kaleb/code/ros_ws/src/robo_copilot/assets/simple_game.cpp")

        self.root.update()
        self.root.mainloop()
        self.root.quit()

    def compile(self, cpp_file, binary_file):
        res = subprocess.run(["g++", "-g3", cpp_file, "-o", binary_file], check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(res.stdout.decode('utf-8'))
        if res.returncode != 0:
            msg = Error()
            msg.type = msg.COMPILE
            msg.stderr = res.stderr.decode('utf-8')
            msg.stdout = ""

            print(msg.stderr)
            self.test_pub.publish(msg)
            return False
        return True

    def get_code(self):
        return self.text_area.get(1.0, END)

    def keypress_cb(self, key):
        self.current_text = self.text_area.get(1.0, END)
        self.code_pub.publish(self.current_text)

    def run_test(self):
        self.save_file()
        # compile the file
        cpp_file = self.file_name
        binary_file = os.path.join(
            os.path.dirname(self.file_name), "tmp.out"
        )
        res = self.compile(cpp_file, binary_file)
        if not res:
            return

        # run via gdb
        gdbmi = GdbController()
        response = gdbmi.write('-file-exec-and-symbols ' + binary_file)
        response = gdbmi.write('-exec-run', raise_error_on_timeout=True, timeout=5)
        gdbmi.exit()

        pprint(response)
        msg = Error()
        stdout_str = ""
        stderr_str = ""

        # parse gdbmi output
        for item in response:
            if item["message"] == "thread-group-exited":
                # check exit code
                if item["payload"]["exit-code"] == '0':
                    msg.type = msg.SUCCESS
                else:
                    msg.type = msg.RUNTIME
            elif item["message"] is None and item["stream"] == "stdout":
                stdout_str += item["payload"]
            elif item["message"] is None and item["stream"] == "stderr":
                stderr_str += item["payload"]
            elif item["message"] == "stopped":
                msg.payload = str(item["payload"])
                # sometimes the output doesn't have a thread-group-exited message
                # with error code for whatever reason, so this works as a backup check
                if item["payload"]["reason"] == "exited-normally":
                    msg.type = msg.SUCCESS
                else:
                    msg.type = msg.RUNTIME

        # publish & cleanup
        msg.stdout = stdout_str
        print(stdout_str)
        self.test_pub.publish(msg)
        os.remove(binary_file)

    def _open_file(self, filename):
        with open(filename, "r") as file:
            self.text_area.insert(1.0, file.read())
            file.close()
        self.file_name = filename

    # Creating all the functions of all the buttons in the NotePad
    def open_file(self):
        file = fd.askopenfilename(defaultextension='.cpp', filetypes=[('All Files', '*.*'), ("C++", "*.cpp"), ("Text File", "*.txt*")])

        if file:
            print(file)
            self.root.title(f"{os.path.basename(file)}")
            self.text_area.delete(1.0, END)
            self._open_file(file)
        else:
            file = None

    def open_new_file(self):
        self.root.title("Untitled - Notepad")
        self.text_area.delete(1.0, END)

    def save_file_as(self):
        file = fd.asksaveasfilename(initialfile='Untitled', defaultextension='.cpp',
                                    filetypes=[('All Files', '*'), ('C++', '.cpp')])
        f = open(file, "w")
        f.write(self.text_area.get(1.0, END))
        f.close()
        self.file_name = file
        self.root.title(f"{os.path.basename(file)} - Editor")
        print("Saved to ", self.file_name)

    def save_file(self):
        if self.file_name is None:
            self.save_file_as()
        else:
            f = open(self.file_name, "w")
            f.write(self.text_area.get(1.0, END))
            f.close()

    def exit_application(self):
        self.root.destroy()

    def copy_text(self):
        self.text_area.event_generate("<<Copy>>")

    def cut_text(self):
        self.text_area.event_generate("<<Cut>>")

    def paste_text(self):
        self.text_area.event_generate("<<Paste>>")

    def select_all(self):
        self.text_area.event_generate("<<Control-Keypress-A>>")

    def delete_last_char(self):
        self.text_area.event_generate("<<KP_Delete>>")

    def about_notepad(self):
        mb.showinfo("About Notepad", "This is just another Notepad, but this is better than all others")

    def about_commands(self):
        commands = """
            Under the File Menu:
            - 'New' clears the entire Text Area
            - 'Open' clears text and opens another file
            - 'Save As' saves your file in the same / another extension

            Under the Edit Menu:
            - 'Copy' copies the selected text to your clipboard
            - 'Cut' cuts the selected text and removes it from the text area
            - 'Paste' pastes the copied/cut text
            - 'Select All' selects the entire text
            - 'Delete' deletes the last character 
            """

        mb.showinfo(title="All commands", message=commands, width=60, height=40)

if __name__ == "__main__":
    tkCppEditorNode()