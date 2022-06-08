from tkinter import *
import tkinter.filedialog as fd
import tkinter.messagebox as mb
import subprocess
import os
import sys
from pygdbmi.gdbcontroller import GdbController
from distutils.spawn import find_executable
from pprint import pprint

class tkCppEditor:
    def __init__(self):
        self.root = Tk()
        self.root.title("TK Editor")
        self.root.geometry("800x600")
        self.root.resizable(True, True)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        menu_bar = Menu(self.root)

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
        self.text_area = Text(self.root, font=("Times New Roman", 14))
        self.text_area.grid(sticky=NSEW)

        scroller = Scrollbar(self.text_area, orient=VERTICAL)
        scroller.pack(side=RIGHT, fill=Y)

        scroller.config(command=self.text_area.yview)
        self.text_area.config(yscrollcommand=scroller.set)
        
        self.text_area.bind("<KeyPress>", self.keypress_cb)

        self.current_text = ""
        self.root.update()
        self.root.mainloop()

    def compile(self, cpp_file, binary_file):
        res = subprocess.run(["g++", "-g3", cpp_file, "-o", binary_file], check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(res.stdout)
        if res.returncode != 0:
            print(res.stderr)
            return False
        return True

    def keypress_cb(self, key):
        self.current_text = self.text_area.get(1.0, END)
        # TODO
        pass

    def run_test(self):
        self.save_file()
        print("Saved to ", self.file_name)
        # compile the file
        cpp_file = self.file_name
        binary_file = os.path.join(
            os.path.dirname(self.file_name), "tmp.out"
        )
        res = self.compile(cpp_file, binary_file)
        if not res:
            return

        gdbmi = GdbController()
        response = gdbmi.write('-file-exec-file ' + binary_file)
        pprint(response)
        os.remove(binary_file)
        # TODO

    # Creating all the functions of all the buttons in the NotePad
    def open_file(self):
        file = fd.askopenfilename(defaultextension='.cpp', filetypes=[('All Files', '*.*'), ("C++", "*.cpp"), ("Text File", "*.txt*")])

        if file != '':
            self.root.title(f"{os.path.basename(file)}")
            self.text_area.delete(1.0, END)
            with open(file, "r") as file_:
                self.text_area.insert(1.0, file_.read())
                file_.close()
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
    tkCppEditor()