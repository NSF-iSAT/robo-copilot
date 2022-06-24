# Writing a C++ Editor, Compiler, & Tester in Python
There are lots of great C++ development environments out there. But for me, because I'm studying how robots can guide users through the debugging process, I specifically needed an interactive C++ editor and testbed that can communicate with my robot's backend. This way the robot can be aware of what edits the user is making, what error messages they get, when they successfully compile (or not!), and so on. So, below I've detailed my process of creating this little app.

### 0. Software prerequisites
* [gdb](https://sourceware.org/gdb/) -- a debugger program for C/C++
* [g++](https://courses.cs.washington.edu/courses/cse373/99au/unix/g++.html) -- my C++ compiler of choice
* [tkinter](https://docs.python.org/3/library/tkinter.html) -- a python library for GUI apps
* [pygdbmi](https://pypi.org/project/pygdbmi/) -- a python library for controlling gdb

### 1. Text Editor
The first step is to create the text editor window, where users can open files, edit code, and save them. I based the bulk of my starter code off of [this tutorial](https://www.studytonight.com/tkinter/text-editor-application-using-tkinter), but things got a bit more complicated when I wanted to add line numbers to the text window. I used the `TextLineNumbers` and `CustomText` classes from [this newbedev post](https://newbedev.com/tkinter-adding-line-number-to-text-widget):

	from tkinter import *
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

Next, I put it all together to make a class for the editor window:

	import os
	class EditorWindow:
        """ Module to draw the code editor window, with line numbers.
            Inputs:
                root: the tkinter root (a Tk() instance)
                edit_cb : a function be triggered when user makes changes.
         """
		def __init__(self, root, edit_cb):
			self.external_bindings = {}

			self.root = root
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

			self.root.config(menu=menu_bar)

			scroller = Scrollbar(self.frame, orient=VERTICAL)
			scroller.pack(side=RIGHT, fill=Y)

			self.text_area = CustomText(self.frame, font=("Courier New", 12), undo=True)
			self.linenumbers = TextLineNumbers(self.frame, width=60)
			self.linenumbers.attach(self.text_area)
			self.linenumbers.pack(side=LEFT, fill=Y)
			self.text_area.pack(side=RIGHT, fill=BOTH, expand=1)

			scroller.config(command=self.text_area.yview)
			self.text_area.config(yscrollcommand=scroller.set)

			self.text_area.bind("<KeyPress>", edit_cb)
			self.text_area.bind("<<Change>>", self._on_change)
			self.text_area.bind("<Configure>", self._on_change)

			self.frame.pack(side=TOP, fill=BOTH, expand=True)

		def _on_change(self, event=None):
			self.linenumbers.redraw()

		def get_contents(self):
			return self.text_area.get(1.0, END)

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
			self.root.title("Untitled - Editor")
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

If you initialize tkinter and run this in a `mainloop()`, you get the following result:

![Editor window image](/docs/assets/editor_window.png)

### 2. Output Window
In addition to the editor window, I wanted my program to launch a separate window where any messages from the robot's backend, like error messages from the C++ code, could be displayed. I also wanted to add a **Test** button that would test the code (once I had implemented compiling & testing!). Lastly, I wanted a text entry box where the user could input text that would (again, once I had done the necessary connectivity) communicate to their program.

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

			self.user_input = StringVar()
			self.last_input = None
			self.entry_field = Entry(bottomframe, font=("Courier New", 12), textvariable = self.user_input)
			self.entry_field.bind("<Return>", self._get_input)
			self.entry_field.pack(side=LEFT, fill=X)

			frame.pack(side=TOP, fill=BOTH, expand=True)
			bottomframe.pack(side=BOTTOM, fill=X)

		def _get_input(self, event=None):
			user_input = self.user_input.get()
			self.entry_field.delete(0, END)
			self.place_text(user_input)
			if not user_input:
				self.last_input = None
			else:
				self.last_input = user_input

		def get_input(self):
			line = self.last_input
			self.last_input = None
			return line

		def place_text(self, new_text):
			# self.test_count += 1
			prev_yview = self.text.yview()

			self.text["state"] = NORMAL
			self.text.insert(END, new_text + "\n")
			self.text["state"] = DISABLED

			self.text.yview_moveto(prev_yview[1])

Note that I'm not doing anything with the input text from the Entry box yet -- just storing it in an attribute, where it can be retrieved later via `get_input`. Once I got the above running and displayed some sample text, it looks like this:

![Output window image](/docs/assets/output_window.png)

### 3. Compiling & Testing
Next, I want the "Test" button to actually compile the C++ code so it can be run by the test code I'll implement later. The compilation itself is as simple as calling g++ via `subprocess`:
	import subprocess
	def compile(cpp_file, binary_file):
		""" Compiles the cpp file to an executable of name binary_file.
			Inputs:
				cpp_file: C++ filename to be compiled.
				binary_file: desired name of compiled binary file produced.
			@return: "(True, "")" if compiled successfully
					 else, (False, <error>) where <error> is the stderr message returned by g++
		"""
		res = subprocess.run(["g++", "-g3", cpp_file, "-o", binary_file], check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		print(res.stdout.decode('utf-8'))
		if res.returncode != 0:
			return (False, res.stderr.decode('utf-8'))
		return (True, "")

Running the compiled program can be more difficult. I started by using a fairly simple call to `pygdbmi`'s GdbController.
	from pygdbmi import GdbController
	gdbmi = GdbController()
	response = gdbmi.write('-file-exec-and-symbols ' + binary_file)
	response = gdbmi.write('-exec-run')
While this gives me the output that I'll need to update the output window display, I also want to be able to communicate with the program while it's running via text entry. In addition, I want to parse the `response` object (a list of dicts) to separate GDB's error messages from the program's intended output.

To implement all this, I wrote a class that launches the Editor and Output windows, and implements compilation, testing, and interactivity with programs. The `run_test` function is passed into the Output window to be triggered when the user presses "Test".

	class CppEditorNode:
		def __init__(self):            
			tk     = Tk()
			self.editor = EditorWindow(tk, self.edit_cb)
			self.output = OutputWindow(Toplevel(), self.run_test)
			self.test_count = 0

			self.editor._open_file("/home/kaleb/code/ros_ws/src/robo_copilot/assets/test1.cpp")

			self.in_debug = False
			self.gdbmi    = None

			while True:
				try:
					if self.in_debug:
						self.monitor_test(self.gdbmi)
					self.tk.update()
				except:
					break
			tk.quit()

		def compile(self, cpp_file, binary_file):
			res = subprocess.run(["g++", "-g3", cpp_file, "-o", binary_file], check=False, stdout=subprocess.	PIPE, stderr=subprocess.PIPE)

			if res.returncode != 0:
				return (False, res.stderr.decode('utf-8'))
			return (True, "")

		def run_test(self):
			if self.in_debug:
				return
			self.editor.save_file()
			self.test_count += 1
			
			cpp_file = self.editor.file_name
			binary_file = os.path.join(
				os.path.dirname(self.editor.file_name), "tmp.out"
			)
			
			result, err = self.compile(cpp_file, binary_file)
			if not result:
				# failed to compile
				msg.type = msg.COMPILE
				msg.stderr = err
				msg.stdout = ""
				self.test_pub.publish(msg)
				self.output.place_text(err)
				return

			# compiled successfully, now run via gdb
			self.gdbmi = gdbmi = GdbController()
			response = gdbmi.write('-file-exec-and-symbols ' + binary_file)
			response = gdbmi.write('-exec-run')\
			
			if self.process_gdb_response(response):
				self.in_debug = True
			else:
				self.in_debug = False
				self.gdbmi = None
				gdbmi.exit()

		def process_gdb_response(self, response):
			""" Parse and appropriately output gdb data from param response.
				@return: True if gdb process is still running, else False
			"""
			msg = Error()
			program_output = ""
			gdb_output = ""
			for item in response:
				if item["type"] == "output":
					program_output += item["payload"]
					self.output.place_text(item["payload"])

				elif item["type"] == "console":
					gdb_output += item["payload"]
					self.output.place_text("\nDEBUG: " + item["payload"])

				elif item["message"] == "stopped":
					if item["payload"]["reason"] == "exited-normally":
						msg.type = msg.SUCCESS
					else:
						msg.type = msg.RUNTIME
						msg.payload = item["payload"]
					return False
			# program is presumably waiting for user input
			return True
			
		def monitor_test(self, gdbmi):
			# check for input
			user_input = self.output.get_input()
			if user_input is None:
				return
			response = gdbmi.write(user_input, raise_error_on_timeout=False)
			# parse new response, and see if program is still running
			self.in_debug = self.process_gdb_response(response)
		