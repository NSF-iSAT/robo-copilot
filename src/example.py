#!/usr/bin/env python

"""
Run with `python -m example`
"""
import subprocess
import os
import sys
from pygdbmi.gdbcontroller import GdbController
from distutils.spawn import find_executable

def compile():
    cpp_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "linked_list.cpp"
    )
    binary_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "tmp.out"
    )
    subprocess.check_output(["g++", "-g3", cpp_file, "-o", binary_file])
    print("success!")
    # os.remove(binary_file)

if __name__ == "__main__":
    compile()