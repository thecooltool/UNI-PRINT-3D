#!/usr/bin/python

import sys
import os
import subprocess
from time import *
from machinekit import launcher
from machinekit import config

launcher.register_exit_handler()
#launcher.set_debug_level(5)
os.chdir(os.path.dirname(os.path.realpath(__file__)))
c = config.Config()
os.environ["MACHINEKIT_INI"] = c.MACHINEKIT_INI

try:
    launcher.check_installation()
    launcher.cleanup_session()
    launcher.load_bbio_file('paralell_cape3.bbio')
    launcher.install_comp('thermistor_check.icomp')
    launcher.install_comp('led_dim.icomp')
    launcher.install_comp('logic_fuse.icomp')
    launcher.start_process("configserver -n Uni-print-3D ~/Machineface")
    launcher.start_process('linuxcnc UNIPRINT-3D_P1.ini')
except subprocess.CalledProcessError:
    launcher.end_session()
    sys.exit(1)

while True:
    sleep(1)
    launcher.check_processes()
