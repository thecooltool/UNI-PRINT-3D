#!/usr/bin/python

import sys
import os
import subprocess
import argparse
from time import *
from machinekit import launcher
from machinekit import config

if sys.version_info >= (3, 0):
    import configparser
else:
    import ConfigParser as configparser


parser = argparse.ArgumentParser(description='starts the Uni-Print-3D confgurations')
parser.add_argument('-e', '--num_extruders', help='number of extruders', nargs='?', type=int, default=1)
parser.add_argument('-a', '--with_abp', help='enable a automated build platform', action='store_true')
parser.add_argument('config', nargs=1, help='path to config file')

args = parser.parse_args()

configName = args.config[0]
numExtruders = args.num_extruders
withAbp = args.with_abp

launcher.register_exit_handler()
#launcher.set_debug_level(5)
os.chdir(os.path.dirname(os.path.realpath(__file__)))
launcher.set_machinekit_ini(config.MACHINEKIT_INI)

if not os.path.isfile(configName):
    sys.stderr.write('Config file %s does not exist' % configName)
    sys.exit(1)

startupIniName = 'startup.ini'
sourceIni = open(configName)  # open ini
lines = sourceIni.readlines()
sourceIni.close()
lines.append('NUM_EXTRUDERS = %i\n' % numExtruders)
lines.append('NUM_FANS = %i\n' % numExtruders)
if withAbp:
    lines.append('ABP = 1\n')
startupIni = open(startupIniName, 'w')
startupIni.writelines(lines)  # copy file contents
startupIni.close()

try:
    launcher.check_installation()
    launcher.cleanup_session()
    launcher.load_bbio_file('paralell_cape3.bbio')
    launcher.install_comp('thermistor_check.icomp')
    cfg = configparser.ConfigParser({'NAME': ''})
    cfg.read(startupIniName)
    machineName = cfg.get('EMC', 'NAME')
    command = 'configserver'
    if machineName is not '':
        command += ' -n %s' % machineName
    command += ' ~/Machineface'
    launcher.start_process(command)
    if os.path.exists('/dev/video0'):  # automatically start videoserver
        launcher.start_process('videoserver -i video.ini Webcam1')
    launcher.start_process('linuxcnc %s' % startupIniName)
    while True:
        launcher.check_processes()
        sleep(1)
except subprocess.CalledProcessError:
    launcher.end_session()
    sys.exit(1)

sys.exit(0)
