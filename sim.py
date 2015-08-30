import os

from machinekit import hal
from machinekit import rtapi as rt
from machinekit import config as c

from fdm.config import velocity_extrusion as ve
import hardware
from fdm.config import base
from fdm.config import storage
from fdm.config import motion


rt.init_RTAPI()
c.load_ini(os.environ['INI_FILE_NAME'])

motion.setup_motion()
storage.init_storage('storage.ini')

# Gantry component for Z Axis
base.init_gantry(axisIndex=2)

base.gantry_read(gantryAxis=2, thread='servo-thread')
hal.addf('motion-command-handler', 'servo-thread')

# setup axis feedback
limits = [
    [c.find('AXIS_0', 'MAX_LIMIT') - 0.2, c.find('AXIS_0', 'MAX_LIMIT')],
    [c.find('AXIS_1', 'MAX_LIMIT') - 0.2, c.find('AXIS_1', 'MAX_LIMIT')],
    [c.find('AXIS_2', 'MIN_LIMIT'), 0.0]
]

for n in range(c.find('TRAJ', 'AXES')):
    axisPos = hal.newsig('axis-%i-pos' % n, hal.HAL_FLOAT)
    hal.Pin('axis.%i.motor-pos-cmd' % n).link(axisPos)
    hal.Pin('axis.%i.motor-pos-fb' % n).link(axisPos)

    # fake limits
    limitHome = hal.newsig('limit-%i-home' % n, hal.HAL_BIT)
    hal.Pin('axis.%i.home-sw-in' % n).link(limitHome)
    wcomp = rt.newinst('wcomp', 'wcomp.axis-%i-home' % n)
    hal.addf(wcomp.name, 'servo-thread')
    wcomp.pin('in').link(axisPos)
    wcomp.pin('out').link(limitHome)
    wcomp.pin('min').set(limits[n][0])
    wcomp.pin('max').set(limits[n][1])

numFans = int(c.find('FDM', 'NUM_FANS'))
numExtruders = int(c.find('FDM', 'NUM_EXTRUDERS'))
numLights = int(c.find('FDM', 'NUM_LIGHTS'))

ve.velocity_extrusion(extruders=numExtruders, thread='servo-thread')

# Fans
for i in range(0, numFans):
    base.setup_fan('f%i' % i, thread='servo-thread')
for i in range(0, numExtruders):
    hardware.setup_exp('exp%i' % i)

# Temperature Signals
base.create_temperature_control(name='hbp', section='HBP',
                                hardwareOkSignal='temp-hw-ok',
                                thread='servo-thread')
# setup feedback
lowpass = rt.newinst('lowpass', 'lowpass.hbp-temp-meas')
hal.addf(lowpass.name, 'servo-thread')
lowpass.pin('in').link('hbp-temp-set')
lowpass.pin('out').link('hbp-temp-meas')
lowpass.pin('gain').set(0.001)
hal.Signal('hbp-temp-set').set(5.0)

for i in range(0, numExtruders):
    base.create_temperature_control(name='e%i' % i, section='EXTRUDER_%i' % i,
                                    coolingFan='f%i' % i, hotendFan='exp%i' % i,
                                    hardwareOkSignal='temp-hw-ok',
                                    thread='servo-thread')
    # setup feedback
    lowpass = rt.newinst('lowpass', 'lowpass.e%i-temp-meas' % i)
    hal.addf(lowpass.name, 'servo-thread')
    lowpass.pin('in').link('e%i-temp-set' % i)
    lowpass.pin('out').link('e%i-temp-meas' % i)
    lowpass.pin('gain').set(0.001)
    hal.Signal('e%i-temp-set' % i).set(5.0)

# temp hw
hal.Signal('temp-hw-ok').set(True)

# Extruder Multiplexer
base.setup_extruder_multiplexer(extruders=numExtruders, thread='servo-thread')
# Stepper Multiplexer
multiplexSections = []
for i in range(0, numExtruders):
    multiplexSections.append('EXTRUDER_%i' % i)
base.setup_stepper_multiplexer(stepgenIndex=4, sections=multiplexSections,
                               selSignal='extruder-sel', thread='servo-thread')

# LEDs
for i in range(0, numLights):
    base.setup_light('l%i' % i, thread='servo-thread')
# HB LED
hardware.setup_hbp_led(thread='servo-thread')

# Standard I/O - EStop, Enables, Limit Switches, Etc
#errorSignals = ['gpio-hw-error', 'pwm-hw-error', 'temp-hw-error',
#                'watchdog-error', 'hbp-error']
#for i in range(0, numExtruders):
#    errorSignals.append('e%i-error' % i)
#base.setup_estop(errorSignals, thread='servo-thread')
base.setup_estop_loopback()
base.setup_tool_loopback()
# Probe
base.setup_probe(thread='servo-thread')

hal.addf('motion-controller', 'servo-thread')
base.gantry_write(gantryAxis=2, thread='servo-thread')

# Storage
storage.read_storage()

# start haltalk server after everything is initialized
# else binding the remote components on the UI might fail
hal.loadusr('haltalk', wait=True)

