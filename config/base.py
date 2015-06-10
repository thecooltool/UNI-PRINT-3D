from machinekit import hal
from machinekit import rtapi as rt
from machinekit import config as c

import rcomps
import storage
import motion


def setup_motion():
    rt.loadrt('trivkins')
    rt.loadrt('tp')

    # motion controller, get name and thread periods from ini file
    rt.loadrt(c.find('EMCMOT', 'EMCMOT'),
              servo_period_nsec=c.find('EMCMOT', 'SERVO_PERIOD'),
              num_joints=c.find('TRAJ', 'AXES'),
              num_aio=51,
              num_dio=21)


def usrcomp_status(compname, signame, thread, resetSignal='estop-reset'):
    sigIn = hal.newsig('%s-error-in' % signame, hal.HAL_BIT)
    sigOut = hal.newsig('%s-error' % signame, hal.HAL_BIT)
    sigOk = hal.newsig('%s-ok' % signame, hal.HAL_BIT)

    sigIn.link('%s.error' % compname)

    logicFuse = rt.newinst('logic_fuse', 'logic-fuse.%s-error' % signame)
    hal.addf(logicFuse.name, thread)
    logicFuse.pin('in').link(sigIn)
    logicFuse.pin('out').link(sigOut)
    logicFuse.pin('reset').link(resetSignal)
    logicFuse.pin('wait').set(0.5)  # TODO replace by better component

    notComp = rt.newinst('not', 'not.%s-no-error' % signame)
    hal.addf(notComp.name, thread)
    notComp.pin('in').link(sigOut)
    notComp.pin('out').link(sigOk)


def usrcomp_watchdog(comps, enableSignal, thread,
                     okSignal=None, errorSignal=None):
    count = len(comps)
    watchdog = rt.loadrt('watchdog', num_inputs=count)
    hal.addf('watchdog.set-timeouts', thread)
    hal.addf('watchdog.process', thread)
    for n, comp in enumerate(comps):
        compname = comp[0]
        comptime = comp[1]
        sigIn = hal.newsig('%s-watchdog' % compname, hal.HAL_BIT)
        hal.Pin('%s.watchdog' % compname).link(sigIn)
        watchdog.pin('input-%i' % n).link(sigIn)
        watchdog.pin('timeout-%i' % n).set(comptime)
    watchdog.pin('enable-in').link(enableSignal)

    if not okSignal:
        okSignal = hal.newsig('watchdog-ok', hal.HAL_BIT)
    watchdog.pin('ok-out').link(okSignal)

    if errorSignal:
        notComp = rt.newinst('not', 'not.watchdog-error')
        hal.addf(notComp.name, thread)
        notComp.pin('in').link(okSignal)
        notComp.pin('out').link(errorSignal)


def setup_stepper(stepgenIndex, section, axisIndex=None,
                  stepgenType='hpg.stepgen', gantry=False,
                  gantryJoint=0, velocitySignal=None, thread=None):
    stepgen = '%s.%02i' % (stepgenType, stepgenIndex)
    axis = 'axis.%i' % axisIndex
    hasMotionAxis = axisIndex and (not gantry or gantryJoint == 0)
    velocityControlled = velocitySignal is not None

    # axis enable chain
    enableIndex = axisIndex
    if not axisIndex:
        enableIndex = 0  # use motor enable signal
    enable = hal.Signal('emcmot-%i-enable' % enableIndex, hal.HAL_BIT)
    if hasMotionAxis:
        enable.link('%s.amp-enable-out' % axis)
    enable.link('%s.enable' % stepgen)

    # expose timing parameters so we can multiplex them later
    sigBase = 'stepgen-%i' % stepgenIndex
    dirsetup = hal.newsig('%s-dirsetup' % sigBase, hal.HAL_U32, init=c.find(section, 'DIRSETUP'))
    dirhold = hal.newsig('%s-dirhold' % sigBase, hal.HAL_U32, init=c.find(section, 'DIRHOLD'))
    steplen = hal.newsig('%s-steplen' % sigBase, hal.HAL_U32, init=c.find(section, 'STEPLEN'))
    stepspace = hal.newsig('%s-stepspace' % sigBase, hal.HAL_U32, init=c.find(section, 'STEPSPACE'))
    scale = hal.newsig('%s-scale' % sigBase, hal.HAL_FLOAT, init=c.find(section, 'SCALE'))
    maxVel = hal.newsig('%s-max-vel' % sigBase, hal.HAL_FLOAT, init=c.find(section, 'STEPGEN_MAX_VEL'))
    maxAcc = hal.newsig('%s-max-acc' % sigBase, hal.HAL_FLOAT, init=c.find(section, 'STEPGEN_MAX_ACC'))
    controlType = hal.newsig('%s-control-type' % sigBase, hal.HAL_FLOAT, init=0)

    hal.Pin('%s.dirsetup' % stepgen).link(dirsetup)
    hal.Pin('%s.dirhold' % stepgen).link(dirhold)

    hal.Pin('%s.steplen' % stepgen).link(steplen)
    hal.Pin('%s.stepspace' % stepgen).link(stepspace)

    hal.Pin('%s.position-scale' % stepgen).link(scale)

    hal.Pin('%s.maxvel' % stepgen).link(maxVel)
    hal.Pin('%s.maxaccel' % stepgen).link(maxAcc)

    hal.Pin('%s.control-type' % stepgen).link(controlType)

    # position command and feedback
    if not velocityControlled:
        if hasMotionAxis:  # per axis fb and cmd
            posCmd = hal.newsig('emcmot-%i-pos-cmd' % axisIndex, hal.HAL_FLOAT)
            posCmd.link('%s.motor-pos-cmd' % axis)
            if not gantry:
                posCmd.link('%s.position-cmd' % stepgen)
            else:
                posCmd.link('gantry.%i.position-cmd' % axisIndex)

            posFb = hal.newsig('emcmot-%i-pos-fb' % axisIndex, hal.HAL_FLOAT)
            if not gantry:
                posFb.link('%s.position-fb' % stepgen)
            else:
                posFb.link('gantry.%i.position-fb' % axisIndex)
                posFb.link('%s.motor-pos-fb' % axis)

        if gantry:  # per joint fb and cmd
            posCmd = hal.newsig('emcmot-%i-%i-pos-cmd' % (axisIndex, gantryJoint), hal.HAL_FLOAT)
            posCmd.link('gantry.%i.joint.%02i.pos-cmd' % (axisIndex, gantryJoint))
            posCmd.link('%s.position-cmd' % stepgen)

            posFb = hal.newsig('emcmot-%i-%i-pos-fb' % (axisIndex, gantryJoint), hal.HAL_FLOAT)
            posFb.link('%s.position-fb' % stepgen)
            posFb.link('gantry.%i.joint.%02i.pos-fb' % (axisIndex, gantryJoint))
    else:  # velocity control
        hal.net(velocitySignal, '%s.velocity-cmd' % stepgen)
        controlType.set(1)  # enable velocity control

    # limits
    if hasMotionAxis:
        limitHome = hal.newsig('limit-%i-home' % axisIndex, hal.HAL_BIT)
        limitMin = hal.newsig('limit-%i-min' % axisIndex, hal.HAL_BIT)
        limitMax = hal.newsig('limit-%i-max' % axisIndex, hal.HAL_BIT)
        limitHome.link('%s.home-sw-in' % axis)
        limitMin.link('%s.neg-lim-sw-in' % axis)
        limitMax.link('%s.pos-lim-sw-in' % axis)

    if gantry:
        if gantryJoint == 0:
            hal.Pin('gantry.%i.search-vel' % axisIndex).set(c.find(section, 'HOME_SEARCH_VEL'))
            hal.Pin('gantry.%i.homing' % axisIndex).link('%s.homing' % axis)
            hal.Pin('gantry.%i.home' % axisIndex).link(limitHome)

            or2 = rt.newinst('or2', 'or2.limit-%i-min' % axisIndex)
            hal.addf(or2.name, thread)
            or2.pin('out').link(limitMin)

            or2 = rt.newinst('or2', 'or2.limit-%i-max' % axisIndex)
            hal.addf(or2.name, thread)
            or2.pin('out').link(limitMax)

        limitHome = hal.newsig('limit-%i-%i-home' % (axisIndex, gantryJoint),
                               hal.HAL_BIT)
        limitMin = hal.newsig('limit-%i-%i-min' % (axisIndex, gantryJoint),
                              hal.HAL_BIT)
        limitMax = hal.newsig('limit-%i-%i-max' % (axisIndex, gantryJoint),
                              hal.HAL_BIT)
        homeOffset = hal.newsig('home-offset-%i-%i' % (axisIndex, gantryJoint),
                                hal.HAL_BIT)
        limitHome.link('gantry.%i.joint.%02i.home' % (axisIndex, gantryJoint))
        limitMin.link('or2.limit-%i-min.in%i' % (axisIndex, gantryJoint))
        limitMax.link('or2.limit-%i-max.in%i' % (axisIndex, gantryJoint))
        homeOffset.link('%s.joint.%02i.home-offset' % (gantry, gantryJoint))

        storage.setup_gantry_storage(axisIndex, gantryJoint)

    # stepper pins configured in hardware setup


def setup_probe(thread):
    probeEnable = hal.newsig('probe-enable', hal.HAL_BIT)
    probeInput = hal.newsig('probe-input', hal.HAL_BIT)
    probeSignal = hal.newsig('probe-signal', hal.HAL_BIT)

    and2 = rt.newinst('and2', 'and2.probe-input')
    hal.addf(and2.name, thread)
    and2.pin('in0').link(probeSignal)
    and2.pin('in1').link(probeEnable)
    and2.pin('out').link(probeInput)

    probeInput += 'motion.probe-input'

    motion.setup_probe_io()


def create_temperature_control(name, section, thread, hardwareOkSignal=None,
                               coolingFan=None, hotendFan=None):
    tempSet = hal.newsig('%s-temp-set' % name, hal.HAL_FLOAT)
    tempMeas = hal.newsig('%s-temp-meas' % name, hal.HAL_FLOAT)
    tempInRange = hal.newsig('%s-temp-in-range' % name, hal.HAL_BIT)
    tempPwm = hal.newsig('%s-temp-pwm' % name, hal.HAL_FLOAT)
    tempPwmMax = hal.newsig('%s-temp-pwm-max' % name, hal.HAL_FLOAT)
    tempLimitMin = hal.newsig('%s-temp-limit-min' % name, hal.HAL_FLOAT)
    tempLimitMax = hal.newsig('%s-temp-limit-max' % name, hal.HAL_FLOAT)
    tempStandby = hal.newsig('%s-temp-standby' % name, hal.HAL_FLOAT)
    tempInLimit = hal.newsig('%s-temp-in-limit' % name, hal.HAL_BIT)
    tempThermOk = hal.newsig('%s-temp-therm-ok' % name, hal.HAL_BIT)
    error = hal.newsig('%s-error' % name, hal.HAL_BIT)
    active = hal.newsig('%s-active' % name, hal.HAL_BIT)

    tempPidPgain = hal.newsig('%s-temp-pid-Pgain' % name, hal.HAL_FLOAT)
    tempPidIgain = hal.newsig('%s-temp-pid-Igain' % name, hal.HAL_FLOAT)
    tempPidDgain = hal.newsig('%s-temp-pid-Dgain' % name, hal.HAL_FLOAT)
    tempPidMaxerrorI = hal.newsig('%s-temp-pid-maxerrorI' % name, hal.HAL_FLOAT)
    tempPidOut = hal.newsig('%s-temp-pid-out' % name, hal.HAL_FLOAT)
    tempPidBias = hal.newsig('%s-temp-pid-bias' % name, hal.HAL_FLOAT)
    tempRangeMin = hal.newsig('%s-temp-range-min' % name, hal.HAL_FLOAT)
    tempRangeMax = hal.newsig('%s-temp-range-max' % name, hal.HAL_FLOAT)
    noErrorIn1 = hal.newsig('%s-no-error-in1' % name, hal.HAL_BIT)
    noErrorIn2 = hal.newsig('%s-no-error-in2' % name, hal.HAL_BIT)
    errorIn = hal.newsig('%s-error-in' % name, hal.HAL_BIT)

    # parameters
    tempLimitMin.set(c.find(section, 'TEMP_LIMIT_MIN'))
    tempLimitMax.set(c.find(section, 'TEMP_LIMIT_MAX'))
    tempStandby.set(c.find(section, 'TEMP_STANDBY'))
    tempPwmMax.set(c.find(section, 'PWM_MAX'))
    tempPidBias.set(c.find(section, 'PID_BIAS'))
    tempPidPgain.set(c.find(section, 'PID_PGAIN'))
    tempPidIgain.set(c.find(section, 'PID_IGAIN'))
    tempPidDgain.set(c.find(section, 'PID_DGAIN'))
    tempPidMaxerrorI.set(c.find(section, 'PID_MAXERRORI'))

    # coolingFan compensation
    if coolingFan:
        tempPidFanBias = hal.newsig('%s-temp-pid-fan-bias' % name, hal.HAL_FLOAT)

        scale = rt.newinst('scale', 'scale.%s-temp-pid-fan-bias')
        hal.addf(scale.name, thread)
        scale.pin('in').link('%s.pwm' % coolingFan)
        scale.pin('out').link(tempPidFanBias)
        scale.pin('gain').set(c.find(section, 'FAN_BIAS'))

        sum2 = rt.newinst('sum2', 'sum2.%s-temp-pid-bias')
        hal.addf(sum2.name, thread)
        sum2.pin('in0').set(c.find(section, 'PID_BIAS'))
        sum2.pin('in1').link(tempPidFanBias)
        sum2.pin('out').link(tempPidBias)

    # PID
    pid = rt.newinst('pid', 'pid.%s' % name)
    hal.addf('%s.do-pid-calcs' % pid.name, thread)
    pid.pin('enable').link('emcmot.00.enable')  # motor enable
    pid.pin('feedback').link(tempMeas)
    pid.pin('command').link(tempSet)
    pid.pin('output').link(tempPidOut)
    pid.pin('maxoutput').link(tempPwmMax)
    pid.pin('bias').link(tempPidBias)
    pid.pin('Pgain').link(tempPidPgain)
    pid.pin('Igain').link(tempPidIgain)
    pid.pin('Dgain').link(tempPidDgain)
    pid.pin('maxerrorI').link(tempMaxerrorI)

    # Limit heater PWM to positive values
    # PWM mimics hm2 implementation, which generates output for negative values
    limit1 = rt.newinst('limit1', 'limit.%s-temp-heaterl')
    hal.addf(limit1.name, thread)
    limit1.pin('in').link(tempPidOut)
    limit1.pin('out').link(tempPwm)
    limit1.pin('min').set(0.0)
    limit1.pin('max').link(tempPwmMax)

    # Temperature checking
    sum2 = rt.newinst('sum2', 'sum2.%s-temp-range-pos')
    hal.addf(sum2.name, thread)
    sum2.pin('in0').link(tempSet)
    sum2.pin('in1').set(c.find(section, 'TEMP_RANGE_POS_ERROR'))
    sum2.pin('out').link(tempRangeMin)

    sum2 = rt.newinst('sum2', 'sum2.%s-temp-range-neg')
    hal.addf(sum2.name, thread)
    sum2.pin('in0').link(tempSet)
    sum2.pin('in1').set(c.find(section, 'TEMP_RANGE_NEG_ERROR'))
    sum2.pin('out').link(tempRangeMax)

    #the output of this component will say if measured temperature is in range of set value
    wcomp = rt.newinst('wcomp', 'wcomp.%s-temp-in-range')
    hal.addf(wcomp.name, thread)
    wcomp.pin('min').link(tempRangeMin)
    wcomp.pin('max').link(tempRangeMax)
    wcomp.pin('in').link(tempMeas)
    wcomp.pin('out').link(tempInRange)

    # limit the output temperature to prevent damage when thermistor is broken/removed
    wcomp = rt.newinst('wcomp', 'wcomp.%s-temp-in-limit')
    hal.addf(wcomp.name, thread)
    wcomp.pin('min').link(tempLimitMin)
    wcomp.pin('max').link(tempLimitMax)
    wcomp.pin('in').link(tempMeas)
    wcomp.pin('out').link(tempInLimit)

    # check the thermistor
    # net e0.temp.meas              => thermistor-check.e0.temp
    # net e0.temp.in-range          => not.e0-temp-range.in
    # net e0.temp.in-range_n        <= not.e0-temp-range.out
    # net e0.temp.in-range_n        => thermistor-check.e0.enable
    # net e0.heaterl                => thermistor-check.e0.pid
    # net e0.therm-ok               <= thermistor-check.e0.no-error
    tempThermOk.set(True)  # for now disabled

    # no error chain
    and2 = rt.newinst('and2', 'and2.%s-no-error-in' % name)
    hal.addf(and2.name, thread)
    and2.pin('in0').link(tempThermOk)
    and2.pin('in1').link(tempInLimit)
    and2.pin('out').link(noErrorIn1)

    and2 = rt.newinst('and2', 'and2.%s-no-error-in' % name)
    hal.addf(and2.name, thread)
    and2.pin('in0').link(noErrorIn1)
    if hardwareOkSignal:
        and2.pin('in1').link(hardwareOkSignal)
    else:
        and2.pin('in1').set(True)
    and2.pin('out').link(noErrorIn2)

    notComp = rt.newinst('not', 'not.%s-error-in' % name)
    hal.addf(notComp.name, thread)
    notComp.pin('in').link(noErrorIn2)
    notComp.pin('out').link(errorIn)

    logicFuse = rt.newinst('logic_fuse', 'logic-fuse.%s-error' % name)
    hal.addf(logicFuse.name, thread)
    logicFuse.pin('in').link(errorIn)
    logicFuse.pin('out').link(error)
    logicFuse.pin('wait').set(0.5)  # TODO configure
    logicFuse.pin('reset').link('estop-error')

    # active chain
    comp = rt.newinst('comp', 'comp.%s-active' % name)
    hal.addf(comp.name, thread)
    comp.pin('in0').set(0.0001)
    comp.pin('hyst').set(0.0)
    comp.pin('in1').link(tempPwm)
    comp.pin('out').link(active)

    # Thermistor checking
    # setp thermistor-check.e0.wait 9.0
    # setp thermistor-check.e0.min-pid 1.5 # disable0.25
    # setp thermistor-check.e0.min-temp 1.5
    # net e0.pid.bias => thermistor-check.e0.bias

    # Hotend fan
    if hotendFan:
        comp = rt.newinst('comp', 'comp.%s-pwm-enable' % hotendFan)
        hal.addf(comp.name, thread)
        comp.pin('in0').set(c.find(section, 'HOTEND_FAN_THRESHOLD', 50.0))
        comp.pin('in1').link(tempMeas)
        comp.pin('hyst').set(c.find(section, 'HOTEND_FAN_HYST', 2.0))
        comp.pin('out').link('%s-pwm-enable' % hotendFan)

        hal.Signal('%s-pwm' % hotendFan).set(1.0)

    rcomps.create_temperature_rcomp(name)
    motion.setup_temperature_io(name)


def setup_extruder_multiplexer(extruders, thread):
    extruderSel = hal.newsig('extruder-sel', hal.HAL_S32)

    select8 = rt.newinst('select8', 'select8.extruder-sel')
    hal.addf(select8.name, thread)
    for n in range(0, extruders):
        select8.pin('out%i' % n).link('e%i-enable' % n)
    select8.pin('sel').link(extruderSel)

    extruderSel.link('iocontrol.0.tool-prep-number')  # driven by T code


def setup_light(name, thread):
    for color in ('r', 'g', 'b', 'w'):
        inSig = hal.newsig('%s-%s' % (name, color), hal.HAL_FLOAT)
        outSig = hal.newsig('%s-%s-out' % (name, color), hal.HAL_FLOAT)

        ledDim = rt.newinst('led_dim', 'led-dim.%s-%s' % (name, color))
        hal.addf(ledDim.name, thread)
        ledDim.pin('input').link(inSig)
        ledDim.pin('output').link(outSig)
        ledDim.pin('factor').set(5.0)
        ledDim.pin('steps').set(256)
        ledDim.pin('max-pwm').set(4095)

    rcomps.create_light_rcomp(name)
    storage.setup_light_storage(name)
    motion.setup_light_io(name)


def setup_fan(name, thread):
    setSig = hal.newsig('%s-set' % name, hal.HAL_FLOAT, init=0.0)
    pwmSig = hal.newsig('%s-pwm' % name, hal.HAL_FLOAT)
    hal.newsig('%s-enable' % name, hal.HAL_FLOAT, init=True)

    scale = rt.newcomp('scale', 'scale.%s')
    hal.addf(scale.name, thread)
    scale.pin('in').link(setSig)
    scale.pin('out').link(pwmSig)
    scale.pin('gain').set(1.0 / 255.0)  # 255 steps from motion

    rcomps.create_fan_rcomp(name)
    motion.setup_fan_io(name)


def setup_estop(errorSignals, thread):
    # Create estop signal chain
    estopUser = hal.newsig('estop-user', hal.HAL_BIT)
    estopReset = hal.newsig('estop-reset', hal.HAL_BIT)
    estopOut = hal.newsig('estop-out', hal.HAL_BIT)
    estopIn = hal.newsig('estop-in', hal.HAL_BIT)
    estopError = hal.newsig('estop-error', hal.HAL_BIT)

    num = len(errorSignals)
    orComp = rt.newinst('or', 'or%i.estop-error' % num, pincount=num)
    hal.addf(orComp.name, thread)
    for n, sig in enumerate(errorSignals):
        orComp.pin('in%i' % n).link(sig)
    orComp.link('out').link(estopError)

    estopLatch = rt.newinst('estop_latch', 'estop-latch')
    hal.addf(estopLatch.name, thread)
    estopLatch.pin('ok-in').link(estopUser)
    estopLatch.pin('fault-in').link(estopError)
    estopLatch.pin('reset').link(estopReset)
    estopLatch.pin('ok-out').link(estopOut)

    estopUser += 'iocontrol.0.user-enable-out'
    estopReset += 'iocontrol.0.user-request-enable'

    # Monitor estop input from hardware
    estopIn += 'iocontrol.0.emc-enable-in'


def setup_tool_loopback():
    # create signals for tool loading loopback
    hal.net('iocontrol.0.tool-prepare', 'iocontrol.0.tool-prepared')
    hal.net('iocontrol.0.tool-change', 'iocontrol.0.tool-changed')


def init_gantry(axisIndex, joints=2, latching=True):
    if latching:
        comp = 'lgantry'
    else:
        comp = 'gantry'
    rt.newinst(comp, 'gantry.%i' % axisIndex, pincount=joints)
    rcomps.create_gantry_rcomp(axisIndex=axisIndex)


def read_gantry(gantryAxis, thread):
    hal.addf('gantry.%i.read' % gantryAxis, thread)


def write_gantry(gantryAxis, thread):
    hal.addf('gantry.%i.write' % gantryAxis, thread)
