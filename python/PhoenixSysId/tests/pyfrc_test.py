'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
from wpilib.simulation import XboxControllerSim
import time
from subsystems.flywheel import FlywheelMechanism

FIRST_SET = 0
SECOND_SET = 4.8

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

def test_sysid_quasistatic(control, robot):
    joysim = XboxControllerSim(robot.container.joystick.getHID())

    joysim.setYButton(True)
    control.step_timing(seconds=0.1, autonomous=False, enabled=True)

    assert robot.container.mechanism.sys_id_control.output > 0.1

    joysim.setYButton(False)

def test_sysid_dynamic(control, robot):
    joysim = XboxControllerSim(robot.container.joystick.getHID())

    joysim.setXButton(True)
    control.step_timing(seconds=0.1, autonomous=False, enabled=True)

    assert robot.container.mechanism.sys_id_control.output > 0.1
    
    joysim.setXButton(False)
