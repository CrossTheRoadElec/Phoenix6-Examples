'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
from wpilib.simulation import XboxControllerSim

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

def test_sysid_quasistatic(control, robot):
    with control.run_robot():
        joysim = XboxControllerSim(robot.container.joystick.getHID())

        joysim.setYButton(True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        assert_almost_equal(robot.container.mechanism.sys_id_control.output, 0.16, 0.01)

        joysim.setYButton(False)

def test_sysid_dynamic(control, robot):
    with control.run_robot():
        joysim = XboxControllerSim(robot.container.joystick.getHID())

        joysim.setBButton(True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        assert_almost_equal(robot.container.mechanism.sys_id_control.output, 4.0, 0.01)
        
        joysim.setBButton(False)
