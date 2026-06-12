'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from typing import TYPE_CHECKING

from wpilib.testing.robot_tests import *
from wpilib.simulation import NiDsXboxControllerSim

if TYPE_CHECKING:
    from wpilib.testing.controller import RobotTestController

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

def test_sysid_quasistatic(control: 'RobotTestController', robot):
    with control.run_robot():
        joysim = NiDsXboxControllerSim(robot.container.joystick.getHID())

        joysim.setYButton(True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        assert_almost_equal(robot.container.mechanism.sys_id_control.output, 0.16, 0.01)

        joysim.setYButton(False)

def test_sysid_dynamic(control: 'RobotTestController', robot):
    with control.run_robot():
        joysim = NiDsXboxControllerSim(robot.container.joystick.getHID())

        joysim.setBButton(True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        assert_almost_equal(robot.container.mechanism.sys_id_control.output, 4.0, 0.01)
        
        joysim.setBButton(False)
