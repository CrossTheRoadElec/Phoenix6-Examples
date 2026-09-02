'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from robot import MyRobot

from typing import TYPE_CHECKING

from wpilib.testing.robot_tests import *
from wpilib.simulation import XboxControllerSim

if TYPE_CHECKING:
    from wpilib.testing.controller import RobotTestController

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

def test_sysid_quasistatic(control: 'RobotTestController', robot: MyRobot):
    with control.run_robot():
        joysim = XboxControllerSim(robot._container.joystick.get_controller())

        joysim.set_y_button(True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        assert_almost_equal(robot._container.mechanism.sys_id_control.output, 0.16, 0.01)

        joysim.set_y_button(False)

def test_sysid_dynamic(control: 'RobotTestController', robot: MyRobot):
    with control.run_robot():
        joysim = XboxControllerSim(robot._container.joystick.get_controller())

        joysim.set_b_button(True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        assert_almost_equal(robot._container.mechanism.sys_id_control.output, 4.0, 0.01)
        
        joysim.set_b_button(False)
