'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from robot import MyRobot

from time import sleep
from typing import TYPE_CHECKING

from pyfrc.tests import *

from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations

from phoenix6 import configs, controls, hardware, signals

if TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController

FIRST_SET = 0
SECOND_SET = 1.0

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

# PID loop means we should be kinda fast, let's target 5 ms
LOOP_PERIOD = 0.005
def wait_with_sim(time: float, fx: hardware.TalonFX, dcmotorsim: DCMotorSim, gear_ratio: float):
    start_time = 0.0
    while start_time < time:
        start_time += LOOP_PERIOD

        dcmotorsim.setInputVoltage(fx.sim_state.motor_voltage)
        dcmotorsim.update(LOOP_PERIOD)
        fx.sim_state.set_raw_rotor_position(gear_ratio * radiansToRotations(dcmotorsim.getAngularPosition()))
        fx.sim_state.set_rotor_velocity(gear_ratio * radiansToRotations(dcmotorsim.getAngularVelocity()))

        sleep(LOOP_PERIOD)

def test_position_closed_loop(control: 'TestController', robot: MyRobot):
    with control.run_robot():
        talonfx = robot.talonfx
        pos = talonfx.get_position(False)

        # wait for the device to start up and enable
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        while talonfx.get_robot_enable(False).wait_for_update(1.0).value != signals.RobotEnableValue.ENABLED:
            pass

        gear_ratio = 12.8
        gearbox = DCMotor.krakenX60FOC(1)
        motorsim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.01, gear_ratio), gearbox)

        talonfx.sim_state.set_raw_rotor_position(radiansToRotations(motorsim.getAngularPosition()))
        talonfx.sim_state.set_supply_voltage(12)

        cfg = configs.TalonFXConfiguration()
        cfg.feedback.sensor_to_mechanism_ratio = gear_ratio
        cfg.slot0.k_p = 60.0
        cfg.slot0.k_i = 0
        cfg.slot0.k_d = 1.0
        assert talonfx.configurator.apply(cfg).is_ok()
        assert talonfx.set_position(FIRST_SET).is_ok()

        pos.wait_for_update(1)
        assert_almost_equal(pos.value, 0, 0.02)

        # Closed loop for 2 seconds to a target of 1 rotation, and verify we're close
        target_control = controls.PositionVoltage(SECOND_SET)
        talonfx.set_control(target_control)

        wait_with_sim(2, talonfx, motorsim, gear_ratio)

        # Verify position is close to target
        pos.wait_for_update(1)
        assert_almost_equal(pos.value, SECOND_SET, 0.02)
