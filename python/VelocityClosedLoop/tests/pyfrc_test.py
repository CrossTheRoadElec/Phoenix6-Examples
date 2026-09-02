'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from robot import MyRobot

from time import sleep
from typing import TYPE_CHECKING

from wpilib.testing.robot_tests import *

from wpilib.simulation import DCMotorSim, DriverStationSim, resume_timing
from wpimath import DCMotor, Models
from wpimath.units import radians_to_rotations

from phoenix6 import configs, controls, hardware, signals

if TYPE_CHECKING:
    from wpilib.testing.controller import RobotTestController

TARGET_VEL = 10.0

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

        dcmotorsim.set_input_voltage(fx.sim_state.motor_voltage)
        dcmotorsim.update(LOOP_PERIOD)
        fx.sim_state.set_raw_rotor_position(gear_ratio * radians_to_rotations(dcmotorsim.get_angular_position()))
        fx.sim_state.set_rotor_velocity(gear_ratio * radians_to_rotations(dcmotorsim.get_angular_velocity()))

        sleep(LOOP_PERIOD)

def test_velocity_closed_loop(control: 'RobotTestController', robot: MyRobot):
    with control.run_robot():
        talonfx = robot.talonfx
        vel = talonfx.get_velocity(False)

        # wait for the device to start up and enable
        DriverStationSim.set_ds_attached(True)
        DriverStationSim.set_enabled(True)
        DriverStationSim.notify_new_data()
        resume_timing()

        while talonfx.get_robot_enable(False).wait_for_update(1.0).value != signals.RobotEnableValue.ENABLED:
            pass

        gear_ratio = 5.0
        gearbox = DCMotor.kraken_x60_foc(1)
        motorsim = DCMotorSim(Models.single_jointed_arm_from_physical_constants(gearbox, 0.03, gear_ratio), gearbox)

        talonfx.sim_state.set_raw_rotor_position(radians_to_rotations(motorsim.get_angular_position()))
        talonfx.sim_state.set_supply_voltage(12)

        cfg = configs.TalonFXConfiguration()
        cfg.feedback.sensor_to_mechanism_ratio = gear_ratio
        cfg.slot0.k_v = 0.62
        cfg.slot0.k_p = 0.75
        cfg.slot0.k_i = 0
        cfg.slot0.k_d = 0.0
        assert talonfx.configurator.apply(cfg).is_ok()

        vel.wait_for_update(1)
        assert_almost_equal(vel.value, 0, 0.02)

        # Closed loop for 2 seconds to a velocity of 10 rps, and verify we're close
        target_control = controls.VelocityVoltage(TARGET_VEL)
        talonfx.set_control(target_control)

        wait_with_sim(2, talonfx, motorsim, gear_ratio)

        # Verify velocity is close to target
        vel.wait_for_update(1)
        assert_almost_equal(vel.value, TARGET_VEL, 0.2)
