'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from time import sleep
from robot import MyRobot

from pyfrc.tests import *
from phoenix6 import hardware, configs, controls
from phoenix6.unmanaged import feed_enable
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations

FIRST_SET = 0
SECOND_SET = 4.8

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

# PID loop means we should be kinda fast, let's target 10ms
LOOP_PERIOD = 0.01
def wait_with_sim(time: float, fx: hardware.TalonFX, dcmotorsim: DCMotorSim):
    feed_enable(0.2)
    sleep(0.1)

    start_time = 0
    while start_time < time:
        feed_enable(0.1)
        start_time += LOOP_PERIOD

        dcmotorsim.setInputVoltage(fx.sim_state.motor_voltage)
        dcmotorsim.update(LOOP_PERIOD)
        fx.sim_state.set_raw_rotor_position(radiansToRotations(dcmotorsim.getAngularPosition()))
        fx.sim_state.set_rotor_velocity(radiansToRotations(dcmotorsim.getAngularVelocity()))

        sleep(LOOP_PERIOD)

def test_position_closed_loop(control, robot: MyRobot):
    with control.run_robot():
        talonfx = robot.talonfx
        gearbox = DCMotor.krakenX60FOC(1)
        motorsim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.01, 1.0), gearbox)
        pos = talonfx.get_position()

        talonfx.sim_state.set_raw_rotor_position(radiansToRotations(motorsim.getAngularPosition()))
        talonfx.sim_state.set_supply_voltage(12)

        cfg = configs.TalonFXConfiguration()
        cfg.feedback.sensor_to_mechanism_ratio = 12.8
        cfg.motion_magic.motion_magic_cruise_velocity = 5
        cfg.motion_magic.motion_magic_acceleration = 10
        cfg.motion_magic.motion_magic_jerk = 100

        cfg.slot0.k_v = 0.12
        cfg.slot0.k_a = 0.01
        cfg.slot0.k_p = 60
        cfg.slot0.k_i = 0
        cfg.slot0.k_d = 0.5

        assert talonfx.configurator.apply(cfg).is_ok()
        assert talonfx.set_position(FIRST_SET).is_ok()

        pos.wait_for_update(1)
        assert_almost_equal(pos.value, 0, 0.02)

        # Closed loop for 1 seconds to a target of 1 rotation, and verify we're close
        target_control = controls.MotionMagicVoltage(position=1.0)
        talonfx.set_control(target_control)

        wait_with_sim(2, talonfx, motorsim)

        # Verify position is close to target
        pos.wait_for_update(1)
        assert_almost_equal(pos.value, 1, 0.02)
