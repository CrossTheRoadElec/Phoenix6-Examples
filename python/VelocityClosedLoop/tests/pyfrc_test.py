'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from time import sleep
from pyfrc.tests import *
from phoenix6 import hardware, configs, controls
from phoenix6.unmanaged import feed_enable
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

# PID loop means we should be kinda fast, let's target 10ms
LOOP_PERIOD = 0.01
def wait_with_sim(time: float, fx: hardware.TalonFX, dcmotorsim: DCMotorSim):
    start_time = 0
    while start_time < time:
        feed_enable(LOOP_PERIOD * 2)
        start_time += LOOP_PERIOD

        dcmotorsim.setInputVoltage(fx.sim_state.motor_voltage)
        dcmotorsim.update(LOOP_PERIOD)
        fx.sim_state.set_raw_rotor_position(radiansToRotations(dcmotorsim.getAngularPosition()))
        fx.sim_state.set_rotor_velocity(radiansToRotations(dcmotorsim.getAngularVelocity()))

        sleep(LOOP_PERIOD)

def test_position_closed_loop():
    talonfx = hardware.TalonFX(1, "sim")
    motorsim = DCMotorSim(DCMotor.krakenX60FOC(1), 1.0, 0.001)
    vel = talonfx.get_velocity()

    talonfx.sim_state.set_raw_rotor_position(radiansToRotations(motorsim.getAngularPosition()))
    talonfx.sim_state.set_rotor_velocity(radiansToRotations(motorsim.getAngularVelocity()))
    talonfx.sim_state.set_supply_voltage(12)

    cfg = configs.TalonFXConfiguration()
    cfg.slot0.k_v = 0.12
    cfg.slot0.k_p = 0.11
    cfg.slot0.k_i = 0
    cfg.slot0.k_d = 0
    assert talonfx.configurator.apply(cfg).is_ok()

    vel.wait_for_update(1)
    assert_almost_equal(vel.value, 0, 0.1)

    # Closed loop for 1 second to a target of 10 rps, and verify we're close
    target_control = controls.VelocityVoltage(velocity=10)
    talonfx.set_control(target_control)

    wait_with_sim(1, talonfx, motorsim)

    # Verify velocity is close to target
    vel.wait_for_update(1)
    assert_almost_equal(vel.value, 1, 0.1)
