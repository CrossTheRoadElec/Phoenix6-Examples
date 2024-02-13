'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from time import sleep
from pyfrc.tests import *
from phoenix6 import hardware, BaseStatusSignal

POS = 1.5
VEL = 10
DELAY = 0.1

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

def test_latency_compensation():
    """
    Verifies that latency compensation is performing as expected
    """
    fx = hardware.TalonFX(1, "sim")
    pos = fx.get_position()
    vel = fx.get_velocity()

    BaseStatusSignal.wait_for_all(1.0, pos, vel)
    fx.set_position(0, 1.0) # Set position to 0 to initialize

    # Position at 1.5 rotations rotating at 10 rotations per second
    fx.sim_state.set_raw_rotor_position(POS)
    fx.sim_state.set_rotor_velocity(VEL)

    # Wait for 2 signals to update to ensure we have a fresh signal
    BaseStatusSignal.wait_for_all(1.0, pos, vel)
    BaseStatusSignal.wait_for_all(1.0, pos, vel)

    # Wait another 100ms so we latency-compensate one full rotation
    sleep(DELAY)

    # Calculate latency-compensated position
    latency_compensated_pos = BaseStatusSignal.get_latency_compensated_value(pos, vel)
    measured_latency = pos.timestamp.get_latency()

    assert_almost_equal(pos.value, POS, 0.01)
    assert_almost_equal(vel.value, VEL, 0.01)
    # This can be more loose since it's time-dependent
    assert_almost_equal(latency_compensated_pos, POS + (VEL * measured_latency), 0.1)
