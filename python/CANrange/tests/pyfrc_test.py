'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
from phoenix6 import hardware, configs, signals, BaseStatusSignal

FIRST_SET = 0
SECOND_SET = 4.8

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

def test_proximity_detect():
    canrange = hardware.CANrange(1, "sim")
    canrange_sim_state = canrange.sim_state

    # Factory-default CANrange
    canrange.configurator.apply(configs.CANrangeConfiguration())

    # Double check both directions make sense
    distance_sequence: list[tuple[float, bool]] = [
        (0.3, True),
        (0.4, True),
        (0.411, False),
        (0.6, False),
        (0.4, False),
        (0.389, True),
        (0.3, True),
    ]

    distance = canrange.get_distance()
    is_detected = canrange.get_is_detected()

    for e in distance_sequence:
        canrange_sim_state.set_distance(e[0])
        BaseStatusSignal.wait_for_all(1, distance, is_detected)

        print(f"Configured for {e[0]} and distance is {distance} and is detected: {is_detected}")
        assert_almost_equal(e[0], distance.value, 0.001)
        assert e[1] == is_detected.value
