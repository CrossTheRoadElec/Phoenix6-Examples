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

def test_float_states():
    candi = hardware.CANdi(1, "sim")
    candi_sim_state = candi.sim_state

    # Factory-default CANdi
    candi.configurator.apply(configs.CANdiConfiguration())

    # Double check all permutations make sense
    permutations : tuple[signals.S1CloseStateValue, signals.S1StateValue, bool] = [
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_FLOATING, signals.S1StateValue.FLOATING, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_FLOATING, signals.S1StateValue.HIGH, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_FLOATING, signals.S1StateValue.LOW, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_FLOATING, signals.S1StateValue.FLOATING, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_FLOATING, signals.S1StateValue.HIGH, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_FLOATING, signals.S1StateValue.LOW, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_HIGH, signals.S1StateValue.FLOATING, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_HIGH, signals.S1StateValue.HIGH, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_HIGH, signals.S1StateValue.LOW, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_HIGH, signals.S1StateValue.FLOATING, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_HIGH, signals.S1StateValue.HIGH, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_HIGH, signals.S1StateValue.LOW, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_LOW, signals.S1StateValue.FLOATING, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_LOW, signals.S1StateValue.HIGH, True),
        (signals.S1CloseStateValue.CLOSE_WHEN_NOT_LOW, signals.S1StateValue.LOW, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_LOW, signals.S1StateValue.FLOATING, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_LOW, signals.S1StateValue.HIGH, False),
        (signals.S1CloseStateValue.CLOSE_WHEN_LOW, signals.S1StateValue.LOW, True),
    ]

    s1_state = candi.get_s1_state()
    s1_closed = candi.get_s1_closed()

    for e in permutations:
        # First configure close state value
        cfg = configs.CANdiConfiguration()
        cfg.digital_inputs.s1_close_state = e[0]
        candi.configurator.apply(cfg)
        candi_sim_state.set_s1_state(e[1])

        BaseStatusSignal.wait_for_all(1, s1_state, s1_closed)
        BaseStatusSignal.wait_for_all(1, s1_state, s1_closed)
        print(f"Configured for {e[0]} and state is {s1_state} and is closed: {s1_closed}")
        assert e[1] == s1_state.value
        assert e[2] == s1_closed.value
    
    # Convert to S2 equivalents
    permutations : tuple[signals.S2CloseStateValue, signals.S2StateValue, bool] = [
        (signals.S2CloseStateValue(a.value), signals.S2StateValue(b.value), c) for (a, b, c) in permutations
    ]

    s2_state = candi.get_s2_state()
    s2_closed = candi.get_s2_closed()
    for e in permutations:
        # First configure close state value
        cfg = configs.CANdiConfiguration()
        cfg.digital_inputs.s2_close_state = e[0]
        candi.configurator.apply(cfg)
        candi_sim_state.set_s2_state(e[1])

        BaseStatusSignal.wait_for_all(1, s2_state, s2_closed)
        BaseStatusSignal.wait_for_all(1, s2_state, s2_closed)
        print(f"Configured for {e[0]} and state is {s2_state} and is closed: {s2_closed}")
        assert e[1] == s2_state.value
        assert e[2] == s2_closed.value


def over_current_test():
    candi = hardware.CANdi(2, "sim")
    candi_sim_state = candi.sim_state

    candi.configurator.apply(configs.CANdiConfiguration())

    is_overcurrent = candi.get_overcurrent()
    output_current = candi.get_output_current()

    BaseStatusSignal.wait_for_all(1, is_overcurrent, output_current)

    is_overcurrent.wait_for_update(1)

    print(f"CANdi output current is {output_current} and is overcurrent: {is_overcurrent}")
    assert is_overcurrent.value == False
    assert_almost_equal(output_current.value, 0, 0.001)

    # CANdi can provide up to 300 mA
    candi_sim_state.set_output_current(0.250)

    BaseStatusSignal.wait_for_all(1, is_overcurrent, output_current);

    print(f"CANdi output current is {output_current} and is overcurrent: {is_overcurrent}")
    assert is_overcurrent.value == False
    assert_almost_equal(output_current.value, 0.250, 0.001)

    # CANdi will fault over 300 mA
    candi_sim_state.set_output_current(1)

    BaseStatusSignal.wait_for_all(1, is_overcurrent, output_current);

    print(f"CANdi output current is {output_current} and is overcurrent: {is_overcurrent}")
    assert is_overcurrent.value == True
    assert_almost_equal(output_current.value, 0, 0.001)

def test_individual_positions():
    CANDI_POSITION_1 = -3.1
    CANDI_POSITION_2 = 1.2
    CANDI_POSITION_3 = 0.7

    candi = hardware.CANdi(2, "sim")
    candi_sim_state = candi.sim_state

    # Factory-default CANdi
    candi.configurator.apply(configs.CANdiConfiguration())

    # Wait for signal to update and assert they match the set positions
    candi_pos_1 = candi.get_pwm1_position()
    candi_pos_2 = candi.get_pwm2_position()
    candi_pos_3 = candi.get_quadrature_position()

    # Make sure both are initially set to 0 before messing with sim state
    candi_sim_state.set_pwm1_position(0)
    candi_sim_state.set_pwm2_position(0)
    candi_sim_state.set_raw_quadrature_position(0)
    candi.set_raw_quadrature_position(0)
    # Wait for sets to take affect
    BaseStatusSignal.wait_for_all(1.0, candi_pos_1, candi_pos_2, candi_pos_3)

    # Set them to different values
    candi_sim_state.set_pwm1_position(CANDI_POSITION_1)
    candi_sim_state.set_pwm2_position(CANDI_POSITION_2)
    candi_sim_state.set_raw_quadrature_position(CANDI_POSITION_3)

    BaseStatusSignal.wait_for_all(1.0, candi_pos_1, candi_pos_2, candi_pos_3)
    BaseStatusSignal.wait_for_all(1.0, candi_pos_1, candi_pos_2, candi_pos_3)

    print(f"CANdi Pos vs expected: {candi_pos_1} vs {CANDI_POSITION_1}")
    assert_almost_equal(candi_pos_1.value, CANDI_POSITION_1, 0.1)
    print(f"CANdi Pos vs expected: {candi_pos_2} vs {CANDI_POSITION_2}")
    assert_almost_equal(candi_pos_2.value, CANDI_POSITION_2, 0.1)
    print(f"CANdi Pos vs expected: {candi_pos_3} vs {CANDI_POSITION_3}")
    assert_almost_equal(candi_pos_3.value, CANDI_POSITION_3, 0.1)
