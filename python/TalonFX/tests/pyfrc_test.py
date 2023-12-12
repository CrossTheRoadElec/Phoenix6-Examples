'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
from phoenix6 import TalonFX, TalonFXConfiguration

FIRST_SET = 0
SECOND_SET = 4.8

def assert_almost_equal(a: float, b: float, range_val: float):
    """
    Assert that a is within range of b
    """
    assert a >= (b - range_val) and a <= (b + range_val)

def test_pos_setter():
    talonfx = talonfx(1, "sim")
    pos = talonfx.get_position()

    cfg = TalonFXConfiguration()
    talonfx.configurator.apply(cfg)

    talonfx.set_position(FIRST_SET)
    pos.wait_for_update(1.0)
    assert_almost_equal(pos.value, FIRST_SET, 0.01)

    talonfx.set_position(SECOND_SET)
    pos.wait_for_update(1.0)
    assert_almost_equal(pos.value, SECOND_SET, 0.01)
