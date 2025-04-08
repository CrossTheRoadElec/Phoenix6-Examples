#!/usr/bin/env python3
"""
    This is a demo program for CANdi usage in Phoenix 6
"""
import wpilib
from wpilib import Timer
from phoenix6 import configs, hardware, signals

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use CANdi
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the devices used
        self.candi = hardware.CANdi(1, "canivore")

        # Configure CANdi
        cfg = configs.CANdiConfiguration()

        # Pulse-width sensor will drive low. Default of FloatDetect will typically work on most sensors.
        cfg.digital_inputs.s1_float_state = signals.S1FloatStateValue.PULL_HIGH
        # This example specifically assumes a hardware limit switch will close S2 to Ground. Default of CloseWhenNotFloating will also work.
        cfg.digital_inputs.s2_close_state = signals.S2CloseStateValue.CLOSE_WHEN_LOW

        # Invert the PWM1 position.
        cfg.pwm1.sensor_direction = True
        # If the PWM 1 position on boot is above 0.75 rotations, treat it as x - 1 rotations.
        # As an example, if the position is 0.87, it will boot to 0.87 - 1 = -0.13 rotations.
        cfg.pwm1.absolute_sensor_discontinuity_point = 0.75

        self.candi.configurator.apply(cfg)

        self.timer = Timer()
        self.timer.start()
        self.controller = wpilib.XboxController(0)

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.hasElapsed(0.1):
            self.timer.reset()
            # get_pwm1_position automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            pos = self.candi.get_pwm1_position()
            print(f"Position is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

            # Get the S2 State StatusSignal without refreshing
            s2_state = self.candi.get_s2_state(False)
            # This time wait for the signal to reduce latency
            s2_state.wait_for_update(0.1)
            print(f"S2 State is {s2_state} with {s2_state.timestamp.get_latency()} seconds of latency")

            print("")

if __name__ == "__main__":
    wpilib.run(MyRobot)
