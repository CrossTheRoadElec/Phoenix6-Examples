#!/usr/bin/env python3
"""
    This is a demo program for candi usage in Phoenix 6
"""
import wpilib
from wpilib import Timer
from phoenix6 import hardware

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use candi
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.candi = hardware.CANdi(1, "canivore")

        self.timer = Timer()
        self.timer.start()
        self.controller = wpilib.XboxController(0)

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.hasElapsed(0.1):
            self.timer.reset()
            # get_position automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            pos = self.candi.get_pwm1_position()
            print(f"Position is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

            # Get the velocity StatusSignal without refreshing
            s2_state = self.candi.get_s2_state(False)
            # This time wait for the signal to reduce latency
            s2_state.wait_for_update(0.1)
            print(f"S2 State is {s2_state} with {s2_state.timestamp.get_latency()} seconds of latency")

            print("")

if __name__ == "__main__":
    wpilib.run(MyRobot)
