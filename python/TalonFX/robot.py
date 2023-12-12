#!/usr/bin/env python3
"""
    This is a demo program for TalonFX usage in Phoenix 6
"""
import wpilib
from wpilib import Timer
from phoenix6 import TalonFX

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.talonfx = TalonFX(1, "canivore")

        self.timer = Timer()
        self.timer.start()

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.hasElapsed(0.1):
            # get_position automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            pos = self.talonfx.get_position()
            print(f"Positions is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

            # Get the velocity StatusSignal
            vel = self.talonfx.get_velocity()
            # This time wait for the signal to reduce latency
            vel.wait_for_update(0.1)
            print(f"Velocity is {vel} with {vel.timestamp.get_latency()} seconds of latency")

            print("")

if __name__ == "__main__":
    wpilib.run(MyRobot)
