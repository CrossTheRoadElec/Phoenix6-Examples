#!/usr/bin/env python3
"""
    This is a demo program for CANcoder usage in Phoenix 6
"""
import wpilib
from wpilib import Timer
from phoenix6 import hardware

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use CANcoder
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.cancoder = hardware.CANcoder(1, "canivore")

        self.timer = Timer()
        self.timer.start()

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.hasElapsed(0.1):
            # get_position automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            pos = self.cancoder.get_position()
            print(f"Positions is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

            # Get the velocity StatusSignal
            vel = self.cancoder.get_velocity()
            # This time wait for the signal to reduce latency
            vel.wait_for_update(0.1)
            print(f"Velocity is {vel} with {vel.timestamp.get_latency()} seconds of latency")

            print("")

if __name__ == "__main__":
    wpilib.run(MyRobot)
