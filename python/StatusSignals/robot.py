#!/usr/bin/env python3
"""
    This is a demo program for StatusSignal usage in Phoenix 6
"""
import wpilib
from wpilib import Timer, XboxController
from phoenix6 import (
    hardware,
    controls,
    SignalLogger,
    BaseStatusSignal,
    unmanaged
)


class MyRobot(wpilib.TimedRobot):
    """
    Example program that provides basic usage on StatusSignals
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.motor = hardware.TalonFX(1, "canivore")
        self.request = controls.DutyCycleOut(0)

        self.pos = self.motor.get_position()
        self.vel = self.motor.get_velocity()

        self.timer = Timer()
        self.timer.start()

        self.joystick = XboxController(0)

        self.motor.set_position(6)

    def robotPeriodic(self) -> None:
        # Drive the motor so we have a changing position/velocity
        self.motor.set_control(self.request.with_output(self.joystick.getLeftY()))

    def teleopInit(self) -> None:
        """Start signal logger for logging purposes"""
        SignalLogger.start()

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.hasElapsed(0.1):
            self.timer.reset()
            BaseStatusSignal.refresh_all(self.pos, self.vel)

            pos_timestamp = self.pos.all_timestamps.get_device_timestamp().time
            print(f"Position is {self.pos} and velocity is {self.vel} at timestamp {pos_timestamp}")

            latency_compensated_pos = BaseStatusSignal.get_latency_compensated_value(
                self.pos, self.vel
            )
            print(f"Latency compensated position is {latency_compensated_pos}")


if __name__ == "__main__":
    wpilib.run(MyRobot)
