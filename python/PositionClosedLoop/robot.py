#!/usr/bin/env python3
"""
    This is a demo program for TalonFX Position PID usage in Phoenix 6
"""
import wpilib
from wpilib import Timer, XboxController
from phoenix6 import hardware, controls, configs, unmanaged

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.talonfx = hardware.TalonFX(1, "canivore")
        self.position_request = controls.PositionVoltage(0)

        cfg = configs.TalonFXConfiguration()
        # Set PID gains
        cfg.slot0.k_p = 10
        cfg.slot0.k_d = 0.2

        # Apply PID gains to motor
        self.talonfx.configurator.apply(cfg)

        self.joystick = XboxController(0)
        self.timer = Timer()
        self.timer.start()

    def teleopInit(self):
        """Set the position of the talonfx to 0 so we know we're centered"""
        self.talonfx.set_position(0)

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        # Target a position of +- 10 rotations depending on joystick position
        self.talonfx.set_control(self.position_request.with_position(self.joystick.getLeftY() * 10))

        if self.timer.hasElapsed(0.1):
            self.timer.reset()
            # Print the position & velocity to see what they are
            pos = self.talonfx.get_position()
            vel = self.talonfx.get_velocity()

            print(f"Position: {pos}")
            print(f"Velocity: {vel}")

            print("")

    def _simulationPeriodic(self):
        """"""
        # If the driver station is enabled, then feed enable for phoenix devices
        if wpilib.DriverStation.isEnabled():
            unmanaged.feed_enable(100)

if __name__ == "__main__":
    wpilib.run(MyRobot)
