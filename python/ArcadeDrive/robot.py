#!/usr/bin/env python3
"""
    This is a demo program for arcade drive in Python with Phoenix 6
"""
import wpilib
from phoenix6 import hardware, controls, unmanaged


class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows to do simple arcade drive in robotpy
    with Phoenix 6
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        canivore_name = "canivore"
        self.front_left_motor = hardware.TalonFX(0, canivore_name)
        self.rear_left_motor = hardware.TalonFX(1, canivore_name)
        self.front_right_motor = hardware.TalonFX(2, canivore_name)
        self.rear_right_motor = hardware.TalonFX(3, canivore_name)

        # Configure the rear motors to follow the front motors
        follow_left_request = controls.Follower(0, False)
        self.rear_left_motor.set_control(follow_left_request)

        follow_right_request = controls.Follower(2, False)
        self.rear_right_motor.set_control(follow_right_request)

        # Keep a reference to the DutyCycleOut control request to update periodically
        self.left_out = controls.DutyCycleOut(output=0)
        self.right_out = controls.DutyCycleOut(output=0)

        # Keep a reference to an Xbox Controller for teleop control
        self.joy = wpilib.XboxController(0)

    def teleopPeriodic(self):
        """Runs the motors with arcade drive"""
        # Get throttle and wheel values for arcade drive
        throttle = self.joy.getLeftY() * -1
        wheel = self.joy.getRightX() * 1

        # And set the DutyCycleOut to the motor controllers
        self.front_left_motor.set_control(self.left_out.with_output(throttle + wheel))
        self.front_right_motor.set_control(self.right_out.with_output(throttle - wheel))

    def _simulationPeriodic(self):
        """"""
        # If the driver station is enabled, then feed enable for phoenix devices
        if wpilib.DriverStation.isEnabled():
            unmanaged.feed_enable(100)


if __name__ == "__main__":
    wpilib.run(MyRobot)
