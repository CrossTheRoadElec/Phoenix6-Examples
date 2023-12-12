#!/usr/bin/env python3
"""
    This is a demo program for tank drive in Python with Phoenix6
"""
import wpilib
from phoenix6 import TalonFX, Follower, DutyCycleOut


class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows to do simple arcade drive in robotpy
    with Phoenix 6
    """

    front_left_motor: TalonFX
    rear_left_motor: TalonFX
    front_right_motor: TalonFX
    rear_right_motor: TalonFX
    left_out: DutyCycleOut
    right_out: DutyCycleOut
    joy: wpilib.XboxController

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        canivore_name = "canivore"
        self.front_left_motor = TalonFX(0, canivore_name)
        self.rear_left_motor = TalonFX(1, canivore_name)
        self.front_right_motor = TalonFX(2, canivore_name)
        self.rear_right_motor = TalonFX(3, canivore_name)

        # Configure the rear motors to follow the front motors
        follow_left_request = Follower(0, False)
        self.rear_left_motor.set_control(follow_left_request)

        follow_right_request = Follower(2, False)
        self.rear_right_motor.set_control(follow_right_request)

        # Keep a reference to the DutyCycleOut control request to update periodically
        self.left_out = DutyCycleOut(output=0)
        self.right_out = DutyCycleOut(output=0)

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


if __name__ == "__main__":
    wpilib.run(MyRobot)
