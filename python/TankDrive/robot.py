#!/usr/bin/env python3
"""
    This is a demo program for tank drive in Python with Phoenix6
"""
import wpilib
from phoenix6 import CoreTalonFX, Follower, DutyCycleOut
from wpilib.drive import DifferentialDrive

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.front_left_motor = CoreTalonFX(0, "Fred")
        self.rear_left_motor = CoreTalonFX(1, "Fred")
        self.front_right_motor = CoreTalonFX(2, "Fred")
        self.rear_right_motor = CoreTalonFX(3, "Fred")

        # Configure the rear motors to follow the front motors
        follow_left_request = Follower(master_id=0, oppose_master_direction=False)
        self.rear_left_motor.set_control(follow_left_request)

        follow_right_request = Follower(master_id=2, oppose_master_direction=False)
        self.rear_right_motor.set_control(follow_right_request)

        # Keep a reference to the DutyCycleOut control request to update periodically
        self.left_out = DutyCycleOut(output=0, enable_foc=True, override_brake_dur_neutral=False)
        self.right_out = DutyCycleOut(output=0, enable_foc=True, override_brake_dur_neutral=False)

        # Keep a reference to an Xbox Controller for teleop control
        self.joy = wpilib.XboxController(0)

    def teleopPeriodic(self):
        """Runs the motors with arcade drive"""
        # Get throttle and wheel values for arcade drive
        throttle = self.joy.getLeftY() * -1
        wheel = self.joy.getRightX() * 1

        # Set DutyCycleOut outputs
        self.left_out.output = throttle + wheel
        self.right_out.output = throttle - wheel

        # And set the DutyCycleOut to the motor controllers
        self.front_left_motor.set_control(self.left_out)
        self.front_right_motor.set_control(self.right_out)


if __name__ == "__main__":
    wpilib.run(MyRobot)