#!/usr/bin/env python3
"""
    This is a demo program for arcade drive in Python with Phoenix 6
"""
import wpilib
from phoenix6 import CANBus, configs, controls, hardware


class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows to do simple arcade drive in robotpy
    with Phoenix 6
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.canivore = CANBus("canivore")
        self.front_left_motor = hardware.TalonFX(0, self.canivore)
        self.rear_left_motor = hardware.TalonFX(1, self.canivore)
        self.front_right_motor = hardware.TalonFX(2, self.canivore)
        self.rear_right_motor = hardware.TalonFX(3, self.canivore)

        cfg = configs.TalonFXConfiguration()
        cfg.motor_output.inverted = configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.front_left_motor.configurator.apply(cfg)

        cfg.motor_output.inverted = configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        self.front_right_motor.configurator.apply(cfg)

        # Configure the rear motors to follow the front motors
        follow_left_request = controls.Follower(0, False)
        self.rear_left_motor.set_control(follow_left_request)

        follow_right_request = controls.Follower(2, False)
        self.rear_right_motor.set_control(follow_right_request)

        # Keep a reference to the DutyCycleOut control request to update periodically
        self.left_out = controls.DutyCycleOut(0)
        self.right_out = controls.DutyCycleOut(0)

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
