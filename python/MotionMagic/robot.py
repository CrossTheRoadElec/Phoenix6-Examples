#!/usr/bin/env python3
"""
    This is a demo program for TalonFX Motion Magic usage in Phoenix 6
"""
import wpilib
from wpilib import XboxController
from phoenix6 import hardware, controls, configs, StatusCode

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.talonfx = hardware.TalonFX(1, "canivore")
        self.motion_magic = controls.MotionMagicVoltage(0)

        self.joystick = XboxController(0)

        cfg = configs.TalonFXConfiguration()

        fdb = cfg.feedback
        fdb.sensor_to_mechanism_ratio = 12.8

        # Configure Motion Magic
        mm = cfg.motion_magic
        mm.motion_magic_cruise_velocity = 5 # 5 rotations per second cruise
        mm.motion_magic_acceleration = 10 # Take approximately 0.5 seconds to reach max vel
        # Take apprximately 0.1 seconds to reach max accel
        mm.motion_magic_jerk = 100

        slot0 = cfg.slot0
        slot0.k_s = 0.25 # Add 0.25 V output to overcome static friction
        slot0.k_v = 0.12 # A velocity target of 1 rps results in 0.12 V output
        slot0.k_a = 0.01 # An acceleration of 1 rps/s requires 0.01 V output
        slot0.k_p = 60 # A position error of 0.2 rotations results in 12 V output
        slot0.k_i = 0 # No output for integrated error
        slot0.k_d = 0.5 # A velocity error of 1 rps results in 0.5 V output

        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.talonfx.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Deadband the joystick
        left_y = self.joystick.getLeftY()
        if abs(left_y) < 0.1:
            left_y = 0

        self.talonfx.set_control(self.motion_magic.with_position(left_y * 10).with_slot(0))
        if (self.joystick.getBButton()):
            self.talonfx.set_position(1)

if __name__ == "__main__":
    wpilib.run(MyRobot)
