#!/usr/bin/env python3
"""
    This is a demo program for TalonFX Position PID usage in Phoenix 6
"""
import wpilib
from wpilib import Timer, XboxController
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

        # Be able to switch which control request to use based on a button press
        # Start at position 0, use slot 0
        self.position_voltage = controls.PositionVoltage(0).with_slot(0)
        # Start at position 0, use slot 1
        self.position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(1)
        # Keep a brake request so we can disable the motor
        self.brake = controls.NeutralOut()

        self.joystick = XboxController(0)

        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_p = 2.4; # An error of 1 rotation results in 2.4 V output
        cfg.slot0.k_i = 0; # No output for integrated error
        cfg.slot0.k_d = 0.1; # A velocity of 1 rps results in 0.1 V output
        # Peak output of 8 V
        cfg.voltage.peak_forward_voltage = 8
        cfg.voltage.peak_reverse_voltage = -8

        cfg.slot1.k_p = 40; # An error of 1 rotation results in 40 A output
        cfg.slot1.k_i = 0; # No output for integrated error
        cfg.slot1.k_d = 2; # A velocity of 1 rps results in 2 A output
        # Peak output of 120 A
        cfg.torque_current.peak_forward_torque_current = 120
        cfg.torque_current.peak_reverse_torque_current = -120

        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.talonfx.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        # Make sure we start at 0
        self.talonfx.set_position(0)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Go for plus/minus 10 rotations
        desired_rotations = self.joystick.getLeftY() * 10
        if abs(desired_rotations) <= 0.1: # Joystick deadzone
            desired_rotations = 0

        if self.joystick.getLeftBumper():
            # Use position voltage
            self.talonfx.set_control(self.position_voltage.with_position(desired_rotations))
        elif self.joystick.getRightBumper():
            # Use position torque
            self.talonfx.set_control(self.position_torque.with_position(desired_rotations))
        else:
            # Disable the motor instead
            self.talonfx.set_control(self.brake)

if __name__ == "__main__":
    wpilib.run(MyRobot)
