#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import tunables
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve
from wpilib import RobotState
from wpimath import Rotation2d
from wpimath.units import rotations_to_radians

from generated.tuner_constants import TunerConstants
from swerve_telemetry import SwerveTelemetry


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotations_to_radians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

        self._logger = SwerveTelemetry(self._max_speed)

        self._joystick = CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        tunables.publish("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configure_button_bindings()

    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.set_default_command(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.get_left_y() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.get_left_x() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.get_right_x() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(RobotState.is_disabled).while_true(
            self.drivetrain.apply_request(lambda: idle).ignoring_disable(True)
        )

        self._joystick.a().while_true(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().while_true(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.get_left_y(), -self._joystick.get_left_x())
                )
            )
        )

        self._joystick.dpad_up().while_true(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        self._joystick.dpad_down().while_true(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )

        # Run SysId routines when holding back (view)/start (menu) and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.view() & self._joystick.y()).while_true(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.FORWARD)
        )
        (self._joystick.view() & self._joystick.x()).while_true(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.REVERSE)
        )
        (self._joystick.menu() & self._joystick.y()).while_true(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.FORWARD)
        )
        (self._joystick.menu() & self._joystick.x()).while_true(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.REVERSE)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.left_bumper().on_true(
            self.drivetrain.run_once(self.drivetrain.seed_field_centric)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def get_autonomous_command(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.get_selected()
