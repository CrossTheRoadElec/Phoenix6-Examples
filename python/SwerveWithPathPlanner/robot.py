#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from phoenix6 import HootAutoReplay

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def __init__(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        super().__init__()

        self._autonomousCommand: commands2.Command | None = None

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self._container = RobotContainer()

        # log and replay timestamp and joystick data
        self._time_and_joystick_replay = (
            HootAutoReplay()
            .with_timestamp_replay()
            .with_joystick_replay()
        )

    def robotPeriodic(self) -> None:
        self._time_and_joystick_replay.update()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def disabledExit(self) -> None:
        pass

    def autonomousInit(self) -> None:
        # This autonomous runs the autonomous command selected by your RobotContainer class.
        self._autonomousCommand = self._container.getAutonomousCommand()

        if self._autonomousCommand:
            commands2.CommandScheduler.getInstance().schedule(self._autonomousCommand)

    def autonomousPeriodic(self) -> None:
        pass

    def autonomousExit(self) -> None:
        pass

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self._autonomousCommand:
            commands2.CommandScheduler.getInstance().cancel(self._autonomousCommand)

    def teleopPeriodic(self) -> None:
        pass

    def teleopExit(self) -> None:
        pass

    def utilityInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def utilityPeriodic(self) -> None:
        pass

    def utilityExit(self) -> None:
        pass
