#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import CommandScheduler, TimedCommandRobot

from robotcontainer import RobotContainer


class MyRobot(TimedCommandRobot):
    """The VM is configured to automatically run this class, and to call the functions corresponding to
    each mode, as described in the TimedRobot documentation. If you change the name of this class or
    the package after creating this project, you must also update the build.gradle file in the
    project.
    """

    def robotInit(self) -> None:
        """This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.container = RobotContainer()

        self.autonomousCommand = None

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            CommandScheduler.getInstance().schedule(self.autonomousCommand)

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            CommandScheduler.getInstance().cancel(self.autonomousCommand)

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        """This function is called periodically during test mode."""
        pass
