#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robot_periodic which runs the scheduler for you
    """

    def __init__(self) -> None:
        """This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        super().__init__()

        self._autonomous_command: commands2.Command | None = None
        self._container = RobotContainer()

    def robot_periodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""
        pass

    def disabled_init(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabled_periodic(self) -> None:
        pass

    def autonomous_init(self) -> None:
        self._autonomous_command = self._container.get_autonomous_command()

        if self._autonomous_command:
            commands2.CommandScheduler.get_instance().schedule(self._autonomous_command)

    def teleop_init(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self._autonomous_command:
            commands2.CommandScheduler.get_instance().cancel(self._autonomous_command)

    def teleop_periodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def test_init(self) -> None:
        # Cancels all running commands at the start of test mode.
        commands2.CommandScheduler.get_instance().cancel_all()

    def test_periodic(self) -> None:
        """This function is called periodically during test mode."""
        pass
