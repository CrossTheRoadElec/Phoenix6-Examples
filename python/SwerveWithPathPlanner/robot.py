#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from phoenix6 import HootAutoReplay, HootReplay

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robot_periodic which runs the scheduler for you
    """

    def __init__(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        super().__init__()

        self._replay = HootReplay("./logs/example.hoot")

        self._autonomous_command: commands2.Command | None = None
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self._container = RobotContainer()

        # log and replay timestamp and Driver Station data
        self._time_and_ds_replay = (
            HootAutoReplay()
            .with_timestamp_replay()
            .with_driver_station_replay()
            .with_joystick_replay()
        )
        self.add_periodic(self._time_and_ds_replay.update, self.DEFAULT_PERIOD, -0.001)

    def robot_periodic(self) -> None:
        pass

    def disabled_init(self) -> None:
        pass

    def disabled_periodic(self) -> None:
        pass

    def disabled_exit(self) -> None:
        pass

    def autonomous_init(self) -> None:
        # This autonomous runs the autonomous command selected by your RobotContainer class.
        self._autonomous_command = self._container.get_autonomous_command()

        if self._autonomous_command:
            commands2.CommandScheduler.get_instance().schedule(self._autonomous_command)

    def autonomous_periodic(self) -> None:
        pass

    def autonomous_exit(self) -> None:
        pass

    def teleop_init(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self._autonomous_command:
            commands2.CommandScheduler.get_instance().cancel(self._autonomous_command)

    def teleop_periodic(self) -> None:
        pass

    def teleop_exit(self) -> None:
        pass

    def utility_init(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.get_instance().cancel_all()

    def utility_periodic(self) -> None:
        pass

    def utility_exit(self) -> None:
        pass
