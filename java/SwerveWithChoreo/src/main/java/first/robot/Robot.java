// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootReplay;

public class Robot extends TimedRobot {
    @SuppressWarnings("unused")
    private final HootReplay replay = new HootReplay("./logs/example.hoot");

    private Command autonomousCommand;
    private final RobotContainer container;

    /* log and replay timestamp and Driver Station data */
    private final HootAutoReplay timeAndDSReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withDriverStationReplay()
        .withJoystickReplay();

    public Robot() {
        container = new RobotContainer();
        addPeriodic(timeAndDSReplay::update, DEFAULT_PERIOD, -0.001);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = container.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
