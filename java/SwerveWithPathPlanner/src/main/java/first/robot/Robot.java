// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.util.Units;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootReplay;

import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimateType;

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

    private final Limelight limelight = null; // new Limelight("limelight")

    public Robot() {
        container = new RobotContainer();
        addPeriodic(timeAndDSReplay::update, DEFAULT_PERIOD, -0.001);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (limelight != null) {
            final var driveState = container.drivetrain.getState();
            final double headingDeg = driveState.Pose.getRotation().getDegrees();
            final double omegaRps = Units.radiansToRotations(driveState.Velocity.omega);

            limelight.setRobotOrientation(headingDeg, Units.rotationsToDegrees(omegaRps), 0, 0, 0, 0, true);
            final var llMeasurement = limelight.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE);
            if (llMeasurement.isValid() && Math.abs(omegaRps) < 2.0) {
                container.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }
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
