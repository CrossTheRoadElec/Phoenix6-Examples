// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import com.ctre.phoenix6.HootAutoReplay;

import com.limelightvision.Limelight;
import com.limelightvision.Limelight.PoseEstimateType;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.util.Units;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final Limelight limelight = null; // new Limelight("limelight")

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        timeAndJoystickReplay.update();
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
            final var driveState = robotContainer.drivetrain.getState();
            final double headingDeg = driveState.Pose.getRotation().getDegrees();
            final double omegaRps = Units.radiansToRotations(driveState.Velocity.omega);

            limelight.setRobotOrientation(headingDeg, Units.rotationsToDegrees(omegaRps), 0, 0, 0, 0);
            final var llMeasurement = limelight.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE);
            if (llMeasurement.isValid() && Math.abs(omegaRps) < 2.0) {
                robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
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
        autonomousCommand = robotContainer.getAutonomousCommand();

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
    public void simulationPeriodic() {}
}
