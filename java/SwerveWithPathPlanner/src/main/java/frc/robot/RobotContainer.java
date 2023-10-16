// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true).withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final double Velocity = 3;
  private final double Acceleration = 10;
  /* Path follower */
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Tests", new PathConstraints(Velocity, Acceleration), new PathConstraints(Velocity, Acceleration));
  Map<String, Command> eventMap = new HashMap<>() {{
    put("marker", new PrintCommand("Passed Marker"));
  }};
  SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      ()->drivetrain.getState().Pose, // Supplier of current robot pose
      drivetrain::seedFieldRelative,  // Consumer for seeding pose against auto
      new PIDConstants(10, 0, 0), // Translation gains
      new PIDConstants(10, 0, 0), // Rotation gains
      (speeds)->drivetrain.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
      eventMap, // Map of events to trigger commands based on where the robot is in the path
      true, // Mirror the path based on red or blue
      drivetrain); // Subsystem for requirements

  Command runAuto = autoBuilder.fullAuto(pathGroup);

  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = pathGroup.get(0).getInitialHolonomicPose();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(()->forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(()->forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }

  public boolean seedPoseButtonDown() {
    return joystick.leftBumper().getAsBoolean();
  }
}
