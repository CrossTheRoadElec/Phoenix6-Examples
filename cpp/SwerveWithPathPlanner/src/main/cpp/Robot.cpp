// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"
#include "LimelightHelpers.hpp"

#include "wpi/commands2/CommandScheduler.hpp"

Robot::Robot() {}

void Robot::RobotPeriodic() {
    timeAndJoystickReplay.Update();
    wpi::cmd::CommandScheduler::GetInstance().Run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (USE_LIMELIGHT) {
        auto const driveState = container.drivetrain.GetState();
        auto const heading = driveState.Pose.Rotation().Degrees();
        auto const omega = driveState.Velocity.omega;

        LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
        auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement && llMeasurement->tagCount > 0 && wpi::units::math::abs(omega) < 2_tps) {
            container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
        }
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    autonomousCommand = container.GetAutonomousCommand();

    if (autonomousCommand) {
        wpi::cmd::CommandScheduler::GetInstance().Schedule(autonomousCommand);
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (autonomousCommand) {
        wpi::cmd::CommandScheduler::GetInstance().Cancel(autonomousCommand);
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::UtilityInit() {
    wpi::cmd::CommandScheduler::GetInstance().CancelAll();
}

void Robot::UtilityPeriodic() {}

void Robot::UtilityExit() {}

#ifndef RUNNING_WPILIB_TESTS
int main() {
    return wpi::StartRobot<Robot>();
}
#endif
