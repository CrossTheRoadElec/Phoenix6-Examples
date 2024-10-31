// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  /*
   * This example of adding Limelight is very simple and may not be sufficient for on-field use.
   * Users typically need to provide a standard deviation that scales with the distance to target
   * and changes with number of tags available.
   *
   * This example is sufficient to show that vision integration is possible, though exact implementation
   * of how to use vision should be tuned per-robot and to the team's specification.
   */
  if (kUseLimelight) {
    auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight");
    if (llMeasurement) {
      m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, utils::FPGAToCurrentTime(llMeasurement->timestampSeconds));
    }
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
