// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

using namespace ctre::phoenixpro;

void Robot::RobotInit() {
  /* Configure devices */
  configs::TalonFXConfiguration appliedConfiguration{};

  /* User can optionally change the configs, or leave it alone to perform a factory default */

  leftLeader.GetConfigurator().Apply(appliedConfiguration);
  leftFollower.GetConfigurator().Apply(appliedConfiguration);
  rightLeader.GetConfigurator().Apply(appliedConfiguration);
  rightFollower.GetConfigurator().Apply(appliedConfiguration);

  /* Set up followers to follow leaders */
  leftFollower.SetControl(controls::Follower{leftLeader.GetDeviceID(), false});
  rightFollower.SetControl(controls::Follower{rightLeader.GetDeviceID(), false});
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  /* Get throttle and wheel from joystick */
  double throttle = joystick.GetLeftY();
  double wheel = joystick.GetRightX();
  /* Set output to control frames */
  leftOut.output = throttle + wheel;
  rightOut.output = throttle - wheel;
  /* And set them to the motors */
  leftLeader.SetControl(leftOut);
  rightLeader.SetControl(rightOut);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {
  /* Zero out controls so we aren't just relying on the enable frame */
  leftOut.output = 0;
  rightOut.output = 0;
  leftLeader.SetControl(leftOut);
  rightLeader.SetControl(rightOut);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
