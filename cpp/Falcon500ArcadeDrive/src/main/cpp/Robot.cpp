// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

using namespace ctre::phoenixpro;

void Robot::RobotInit() {
  /* Configure devices */
  configs::TalonFXConfiguration leftConfiguration{};
  configs::TalonFXConfiguration rightConfiguration{};

  /* User can optionally change the configs, or leave it alone to perform a factory default */
  leftConfiguration.MotorOutput.Inverted = false;
  rightConfiguration.MotorOutput.Inverted = true;

  leftLeader.GetConfigurator().Apply(leftConfiguration);
  leftFollower.GetConfigurator().Apply(leftConfiguration);
  rightLeader.GetConfigurator().Apply(rightConfiguration);
  rightFollower.GetConfigurator().Apply(rightConfiguration);
    
  /* Currently in simulation, we do not support FOC, so disable it while simulating */
  if(ctre::phoenixpro::IsSimulation())
  {
    leftOut.EnableFOC = false;
    rightOut.EnableFOC = false;
  }

  /* Set up followers to follow leaders */
  leftFollower.SetControl(controls::Follower{leftLeader.GetDeviceID(), false});
  rightFollower.SetControl(controls::Follower{rightLeader.GetDeviceID(), false});
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  /* Get forward and rotational throttle from joystick */
  /* invert the joystick Y because forward Y is negative */
  double fwd = -joystick.GetLeftY();
  double rot = joystick.GetRightX();
  /* Set output to control frames */
  leftOut.Output = fwd + rot;
  rightOut.Output = fwd - rot;
  if(!joystick.GetAButton())
  {
    /* And set them to the motors */
    leftLeader.SetControl(leftOut);
    rightLeader.SetControl(rightOut);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {
  /* Zero out controls so we aren't just relying on the enable frame */
  leftOut.Output = 0;
  rightOut.Output = 0;
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
