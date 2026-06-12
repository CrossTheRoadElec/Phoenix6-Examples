// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"
#include <iostream>

using namespace ctre::phoenix6;

Robot::Robot() {
  /* Configure devices */
  configs::TalonFXConfiguration leftConfiguration{};
  configs::TalonFXConfiguration rightConfiguration{};

  /* User can optionally change the configs, or leave it alone to perform a factory default */
  leftConfiguration.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  rightConfiguration.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

  leftLeader.GetConfigurator().Apply(leftConfiguration);
  leftFollower.GetConfigurator().Apply(leftConfiguration);
  rightLeader.GetConfigurator().Apply(rightConfiguration);
  rightFollower.GetConfigurator().Apply(rightConfiguration);

  /* Set up followers to follow leaders */
  leftFollower.SetControl(controls::Follower{leftLeader.GetDeviceID(), false});
  rightFollower.SetControl(controls::Follower{rightLeader.GetDeviceID(), false});
}

void Robot::RobotPeriodic() {
  if (++printCount >= 10) {
    printCount = 0;
    std::cout << "Left out: " << leftLeader.GetThrottle() << std::endl;
    std::cout << "Right out: " << rightLeader.GetThrottle() << std::endl;
    std::cout << "Left Pos: " << leftLeader.GetPosition() << std::endl;
    std::cout << "Right Pos: " << rightLeader.GetPosition() << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  /* Get forward and rotational throttle from joystick */
  /* invert the joystick X/Y because forward Y is negative and left X is negative */
  double fwd = -joystick.GetLeftY();
  double rot = -joystick.GetRightX();
  /* Set output to control frames */
  leftOut.Output = fwd - rot;
  rightOut.Output = fwd + rot;
  if (!joystick.GetAButton()) {
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

void Robot::UtilityInit() {}
void Robot::UtilityPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_WPILIB_TESTS
int main() {
  return wpi::StartRobot<Robot>();
}
#endif
