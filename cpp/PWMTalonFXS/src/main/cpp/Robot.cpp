// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

Robot::Robot() {
  // Configure the connected motor for Talon FXS
  m_motor.SetMotorArrangement(wpi::PWMTalonFXS::MotorArrangement::Minion_JST);

  // Set neutral mode to brake
  m_motor.SetNeutralMode(true);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // Command motor output
  m_motor.SetThrottle(-m_joy.GetLeftY());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::UtilityInit() {}
void Robot::UtilityPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_WPILIB_TESTS
int main() {
  return wpi::StartRobot<Robot>();
}
#endif
