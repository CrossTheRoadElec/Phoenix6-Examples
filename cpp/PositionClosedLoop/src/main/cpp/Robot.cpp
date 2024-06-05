// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

using namespace ctre::phoenix6;

void Robot::RobotInit() {
  configs::TalonFXConfiguration configs{};
  configs.Slot0.kP = 2.4; // An error of 1 rotations results in 1.2 V output
  configs.Slot0.kI = 0; // No output for integrated error
  configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
  // Peak output of 8 V
  configs.Voltage.PeakForwardVoltage = 8;
  configs.Voltage.PeakReverseVoltage = -8;

  configs.Slot1.kP = 60; // An error of 1 rotations results in 60 A output
  configs.Slot1.kI = 0; // No output for integrated error
  configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
  // Peak output of 120 amps
  configs.TorqueCurrent.PeakForwardTorqueCurrent = 120;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -120;

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_fx.GetConfigurator().Apply(configs);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status.GetName() << std::endl;
  }

  /* Make sure we start at 0 */
  m_fx.SetPosition(0_tr);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  auto desiredRotations = m_joystick.GetLeftY() * 10_tr; // Go for plus/minus 10 rotations
  if (units::math::abs(desiredRotations) <= 0.1_tr) { // joystick deadzone
    desiredRotations = 0_tr;
  }

  if (m_joystick.GetLeftBumper()) {
    /* Use position voltage */
    m_fx.SetControl(m_positionVoltage.WithPosition(desiredRotations));
  } else if (m_joystick.GetRightBumper()) {
    /* Use position torque */
    m_fx.SetControl(m_positionTorque.WithPosition(desiredRotations));
  } else {
    /* Disable the motor instead */
    m_fx.SetControl(m_brake);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
