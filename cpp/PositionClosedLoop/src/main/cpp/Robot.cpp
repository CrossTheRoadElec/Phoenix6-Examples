// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

using namespace ctre::phoenix6;

void Robot::RobotInit() {
  configs::TalonFXConfiguration configs{};
  
  configs.Slot0.kP = 24; // An error of 0.5 rotations results in 12V output
  configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output

  configs.Voltage.PeakForwardVoltage = 8;  // Peak output of 8 volts
  configs.Voltage.PeakReverseVoltage = -8; // Peak output of 8 volts
  
  configs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
  configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 130;  // Peak output of 130 amps
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -130; // Peak output of 130 amps
  
  /* Percent supply gains when we get a Slot 2 */
  // configs.Slot1.kP = 1; // An error of 1 rotations results in 100% output
  // configs.Slot1.kD = 0.01; // A change of 1 rotation per second results in 1% output
  // configs.Slot1.PeakOutput = 0.7; // Peak output of 70% amps

  m_fx.GetConfigurator().Apply(configs);

  /* Make sure we start at 0 */
  m_fx.SetRotorPosition(0_tr);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  auto desiredRotations = m_joystick.GetLeftY() * 10_tr; // Go for plus/minus 10 rotations
  if (m_joystick.GetLeftBumper())
  {
    /* Use voltage position */
    m_fx.SetControl(m_voltagePosition.WithPosition(desiredRotations));
  }
  else if (m_joystick.GetRightBumper())
  {
    /* Use torque position */
    m_fx.SetControl(m_torquePosition.WithPosition(desiredRotations));
  }
  else
  {
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
