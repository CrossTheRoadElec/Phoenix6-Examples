// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

using namespace ctre::phoenixpro;

void Robot::RobotInit() {
  configs::TalonFXConfiguration configs{};

  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
  configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
  configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.0001 volts output
  configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  configs.Slot0.PeakOutput = 8; // Peak output of 8 volts
  
  /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
  configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
  configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
  configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
  configs.Slot1.PeakOutput = 40; // Peak output of 40 amps
  
  /* Percent supply gains when we get a Slot 2 */
  // configs.Slot1.kP = 0.01; // An error of 100 rotations per second results in 100% output
  // configs.Slot1.kI = 0.04; // An error of 1 rotation per second increases output by 0.04V every second
  // configs.Slot1.kD = 0.00001; // A change of 1 rotation per second squared results in 0.00001 volts output
  // configs.Slot1.kV = 0.013; // Approximately 1.3% for each rotation per second
  // configs.Slot1.PeakOutput = 0.7; // Peak output of 70%

  m_fx.GetConfigurator().Apply(configs);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  double joyValue = m_joystick.GetLeftY();
  if (joyValue > -0.1 && joyValue < 0.1) joyValue = 0;

  auto desiredRotationsPerSecond = joyValue * 50_tps; // Go for plus/minus 10 rotations per second

  if (m_joystick.GetLeftBumper())
  {
    /* Use voltage velocity */
    m_fx.SetControl(m_voltageVelocity.WithVelocity(desiredRotationsPerSecond));
  }
  else if (m_joystick.GetRightBumper())
  {
    auto friction_torque = (joyValue > 0) ? 1_A : -1_A; // To account for friction, we add this to the arbitrary feed forward
    /* Use torque velocity */
    m_fx.SetControl(m_torqueVelocity.WithVelocity(desiredRotationsPerSecond).WithFeedForward(friction_torque));
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
