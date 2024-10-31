// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

using namespace ctre::phoenix6;

void Robot::RobotInit() {
  configs::TalonFXConfiguration configs{};

  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
  configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
  configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
  configs.Slot0.kI = 0; // No output for integrated error
  configs.Slot0.kD = 0; // No output for error derivative
  // Peak output of 8 volts
  configs.Voltage.PeakForwardVoltage = 8_V;
  configs.Voltage.PeakReverseVoltage = -8_V;
  
  /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
  configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
  configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
  configs.Slot1.kI = 0; // No output for integrated error
  configs.Slot1.kD = 0; // No output for error derivative
  // Peak output of 40 A
  configs.TorqueCurrent.PeakForwardTorqueCurrent = 40_A;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -40_A;

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_fx.GetConfigurator().Apply(configs);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status.GetName() << std::endl;
  }

  m_fllr.SetControl(controls::Follower{m_fx.GetDeviceID(), false});
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  double joyValue = m_joystick.GetLeftY();
  if (fabs(joyValue) < 0.1) joyValue = 0;

  auto desiredRotationsPerSecond = joyValue * 50_tps; // Go for plus/minus 50 rotations per second

  if (m_joystick.GetLeftBumperButton()) {
    /* Use velocity voltage */
    m_fx.SetControl(m_velocityVoltage.WithVelocity(desiredRotationsPerSecond));
  } else if (m_joystick.GetRightBumperButton()) {
    /* Use velocity torque */
    m_fx.SetControl(m_velocityTorque.WithVelocity(desiredRotationsPerSecond));
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
