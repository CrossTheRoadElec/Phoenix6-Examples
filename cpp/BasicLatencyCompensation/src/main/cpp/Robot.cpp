// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

using namespace ctre::phoenix6;

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  /* Perform basic latency compensation based on latency and current derivative */
  /* First refresh the signal */
  BaseStatusSignal::RefreshAll(m_fxpos, m_fxvel, m_ccpos, m_ccvel, m_p2yaw, m_p2yawRate);

  /* Use the helper function to apply latency compensation to the signals */
  /* Since these are already refreshed we don't need to inline the refresh call */
  auto ccCompensatedPos = BaseStatusSignal::GetLatencyCompensatedValue(m_ccpos, m_ccvel);
  auto fxCompensatedPos = BaseStatusSignal::GetLatencyCompensatedValue(m_fxpos, m_fxvel);
  auto p2CompensatedYaw = BaseStatusSignal::GetLatencyCompensatedValue(m_p2yaw, m_p2yawRate);

  /* Print out both values so it shows how they perform */
  if (m_printCount++ > 10 && m_joystick.GetAButton()) {
    m_printCount = 0;
    printf("CANcoder: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f\n", m_ccpos.GetValue().value(), ccCompensatedPos.value(), (m_ccpos.GetValue() - ccCompensatedPos).value());
    printf("Talon FX: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f\n", m_fxpos.GetValue().value(), fxCompensatedPos.value(), (m_fxpos.GetValue() - fxCompensatedPos).value());
    printf("Pigeon2 : Yaw: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f\n", m_p2yaw.GetValue().value(), p2CompensatedYaw.value(), (m_p2yaw.GetValue() - p2CompensatedYaw).value());
    printf("\n\n");
  }
  m_fx.SetControl(m_dutycycle.WithOutput(m_joystick.GetLeftY()));

  if (m_joystick.GetLeftBumperButtonPressed()) {
    /* Speed up the signals to reduce the latency */
    /* Make them 1000 Hz (1 ms) for this example */
    BaseStatusSignal::SetUpdateFrequencyForAll(1000_Hz, m_fxpos, m_ccpos, m_p2yaw);
  }
  if (m_joystick.GetRightBumperButtonPressed()) {
    /* Slow down the signals to increase the latency */
    /* Make them 10 Hz (100 ms) for this example */
    BaseStatusSignal::SetUpdateFrequencyForAll(10_Hz, m_fxpos, m_ccpos, m_p2yaw);
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

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
