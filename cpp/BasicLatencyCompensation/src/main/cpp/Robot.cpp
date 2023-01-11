// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  /* Perform basic latency compensation based on latency and current derivative */
  /* First refresh the signal */
  m_ccpos.Refresh();
  m_fxpos.Refresh();
  m_p2yaw.Refresh();

  /* Then get the rates of each of the signal */
  auto ccVel = m_cc.GetVelocity().GetValue();
  auto fxVel = m_fx.GetVelocity().GetValue();
  /**
   * Pigeon2 can only perform this latency compensation if the Z axis is straight up, since the
   * angular velocity Z value comes from the pre-mount orientation gyroscope.
   * For more information on what signals have what algorithms applied to them,
   * see section 1.6 of the Pigeon 2's User's Guide
   * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf 
   */
  auto p2Rate = m_p2.GetAngularVelocityZ().GetValue();

  /* Multiply the latency (in seconds) by the rates (in seconds) to get the amount to offset by */
  /* This automatically uses the best timestamp, CANivore will perform better than RIO since its timestamp */
  /* is more accurate */
  auto ccPosOffset = ccVel * m_ccpos.GetTimestamp().GetLatency();
  auto fxPosOffset = fxVel * m_fxpos.GetTimestamp().GetLatency();
  auto p2YawOffset = p2Rate * m_p2yaw.GetTimestamp().GetLatency();

  /* And add it to the current signal to get the latency-compensated signal */
  auto ccCompensatedPos = m_ccpos.GetValue() + ccPosOffset;
  auto fxCompensatedPos = m_fxpos.GetValue() + fxPosOffset;
  auto p2CompensatedYaw = m_p2yaw.GetValue() + p2YawOffset;

  /* Print out both values so it shows how they perform */
  if(m_printCount++ > 10 && m_joystick.GetAButton()) {
    m_printCount = 0;
    printf("CANcoder: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f\n", m_ccpos.GetValue().value(), ccCompensatedPos.value(), ccPosOffset.value());
    printf("Talon FX: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f\n", m_fxpos.GetValue().value(), fxCompensatedPos.value(), fxPosOffset.value());
    printf("Pigeon2 : Yaw: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f\n", m_p2yaw.GetValue().value(), p2CompensatedYaw.value(), p2YawOffset.value());
    printf("\n\n");
  }
  m_fx.SetControl(m_dutycycle.WithOutput(m_joystick.GetLeftY()));

  if(m_joystick.GetLeftBumperPressed()) {
    /* Speed up the signals to reduce the latency */
    m_fxpos.SetUpdateFrequency(1000_Hz); // Make it 1ms for this example
    m_ccpos.SetUpdateFrequency(1000_Hz); // Make it 1ms for this example
    m_p2yaw.SetUpdateFrequency(1000_Hz); // Make it 1ms for this example
  }
  if(m_joystick.GetRightBumperPressed()) {
    /* Slow down the signals to increase the latency */
    m_fxpos.SetUpdateFrequency(10_Hz); // Make it 100ms for this example
    m_ccpos.SetUpdateFrequency(10_Hz); // Make it 100ms for this example
    m_p2yaw.SetUpdateFrequency(10_Hz); // Make it 100ms for this example
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
