// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {

  if(m_joystick.GetLeftBumper()) {
    m_waitForAllTimeout = 0.1_s;
    std::cout << "Timeout is now at " << m_waitForAllTimeout.value() << std::endl;
  }
  if(m_joystick.GetRightBumper()) {
    m_waitForAllTimeout = 0_s;
    std::cout << "Timeout is now at " << m_waitForAllTimeout.value() << std::endl;
  }

  /* If we press the A button, test what happens when we wait on lots of signals (normal use case) */
    if(m_joystick.GetAButtonPressed()) {
      ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(m_waitForAllTimeout, m_lotsOfSignals);
      std::cout << "Status of waiting on signals (normal use case): " << status.GetName() << std::endl;
      for(auto const &sig : m_lotsOfSignals) {
        std::cout << "Signal status: " << sig->GetStatus().GetName() << std::endl;
      }
    }
    /* If we press the B button, test what happens when we wait on signals from different busses */
    if(m_joystick.GetBButtonPressed()) {
      ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(m_waitForAllTimeout, m_signalsAcrossCANbuses);
      std::cout << "Status of waiting on signals across different CAN busses: " << status.GetName() << std::endl;
      for(auto const& sig : m_signalsAcrossCANbuses) {
        std::cout << "Signal status: " << sig->GetStatus().GetName() << std::endl;
      }
    }
    /* If we press the Y button, test what happens when we wait on no signals */
    if(m_joystick.GetYButtonPressed()) {
      ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(m_waitForAllTimeout, m_noSignals);
      std::cout << "Status of waiting on no signals: " << status.GetName() << std::endl;
      for(auto const& sig : m_noSignals) {
        std::cout << "Signal status: " << sig->GetStatus().GetName() << std::endl;
      }
    }
    /* If we press the X button, test what happens when we wait on signals with the transcient motor controller */
    if(m_joystick.GetXButtonPressed()) {
      ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(m_waitForAllTimeout, {&m_canbus1signal1,
    &m_canbus1signal2,
    &m_canbus1transcient1,
    &m_canbus1transcient2});
      std::cout << "Status of waiting on transcient signals: " << status.GetName() << std::endl;
      for(auto const& sig : m_tanscientSignals) {
        std::cout << "Signal status: " << sig->GetStatus().GetName() << std::endl;
      }
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
