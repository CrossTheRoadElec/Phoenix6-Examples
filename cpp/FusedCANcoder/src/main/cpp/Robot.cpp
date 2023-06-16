// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

using namespace ctre::phoenix6;

void Robot::RobotInit() {
  /* Configure CANcoder to zero the magnet appropriately */
  configs::CANcoderConfiguration cc_cfg{};
  cc_cfg.MagnetSensor.AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
  cc_cfg.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  cc_cfg.MagnetSensor.MagnetOffset = 0.4;
  m_cc.GetConfigurator().Apply(cc_cfg);

  configs::TalonFXConfiguration fx_cfg{};
  fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.GetDeviceID();
  fx_cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
  fx_cfg.Feedback.RotorToSensorRatio = 12.8;

  m_fx.GetConfigurator().Apply(fx_cfg);
}
void Robot::RobotPeriodic() {
  if (printCount++ > 10) {
    printCount = 0;
    // If any faults happen, print them out. Sticky faults will always be present if live-fault occurs
    f_fusedSensorOutOfSync.Refresh();
    sf_fusedSensorOutOfSync.Refresh();
    f_missingRemoteSensor.Refresh();
    sf_missingRemoteSensor.Refresh();
    bool anyFault = sf_fusedSensorOutOfSync.GetValue() || sf_missingRemoteSensor.GetValue();
    if(anyFault) {
      std::cout << "A fault has occurred:" << std::endl;;
      /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
      if(f_fusedSensorOutOfSync.GetValue()) {
        std::cout << "Fused sensor out of sync live-faulted" << std::endl;
      } else if (sf_fusedSensorOutOfSync.GetValue()) {
        std::cout << "Fused sensor out of sync sticky-faulted" << std::endl;
      }
      /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
      if(f_missingRemoteSensor.GetValue()) {
        std::cout << "Missing remote sensor live-faulted" << std::endl;
      } else if (sf_missingRemoteSensor.GetValue()) {
        std::cout << "Missing remote sensor sticky-faulted" << std::endl;
      }
    }

    if(m_joystick.GetAButton()) {
      /* Clear sticky faults */
      m_fx.ClearStickyFaults();
    }

    /* Print out current position and velocity */
    fx_pos.Refresh(); fx_vel.Refresh();
    cc_pos.Refresh(); cc_vel.Refresh();
    std::cout << "FX Position: " << fx_pos << " FX Vel: " << fx_vel << std::endl;
    std::cout << "CC Position: " << cc_pos << " CC Vel: " << cc_vel << std::endl;
    std::cout << "" << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  double output = m_joystick.GetLeftY();
  if (output > -0.1 && output < 0.1) output = 0;
  if(!m_joystick.GetBButton())
    output *= 0.1;
  m_fx.SetControl(m_dutyCycleControl.WithOutput(output));
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
