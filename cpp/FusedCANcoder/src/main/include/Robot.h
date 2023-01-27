// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/phoenixpro/TalonFX.hpp>
#include <ctre/phoenixpro/CANcoder.hpp>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;


  ctre::phoenixpro::hardware::TalonFX m_fx{1, "fred"};
  ctre::phoenixpro::hardware::CANcoder m_cc{1, "fred"};
  ctre::phoenixpro::StatusSignalValue<bool> f_fusedSensorOutOfSync = m_fx.GetFault_FusedSensorOutOfSync();
  ctre::phoenixpro::StatusSignalValue<bool> sf_fusedSensorOutOfSync = m_fx.GetStickyFault_FusedSensorOutOfSync();
  ctre::phoenixpro::StatusSignalValue<bool> f_missingRemoteSensor = m_fx.GetFault_MissingRemoteSensor();
  ctre::phoenixpro::StatusSignalValue<bool> sf_missingRemoteSensor = m_fx.GetStickyFault_MissingRemoteSensor();

  ctre::phoenixpro::StatusSignalValue<units::turn_t> fx_pos = m_fx.GetPosition();
  ctre::phoenixpro::StatusSignalValue<units::turns_per_second_t> fx_vel = m_fx.GetVelocity();
  ctre::phoenixpro::StatusSignalValue<units::turn_t> cc_pos = m_cc.GetPosition();
  ctre::phoenixpro::StatusSignalValue<units::turns_per_second_t> cc_vel = m_cc.GetVelocity();

  ctre::phoenixpro::controls::DutyCycleOut m_dutyCycleControl{0};

  frc::XboxController m_joystick{0};

  int printCount = 0;
};
