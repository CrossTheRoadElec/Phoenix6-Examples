// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

class Robot : public frc::TimedRobot {
  ctre::phoenix6::hardware::TalonFX m_fx{1, "canivore"};
  ctre::phoenix6::hardware::CANcoder m_cc{1, "canivore"};
  ctre::phoenix6::StatusSignal<bool> &f_fusedSensorOutOfSync = m_fx.GetFault_FusedSensorOutOfSync(false);
  ctre::phoenix6::StatusSignal<bool> &sf_fusedSensorOutOfSync = m_fx.GetStickyFault_FusedSensorOutOfSync(false);
  ctre::phoenix6::StatusSignal<bool> &f_remoteSensorInvalid = m_fx.GetFault_RemoteSensorDataInvalid(false);
  ctre::phoenix6::StatusSignal<bool> &sf_remoteSensorInvalid = m_fx.GetStickyFault_RemoteSensorDataInvalid(false);

  ctre::phoenix6::StatusSignal<units::turn_t> &fx_pos = m_fx.GetPosition(false);
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &fx_vel = m_fx.GetVelocity(false);
  ctre::phoenix6::StatusSignal<units::turn_t> &cc_pos = m_cc.GetPosition(false);
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> &cc_vel = m_cc.GetVelocity(false);

  ctre::phoenix6::controls::DutyCycleOut m_dutyCycleControl{0};

  frc::XboxController m_joystick{0};

  int printCount = 0;

 public:
  Robot();
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
};
