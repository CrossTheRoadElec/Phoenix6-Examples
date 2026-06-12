// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include "wpi/driverstation/NiDsXboxController.hpp"
#include "wpi/framework/TimedRobot.hpp"

class Robot : public wpi::TimedRobot {
  static constexpr ctre::phoenix6::CANBus CANBUS{""};
  ctre::phoenix6::hardware::CANcoder m_cc{0, CANBUS};
  ctre::phoenix6::hardware::TalonFX m_fx{0, CANBUS};
  ctre::phoenix6::hardware::Pigeon2 m_p2{0, CANBUS};
  int m_printCount = 0;

  ctre::phoenix6::controls::DutyCycleOut m_dutycycle{0};

  wpi::NiDsXboxController m_joystick{0};

  ctre::phoenix6::StatusSignal<wpi::units::turn_t> &m_ccpos = m_cc.GetPosition();
  ctre::phoenix6::StatusSignal<wpi::units::turn_t> &m_fxpos = m_fx.GetPosition();
  ctre::phoenix6::StatusSignal<wpi::units::degree_t> &m_p2yaw = m_p2.GetYaw();
  ctre::phoenix6::StatusSignal<wpi::units::turns_per_second_t> &m_ccvel = m_cc.GetVelocity();
  ctre::phoenix6::StatusSignal<wpi::units::turns_per_second_t> &m_fxvel = m_fx.GetVelocity();
  ctre::phoenix6::StatusSignal<wpi::units::degrees_per_second_t> &m_p2yawRate = m_p2.GetAngularVelocityZWorld();

 public:
  Robot();
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void UtilityInit() override;
  void UtilityPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};
