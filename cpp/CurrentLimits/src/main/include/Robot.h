// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenixpro/TalonFX.hpp>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

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

  ctre::phoenixpro::hardware::TalonFX m_fx{0, "Fred"};
  ctre::phoenixpro::controls::DutyCycleOut m_output{0};
  ctre::phoenixpro::configs::CurrentLimitsConfigs m_currentLimits{};
  
  frc::XboxController m_joystick{0};

  int printCount = 0;
};
