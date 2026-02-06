// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

class Robot : public frc::TimedRobot {
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

private:
  ctre::phoenix6::hardware::TalonFX m_fx{0, ctre::phoenix6::CANBus{"*"}};
  ctre::phoenix6::hardware::CANcoder m_cc{1, ctre::phoenix6::CANBus{"*"}};
  ctre::phoenix6::controls::DutyCycleOut m_out{0};

  frc::XboxController m_joystick{0};
};
