// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/phoenix6/TalonFX.hpp>

class Robot : public frc::TimedRobot {
  ctre::phoenix6::hardware::TalonFX m_motor{1, ctre::phoenix6::CANBus{"canivore"}};
  ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};
  frc::XboxController m_joystick{0};
  int m_printCount = 0;

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
