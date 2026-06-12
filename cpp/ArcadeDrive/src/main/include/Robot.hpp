// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "wpi/driverstation/NiDsXboxController.hpp"
#include "wpi/framework/TimedRobot.hpp"

class Robot : public wpi::TimedRobot {
 private:
  static constexpr ctre::phoenix6::CANBus CANBUS = ctre::phoenix6::CANBus::Systemcore(1);

  ctre::phoenix6::hardware::TalonFX leftLeader{1, CANBUS};
  ctre::phoenix6::hardware::TalonFX leftFollower{2, CANBUS};
  ctre::phoenix6::hardware::TalonFX rightLeader{3, CANBUS};
  ctre::phoenix6::hardware::TalonFX rightFollower{4, CANBUS};

  ctre::phoenix6::controls::DutyCycleOut leftOut{0}; // Initialize output to 0%
  ctre::phoenix6::controls::DutyCycleOut rightOut{0}; // Initialize output to 0%

  wpi::NiDsXboxController joystick{0};

  int printCount{};

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
