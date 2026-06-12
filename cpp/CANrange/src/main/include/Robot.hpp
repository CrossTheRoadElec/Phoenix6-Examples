// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/CANrange.hpp"
#include "wpi/framework/TimedRobot.hpp"
#include "wpi/system/Timer.hpp"

class Robot : public wpi::TimedRobot {
 private:
  ctre::phoenix6::hardware::CANrange canRange{1, ctre::phoenix6::CANBus::Systemcore(1)};
  wpi::units::time::second_t currentTime{wpi::Timer::GetTimestamp()};

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
