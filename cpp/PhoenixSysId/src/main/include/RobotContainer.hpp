// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/commands2/CommandPtr.hpp"
#include "wpi/commands2/button/CommandNiDsXboxController.hpp"

#include "subsystems/FlywheelMechanism.hpp"

class RobotContainer {
public:
  RobotContainer();

  wpi::cmd::CommandPtr GetAutonomousCommand();

private:
  wpi::cmd::CommandNiDsXboxController m_joystick{0};
  FlywheelMechanism m_mechanism{};

  void ConfigureBindings();
};
