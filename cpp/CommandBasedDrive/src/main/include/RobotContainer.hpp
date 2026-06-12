// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/commands2/button/CommandNiDsXboxController.hpp"
#include "wpi/commands2/Command.hpp"
#include "wpi/commands2/RunCommand.hpp"

#include "commands/DriveStraightCommand.hpp"
#include "subsystems/DriveSubsystem.hpp"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
private:
  // The robot's subsystems and joysticks are defined here...
  wpi::cmd::CommandNiDsXboxController m_joystick{0};

public:
  DriveSubsystem m_driveSubsystem{};

  RobotContainer();

  wpi::cmd::CommandPtr GetAutonomousCommand();

private:
  void ConfigureButtonBindings();
};
