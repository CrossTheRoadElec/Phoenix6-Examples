// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  m_driveSubsystem.SetDefaultCommand(
    m_driveSubsystem.Run([this] {
      /* invert the joystick Y because forward Y is negative */
      m_driveSubsystem.ArcadeDrive(-m_joystick.GetLeftY(), m_joystick.GetRightX());
    })
  );

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
  // Configure your button bindings here
  m_joystick.LeftBumper().WhileTrue(
    DriveStraightCommand{
      m_driveSubsystem,
      [this] { return -m_joystick.GetLeftY(); }
    }.ToPtr()
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  return frc2::cmd::Print("No autonomous command configured");
}
