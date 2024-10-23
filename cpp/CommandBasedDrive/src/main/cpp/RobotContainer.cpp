// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  frc2::RunCommand teleopDrive {[this]()
                                 {
                                   /* invert the joystick Y because forward Y is negative */
                                   m_driveSubsystem.ArcadeDrive(-m_joystick.GetLeftY(), m_joystick.GetRightX());
                                 },
                                 {&m_driveSubsystem}};
  m_driveSubsystem.SetDefaultCommand(std::move(teleopDrive));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
  // Configure your button bindings here
  frc2::Trigger{[this]()
                { return m_joystick.GetLeftBumperButton(); }}
      .WhileTrue(&m_driveStraightCommand);
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  return nullptr;
}
