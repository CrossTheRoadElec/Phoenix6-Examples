// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.hpp"

#include "wpi/commands2/Commands.hpp"

using namespace ctre::phoenix6;

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    /* Default command is duty cycle control with the left up/down stick */
    m_mechanism.SetDefaultCommand(m_mechanism.JoystickDriveCommand([this] { return m_joystick.GetLeftY(); }));

    /* Manually start logging with left bumper before running any tests,
     * and stop logging with right bumper after we're done with ALL tests.
     * This isn't necessary but is convenient to reduce the size of the hoot file.*/
    m_joystick.LeftBumper().OnTrue(wpi::cmd::RunOnce(SignalLogger::Start));
    m_joystick.RightBumper().OnTrue(wpi::cmd::RunOnce(SignalLogger::Stop));

    /*
     * Joystick Y = quasistatic forward
     * Joystick A = quasistatic reverse
     * Joystick B = dynamic forward
     * Joystick X = dyanmic reverse
     */
    m_joystick.Y().WhileTrue(m_mechanism.SysIdQuasistatic(wpi::cmd::sysid::Direction::kForward));
    m_joystick.A().WhileTrue(m_mechanism.SysIdQuasistatic(wpi::cmd::sysid::Direction::kReverse));
    m_joystick.B().WhileTrue(m_mechanism.SysIdDynamic(wpi::cmd::sysid::Direction::kForward));
    m_joystick.X().WhileTrue(m_mechanism.SysIdDynamic(wpi::cmd::sysid::Direction::kReverse));
}

wpi::cmd::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return wpi::cmd::Print("No autonomous command configured");
}
