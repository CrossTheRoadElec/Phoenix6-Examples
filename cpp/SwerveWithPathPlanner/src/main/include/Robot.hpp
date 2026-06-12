// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "RobotContainer.hpp"

#include "ctre/phoenix6/HootAutoReplay.hpp"
#include "wpi/commands2/CommandPtr.hpp"
#include "wpi/framework/TimedRobot.hpp"

class Robot : public wpi::TimedRobot {
public:
    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void UtilityInit() override;
    void UtilityPeriodic() override;
    void UtilityExit() override;

private:
    static constexpr bool USE_LIMELIGHT = false;

    wpi::cmd::Command *autonomousCommand;

    RobotContainer container;

    /* log and replay timestamp and joystick data */
    ctre::phoenix6::HootAutoReplay timeAndJoystickReplay = ctre::phoenix6::HootAutoReplay{}
        .WithTimestampReplay()
        .WithJoystickReplay();
};
