// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/DriveSubsystem.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Notifier.h>
#include <units/time.h>

class DriveStraightCommand : public frc2::CommandHelper<frc2::Command, DriveStraightCommand>
{
private:
    static constexpr units::time::second_t MAX_UPDATE_PERIOD{0.050_s};

public:
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit DriveStraightCommand(DriveSubsystem &subsystem, std::function<double()> throttle);

    void Initialize() override;
    void End(bool isInterrupted) override;

private:
    void DriveStraightExecution();

    DriveSubsystem &m_driveSubsystem;
    std::function<double()> m_throttle;
    ctre::phoenix6::StatusSignal<units::angle::degree_t> &m_yawGetter;
    units::angle::degree_t m_holdYaw;
    frc::Notifier m_driveStraightThread;
};
