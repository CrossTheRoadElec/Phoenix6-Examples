// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/commands2/Command.hpp"
#include "wpi/commands2/CommandHelper.hpp"
#include "wpi/system/Notifier.hpp"
#include "wpi/units/time.hpp"

#include "subsystems/DriveSubsystem.hpp"

class DriveStraightCommand : public wpi::cmd::CommandHelper<wpi::cmd::Command, DriveStraightCommand> {
private:
    static constexpr wpi::units::time::second_t MAX_UPDATE_PERIOD{0.050_s};

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
    ctre::phoenix6::StatusSignal<wpi::units::angle::degree_t> &m_yawGetter;
    wpi::units::angle::degree_t m_holdYaw;
    wpi::Notifier m_driveStraightThread;
};
