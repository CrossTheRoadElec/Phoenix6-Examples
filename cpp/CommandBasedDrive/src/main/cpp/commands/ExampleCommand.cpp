// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStraightCommand.h"

DriveStraightCommand::DriveStraightCommand(DriveSubsystem &subsystem,
                                           std::function<double()> throttle) : m_driveSubsystem{subsystem},
                                                                               m_throttle{throttle},
                                                                               m_yawGetter{subsystem.GetYaw()},
                                                                               m_holdYaw{0},
                                                                               m_driveStraightThread{[this]()
                                                                                                     { DriveStraightExecution(); }}
{
    AddRequirements(&subsystem);
}

void DriveStraightCommand::DriveStraightExecution()
{
    /* Get our current yaw and find the error from the yaw we want to hold */
    auto err = m_holdYaw - m_yawGetter.WaitForUpdate(MAX_UPDATE_PERIOD).GetUnitValue();
    /* Simple P-loop, where 100 degrees off corresponds to 100% output */
    auto kP{1.0 / 100_deg};
    double correction = err * kP;
    /* And apply it to the arcade drive */
    m_driveSubsystem.ArcadeDrive(m_throttle(), correction);
}

void DriveStraightCommand::Initialize()
{
    /* On initialize, latch the current yaw and begin correction */
    m_holdYaw = m_yawGetter.WaitForUpdate(MAX_UPDATE_PERIOD).GetUnitValue();
    /* Update as fast as possible, the waitForUpdate will manage the loop period */
    m_driveStraightThread.StartPeriodic(0_s);
}

void DriveStraightCommand::End(bool isInterrupted)
{
    /* Stop the notifier */
    m_driveStraightThread.Stop();
}
