// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

using namespace ctre::phoenixpro;

DriveSubsystem::DriveSubsystem()
{
    /* Initialize all the devices */
    InitializeLeftDriveTalonFX(m_leftLeader.GetConfigurator());
    InitializeLeftDriveTalonFX(m_leftFollower.GetConfigurator());
    InitializeRightDriveTalonFX(m_rightLeader.GetConfigurator());
    InitializeRightDriveTalonFX(m_rightFollower.GetConfigurator());
    InitializePigeon2(m_pigeon2.GetConfigurator());

    /* Set followers to follow leader */
    m_leftFollower.SetControl(controls::Follower{m_leftLeader.GetDeviceID(), false});
    m_rightFollower.SetControl(controls::Follower{m_rightLeader.GetDeviceID(), false});

    /* Make sure all critical signals are synchronized */
    /* Setting all these signals to 100hz means they get sent at the same time if they're all on a CANivore */
    m_pigeon2.GetYaw().SetUpdateFrequency(100_Hz);
    m_leftLeader.GetPosition().SetUpdateFrequency(100_Hz);
    m_rightLeader.GetPosition().SetUpdateFrequency(100_Hz);

    /* Set the update frequency of the main requests to 0 so updates are sent immediately in the arcadeDrive method */
    m_leftOut.UpdateFreqHz = 0_Hz;
    m_rightOut.UpdateFreqHz = 0_Hz;
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot)
{
    m_leftOut.output = fwd + rot;
    m_rightOut.output = fwd - rot;
    m_leftLeader.SetControl(m_leftOut);
    m_rightLeader.SetControl(m_rightOut);
}

void DriveSubsystem::InitializeLeftDriveTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator& cfg)
{
    configs::TalonFXConfiguration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */
    toApply.MotorOutput.Inverted = false;

    cfg.Apply(toApply);

    /* And initialize position to 0 */
    cfg.SetRotorPosition(0_tr);
}
void DriveSubsystem::InitializeRightDriveTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator& cfg)
{
    configs::TalonFXConfiguration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */
    toApply.MotorOutput.Inverted = true;

    cfg.Apply(toApply);

    /* And initialize position to 0 */
    cfg.SetRotorPosition(0_tr);
}
void DriveSubsystem::InitializePigeon2(ctre::phoenixpro::configs::Pigeon2Configurator& cfg)
{
    configs::Pigeon2Configuration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */

    cfg.Apply(toApply);

    /* And initialize yaw to 0 */
    cfg.SetYaw(0_deg);
}
